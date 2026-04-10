/**
 * @file blinds_controller.c
 * @brief Window blind controller implementation.
 *
 * State machine overview:
 *
 *   +--------+   manual/auto trigger    +--------------+
 *   |  IDLE  | -----------------------> | MOVING_UP    |
 *   |        | <--- top limit hit ----  +--------------+
 *   |        |
 *   |        |   manual/auto trigger    +--------------+
 *   |        | -----------------------> | MOVING_DOWN  |
 *   |        | <- bottom limit hit ---  +--------------+
 *   +--------+
 *
 * Manual mode:
 *   - Entered on any button press.
 *   - Motor runs while the button is held.
 *   - Motor stops on button release (or limit switch).
 *   - Returns to AUTO after BLINDS_MANUAL_TIMEOUT_S seconds of no button
 *     activity.
 *
 * Auto mode:
 *   - Between BLINDS_DAYTIME_START_HOUR and BLINDS_NIGHTTIME_START_HOUR:
 *     motor runs UP until the top limit switch fires.
 *   - Outside that window: motor runs DOWN until the bottom limit switch fires.
 *   - Once at the correct limit, the motor stays idle until the schedule
 *     transitions.
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

#include "blinds_controller.h"
#include "button_debouncer.h"
#include "h_bridge_controller.h"
#include "DS3231_HAL.h"
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/** Number of task calls that make up BLINDS_MANUAL_TIMEOUT_S. */
#define MANUAL_TIMEOUT_CALLS \
    ((uint32_t)(BLINDS_MANUAL_TIMEOUT_S) * 1000U / (BLINDS_TASK_PERIOD_MS))

/*******************************************************************************/
/*                               DATA TYPES                                    */
/*******************************************************************************/

typedef enum
{
    BLINDS_STATE_IDLE = 0,
    BLINDS_STATE_MOVING_UP,
    BLINDS_STATE_MOVING_DOWN,
} Blinds_InternalState;

typedef enum
{
    BLINDS_MODE_AUTO = 0,
    BLINDS_MODE_MANUAL,
} Blinds_Mode;

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

static void         motor_stop(void);
static void         motor_start_up(void);
static void         motor_start_down(void);
static bool         is_daytime(uint8_t hour);
static void         handle_limit_switches(bool at_top, bool at_bottom);
static void         handle_manual_buttons(bool at_top, bool at_bottom);
static void         handle_auto_control(bool at_top, bool at_bottom);
static void         poll_rtc(void);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

static Blinds_InternalState s_state            = BLINDS_STATE_IDLE;
static Blinds_Mode          s_mode             = BLINDS_MODE_AUTO;

/* Cached RTC hour — updated every BLINDS_RTC_POLL_CALLS task calls. */
static uint8_t              s_current_hour     = 0U;
static uint8_t              s_rtc_poll_count   = 0U;

/* Manual mode inactivity counter. Counts task calls since the last button
 * press/release event. When it reaches MANUAL_TIMEOUT_CALLS the controller
 * reverts to AUTO. */
static uint32_t             s_manual_idle_count = 0U;

/*******************************************************************************/
/*                          GLOBAL FUNCTION DEFINITIONS                        */
/*******************************************************************************/

Blinds_Status Blinds_Init(void)
{
    /* --- 1. Button debouncer ------------------------------------------------ */
    if (Debounce_Init() != DEBOUNCE_OK)
    {
        LOG("[Blinds] ERROR: Debounce_Init failed.\n");
        return BLINDS_ERROR_DEBOUNCE_INIT;
    }

    if (Debounce_AddButton(BLINDS_BTN_UP_PIN, GPIO_PULL_UP, true) != DEBOUNCE_OK)
    {
        LOG("[Blinds] ERROR: Failed to register BTN_UP (GP%u).\n", BLINDS_BTN_UP_PIN);
        return BLINDS_ERROR_DEBOUNCE_INIT;
    }
    if (Debounce_AddButton(BLINDS_BTN_DOWN_PIN, GPIO_PULL_UP, true) != DEBOUNCE_OK)
    {
        LOG("[Blinds] ERROR: Failed to register BTN_DOWN (GP%u).\n", BLINDS_BTN_DOWN_PIN);
        return BLINDS_ERROR_DEBOUNCE_INIT;
    }
    if (Debounce_AddButton(BLINDS_LIMIT_TOP_PIN, GPIO_PULL_UP, true) != DEBOUNCE_OK)
    {
        LOG("[Blinds] ERROR: Failed to register LIMIT_TOP (GP%u).\n", BLINDS_LIMIT_TOP_PIN);
        return BLINDS_ERROR_DEBOUNCE_INIT;
    }
    if (Debounce_AddButton(BLINDS_LIMIT_BOTTOM_PIN, GPIO_PULL_UP, true) != DEBOUNCE_OK)
    {
        LOG("[Blinds] ERROR: Failed to register LIMIT_BOTTOM (GP%u).\n", BLINDS_LIMIT_BOTTOM_PIN);
        return BLINDS_ERROR_DEBOUNCE_INIT;
    }

    /* --- 2. H-bridge motor driver ------------------------------------------ */
    if (HBridge_Init() != HBRIDGE_OK)
    {
        LOG("[Blinds] ERROR: HBridge_Init failed.\n");
        return BLINDS_ERROR_MOTOR_INIT;
    }
    if (HBridge_SetEnable(HBRIDGE_SIDE_BOTH, true) != HBRIDGE_OK)
    {
        LOG("[Blinds] ERROR: HBridge_SetEnable failed.\n");
        return BLINDS_ERROR_MOTOR_INIT;
    }

    /* --- 3. DS3231 RTC ------------------------------------------------------ */
    DS3231_Status rtc_status = DS3231_Init();
    if (rtc_status != DS3231_OK)
    {
        /* DS3231_Init clears the OSF flag; a return value other than OK means
         * the device is not responding or the I2C bus failed — treat as fatal. */
        LOG("[Blinds] ERROR: DS3231_Init failed (status=%d).\n", (int)rtc_status);
        return BLINDS_ERROR_RTC_INIT;
    }

    /* Prime the cached hour so the first auto decision uses real data. */
    DS3231_DateTime dt;
    if (DS3231_GetDateTime(&dt) == DS3231_OK)
    {
        s_current_hour = dt.hours;
    }

    LOG("[Blinds] Initialized. Mode: AUTO. Current hour: %u.\n", s_current_hour);
    return BLINDS_OK;
}

void Blinds_MainFunction(void)
{
    /* Snapshot debounced limit-switch states once per cycle. */
    bool at_top    = Debounce_IsHeld(BLINDS_LIMIT_TOP_PIN);
    bool at_bottom = Debounce_IsHeld(BLINDS_LIMIT_BOTTOM_PIN);

    /* Process in priority order:
     *  1. Safety stop at limits (highest priority — always checked).
     *  2. Manual button commands.
     *  3. Auto schedule (only when manual is idle). */
    handle_limit_switches(at_top, at_bottom);
    handle_manual_buttons(at_top, at_bottom);
    poll_rtc();
    handle_auto_control(at_top, at_bottom);
}

/*******************************************************************************/
/*                         STATIC FUNCTION DEFINITIONS                         */
/*******************************************************************************/

/**
 * @brief Stop motor and return to IDLE / AUTO when a limit switch fires
 *        while the motor is running.
 */
static void handle_limit_switches(bool at_top, bool at_bottom)
{
    if (s_state == BLINDS_STATE_MOVING_UP && at_top)
    {
        motor_stop();
        s_state = BLINDS_STATE_IDLE;
        s_mode  = BLINDS_MODE_AUTO; /* Position is known — safe to hand back to auto. */
        s_manual_idle_count = 0U;
        LOG("[Blinds] TOP limit reached. Motor stopped.\n");
    }
    else if (s_state == BLINDS_STATE_MOVING_DOWN && at_bottom)
    {
        motor_stop();
        s_state = BLINDS_STATE_IDLE;
        s_mode  = BLINDS_MODE_AUTO;
        s_manual_idle_count = 0U;
        LOG("[Blinds] BOTTOM limit reached. Motor stopped.\n");
    }
}

/**
 * @brief Handle UP/DOWN button presses, holds, and releases.
 *
 * A button press puts the controller into MANUAL mode and starts the motor.
 * The motor runs for as long as the button is held.  On release the motor
 * stops and the manual-inactivity timer starts.  After BLINDS_MANUAL_TIMEOUT_S
 * seconds of no button activity the mode reverts to AUTO.
 */
static void handle_manual_buttons(bool at_top, bool at_bottom)
{
    bool up_pressed   = Debounce_Pressed(BLINDS_BTN_UP_PIN);
    bool down_pressed = Debounce_Pressed(BLINDS_BTN_DOWN_PIN);
    bool up_held      = Debounce_IsHeld(BLINDS_BTN_UP_PIN);
    bool down_held    = Debounce_IsHeld(BLINDS_BTN_DOWN_PIN);

    /* Any button event resets the inactivity timer. */
    if (up_pressed || down_pressed || up_held || down_held)
    {
        s_manual_idle_count = 0U;
    }

    /* --- UP button pressed ------------------------------------------------- */
    if (up_pressed && !at_top)
    {
        /* Stop any ongoing downward movement before reversing. */
        if (s_state == BLINDS_STATE_MOVING_DOWN)
        {
            motor_stop();
        }
        s_mode  = BLINDS_MODE_MANUAL;
        s_state = BLINDS_STATE_MOVING_UP;
        motor_start_up();
        LOG("[Blinds] Manual UP.\n");
    }

    /* --- DOWN button pressed ----------------------------------------------- */
    if (down_pressed && !at_bottom)
    {
        if (s_state == BLINDS_STATE_MOVING_UP)
        {
            motor_stop();
        }
        s_mode  = BLINDS_MODE_MANUAL;
        s_state = BLINDS_STATE_MOVING_DOWN;
        motor_start_down();
        LOG("[Blinds] Manual DOWN.\n");
    }

    /* --- Stop when both buttons released (manual mode only) ---------------- */
    if (s_mode == BLINDS_MODE_MANUAL && s_state != BLINDS_STATE_IDLE)
    {
        if (!up_held && !down_held)
        {
            motor_stop();
            s_state = BLINDS_STATE_IDLE;
            LOG("[Blinds] Button released. Motor stopped (manual idle).\n");
        }
    }

    /* --- Manual mode timeout ----------------------------------------------- */
    if (s_mode == BLINDS_MODE_MANUAL && s_state == BLINDS_STATE_IDLE)
    {
        s_manual_idle_count++;
        if (s_manual_idle_count >= MANUAL_TIMEOUT_CALLS)
        {
            s_mode = BLINDS_MODE_AUTO;
            s_manual_idle_count = 0U;
            LOG("[Blinds] Manual timeout. Returning to AUTO.\n");
        }
    }
}

/**
 * @brief Periodically read the DS3231 RTC and cache the current hour.
 */
static void poll_rtc(void)
{
    s_rtc_poll_count++;
    if (s_rtc_poll_count >= BLINDS_RTC_POLL_CALLS)
    {
        s_rtc_poll_count = 0U;
        DS3231_DateTime dt;
        if (DS3231_GetDateTime(&dt) == DS3231_OK)
        {
            s_current_hour = dt.hours;
        }
        else
        {
            LOG("[Blinds] WARNING: RTC read failed.\n");
        }
    }
}

/**
 * @brief Drive the motor toward the target position dictated by the schedule.
 *
 * Only acts when in AUTO mode and the motor is already idle.
 * The motor will run until the relevant limit switch fires (handled by
 * handle_limit_switches() on the next cycle).
 */
static void handle_auto_control(bool at_top, bool at_bottom)
{
    if (s_mode != BLINDS_MODE_AUTO) return;
    if (s_state != BLINDS_STATE_IDLE)  return;

    if (is_daytime(s_current_hour) && !at_top)
    {
        motor_start_up();
        s_state = BLINDS_STATE_MOVING_UP;
        LOG("[Blinds] Auto: raising blinds (hour=%u).\n", s_current_hour);
    }
    else if (!is_daytime(s_current_hour) && !at_bottom)
    {
        motor_start_down();
        s_state = BLINDS_STATE_MOVING_DOWN;
        LOG("[Blinds] Auto: lowering blinds (hour=%u).\n", s_current_hour);
    }
}

/* --- Motor helpers --------------------------------------------------------- */

static void motor_stop(void)
{
    HBridge_Stop();
}

static void motor_start_up(void)
{
    HBridge_SetSpeed((uint8_t)BLINDS_MOTOR_SPEED, BLINDS_MOTOR_DIR_UP);
}

static void motor_start_down(void)
{
    HBridge_SetSpeed((uint8_t)BLINDS_MOTOR_SPEED, BLINDS_MOTOR_DIR_DOWN);
}

/**
 * @brief Return true during the configured daytime window.
 * @param hour  Current hour in 24 h format (0–23).
 */
static bool is_daytime(uint8_t hour)
{
    return (hour >= BLINDS_DAYTIME_START_HOUR) &&
           (hour <  BLINDS_NIGHTTIME_START_HOUR);
}
