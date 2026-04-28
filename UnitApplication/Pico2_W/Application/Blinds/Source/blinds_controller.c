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
 *   - Between local sunrise and (local sunset + BLINDS_SUNSET_OFFSET_HOURS):
 *     motor runs UP until the top limit switch fires.
 *   - Outside that window: motor runs DOWN until the bottom limit switch fires.
 *   - Once at the correct limit, the motor stays idle until the schedule
 *     transitions.
 *   - Sunrise/sunset are recomputed from the DS3231 date whenever the date
 *     rolls over; DST is applied automatically (EU rules).
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

#include "blinds_controller.h"
#include "button_debouncer.h"
#include "h_bridge_controller.h"
#include "DS3231_HAL.h"
#include "VEML7700.h"
#include "time_calculations.h"
#include "flash_operations.h"
#include "Common.h"

/*******************************************************************************/
/*                                 MACROS                                      */
/*******************************************************************************/

/** Number of task calls that make up BLINDS_MANUAL_TIMEOUT_S. */
#define MANUAL_TIMEOUT_CALLS \
    ((uint32_t)(BLINDS_MANUAL_TIMEOUT_S) * 1000U / (BLINDS_TASK_PERIOD_MS))

/** Number of task calls within which a second tap counts as a double-tap. */
#define DOUBLE_TAP_WINDOW_CALLS \
    ((uint32_t)(BLINDS_DOUBLE_TAP_WINDOW_MS) / (BLINDS_TASK_PERIOD_MS))

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

typedef enum
{
    SUN_STATE_IDLE = 0,   /* Monitoring lux; no event active.                   */
    SUN_STATE_GOING_DOWN, /* Motor moving to BLINDS_SUN_HALFWAY_PERMILLE.        */
    SUN_STATE_AT_HALFWAY, /* Holding at halfway; waiting for lux to drop.        */
    SUN_STATE_RETURNING,  /* Motor moving back to the saved position.            */
    SUN_STATE_CANCELLED,  /* Event interrupted; waiting for lux to clear.        */
} Sun_State;

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

static void         motor_stop(void);
static void         motor_start_up(void);
static void         motor_start_down(void);
static bool         is_daytime(float time_of_day_h);
static float        rtc_decimal_hours(const DS3231_DateTime *dt);
static void         recalculate_sun_schedule(const DS3231_DateTime *dt);
static void         handle_limit_switches(bool at_top, bool at_bottom);
static void         handle_manual_buttons(bool at_top, bool at_bottom);
static void         handle_sun_control(bool at_top, bool at_bottom);
static void         handle_auto_control(bool at_top, bool at_bottom);
static void         poll_rtc(void);

/* Motion bookkeeping — wrap motor_start_up/down so every motion is timed,
 * watchdogged, and contributes to position tracking / calibration. */
static void         start_motion(Blinds_InternalState dir, bool at_top, bool at_bottom);
static void         stop_motion(void);
static void         motion_update_position(void);
static bool         motion_check_watchdog(void);
static void         motion_try_calibrate(bool reached_top);
static uint32_t     expected_travel_ms(Blinds_InternalState dir);

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/

static Blinds_InternalState s_state            = BLINDS_STATE_IDLE;
static Blinds_Mode          s_mode             = BLINDS_MODE_AUTO;

/* Cached RTC snapshot — refreshed every BLINDS_RTC_POLL_CALLS task calls. */
static DS3231_DateTime      s_rtc_snapshot     = {0};
static uint8_t              s_rtc_poll_count   = 0U;

/* Cached sun-schedule for the current calendar day (decimal hours, local time).
 * s_sunset_cutoff_hours already includes BLINDS_SUNSET_OFFSET_HOURS. The date
 * for which they were computed is tracked so poll_rtc() can detect a rollover
 * and recompute. s_schedule_date == 0 means "not yet computed". */
static float                s_sunrise_hours       = 0.0f;
static float                s_sunset_cutoff_hours = 0.0f;
static uint8_t              s_schedule_date       = 0U;
static uint8_t              s_schedule_month      = 0U;

/* Manual mode inactivity counter. Counts task calls since the last button
 * press/release event. When it reaches MANUAL_TIMEOUT_CALLS the controller
 * reverts to AUTO. */
static uint32_t             s_manual_idle_count = 0U;

/* Double-tap full-run state.
 * s_up_tap_countdown / s_down_tap_countdown: decremented every task call after
 *   a single tap; a second tap while non-zero triggers a full run.
 * s_full_run: true ONLY during a double-tap full-run (NOT during auto). It
 *   keeps the motor going past button release and gates the interrupt-on-
 *   press block. Auto runs use the normal manual-button branches for
 *   interruption — overloading s_full_run for auto would make any button
 *   press merely stop-and-restart the auto run. */
static uint32_t             s_up_tap_countdown   = 0U;
static uint32_t             s_down_tap_countdown = 0U;
static bool                 s_full_run           = false;

/* Latched on a watchdog trip. Blocks ALL subsequent motor operation — auto,
 * double-tap, AND hold-to-move — until power-cycle or reflash. A trip means
 * something has gone seriously wrong (broken limit, mechanical jam); silently
 * resuming would risk driving the motor into the end stop again. Reset only
 * by reboot (this variable lives in RAM). */
static bool                 s_fault_latched = false;

/* Monotonic task-cycle counter. Used as the time base for watchdog and
 * calibration measurements (each tick = BLINDS_TASK_PERIOD_MS). */
static uint32_t             s_call_count         = 0U;

/* Travel-time calibration (ms), loaded from flash at init and updated
 * opportunistically when a motion completes from one limit to the other.
 * 0 means "uncalibrated" — the watchdog falls back to BLINDS_TRAVEL_DEFAULT_MS. */
static uint32_t             s_travel_up_ms       = 0U;
static uint32_t             s_travel_down_ms     = 0U;

/* Snapshot of the active motion. Captured by start_motion() and consumed by
 * motion_check_watchdog() / motion_try_calibrate() / motion_update_position(). */
static uint32_t             s_motion_start_calls    = 0U;
static bool                 s_motion_started_at_top    = false;
static bool                 s_motion_started_at_bottom = false;
static uint16_t             s_motion_start_position    = BLINDS_POSITION_UNKNOWN;

/* Dead-reckoning position estimate, 0 (bottom) .. 1000 (top). Re-zeroed at
 * every limit hit; invalidated to BLINDS_POSITION_UNKNOWN by a watchdog trip
 * or whenever a motion starts from an unknown position with no calibration. */
static uint16_t             s_position_permille  = BLINDS_POSITION_UNKNOWN;

/* Auto-schedule state.
 * s_auto_target_reached: set when the blinds arrive at the auto target for
 *   the current period (top during daytime, bottom during nighttime). Cleared
 *   when the period transitions so the next target triggers a fresh movement.
 * s_auto_last_daytime: tracks which period was active when the last auto
 *   movement completed, used to detect the transition. */
static bool                 s_auto_target_reached = false;
static bool                 s_auto_last_daytime   = false;

/* Sun automation state.
 * s_sun_state:          current phase of the sun-event state machine.
 * s_sun_saved_position: position at the moment the sun event was triggered;
 *                        the blinds return here when lux drops.
 * s_sun_poll_count:     call counter for throttling VEML7700 reads.
 * s_sun_lux:            most recent valid lux reading (0.0 on init, so the
 *                        event cannot fire until the first successful poll).
 * s_sun_sensor_ok:      false if VEML7700_Init failed — disables sun logic. */
static Sun_State            s_sun_state          = SUN_STATE_IDLE;
static uint16_t             s_sun_saved_position = BLINDS_POSITION_UNKNOWN;
static uint32_t             s_sun_poll_count     = 0U;
static float                s_sun_lux            = 0.0f;
static bool                 s_sun_sensor_ok      = false;

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

    /* Prime the cached snapshot and sun-schedule so the first auto decision
     * uses real data. */
    /* On 13.04.2026 when I set the RTC, it was exactly synced with https://time.is/ (Polish time)
     * We'll see how this DS3231 drifts over time - I'm very curious! (TODO - check this in the future!) */
    DS3231_DateTime dt;
    if (DS3231_GetDateTime(&dt) == DS3231_OK)
    {
        s_rtc_snapshot = dt;
        recalculate_sun_schedule(&dt);
        LOG("[Blinds] RTC: 20%02u-%02u-%02u %02u:%02u:%02u\n",
            dt.year, dt.month, dt.date,
            dt.hours, dt.minutes, dt.seconds);
    }
    else
    {
        LOG("[Blinds] WARNING: RTC read failed at init. Auto schedule unset.\n");
    }

    /* Seed the period tracker so the first boot triggers one auto movement
     * to bring the blinds to the correct position for the current period. */
    s_auto_last_daytime   = !is_daytime(rtc_decimal_hours(&s_rtc_snapshot)); /* force mismatch */
    s_auto_target_reached = false;

    /* --- 4. VEML7700 ambient light sensor (non-fatal — sun automation is
     *        disabled if the sensor is absent or fails to respond). ----------- */
    VEML7700_Status veml_status = VEML7700_Init();
    if (veml_status == VEML7700_OK)
    {
        s_sun_sensor_ok = true;
        LOG("[Blinds] VEML7700 ambient light sensor ready.\n");
    }
    else
    {
        LOG("[Blinds] WARNING: VEML7700 init failed (status=%d). Sun automation disabled.\n",
            (int)veml_status);
    }

    /* --- 5. Travel-time calibration ---------------------------------------- */
    if (read_blinds_calibration(&s_travel_up_ms, &s_travel_down_ms))
    {
        LOG("[Blinds] Calibration loaded: T_up=%u ms, T_down=%u ms.\n",
            (unsigned)s_travel_up_ms, (unsigned)s_travel_down_ms);
    }
    else
    {
        s_travel_up_ms   = 0U;
        s_travel_down_ms = 0U;
        LOG("[Blinds] No travel calibration in flash. Using default %u ms watchdog "
            "until first opportunistic calibration completes.\n",
            (unsigned)BLINDS_TRAVEL_DEFAULT_MS);
    }

    LOG("[Blinds] Initialized. Mode: AUTO.\n");
    return BLINDS_OK;
}

void Blinds_MainFunction(void)
{
    s_call_count++;

    /* Snapshot debounced limit-switch states once per cycle. */
    bool at_top    = Debounce_IsHeld(BLINDS_LIMIT_TOP_PIN);
    bool at_bottom = Debounce_IsHeld(BLINDS_LIMIT_BOTTOM_PIN);

    /* Process in priority order:
     *  1. Watchdog (catches a runaway motor before any other state change
     *     this cycle — e.g. broken limit switch).
     *  2. Safety stop at limits.
     *  3. Manual button commands (incl. double-tap full-run gesture).
     *  4. Sun automation (lux-triggered half-position event).
     *  5. Auto schedule (only when manual is idle and no sun event is active).
     *  6. Update dead-reckoned position estimate. */
    (void)motion_check_watchdog();
    handle_limit_switches(at_top, at_bottom);
    handle_manual_buttons(at_top, at_bottom);
    poll_rtc();
    handle_sun_control(at_top, at_bottom);
    handle_auto_control(at_top, at_bottom);
    motion_update_position();
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
    float now_h = rtc_decimal_hours(&s_rtc_snapshot);

    if (s_state == BLINDS_STATE_MOVING_UP && at_top)
    {
        motion_try_calibrate(true);   /* before stop_motion clears the snapshot */
        stop_motion();
        s_mode  = BLINDS_MODE_AUTO;   /* Position is known — safe to hand back to auto. */
        s_manual_idle_count    = 0U;
        s_auto_target_reached  = true;  /* Don't raise again until day/night flips. */
        s_auto_last_daytime    = is_daytime(now_h);
        s_position_permille    = 1000U;  /* Re-zero dead reckoning. */
        LOG("[Blinds] TOP limit reached. Motor stopped.\n");
    }
    else if (s_state == BLINDS_STATE_MOVING_DOWN && at_bottom)
    {
        motion_try_calibrate(false);
        stop_motion();
        s_mode  = BLINDS_MODE_AUTO;
        s_manual_idle_count    = 0U;
        s_auto_target_reached  = true;  /* Don't lower again until day/night flips. */
        s_auto_last_daytime    = is_daytime(now_h);
        s_position_permille    = 0U;
        LOG("[Blinds] BOTTOM limit reached. Motor stopped.\n");
    }
}

/**
 * @brief Handle UP/DOWN button presses, holds, and releases.
 *
 * Single tap: motor runs while the button is held; stops on release.
 * Double-tap (second press within BLINDS_DOUBLE_TAP_WINDOW_MS): motor runs to
 *   the corresponding limit switch without needing to hold the button.
 *   Any subsequent press interrupts the full run.
 * After a watchdog trip (s_fault_latched): all input is ignored until reboot.
 * After BLINDS_MANUAL_TIMEOUT_S seconds of button inactivity the mode reverts
 *   to AUTO.
 */
static void handle_manual_buttons(bool at_top, bool at_bottom)
{
    /* Fault latch: once the watchdog has tripped, refuse to drive the motor
     * again until the device is power-cycled. See motion_check_watchdog(). */
    if (s_fault_latched) return;

    bool up_pressed   = Debounce_Pressed(BLINDS_BTN_UP_PIN);
    bool down_pressed = Debounce_Pressed(BLINDS_BTN_DOWN_PIN);
    bool up_held      = Debounce_IsHeld(BLINDS_BTN_UP_PIN);
    bool down_held    = Debounce_IsHeld(BLINDS_BTN_DOWN_PIN);

    /* Advance double-tap countdown windows. */
    if (s_up_tap_countdown   > 0U) s_up_tap_countdown--;
    if (s_down_tap_countdown > 0U) s_down_tap_countdown--;

    /* Any button event resets the inactivity timer. */
    if (up_pressed || down_pressed || up_held || down_held)
    {
        s_manual_idle_count = 0U;
    }

    /* --- Interrupt a full run on any button press -------------------------- */
    if (s_full_run && (up_pressed || down_pressed))
    {
        stop_motion();
        s_up_tap_countdown   = 0U;
        s_down_tap_countdown = 0U;
        LOG("[Blinds] Full run interrupted by button press.\n");
        return;
    }

    /* --- UP button pressed ------------------------------------------------- */
    if (up_pressed && !at_top)
    {
        if (s_up_tap_countdown > 0U)
        {
            /* Double-tap: upgrade to full run — motor keeps going without hold. */
            s_up_tap_countdown = 0U;
            if (s_state != BLINDS_STATE_IDLE) stop_motion();  /* clean transition */
            s_full_run = true;
            s_mode     = BLINDS_MODE_MANUAL;
            start_motion(BLINDS_STATE_MOVING_UP, at_top, at_bottom);
            LOG("[Blinds] Full run UP triggered.\n");
        }
        else
        {
            /* First tap: normal hold-to-move. Clear the opposite countdown so a
             * direction change can't accidentally arm a full run. */
            s_up_tap_countdown   = DOUBLE_TAP_WINDOW_CALLS;
            s_down_tap_countdown = 0U;
            if (s_state != BLINDS_STATE_IDLE)
            {
                stop_motion();
                s_auto_target_reached = true;
            }
            s_mode = BLINDS_MODE_MANUAL;
            start_motion(BLINDS_STATE_MOVING_UP, at_top, at_bottom);
            LOG("[Blinds] Manual UP.\n");
        }
    }

    /* --- DOWN button pressed ----------------------------------------------- */
    if (down_pressed && !at_bottom)
    {
        if (s_down_tap_countdown > 0U)
        {
            /* Double-tap: upgrade to full run. */
            s_down_tap_countdown = 0U;
            if (s_state != BLINDS_STATE_IDLE) stop_motion();
            s_full_run = true;
            s_mode     = BLINDS_MODE_MANUAL;
            start_motion(BLINDS_STATE_MOVING_DOWN, at_top, at_bottom);
            LOG("[Blinds] Full run DOWN triggered.\n");
        }
        else
        {
            s_down_tap_countdown = DOUBLE_TAP_WINDOW_CALLS;
            s_up_tap_countdown   = 0U;
            if (s_state != BLINDS_STATE_IDLE)
            {
                stop_motion();
                s_auto_target_reached = true;
            }
            s_mode = BLINDS_MODE_MANUAL;
            start_motion(BLINDS_STATE_MOVING_DOWN, at_top, at_bottom);
            LOG("[Blinds] Manual DOWN.\n");
        }
    }

    /* --- Stop on button release (hold-to-move only, not during full run) --- */
    if (s_mode == BLINDS_MODE_MANUAL && s_state != BLINDS_STATE_IDLE && !s_full_run)
    {
        if (!up_held && !down_held)
        {
            stop_motion();
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
 * @brief Periodically read the DS3231 RTC and refresh the cached snapshot.
 *        Recomputes the sunrise/sunset schedule whenever the calendar date
 *        changes (so the rollover that matters happens at local midnight).
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
            s_rtc_snapshot = dt;
            if (dt.date != s_schedule_date || dt.month != s_schedule_month)
            {
                recalculate_sun_schedule(&dt);
            }
        }
        else
        {
            LOG("[Blinds] WARNING: RTC read failed.\n");
        }
    }
}

/**
 * @brief Lux-triggered sun-position automation.
 *
 * When the VEML7700 reads above BLINDS_SUN_LUX_THRESHOLD the blinds are
 * lowered to BLINDS_SUN_HALFWAY_PERMILLE (if they are currently above it).
 * The prior position is saved and restored once lux falls below
 * (BLINDS_SUN_LUX_THRESHOLD - BLINDS_SUN_LUX_HYSTERESIS).
 *
 * The event is one-shot: any manual button activity while an event is in
 * progress transitions to SUN_STATE_CANCELLED, which suppresses re-arming
 * until lux drops below the hysteresis floor and then rises again.
 *
 * State machine:
 *   IDLE → (lux high, above halfway, auto+idle) → GOING_DOWN
 *   GOING_DOWN → (position reached halfway)     → AT_HALFWAY
 *   GOING_DOWN → (manual interruption)          → CANCELLED
 *   AT_HALFWAY → (lux low)                      → RETURNING
 *   AT_HALFWAY → (manual interruption)          → CANCELLED
 *   RETURNING  → (position reached saved)       → IDLE
 *   RETURNING  → (manual interruption)          → CANCELLED
 *   CANCELLED  → (lux low)                      → IDLE  (re-armed)
 */
static void handle_sun_control(bool at_top, bool at_bottom)
{
    if (!s_sun_sensor_ok) return;
    if (s_fault_latched)  return;

    /* Throttled lux poll — keep s_sun_lux up-to-date regardless of state. */
    s_sun_poll_count++;
    if (s_sun_poll_count >= BLINDS_SUN_POLL_CALLS)
    {
        s_sun_poll_count = 0U;
        float lux;
        if (VEML7700_GetLuxCorrected(&lux) == VEML7700_OK)
        {
            s_sun_lux = lux;
        }
    }

    bool lux_high = (s_sun_lux >= BLINDS_SUN_LUX_THRESHOLD);
    bool lux_low  = (s_sun_lux <  (BLINDS_SUN_LUX_THRESHOLD - BLINDS_SUN_LUX_HYSTERESIS));

    /* The time-based auto schedule must always win at day/night boundaries.
     * Cancel any active event when night begins so the sunset lowering is
     * never blocked or delayed by a lingering sun event. */
    if (s_sun_state != SUN_STATE_IDLE && s_sun_state != SUN_STATE_CANCELLED)
    {
        if (!is_daytime(rtc_decimal_hours(&s_rtc_snapshot)))
        {
            if (s_state != BLINDS_STATE_IDLE) stop_motion();
            s_sun_saved_position = BLINDS_POSITION_UNKNOWN;
            s_sun_state          = SUN_STATE_IDLE;
            LOG("[Sun] Night began. Sun event cleared; auto schedule takes over.\n");
            return;
        }
    }

    switch (s_sun_state)
    {
        case SUN_STATE_IDLE:
            if (lux_high
                && s_mode  == BLINDS_MODE_AUTO
                && s_state == BLINDS_STATE_IDLE
                && s_position_permille != BLINDS_POSITION_UNKNOWN
                && s_position_permille >  BLINDS_SUN_HALFWAY_PERMILLE)
            {
                s_sun_saved_position = s_position_permille;
                s_sun_state          = SUN_STATE_GOING_DOWN;
                start_motion(BLINDS_STATE_MOVING_DOWN, at_top, at_bottom);
                LOG("[Sun] %.0f lx >= threshold. Lowering to %u/1000. "
                    "Saved position: %u.\n",
                    (double)s_sun_lux,
                    (unsigned)BLINDS_SUN_HALFWAY_PERMILLE,
                    (unsigned)s_sun_saved_position);
            }
            break;

        case SUN_STATE_GOING_DOWN:
            if (s_mode == BLINDS_MODE_MANUAL)
            {
                /* Motor already stopped by handle_manual_buttons(). */
                s_sun_state = SUN_STATE_CANCELLED;
                LOG("[Sun] Lowering interrupted by manual input. Event cancelled.\n");
                break;
            }
            if (s_position_permille != BLINDS_POSITION_UNKNOWN
                && s_position_permille <= BLINDS_SUN_HALFWAY_PERMILLE)
            {
                stop_motion();
                s_sun_state = SUN_STATE_AT_HALFWAY;
                LOG("[Sun] Reached halfway (%u/1000). Waiting for lux to drop.\n",
                    (unsigned)s_position_permille);
            }
            break;

        case SUN_STATE_AT_HALFWAY:
            if (s_mode == BLINDS_MODE_MANUAL)
            {
                s_sun_state = SUN_STATE_CANCELLED;
                LOG("[Sun] Manual input while at halfway. Event cancelled.\n");
                break;
            }
            if (lux_low)
            {
                if (s_sun_saved_position != BLINDS_POSITION_UNKNOWN
                    && s_sun_saved_position > BLINDS_SUN_HALFWAY_PERMILLE)
                {
                    s_sun_state = SUN_STATE_RETURNING;
                    start_motion(BLINDS_STATE_MOVING_UP, at_top, at_bottom);
                    LOG("[Sun] %.0f lx below threshold. Returning to %u/1000.\n",
                        (double)s_sun_lux, (unsigned)s_sun_saved_position);
                }
                else
                {
                    s_sun_state          = SUN_STATE_IDLE;
                    s_sun_saved_position = BLINDS_POSITION_UNKNOWN;
                    LOG("[Sun] Lux dropped. No valid saved position. Event complete.\n");
                }
            }
            break;

        case SUN_STATE_RETURNING:
            if (s_mode == BLINDS_MODE_MANUAL)
            {
                s_sun_state = SUN_STATE_CANCELLED;
                LOG("[Sun] Return interrupted by manual input. Event cancelled.\n");
                break;
            }
            if (at_top
                || (s_position_permille != BLINDS_POSITION_UNKNOWN
                    && s_position_permille >= s_sun_saved_position))
            {
                stop_motion();
                s_sun_saved_position = BLINDS_POSITION_UNKNOWN;
                s_sun_state          = SUN_STATE_IDLE;
                LOG("[Sun] Returned to saved position. Event complete.\n");
            }
            break;

        case SUN_STATE_CANCELLED:
            if (lux_low)
            {
                s_sun_state = SUN_STATE_IDLE;
                LOG("[Sun] Lux cleared after cancellation. Re-armed.\n");
            }
            break;
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
    /* Fault latch blocks auto too — otherwise the scheduler would happily
     * retry against a broken limit switch and retrip indefinitely. */
    if (s_fault_latched) return;
    if (s_mode != BLINDS_MODE_AUTO) return;
    if (s_state != BLINDS_STATE_IDLE)  return;

    /* Sun automation takes priority during an active event — prevent auto from
     * fighting the sun position or lifting the blinds while they are at halfway. */
    if (s_sun_state != SUN_STATE_IDLE && s_sun_state != SUN_STATE_CANCELLED) return;

    float now_h  = rtc_decimal_hours(&s_rtc_snapshot);
    bool  daytime = is_daytime(now_h);

    /* Detect a day/night transition and allow one new auto movement. */
    if (daytime != s_auto_last_daytime)
    {
        s_auto_last_daytime   = daytime;
        s_auto_target_reached = false;
        LOG("[Blinds] Auto: schedule period changed. daytime=%d.\n", (int)daytime);
    }

    /* Already at the target for the current period — nothing to do. */
    if (s_auto_target_reached) return;

    if (daytime && !at_top)
    {
        /* Note: s_full_run stays false. Auto runs are interrupted via the
         * normal manual-button branches (which set s_mode = MANUAL and
         * s_auto_target_reached = true), not via the full-run interrupt
         * block. The watchdog still bounds the run — it triggers off
         * s_state != IDLE and does not depend on s_full_run. */
        start_motion(BLINDS_STATE_MOVING_UP, at_top, at_bottom);
        LOG("[Blinds] Auto: raising blinds (%02u:%02u).\n",
            s_rtc_snapshot.hours, s_rtc_snapshot.minutes);
    }
    else if (!daytime && !at_bottom)
    {
        start_motion(BLINDS_STATE_MOVING_DOWN, at_top, at_bottom);
        LOG("[Blinds] Auto: lowering blinds (%02u:%02u).\n",
            s_rtc_snapshot.hours, s_rtc_snapshot.minutes);
    }
    else
    {
        /* Already at the correct limit on first check — nothing to do. */
        s_auto_target_reached = true;
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
 * @brief Return true when the given local time-of-day falls between sunrise
 *        and (sunset + BLINDS_SUNSET_OFFSET_HOURS).
 * @param time_of_day_h  Local time of day as decimal hours (0.0–24.0).
 */
static bool is_daytime(float time_of_day_h)
{
    return (time_of_day_h >= s_sunrise_hours) &&
           (time_of_day_h <  s_sunset_cutoff_hours);
}

/**
 * @brief Convert a DS3231 snapshot to local time-of-day in decimal hours.
 */
static float rtc_decimal_hours(const DS3231_DateTime *dt)
{
    return (float)dt->hours
         + (float)dt->minutes / 60.0f
         + (float)dt->seconds / 3600.0f;
}

/**
 * @brief Recompute cached sunrise / (sunset + offset) for the given date.
 */
static void recalculate_sun_schedule(const DS3231_DateTime *dt)
{
    int  day_of_year = (int)CalculateDayOfYear(dt->year, dt->month, dt->date);
    bool dst_active  = isDST(dt->year, dt->month, dt->date);
    int  timezone    = TIME_ZONE_PLUS_TO_E + (dst_active ? 1 : 0);

    float sunrise_h = 0.0f;
    float sunset_h  = 0.0f;
    CalculateSunriseSunset((float)LATITUDE_SIEROSZEWICE, (float)LONGITUDE_SIEROSZEWICE,
                           day_of_year, timezone, &sunrise_h, &sunset_h);

    s_sunrise_hours       = sunrise_h;
    s_sunset_cutoff_hours = sunset_h + BLINDS_SUNSET_OFFSET_HOURS;
    s_schedule_date       = dt->date;
    s_schedule_month      = dt->month;

    uint8_t sr_h = (uint8_t)sunrise_h;
    uint8_t sr_m = (uint8_t)((sunrise_h - (float)sr_h) * 60.0f);
    uint8_t ss_h = (uint8_t)s_sunset_cutoff_hours;
    uint8_t ss_m = (uint8_t)((s_sunset_cutoff_hours - (float)ss_h) * 60.0f);
    LOG("[Blinds] Sun schedule for %02u-%02u: sunrise=%02u:%02u, lower-at=%02u:%02u (DST=%d).\n",
        dt->month, dt->date, sr_h, sr_m, ss_h, ss_m, (int)dst_active);
}

/* --- Motion bookkeeping --------------------------------------------------- */

/**
 * @brief Travel time (ms) the watchdog and position estimator should assume
 *        for the given direction. Falls back to the conservative default
 *        when no calibration is available yet.
 */
static uint32_t expected_travel_ms(Blinds_InternalState dir)
{
    if (dir == BLINDS_STATE_MOVING_UP)
    {
        return (s_travel_up_ms != 0U) ? s_travel_up_ms : BLINDS_TRAVEL_DEFAULT_MS;
    }
    return (s_travel_down_ms != 0U) ? s_travel_down_ms : BLINDS_TRAVEL_DEFAULT_MS;
}

/**
 * @brief Start a motion in the given direction. Captures the call-count
 *        timestamp, the limit-switch state at start (used by the
 *        opportunistic calibrator), and the starting position (used as the
 *        anchor for dead reckoning so direction changes don't accumulate
 *        rounding error).
 */
static void start_motion(Blinds_InternalState dir, bool at_top, bool at_bottom)
{
    s_state                    = dir;
    s_motion_start_calls       = s_call_count;
    s_motion_started_at_top    = at_top;
    s_motion_started_at_bottom = at_bottom;
    s_motion_start_position    = s_position_permille;

    if (dir == BLINDS_STATE_MOVING_UP) motor_start_up();
    else                               motor_start_down();
}

/**
 * @brief Stop the motor and clear motion state. Does NOT touch position,
 *        calibration, or the lock-out flag — those are owned by the limit
 *        switch handler and the watchdog.
 */
static void stop_motion(void)
{
    motor_stop();
    s_state    = BLINDS_STATE_IDLE;
    s_full_run = false;
}

/**
 * @brief Integrate the dead-reckoned position estimate from the motion's
 *        start anchor. Recomputed from start each tick (rather than
 *        accumulated) to avoid integer-division drift.
 */
static void motion_update_position(void)
{
    if (s_state == BLINDS_STATE_IDLE) return;
    if (s_motion_start_position == BLINDS_POSITION_UNKNOWN) return;

    uint32_t elapsed_ms = (s_call_count - s_motion_start_calls) * BLINDS_TASK_PERIOD_MS;
    uint32_t t_full_ms  = expected_travel_ms(s_state);
    uint32_t delta      = (1000U * elapsed_ms) / t_full_ms;

    if (s_state == BLINDS_STATE_MOVING_UP)
    {
        uint32_t pos = (uint32_t)s_motion_start_position + delta;
        s_position_permille = (pos > 1000U) ? 1000U : (uint16_t)pos;
    }
    else
    {
        if (delta >= s_motion_start_position) s_position_permille = 0U;
        else                                  s_position_permille = (uint16_t)(s_motion_start_position - delta);
    }
}

/**
 * @brief Trip the watchdog if the current motion has run longer than
 *        BLINDS_TRAVEL_WATCHDOG_PCT % of the expected travel time. On a trip:
 *        stops the motor, marks position as unknown, and latches
 *        s_fault_latched, which blocks all further motor operation until the
 *        device is power-cycled or reflashed. The trip means something has
 *        gone seriously wrong (broken limit, mechanical jam); resuming
 *        automatically would risk further mechanical damage.
 */
static bool motion_check_watchdog(void)
{
    if (s_state == BLINDS_STATE_IDLE) return false;

    uint32_t elapsed_ms  = (s_call_count - s_motion_start_calls) * BLINDS_TASK_PERIOD_MS;
    uint32_t expected_ms = expected_travel_ms(s_state);
    uint32_t limit_ms    = (expected_ms * BLINDS_TRAVEL_WATCHDOG_PCT) / 100U;
    if (elapsed_ms < limit_ms) return false;

    Blinds_InternalState dir = s_state;
    stop_motion();
    s_position_permille  = BLINDS_POSITION_UNKNOWN;
    s_fault_latched      = true;
    s_up_tap_countdown   = 0U;
    s_down_tap_countdown = 0U;
    LOG("[Blinds] SAFETY: watchdog tripped after %u ms moving %s (expected %u ms). "
        "Motor stopped. FAULT LATCHED — all motor operation disabled until "
        "power-cycle or reflash.\n",
        (unsigned)elapsed_ms,
        (dir == BLINDS_STATE_MOVING_UP) ? "UP" : "DOWN",
        (unsigned)expected_ms);
    return true;
}

/**
 * @brief Opportunistic calibration: if the motion that just reached this
 *        limit started at the opposite limit, the elapsed time is a valid
 *        full-travel measurement. Stored in RAM and persisted to flash if
 *        it differs meaningfully from what's already stored.
 */
static void motion_try_calibrate(bool reached_top)
{
    bool valid_start = reached_top ? s_motion_started_at_bottom
                                   : s_motion_started_at_top;
    if (!valid_start) return;

    uint32_t elapsed_ms = (s_call_count - s_motion_start_calls) * BLINDS_TASK_PERIOD_MS;
    if (elapsed_ms < BLINDS_TRAVEL_MIN_MS || elapsed_ms > BLINDS_TRAVEL_MAX_MS)
    {
        LOG("[Blinds] Calibration: rejecting implausible measurement (%u ms).\n",
            (unsigned)elapsed_ms);
        return;
    }

    uint32_t *stored = reached_top ? &s_travel_up_ms : &s_travel_down_ms;
    uint32_t  old_val = *stored;
    *stored = elapsed_ms;

    uint32_t delta = (elapsed_ms > old_val) ? (elapsed_ms - old_val)
                                            : (old_val - elapsed_ms);
    if (old_val == 0U || delta >= BLINDS_TRAVEL_WRITE_DELTA_MS)
    {
        if (write_blinds_calibration(s_travel_up_ms, s_travel_down_ms))
        {
            LOG("[Blinds] Calibration saved: T_up=%u ms, T_down=%u ms.\n",
                (unsigned)s_travel_up_ms, (unsigned)s_travel_down_ms);
        }
        else
        {
            LOG("[Blinds] WARNING: failed to persist calibration to flash.\n");
        }
    }
}