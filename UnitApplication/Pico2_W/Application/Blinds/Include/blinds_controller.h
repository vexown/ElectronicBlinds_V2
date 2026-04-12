/**
 * @file blinds_controller.h
 * @brief Window blind controller for the Electronic Blinds V2 project.
 *
 * Controls a 12 V DC motor through a BTS7960B H-bridge to raise and lower
 * window blinds automatically based on time of day (DS3231 RTC) and
 * manually via UP/DOWN buttons.
 *
 * @section Pins
 *
 * | Signal          | GPIO | Direction |
 * |-----------------|------|-----------|
 * | Button UP       | GP21 | Input, pull-up, active-low |
 * | Button DOWN     | GP20 | Input, pull-up, active-low |
 * | Limit switch A  | GP15 | Input, pull-up, active-low (top position)    |
 * | Limit switch B  | GP14 | Input, pull-up, active-low (bottom position) |
 * | LPWM (forward)  | GP16 | PWM out (managed by h_bridge_controller)     |
 * | RPWM (reverse)  | GP17 | PWM out (managed by h_bridge_controller)     |
 * | L_EN            | GP18 | Digital out (managed by h_bridge_controller) |
 * | R_EN            | GP19 | Digital out (managed by h_bridge_controller) |
 * | I2C0 SDA        | GP4  | I2C (managed by DS3231_HAL)                 |
 * | I2C0 SCL        | GP5  | I2C (managed by DS3231_HAL)                 |
 *
 * @note GP4/GP5 belong to I2C0 on the RP2350; the DS3231_HAL is pre-configured
 *       for I2C_INSTANCE_0 on those pins.
 *
 * @section Usage
 *
 *   // In your FreeRTOS task:
 *   void blindsTask(void *params)
 *   {
 *       Blinds_Status s = Blinds_Init();
 *       if (s != BLINDS_OK) { // handle error }
 *
 *       TickType_t xLastWakeTime = xTaskGetTickCount();
 *       for (;;)
 *       {
 *           vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(BLINDS_TASK_PERIOD_MS));
 *           Blinds_MainFunction();
 *       }
 *   }
 *
 * @section AutoSchedule
 *
 * Between BLINDS_DAYTIME_START_HOUR and BLINDS_NIGHTTIME_START_HOUR the
 * controller raises the blinds; outside that window it lowers them.
 * Adjust the hour constants below to match your location/preference.
 *
 * @section ManualOverride
 *
 * Pressing UP or DOWN switches to manual mode and runs the motor while the
 * button is held (or until the corresponding limit switch fires).
 * After BLINDS_MANUAL_TIMEOUT_S seconds of button inactivity the controller
 * returns to automatic mode.
 */

#ifndef BLINDS_CONTROLLER_H
#define BLINDS_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************/
/*                                 PIN DEFINITIONS                             */
/*******************************************************************************/

/** @brief GP21 — Button UP, active-low, internal pull-up. */
#define BLINDS_BTN_UP_PIN           21U

/** @brief GP20 — Button DOWN, active-low, internal pull-up. */
#define BLINDS_BTN_DOWN_PIN         20U

/** @brief GP15 — Limit switch A (top position), active-low, internal pull-up. */
#define BLINDS_LIMIT_TOP_PIN        15U

/** @brief GP14 — Limit switch B (bottom position), active-low, internal pull-up. */
#define BLINDS_LIMIT_BOTTOM_PIN     14U

/*******************************************************************************/
/*                             AUTOMATIC SCHEDULE                              */
/*******************************************************************************/

/** @brief Hour (0–23) at which the blinds are raised each day. */
#define BLINDS_DAYTIME_START_HOUR   10U

/** @brief Hour (0–23) at which the blinds are lowered each day. */
#define BLINDS_NIGHTTIME_START_HOUR 17U

/*******************************************************************************/
/*                              MOTOR CONFIGURATION                            */
/*******************************************************************************/

/** @brief PWM duty-cycle magnitude, 0–255. Adjust for your motor's load. */
#define BLINDS_MOTOR_SPEED          125U

/**
 * @brief H-bridge direction that physically raises the blinds.
 *
 * If the blinds move the wrong way, swap HBRIDGE_FORWARD / HBRIDGE_REVERSE
 * here rather than rewiring the motor.
 */
#define BLINDS_MOTOR_DIR_UP         HBRIDGE_REVERSE

/** @brief H-bridge direction that physically lowers the blinds. */
#define BLINDS_MOTOR_DIR_DOWN       HBRIDGE_FORWARD

/*******************************************************************************/
/*                               TIMING CONSTANTS                              */
/*******************************************************************************/

/** @brief Recommended FreeRTOS task period in milliseconds. */
#define BLINDS_TASK_PERIOD_MS       50U

/**
 * @brief Seconds of button inactivity before automatic mode resumes.
 *
 * When the user releases a button mid-travel the controller stays in manual
 * mode for this many seconds, then hands control back to the scheduler.
 */
#define BLINDS_MANUAL_TIMEOUT_S     60U

/** @brief How often (in task calls) to read the RTC. 20 × 50 ms = 1 s. */
#define BLINDS_RTC_POLL_CALLS       20U

/*******************************************************************************/
/*                                 STATUS CODES                                */
/*******************************************************************************/

typedef enum
{
    BLINDS_OK = 0,
    BLINDS_ERROR_DEBOUNCE_INIT, /**< Debouncer failed to initialise.    */
    BLINDS_ERROR_MOTOR_INIT,    /**< H-bridge driver failed to initialise. */
    BLINDS_ERROR_RTC_INIT,      /**< DS3231 driver failed to initialise. */
} Blinds_Status;

/*******************************************************************************/
/*                                 PUBLIC API                                  */
/*******************************************************************************/

/**
 * @brief  Initialise all peripherals used by the blinds controller.
 *
 * Must be called once before the first call to Blinds_MainFunction().
 * Initialises (in order):
 *  - Button debouncer (hardware timer ISR)
 *  - H-bridge GPIO and PWM channels
 *  - DS3231 I2C RTC
 *
 * @return BLINDS_OK on success, otherwise the relevant BLINDS_ERROR_* code.
 */
Blinds_Status Blinds_Init(void);

/**
 * @brief  Execute one control cycle of the blinds state machine.
 *
 * Call this from a periodic FreeRTOS task at BLINDS_TASK_PERIOD_MS intervals.
 * The function is non-blocking; all debouncing runs in interrupt context via
 * the hardware timer started by Blinds_Init().
 */
void Blinds_MainFunction(void);

#endif /* BLINDS_CONTROLLER_H */
