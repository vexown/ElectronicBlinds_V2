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
 * The controller raises the blinds at local sunrise and lowers them
 * BLINDS_SUNSET_OFFSET_HOURS after local sunset. Sunrise and sunset are
 * computed from the DS3231 date using the NOAA solar algorithm for the
 * configured latitude/longitude (see time_calculations.h). DST is applied
 * automatically for the EU schedule.
 *
 * @section ManualOverride
 *
 * Pressing UP or DOWN switches to manual mode and runs the motor while the
 * button is held (or until the corresponding limit switch fires).
 * After BLINDS_MANUAL_TIMEOUT_S seconds of button inactivity the controller
 * returns to automatic mode.
 *
 * Double-tapping UP or DOWN (two presses within BLINDS_DOUBLE_TAP_WINDOW_MS)
 * starts a full run to the corresponding limit switch without requiring the
 * button to be held. Any subsequent button press interrupts the run.
 *
 * @section Safety
 *
 * Any motor run (auto, double-tap full-run, or hold-to-move) is bounded by a
 * watchdog: if travel time exceeds BLINDS_TRAVEL_WATCHDOG_PCT of the
 * calibrated travel for the current direction, the motor is force-stopped
 * and the controller latches into a permanent fault state that disables ALL
 * further motor operation (auto, double-tap, AND hold-to-move) until the
 * device is power-cycled or reflashed. A watchdog trip means something has
 * gone seriously wrong (broken limit switch, mechanical jam, miswiring);
 * resuming automatically could drive the mechanism into the end stop
 * repeatedly, so manual intervention is required.
 *
 * Travel-time calibration is learned opportunistically: any run that starts
 * at one limit and ends at the opposite limit is timed and (if it differs
 * from the stored value by more than BLINDS_TRAVEL_WRITE_DELTA_MS) persisted
 * to flash via the boot metadata sector.
 *
 * Position is tracked by dead reckoning between limit hits (linear
 * interpolation against the calibrated travel time). The estimate is
 * re-zeroed at every limit, so errors do not accumulate across full cycles.
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

/**
 * @brief Delay (in decimal hours) between local sunset and the auto lowering.
 *
 * The blinds are lowered at (sunset + this offset). 0.5 means 30 minutes after
 * sunset, giving some dusk before the blinds drop.
 */
#define BLINDS_SUNSET_OFFSET_HOURS  0.5f

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

/**
 * @brief Maximum gap (ms) between two taps to be recognised as a double-tap.
 *
 * Double-tapping UP or DOWN starts a full run to the corresponding limit
 * without requiring the button to be held. A single subsequent tap interrupts.
 */
#define BLINDS_DOUBLE_TAP_WINDOW_MS 500U

/*******************************************************************************/
/*                       SAFETY / CALIBRATION CONSTANTS                        */
/*******************************************************************************/

/**
 * @brief Watchdog trip threshold as a percentage of calibrated travel time.
 *
 * The motor is force-stopped if travel time exceeds this fraction of the
 * calibrated value (e.g. 120 means stop after 1.20 × expected travel).
 * Lower = stricter protection but more false positives from motor variance.
 */
#define BLINDS_TRAVEL_WATCHDOG_PCT  110U

/**
 * @brief Travel time (ms) the watchdog assumes when no calibration exists.
 *
 * Used on first boot and after any uncalibrated firmware upgrade. Picked
 * conservatively so the watchdog still trips before damage on any plausible
 * blind, while not interrupting a normal slow blind.
 */
#define BLINDS_TRAVEL_DEFAULT_MS    60000U

/** @brief Minimum plausible full-travel time (ms). Shorter measurements are
 *  rejected as glitches rather than written to calibration. */
#define BLINDS_TRAVEL_MIN_MS        2000U

/** @brief Maximum plausible full-travel time (ms). */
#define BLINDS_TRAVEL_MAX_MS        120000U

/**
 * @brief Minimum delta (ms) between a fresh measurement and the stored
 *        calibration before the new value is written to flash.
 *
 * Avoids burning a flash erase cycle on every single full run for tiny
 * measurement-to-measurement jitter.
 */
#define BLINDS_TRAVEL_WRITE_DELTA_MS 500U

/** @brief Sentinel position value meaning "we have no idea where the blind
 *  is — recover by reaching a limit switch." */
#define BLINDS_POSITION_UNKNOWN     0xFFFFU

/*******************************************************************************/
/*                           SUN AUTOMATION CONSTANTS                          */
/*******************************************************************************/

/**
 * @brief Lux level (inclusive) that triggers the sun-lowering event.
 *
 * When the VEML7700 reads at or above this value the blinds are automatically
 * lowered to BLINDS_SUN_HALFWAY_PERMILLE. 
 */
#define BLINDS_SUN_LUX_THRESHOLD       5000.0f

/**
 * @brief Hysteresis band (lux) below the threshold for the return trip.
 *
 * The blinds return to their saved position only when lux drops below
 * (BLINDS_SUN_LUX_THRESHOLD - BLINDS_SUN_LUX_HYSTERESIS). Prevents
 * rapid toggling when illuminance hovers near the trigger point.
 */
#define BLINDS_SUN_LUX_HYSTERESIS      1000.0f

/**
 * @brief Sun-position target as a per-mille fraction of the travel range.
 *
 * 0 = fully closed (bottom limit), 1000 = fully open (top limit).
 * 500 means half-way. The event fires only when the current position is
 * ABOVE this value; if the blinds are already at or below it nothing happens.
 */
#define BLINDS_SUN_HALFWAY_PERMILLE    500U

/**
 * @brief How often (in task calls) to poll the VEML7700 for a lux reading.
 * 10 × 50 ms = 500 ms — well above the 100 ms integration time.
 */
#define BLINDS_SUN_POLL_CALLS          ((uint32_t)(500U) / BLINDS_TASK_PERIOD_MS)

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
