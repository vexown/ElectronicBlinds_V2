/**
 * @file h_bridge_controller.h
 * @brief Simple H-bridge motor controller interface for the Pico2 W.
 *        This driver is mainly intended for BTS7960B H-bridge modules,
 *        but can be used/adapted to other similar designs.
 *
 * @section BTS7960B Description
 *
 * The BTS7960B is a fully integrated high-current half-bridge driver.
 * Two of these are combined on common modules to form a full H-bridge
 * capable of driving DC motors bidirectionally. Each half-bridge has
 * a PWM input for speed control and an enable pin.
 *
 *
 * @section Wiring (Feel free to adapt as needed)
 *
 * Pico 2 W | H-Bridge Module
 * ---------|----------------
 * 3.3V     | VCC (some modules may require 5V, check your specific model)
 * GND      | GND
 * GP16     | LPWM (Speed control for one direction)
 * GP17     | RPWM (Speed control for the opposite direction)
 * GP18     | L_EN (Enable pin for one half of the H-bridge)
 * GP19     | R_EN (Enable pin for the other half of the H-bridge)
 *
 */

#ifndef H_BRIDGE_CONTROLLER_H
#define H_BRIDGE_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

/* -----------------------------------------------------------------------
 * Pin & PWM Configuration
 */

#define HBRIDGE_LPWM_PIN    16   /* PWM pin for left/forward direction  */
#define HBRIDGE_RPWM_PIN    17   /* PWM pin for right/reverse direction */
#define HBRIDGE_L_EN_PIN    18   /* Enable pin for left half-bridge     */
#define HBRIDGE_R_EN_PIN    19   /* Enable pin for right half-bridge    */

#define HBRIDGE_PWM_FREQ_HZ 1000 /* Default PWM frequency in Hz        */

/* -----------------------------------------------------------------------
 * Types & Enumerations
 */

/* Status codes returned by all HBridge functions. */
typedef enum
{
    HBRIDGE_OK = 0,
    HBRIDGE_ERROR_INIT,
    HBRIDGE_ERROR_NOT_INITIALIZED,
    HBRIDGE_ERROR_INVALID_SPEED,
    HBRIDGE_ERROR_PWM,
    HBRIDGE_ERROR_GPIO
} HBridge_Status;

/* Motor direction. */
typedef enum
{
    HBRIDGE_FORWARD = 0,
    HBRIDGE_REVERSE
} HBridge_Direction;

/* Which half-bridge side(s) to enable/disable. */
typedef enum
{
    HBRIDGE_SIDE_LEFT = 0,
    HBRIDGE_SIDE_RIGHT,
    HBRIDGE_SIDE_BOTH
} HBridge_Side;

/* -----------------------------------------------------------------------
 * Public API
 */

/*
 * @brief  Initialize PWM channels and GPIO enable pins for the H-bridge.
 * @return HBRIDGE_OK on success, error code otherwise.
 */
HBridge_Status HBridge_Init(void);

/*
 * @brief  Enable or disable one or both sides of the H-bridge.
 * @param  side    Which side(s) to affect.
 * @param  enable  true = enable, false = disable.
 * @return HBRIDGE_OK on success, error code otherwise.
 */
HBridge_Status HBridge_SetEnable(HBridge_Side side, bool enable);

/*
 * @brief  Set motor speed and direction.
 * @param  speed      Speed value 0-255 (0 = stopped, 255 = full speed).
 * @param  direction  HBRIDGE_FORWARD or HBRIDGE_REVERSE.
 * @return HBRIDGE_OK on success, error code otherwise.
 */
HBridge_Status HBridge_SetSpeed(uint8_t speed, HBridge_Direction direction);

/*
 * @brief  Stop the motor (sets both PWM outputs to 0% duty cycle).
 * @return HBRIDGE_OK on success, error code otherwise.
 */
HBridge_Status HBridge_Stop(void);

/*
 * @brief  Run the motor at a given speed/direction for a fixed duration (blocking).
 * @param  speed       Speed value 0-255.
 * @param  direction   HBRIDGE_FORWARD or HBRIDGE_REVERSE.
 * @param  duration_ms Duration in milliseconds. The call blocks for this long,
 *                     then stops the motor before returning.
 * @return HBRIDGE_OK on success, error code otherwise.
 */
HBridge_Status HBridge_RunFor(uint8_t speed, HBridge_Direction direction, uint32_t duration_ms);


#endif /* H_BRIDGE_CONTROLLER_H */
