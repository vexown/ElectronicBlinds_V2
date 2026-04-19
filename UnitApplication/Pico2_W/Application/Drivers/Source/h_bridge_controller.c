/**
 * @file h_bridge_controller.c
 * @brief H-bridge motor controller implementation using PWM_HAL and GPIO_HAL.
 */

#include "h_bridge_controller.h"
#include "PWM_HAL.h"
#include "GPIO_HAL.h"
#include "pico/stdlib.h"

/* Track whether HBridge_Init() has been called successfully. */
static bool s_initialized = false;

HBridge_Status HBridge_Init(void)
{
    if (s_initialized) return HBRIDGE_OK;

    /* Configure enable pins as GPIO outputs (default LOW = disabled) */
    if (GPIO_Init(HBRIDGE_L_EN_PIN, GPIO_OUTPUT, GPIO_PULL_NONE) != GPIO_OK) return HBRIDGE_ERROR_GPIO;
    if (GPIO_Init(HBRIDGE_R_EN_PIN, GPIO_OUTPUT, GPIO_PULL_NONE) != GPIO_OK) return HBRIDGE_ERROR_GPIO;

    GPIO_Clear(HBRIDGE_L_EN_PIN);
    GPIO_Clear(HBRIDGE_R_EN_PIN);

    /* Configure PWM channels (start at 0 % duty) */
    PWM_Config lpwm_cfg =
    {
        .pin           = HBRIDGE_LPWM_PIN,
        .frequency     = HBRIDGE_PWM_FREQ_HZ,
        .duty_cycle    = 0.0f,
        .invert_output = false
    };
    if (PWM_Init(&lpwm_cfg) != PWM_OK) return HBRIDGE_ERROR_PWM;

    PWM_Config rpwm_cfg =
    {
        .pin           = HBRIDGE_RPWM_PIN,
        .frequency     = HBRIDGE_PWM_FREQ_HZ,
        .duty_cycle    = 0.0f,
        .invert_output = false
    };
    if (PWM_Init(&rpwm_cfg) != PWM_OK) return HBRIDGE_ERROR_PWM;

    /* Start PWM outputs (they idle at 0 % until duty is changed) */
    PWM_Start(HBRIDGE_LPWM_PIN);
    PWM_Start(HBRIDGE_RPWM_PIN);

    s_initialized = true;
    return HBRIDGE_OK;
}

HBridge_Status HBridge_SetEnable(HBridge_Side side, bool enable)
{
    if (!s_initialized) return HBRIDGE_ERROR_NOT_INITIALIZED;

    GPIO_Status left_status  = GPIO_OK;
    GPIO_Status right_status = GPIO_OK;

    if (side == HBRIDGE_SIDE_LEFT || side == HBRIDGE_SIDE_BOTH)
    {
        left_status = enable ? GPIO_Set(HBRIDGE_L_EN_PIN)
                             : GPIO_Clear(HBRIDGE_L_EN_PIN);
    }
    if (side == HBRIDGE_SIDE_RIGHT || side == HBRIDGE_SIDE_BOTH)
    {
        right_status = enable ? GPIO_Set(HBRIDGE_R_EN_PIN)
                              : GPIO_Clear(HBRIDGE_R_EN_PIN);
    }

    if (left_status != GPIO_OK || right_status != GPIO_OK) return HBRIDGE_ERROR_GPIO;
    return HBRIDGE_OK;
}

HBridge_Status HBridge_SetSpeed(uint8_t speed, HBridge_Direction direction)
{
    if (!s_initialized) return HBRIDGE_ERROR_NOT_INITIALIZED;

    /* Convert 0-255 to 0.0-100.0 % */
    float duty = (speed / 255.0f) * 100.0f;

    /*
     * Forward  : LPWM = duty,  RPWM = 0 %
     * Reverse  : LPWM = 0 %,   RPWM = duty
     */
    PWM_Status lpwm_status;
    PWM_Status rpwm_status;

    if (direction == HBRIDGE_FORWARD)
    {
        lpwm_status = PWM_SetDutyCycle(HBRIDGE_LPWM_PIN, duty);
        rpwm_status = PWM_SetDutyCycle(HBRIDGE_RPWM_PIN, 0.0f);
    }
    else
    {
        lpwm_status = PWM_SetDutyCycle(HBRIDGE_LPWM_PIN, 0.0f);
        rpwm_status = PWM_SetDutyCycle(HBRIDGE_RPWM_PIN, duty);
    }

    if (lpwm_status != PWM_OK || rpwm_status != PWM_OK) return HBRIDGE_ERROR_PWM;
    return HBRIDGE_OK;
}

HBridge_Status HBridge_Stop(void)
{
    if (!s_initialized) return HBRIDGE_ERROR_NOT_INITIALIZED;

    PWM_Status lpwm_status = PWM_SetDutyCycle(HBRIDGE_LPWM_PIN, 0.0f);
    PWM_Status rpwm_status = PWM_SetDutyCycle(HBRIDGE_RPWM_PIN, 0.0f);

    if (lpwm_status != PWM_OK || rpwm_status != PWM_OK) return HBRIDGE_ERROR_PWM;
    return HBRIDGE_OK;
}

HBridge_Status HBridge_RunFor(uint8_t speed, HBridge_Direction direction,
                              uint32_t duration_ms)
{
    HBridge_Status status = HBridge_SetSpeed(speed, direction);
    if (status != HBRIDGE_OK) return status;

    sleep_ms(duration_ms);

    return HBridge_Stop();
}
