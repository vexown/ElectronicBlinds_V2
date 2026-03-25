/**
 * File: GPIO_HAL.c
 * Description: High-level HAL for easier use of GPIO on Raspberry Pico W
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

#include "GPIO_HAL.h"

#define MAX_GPIO_PIN 28  // Pico has 29 GPIO pins (0-28)

static bool is_valid_pin(uint8_t pin) {
    return pin <= MAX_GPIO_PIN;
}

GPIO_Status GPIO_Init(uint8_t pin, GPIO_Direction direction, GPIO_Pull pull) {
    if (!is_valid_pin(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }

    // Initialize GPIO
    gpio_init(pin);

    // Set direction
    if (direction == GPIO_OUTPUT) {
        gpio_set_dir(pin, GPIO_OUT);
    } else {
        gpio_set_dir(pin, GPIO_IN);
    }

    // Configure pull-up/pull-down
    switch (pull) {
        case GPIO_PULL_UP:
            gpio_pull_up(pin);
            break;
        case GPIO_PULL_DOWN:
            gpio_pull_down(pin);
            break;
        case GPIO_PULL_NONE:
            gpio_disable_pulls(pin);
            break;
        default:
            return GPIO_ERROR_INVALID_MODE;
    }

    return GPIO_OK;
}

GPIO_Status GPIO_Set(uint8_t pin) {
    if (!is_valid_pin(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }
    
    gpio_put(pin, 1);
    return GPIO_OK;
}

GPIO_Status GPIO_Clear(uint8_t pin) {
    if (!is_valid_pin(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }
    
    gpio_put(pin, 0);
    return GPIO_OK;
}

GPIO_Status GPIO_Write(uint8_t pin, GPIO_State state) {
    if (!is_valid_pin(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }
    
    gpio_put(pin, state == GPIO_HIGH ? 1 : 0);
    return GPIO_OK;
}

GPIO_Status GPIO_Read(uint8_t pin, GPIO_State* state) {
    if (!is_valid_pin(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }
    
    *state = gpio_get(pin) ? GPIO_HIGH : GPIO_LOW;
    return GPIO_OK;
}

GPIO_Status GPIO_Toggle(uint8_t pin) {
    if (!is_valid_pin(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }
    
    gpio_put(pin, !gpio_get(pin));
    return GPIO_OK;
}






