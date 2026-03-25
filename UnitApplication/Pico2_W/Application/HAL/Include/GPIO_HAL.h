#ifndef GPIO_HAL_H
#define GPIO_HAL_H

/* Key features */
/*
    - Simple function names that are intuitive to use
    - Error checking for invalid pins
    - Enumerated types for direction, pull configuration, and pin states
    - Status returns for error handling
    - Built-in pin toggle functionality
    - Clear separation between interface (.h) and implementation (.c)
*/

/* Example use */
/*
// Example usage
void main() {
    // Initialize LED pin as output
    GPIO_Init(25, GPIO_OUTPUT, GPIO_PULL_NONE);
    
    // Initialize button pin as input with pull-up
    GPIO_Init(15, GPIO_INPUT, GPIO_PULL_UP);
    
    while (1) {
        GPIO_State button_state;
        GPIO_Read(15, &button_state);
        
        if (button_state == GPIO_LOW) {  // Button pressed
            GPIO_Set(25);    // Turn LED on
        } else {
            GPIO_Clear(25);  // Turn LED off
        }
    }
}
*/

#include "pico/stdlib.h"

// GPIO direction
typedef enum {
    GPIO_INPUT = 0,
    GPIO_OUTPUT
} GPIO_Direction;

// GPIO pull configuration
typedef enum {
    GPIO_PULL_NONE = 0,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
} GPIO_Pull;

// GPIO state
typedef enum {
    GPIO_LOW = 0,
    GPIO_HIGH
} GPIO_State;

// Error codes
typedef enum {
    GPIO_OK = 0,
    GPIO_ERROR_INVALID_PIN,
    GPIO_ERROR_INVALID_MODE
} GPIO_Status;

// Initialize GPIO pin
GPIO_Status GPIO_Init(uint8_t pin, GPIO_Direction direction, GPIO_Pull pull);

// Set GPIO pin to high
GPIO_Status GPIO_Set(uint8_t pin);

// Clear GPIO pin to low
GPIO_Status GPIO_Clear(uint8_t pin);

// Write value to GPIO pin
GPIO_Status GPIO_Write(uint8_t pin, GPIO_State state);

// Read GPIO pin state
GPIO_Status GPIO_Read(uint8_t pin, GPIO_State* state);

// Toggle GPIO pin
GPIO_Status GPIO_Toggle(uint8_t pin);

#endif