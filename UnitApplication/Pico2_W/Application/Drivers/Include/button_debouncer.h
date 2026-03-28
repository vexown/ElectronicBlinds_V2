/*
 * @file button_debouncer.h
 * @brief Robust button debouncer using 8-bit shift-register pattern matching.
 *
 * @section Method
 *
 * A hardware timer ISR samples each registered button every DEBOUNCE_SAMPLE_MS
 * milliseconds, shifting the reading into an 8-bit history register.
 *
 * Instead of a naive delay or simple counter, a bitmask is applied to the
 * history so that only the oldest and newest samples matter — the noisy
 * middle bits are "don't-care." A press is confirmed only when the edges
 * of the history match the expected stable-high → stable-low transition
 * (or vice-versa for active-high buttons). Once detected, the history is
 * forcibly reset to a solid state, providing software hysteresis that
 * prevents re-triggering from residual chatter.
 *
 * This is the "Ganssle / Elliot Williams" approach: Defer logic, not Eager.
 *
 *
 * @section Tuning
 *
 * DEBOUNCE_SAMPLE_MS | History depth | Effective window
 * -------------------|---------------|------------------
 *  5 ms              | 8 samples     | 40 ms (responsive, good-quality switches)
 * 10 ms              | 8 samples     | 80 ms (safe for old/scratchy switches)
 *
 * History depth is currently 8-bit (uint8_t). If you encounter switches that
 * still double-trigger, prefer widening to 16-bit (uint16_t) over slowing the
 * sample rate — this doubles the window to 80ms while keeping the same 5ms
 * temporal resolution. Update the history field in Debounce_Entry and adjust
 * DEBOUNCE_MASK accordingly (e.g. 0xE007 for 3 oldest + 3 newest of 16 bits).
 *
 *
 * @section Usage
 *
 *   Debounce_Init();
 *   Debounce_AddButton(15, GPIO_PULL_UP, true);   // active-low with pull-up
 *
 *   while (1)
 *   {
 *       if (Debounce_Pressed(15))  printf("Button pressed!\n");
 *       if (Debounce_Released(15)) printf("Button released!\n");
 *       // ...
 *   }
 */

#ifndef BUTTON_DEBOUNCER_H
#define BUTTON_DEBOUNCER_H

#include <stdint.h>
#include <stdbool.h>
#include "GPIO_HAL.h"

/* -----------------------------------------------------------------------
 * Configuration
 */

#define DEBOUNCE_MAX_BUTTONS  8    // Max simultaneous buttons
/* Timer tick interval in ms             
 * If you find your project in a very "noisy" environment (near large motors or relays), 
 * you can increase DEBOUNCE_SAMPLE_MS to 10ms. This extends the window to 80ms, 
 * providing a "tank-like" immunity to electrical noise at the cost of a tiny, 
 * barely-perceptible delay. */
#define DEBOUNCE_SAMPLE_MS    5    
/*
 * Mask that selects which bits of the 8-sample history we care about.
 *   0b11000111  →  oldest 2 bits + newest 3 bits
 * The 3 middle bits are "don't-care" (bounce zone).
 */
#define DEBOUNCE_MASK         0xC7 // 0b11000111 

/* -----------------------------------------------------------------------
 * Types
 */

typedef enum
{
    DEBOUNCE_OK = 0,
    DEBOUNCE_ERROR_INVALID_PIN,
    DEBOUNCE_ERROR_MAX_BUTTONS,
    DEBOUNCE_ERROR_NOT_FOUND,
    DEBOUNCE_ERROR_NOT_INITIALIZED
} Debounce_Status;

/* -----------------------------------------------------------------------
 * Public API
 */

/*
 * @brief  Start the debounce timer ISR. Call once at startup.
 * @return DEBOUNCE_OK on success.
 */
Debounce_Status Debounce_Init(void);

/*
 * @brief  Register a button for debouncing.
 * @param  pin        GPIO pin number.
 * @param  pull       Pull configuration (GPIO_PULL_UP, GPIO_PULL_DOWN, or GPIO_PULL_NONE).
 * @param  active_low true if the button reads LOW when pressed (common with pull-up).
 * @return DEBOUNCE_OK on success, error code otherwise.
 */
Debounce_Status Debounce_AddButton(uint8_t pin, GPIO_Pull pull, bool active_low);

/*
 * @brief  Check if a press event occurred (and clear the flag).
 * @param  pin  GPIO pin of a registered button.
 * @return true if the button was pressed since the last call.
 */
bool Debounce_Pressed(uint8_t pin);

/*
 * @brief  Check if a release event occurred (and clear the flag).
 * @param  pin  GPIO pin of a registered button.
 * @return true if the button was released since the last call.
 */
bool Debounce_Released(uint8_t pin);

/*
 * @brief  Check the current debounced state of the button.
 * @param  pin  GPIO pin of a registered button.
 * @return true if the button is currently held down.
 */
bool Debounce_IsHeld(uint8_t pin);

#endif /* BUTTON_DEBOUNCER_H */
