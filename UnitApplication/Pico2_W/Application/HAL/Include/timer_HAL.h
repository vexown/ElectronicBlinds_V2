/**
 * @file timer_HAL.h
 * @brief Hardware Abstraction Layer for Timer functionality on RP2350
 * 
 * This HAL provides a unified interface for both hardware and software timers.
 * Hardware timers utilize the RP2350's timer peripherals while software timers
 * are implemented using the system time with microsecond resolution.
 * 
 * Features:
 * - Hardware and software timer support
 * - One-shot and periodic timer modes
 * - Configurable resolution:
 *   * Hardware Timers: 1µs resolution (direct hardware support)
 *   * Software Timers: 100µs resolution (to maintain system stability)
 * - Callback support
 * - Period modification during runtime
 * - Remaining time queries
 * - Resource management with timer pool
 * 
 * The HAL automatically handles timer resource allocation and provides
 * comprehensive error checking for all operations.
 * 
 * @note Maximum of 8 software timers can be active simultaneously
 * @note For timing requirements < 100µs, use hardware timers
 * @note Software timer resolution is limited to protect system performance
 */

#ifndef TIMER_HAL_H
#define TIMER_HAL_H

#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdint.h>

#if 0 /* Test Code - you can use this to test the timer HAL or as a reference to get an idea how to use it */

#include "timer_HAL.h"

// Test variables
static volatile uint32_t hw_timer_count = 0;
static volatile uint32_t sw_timer_count = 0;
static timer_handle_t hw_timer;
static timer_handle_t sw_timer;
static timer_handle_t oneshot_timer;

// Test callback functions
static void hw_timer_callback(void) {
    hw_timer_count++;
}

static void sw_timer_callback(void) {
    sw_timer_count++;
}

static void oneshot_callback(void) {
    printf("One-shot timer fired!\n");
}

// Test functions
static void test_hardware_timer(void) {
    printf("\n=== Hardware Timer Test ===\n");
    
    timer_config_t config = {
        .period_us = 1000000,  // 1 second
        .repeat = true,
        .callback = hw_timer_callback,
        .hw_timer = true
    };
    
    timer_status_t status = timer_init(&config, &hw_timer);
    assert(status == TIMER_OK);
    
    status = timer_start(&hw_timer);
    assert(status == TIMER_OK);
    
    // Let it run for 3 seconds
    sleep_ms(3000);
    
    status = timer_stop(&hw_timer);
    assert(status == TIMER_OK);
    
    printf("Hardware timer ticks: %lu (expected ~3)\n", hw_timer_count);
}

static void test_software_timer(void) {
    printf("\n=== Software Timer Test ===\n");
    
    timer_config_t config = {
        .period_us = 490000,   // 490ms for reliable timing
        .repeat = true,
        .callback = sw_timer_callback,
        .hw_timer = false
    };
    
    sw_timer_count = 0;
    timer_status_t status = timer_init(&config, &sw_timer);
    assert(status == TIMER_OK);
    
    printf("Starting software timer...\n");
    status = timer_start(&sw_timer);
    assert(status == TIMER_OK);
    
    sleep_ms(2000);  // Run for exactly 2 seconds
    
    status = timer_stop(&sw_timer);
    assert(status == TIMER_OK);
    
    printf("Software timer ticks: %lu (expected 4)\n", sw_timer_count);
}

static void test_oneshot_timer(void) {
    printf("\n=== One-shot Timer Test ===\n");
    
    timer_config_t config = {
        .period_us = 2000000,  // 2 seconds
        .repeat = false,
        .callback = oneshot_callback,
        .hw_timer = true
    };
    
    timer_status_t status = timer_init(&config, &oneshot_timer);
    assert(status == TIMER_OK);
    
    uint32_t remaining;
    status = timer_start(&oneshot_timer);
    assert(status == TIMER_OK);
    
    // Check remaining time
    sleep_ms(1000);
    status = timer_get_remaining(&oneshot_timer, &remaining);
    assert(status == TIMER_OK);
    printf("Remaining time: %lu us\n", remaining);
    
    // Wait for completion
    sleep_ms(1500);
}

static void test_period_update(void) {
    printf("\n=== Period Update Test ===\n");
    
    // Update hardware timer period
    timer_status_t status = timer_update_period(&hw_timer, 250000);  // 250ms
    assert(status == TIMER_OK);
    
    hw_timer_count = 0;
    status = timer_start(&hw_timer);
    assert(status == TIMER_OK);
    
    // Let it run for 1 second
    sleep_ms(1000);
    
    status = timer_stop(&hw_timer);
    assert(status == TIMER_OK);
    
    printf("Fast timer ticks: %lu (expected ~4)\n", hw_timer_count);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);  // Wait for serial connection
    
    printf("=== Timer HAL Test Suite ===\n");
    
    test_hardware_timer();
    test_software_timer();
    test_oneshot_timer();
    test_period_update();
    
    printf("\nAll timer tests complete!\n");
    
    while(1) {
        sleep_ms(1000);
    }
}

#endif

/* Error codes */
typedef enum 
{
    TIMER_OK = 0,
    TIMER_ERROR_INVALID_TIMER,
    TIMER_ERROR_ALREADY_RUNNING,
    TIMER_ERROR_NOT_RUNNING,
    TIMER_ERROR_INVALID_CALLBACK,
    TIMER_ERROR_INVALID_PERIOD
} timer_status_t;

/* Timer configuration structure */
typedef struct 
{
    uint32_t period_us;         // Timer period in microseconds
    bool repeat;                // True for periodic timer, false for one-shot
    void (*callback)(void);     // Timer callback function
    bool hw_timer;              // True to use hardware timer, false for software timer
} timer_config_t;

/* Timer handle structure */
typedef struct 
{
    uint8_t timer_id;           // Internal timer identifier
    timer_config_t config;      // Timer configuration
    bool is_running;            // Current timer state
    uint64_t last_trigger_time;        // Last start time in microseconds
    repeating_timer_t hw_timer; // Hardware timer instance (if used)
} timer_handle_t;

/**
 * @brief Initialize a timer with the specified configuration
 * 
 * @param config Pointer to timer configuration structure
 * @param handle Pointer to timer handle structure to be initialized
 * @return timer_status_t Status code
 */
timer_status_t timer_init(const timer_config_t* config, timer_handle_t* handle);

/**
 * @brief Start the specified timer
 * 
 * @param handle Pointer to timer handle
 * @return timer_status_t Status code
 */
timer_status_t timer_start(timer_handle_t* handle);

/**
 * @brief Stop the specified timer
 * 
 * @param handle Pointer to timer handle
 * @return timer_status_t Status code
 */
timer_status_t timer_stop(timer_handle_t* handle);

/**
 * @brief Reset the specified timer to its initial state
 * 
 * @param handle Pointer to timer handle
 * @return timer_status_t Status code
 */
timer_status_t timer_reset(timer_handle_t* handle);

/**
 * @brief Get remaining time until next timer expiration
 * 
 * @param handle Pointer to timer handle
 * @param remaining_us Pointer to store remaining time in microseconds
 * @return timer_status_t Status code
 */
timer_status_t timer_get_remaining(timer_handle_t* handle, uint32_t* remaining_us);

/**
 * @brief Update timer period
 * 
 * @param handle Pointer to timer handle
 * @param new_period_us New period in microseconds
 * @return timer_status_t Status code
 */
timer_status_t timer_update_period(timer_handle_t* handle, uint32_t new_period_us);

#endif // TIMER_HAL_H