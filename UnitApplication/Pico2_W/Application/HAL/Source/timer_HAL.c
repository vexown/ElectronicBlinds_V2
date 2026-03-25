
/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
#include "timer_HAL.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include <string.h>

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
#define MAX_SW_TIMERS 8
#define SW_TIMER_CHECK_PERIOD_US 100  // 100µs resolution

#ifdef DEBUG_BUILD
    #define TIMER_DEBUG_ENABLED 1 // Enable debug pin (GPIO 16) for timing measurements
    #define TIMER_DEBUG_PIN 16
#else
    #define TIMER_DEBUG_ENABLED 0
#endif

/*******************************************************************************/
/*                             STATIC VARIABLES                                */
/*******************************************************************************/
static uint8_t next_timer_id = 0; 
static repeating_timer_t sw_timer_check;
static bool sw_timer_system_initialized = false;
static timer_handle_t* sw_timers[MAX_SW_TIMERS] = {NULL}; // Software timer pool

/*******************************************************************************/
/*                        STATIC FUNCTION DECLARATIONS                         */
/*******************************************************************************/

/**
 * @brief Hardware timer callback function
 * 
 * @param rt Pointer to repeating timer structure
 * @return bool True to keep repeating, false to stop
 */
static bool hw_timer_callback(repeating_timer_t* rt) 
{
    timer_handle_t* handle = (timer_handle_t*)rt->user_data;
    if (handle && handle->config.callback) 
    {
#if TIMER_DEBUG_ENABLED
        gpio_xor_mask(1u << TIMER_DEBUG_PIN);  // Toggle pin
#endif
        handle->config.callback();
    }

    return handle->config.repeat;
}

/**
 * @brief Update software timers by checking elapsed time and calling callbacks
 * 
 * This function implements a software timer system that uses a single hardware timer
 * to manage multiple virtual timers. It is called periodically (every 100μs) by the
 * sw_timer_check hardware timer to:
 * 
 * 1. Check all active software timers in the timer pool
 * 2. Calculate elapsed periods for each timer since their last update
 * 3. Execute callbacks for any elapsed periods
 * 4. Handle missed callbacks if system was busy
 * 5. Maintain precise timing by updating start times
 * 
 * Benefits over individual hardware timers:
 * - Manages multiple timers with single hardware resource
 * - Supports unlimited number of timers (within memory constraints)
 * - Handles system busy periods by catching up on missed callbacks
 * - More memory efficient than using multiple hardware timers
 * 
 * @note This function is called automatically by sw_timer_check_callback, which
 *       is registered during timer_init for the first software timer
 * 
 * @note The checking period (100μs) determines the minimum practical resolution
 *       for software timers. Hardware timers should be used for microsecond
 *       precision requirements.
 */
static void update_sw_timers(void) 
{
    /* Get the current 64 bit timestamp value in microseconds from the default System Timer instance (TIMER0 I believe) */
    uint64_t current_time = time_us_64();
    
    /* Iterate through all possible software timer slots */
    for (int i = 0; i < MAX_SW_TIMERS; i++) 
    {
        /* Check if this slot has a valid timer that is currently running */
        if (sw_timers[i] && sw_timers[i]->is_running) 
        {
            /* Calculate how much time has passed since the last trigger 
               This handles cases where the system might have been busy 
               and missed some timer checks */
            uint64_t elapsed = current_time - sw_timers[i]->last_trigger_time;
            
            /* Calculate how many complete periods have elapsed
               Example: if period is 1000µs and elapsed is 2500µs,
               then 2 complete periods have passed */
            uint64_t periods = elapsed / sw_timers[i]->config.period_us;
            
            /* If at least one period has elapsed, we need to trigger callbacks */
            if (periods > 0) 
            {
                /* Call the callback for each missed period to maintain timing accuracy
                   Example: if 2 periods passed, call callback twice */
                for (uint32_t j = 0; j < periods; j++) 
                {
#if TIMER_DEBUG_ENABLED
                    gpio_xor_mask(1u << TIMER_DEBUG_PIN);  // Toggle pin
#endif
                    if (sw_timers[i]->config.callback) 
                    {
                        sw_timers[i]->config.callback();
                    }
                }
                
                /* Update the last trigger time by adding exact period intervals
                   This maintains precise timing by avoiding drift that would occur
                   if we simply set last_trigger_time = current_time */
                sw_timers[i]->last_trigger_time += periods * sw_timers[i]->config.period_us;
                
                /* For one-shot timers, stop after first trigger */
                if (!sw_timers[i]->config.repeat) 
                {
                    sw_timers[i]->is_running = false;
                    break;
                }
            }
        }
    }
}

/**
 * @brief Software timer check callback function
 * 
 * @param rt Pointer to repeating timer structure
 * @return bool True to keep repeating, false to stop
 */
static bool sw_timer_check_callback(repeating_timer_t *rt) 
{
    (void)rt;  // Unused parameter

    /* This callback is called every SW_TIMER_CHECK_PERIOD_US (100µs) to check software timers
       Update software timers currently managed by the system */
    update_sw_timers();

    return true;  // Keep repeating
}

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

timer_status_t timer_init(const timer_config_t* config, timer_handle_t* handle) 
{
    if (!config || !handle) return TIMER_ERROR_INVALID_TIMER;

#if TIMER_DEBUG_ENABLED
    static bool debug_pin_initialized = false;
    if (!debug_pin_initialized) 
    {
        gpio_init(TIMER_DEBUG_PIN);
        gpio_set_dir(TIMER_DEBUG_PIN, GPIO_OUT);
        gpio_put(TIMER_DEBUG_PIN, 0);
        debug_pin_initialized = true;
    }
#endif

    /* If the configured timer is a software timer, initialize the "system" that manages them */
    if (!sw_timer_system_initialized && !config->hw_timer) 
    {
        /* Use pico-sdk function to add a new repeating timer */
        /* param 1 - the repeat delay in microseconds; if >0 then this is the delay between one callback ending and the next starting; 
                                                       if <0 then this is the negative of the time between the starts of the callbacks. 
                                                       The value of 0 is treated as 1
           param 2 - the repeating timer callback function
           param 3 - user data to pass to store in the repeating_timer structure for use by the callback.
           param 4 - the pointer to the user owned structure to store the repeating timer info in. 
                     BEWARE this storage location must outlive the repeating timer, so be careful of using stack space */
        if (!add_repeating_timer_us(SW_TIMER_CHECK_PERIOD_US, sw_timer_check_callback, NULL, &sw_timer_check)) 
        {
            return TIMER_ERROR_INVALID_TIMER;
        }

        sw_timer_system_initialized = true;
    }

    if (config->period_us == 0) return TIMER_ERROR_INVALID_PERIOD;
    
    if (!config->callback) return TIMER_ERROR_INVALID_CALLBACK;

    
    /* Initialize the timer handle */
    memset(handle, 0, sizeof(timer_handle_t));
    handle->timer_id = next_timer_id++;
    handle->config = *config;
    
    /* Add the timer to the software timer pool if it's a software timer */
    if (!config->hw_timer) 
    {
        for (int i = 0; i < MAX_SW_TIMERS; i++) 
        {
            if (!sw_timers[i]) 
            {
                sw_timers[i] = handle;
                break;
            }
        }
    }
    
    return TIMER_OK;
}

timer_status_t timer_start(timer_handle_t* handle) 
{
    if (!handle) return TIMER_ERROR_INVALID_TIMER;

    if (handle->is_running) return TIMER_ERROR_ALREADY_RUNNING;
    
    if (handle->config.hw_timer) 
    {
        /* Setup hardware timer */
        if (!add_repeating_timer_us(handle->config.period_us, hw_timer_callback, handle, &handle->hw_timer)) 
        {
            return TIMER_ERROR_INVALID_TIMER;
        }
    } 
    else 
    {
        /* Setup software timer */
        handle->last_trigger_time = time_us_64();
    }
    
    handle->is_running = true;
    return TIMER_OK;
}

timer_status_t timer_stop(timer_handle_t* handle) 
{
    if (!handle) return TIMER_ERROR_INVALID_TIMER;

    if (!handle->is_running) return TIMER_ERROR_NOT_RUNNING;

    if (handle->config.hw_timer) cancel_repeating_timer(&handle->hw_timer);
        
    handle->is_running = false;
    return TIMER_OK;
}

timer_status_t timer_reset(timer_handle_t* handle) 
{
    if (!handle) return TIMER_ERROR_INVALID_TIMER;
    
    timer_status_t status = timer_stop(handle);
    if (status != TIMER_OK) return status;
    
    return timer_start(handle);
}

timer_status_t timer_get_remaining(timer_handle_t* handle, uint32_t* remaining_us) 
{
    if (!handle || !remaining_us) return TIMER_ERROR_INVALID_TIMER;
    
    if (!handle->is_running) return TIMER_ERROR_NOT_RUNNING;
    
    if (handle->config.hw_timer) 
    {
        /* For hardware timers, we can only approximate */
        *remaining_us = handle->config.period_us;
    } 
    else 
    {
        uint64_t current_time = time_us_64();
        uint64_t elapsed = current_time - handle->last_trigger_time;
        
        if (elapsed >= handle->config.period_us) 
        {
            *remaining_us = 0;
        } 
        else 
        {
            uint64_t remaining_u64 = handle->config.period_us - elapsed;
            /* Ensure we don't overflow uint32_t */
            *remaining_us = (remaining_u64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)remaining_u64;
        }
    }
    
    return TIMER_OK;
}

timer_status_t timer_update_period(timer_handle_t* handle, uint32_t new_period_us) 
{
    if (!handle) return TIMER_ERROR_INVALID_TIMER;
    
    if (new_period_us == 0) return TIMER_ERROR_INVALID_PERIOD;
    
    bool was_running = handle->is_running;
    
    if (was_running) 
    {
        timer_status_t status = timer_stop(handle);
        if (status != TIMER_OK) return status;
    }
    
    handle->config.period_us = new_period_us;
    
    if (was_running) return timer_start(handle);
    
    return TIMER_OK;
}
