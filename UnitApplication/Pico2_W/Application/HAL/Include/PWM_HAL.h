/**
 * @note On RP2350:
 * - 12 PWM slices available (vs 8 on RP2040)
 * - Each slice has two outputs (A/B)
 * - GPIOs 0-29 can be used for PWM
 */

#ifndef PWM_HAL_H
#define PWM_HAL_H

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "pico/stdlib.h"

/* Test Code - can be used to test the PWM HAL or to get an idea of how to use it */
#if 0 
#include "PWM_HAL.h"

void test_PWM_HAL(void)
{
    PWM_Status status;
    PWM_Config config;
    const uint8_t TEST_PIN = 16;  // Choose accessible GPIO

    printf("\n=== PWM HAL Test Start ===\n");

    // Test 1: Valid initialization
    printf("\nTest 1: PWM Init\n");
    config.pin = TEST_PIN;
    config.frequency = 1000;      // 1kHz
    config.duty_cycle = 50.0f;    // 50%
    config.invert_output = false;
    
    status = PWM_Init(&config);
    printf("Init status: %d (Expected: %d)\n", status, PWM_OK);

    // Test 2: Invalid parameters
    printf("\nTest 2: Invalid Parameters\n");
    config.frequency = 0;  // Invalid frequency
    status = PWM_Init(&config);
    printf("Invalid freq status: %d (Expected: %d)\n", status, PWM_ERROR_INVALID_FREQUENCY);

    config.frequency = 1000;
    config.duty_cycle = 150.0f;  // Invalid duty
    status = PWM_Init(&config);
    printf("Invalid duty status: %d (Expected: %d)\n", status, PWM_ERROR_INVALID_DUTY);

    // Test 3: Start/Stop
    printf("\nTest 3: Start/Stop Test\n");
    config.duty_cycle = 50.0f;  // Restore valid duty
    PWM_Init(&config);
    
    status = PWM_Start(TEST_PIN);
    printf("Start status: %d\n", status);
    sleep_ms(1000);
    
    status = PWM_Stop(TEST_PIN);
    printf("Stop status: %d\n", status);

    // Test 4: Duty Cycle Changes
    printf("\nTest 4: Duty Cycle Test\n");
    PWM_Start(TEST_PIN);
    
    printf("Setting 25%% duty cycle\n");
    status = PWM_SetDutyCycle(TEST_PIN, 25.0f);
    sleep_ms(1000);
    
    printf("Setting 75%% duty cycle\n");
    status = PWM_SetDutyCycle(TEST_PIN, 75.0f);
    sleep_ms(1000);

    // Test 5: Frequency Changes
    printf("\nTest 5: Frequency Test\n");
    printf("Setting 2kHz\n");
    status = PWM_SetFrequency(TEST_PIN, 2000);
    sleep_ms(1000);
    
    printf("Setting 500Hz\n");
    status = PWM_SetFrequency(TEST_PIN, 500);
    sleep_ms(1000);

    // Test 6: DeInit
    printf("\nTest 6: DeInit Test\n");
    status = PWM_DeInit(TEST_PIN);
    printf("DeInit status: %d\n", status);

    // Test 7: Operations after DeInit
    printf("\nTest 7: Post-DeInit Test\n");
    status = PWM_SetDutyCycle(TEST_PIN, 50.0f);
    printf("Post-deinit operation status: %d (Expected: %d)\n", 
           status, PWM_ERROR_NOT_INITIALIZED);

    printf("\n=== PWM HAL Test Complete ===\n");
}
#endif


typedef enum 
{
    PWM_OK = 0,
    PWM_ERROR_INVALID_PIN,
    PWM_ERROR_INVALID_FREQUENCY,
    PWM_ERROR_INVALID_DUTY,
    PWM_ERROR_NOT_INITIALIZED,
    PWM_ERROR_NULL_POINTER
} PWM_Status;

typedef struct 
{
    uint8_t pin;           // GPIO pin number
    uint32_t frequency;    // PWM frequency in Hz
    float duty_cycle;      // Duty cycle (0.0 to 100.0)
    bool invert_output;    // Invert PWM output
} PWM_Config;

PWM_Status PWM_Init(const PWM_Config* config);
PWM_Status PWM_Start(uint8_t pin);
PWM_Status PWM_Stop(uint8_t pin);
PWM_Status PWM_SetDutyCycle(uint8_t pin, float duty_cycle);
PWM_Status PWM_SetFrequency(uint8_t pin, uint32_t frequency);
PWM_Status PWM_DeInit(uint8_t pin);

#endif // PWM_HAL_H