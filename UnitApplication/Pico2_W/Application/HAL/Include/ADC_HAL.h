#ifndef ADC_HAL_H
#define ADC_HAL_H

/**
 * Raspberry Pi Pico W ADC Overview:
 * - 3 Usable ADC Channels:
 *     - GPIO26 (ADC0)
 *     - GPIO27 (ADC1)
 *     - GPIO28 (ADC2)
 * - 1 Internal ADC Channel:
 *     - ADC4: On-chip temperature sensor (not mapped to a GPIO pin).
 * - 12-bit resolution (0-4095), reference voltage (VREF) is 3.3V:
 *     - Voltage resolution: ~0.8 mV per step.
 * - ADC input voltage range: 0V to 3.3V (Exceeding this may damage the MCU).
 */

/* Key features */
/*
    - Simplified ADC usage with clear function interfaces
    - Error checking for invalid channels and null pointers
    - Enumerated types for error handling
    - Voltage and temperature readings supported
    - Clear separation between interface (.h) and implementation (.c)
*/

/* Example use */
/* 
void main() {
    uint16_t raw_adc_value;
    float voltage, temperature;

    // Initialize ADC channel 0
    if (ADC_Init(0) == ADC_OK) {
        // Read raw ADC value
        ADC_Read(0, &raw_adc_value);

        // Read voltage
        ADC_ReadVoltage(0, &voltage);
    }

    // Read temperature sensor
    ADC_ReadTemperature(&temperature);
}
*/

#include "pico/stdlib.h"
#include "hardware/adc.h"

// Constants
#define ADC_REF_VOLTAGE 3.3f  // Reference voltage in volts
#define ADC_RESOLUTION 12     // ADC resolution is 12 bits (0-4095)
#define ADC_TEMPERATURE_CHANNEL 4  // Internal temperature sensor channel

// Error codes
typedef enum 
{
    ADC_OK = 0,
    ADC_ERROR_INVALID_CHANNEL,
    ADC_ERROR_NULL_POINTER
} ADC_Status;

// Initialize ADC for a specific channel
ADC_Status ADC_Init(uint8_t channel);

// Read raw ADC value
ADC_Status ADC_Read(uint8_t channel, uint16_t* value);

// Read ADC value and convert to voltage
ADC_Status ADC_ReadVoltage(uint8_t channel, float* voltage);

// Read internal temperature sensor
ADC_Status ADC_ReadTemperature(float* temperature);

#endif
