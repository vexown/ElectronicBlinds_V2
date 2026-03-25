/**
 * File: ADC_HAL.c
 * Description: High-level HAL for easier use of ADC on Raspberry Pico W
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

#include "ADC_HAL.h"

#define MAX_ADC_CHANNEL 4  // Pico supports ADC channels 0-3, channel 4 is the internal temperature sensor

static bool is_valid_channel(uint8_t channel) 
{
    return channel <= MAX_ADC_CHANNEL;
}

ADC_Status ADC_Init(uint8_t channel) 
{
    if (!is_valid_channel(channel)) {return ADC_ERROR_INVALID_CHANNEL;}

    // Initialize ADC hardware
    adc_init();

    // Select input channel
    adc_select_input(channel);

    return ADC_OK;
}

ADC_Status ADC_Read(uint8_t channel, uint16_t* value) 
{
    if (!is_valid_channel(channel)) {return ADC_ERROR_INVALID_CHANNEL;}

    if (value == NULL) {return ADC_ERROR_NULL_POINTER;}

    // Select input channel and read the raw value
    adc_select_input(channel);
    *value = adc_read();

    return ADC_OK;
}

ADC_Status ADC_ReadVoltage(uint8_t channel, float* voltage) 
{
    if (!is_valid_channel(channel)) {return ADC_ERROR_INVALID_CHANNEL;}

    if (voltage == NULL) {return ADC_ERROR_NULL_POINTER;}

    // Select input channel and read the raw value
    adc_select_input(channel);
    uint16_t raw_value = adc_read();

    // Convert raw value to voltage
    *voltage = (raw_value * ADC_REF_VOLTAGE) / (float)(1 << ADC_RESOLUTION);

    return ADC_OK;
}

ADC_Status ADC_ReadTemperature(float* temperature) 
{
    if (temperature == NULL) {return ADC_ERROR_NULL_POINTER;}

    // Configure channel 4 (internal temperature sensor)
    adc_set_temp_sensor_enabled(true);
    adc_select_input(ADC_TEMPERATURE_CHANNEL);

    // Read raw ADC value
    uint16_t raw_value = adc_read();

    // Convert raw value to temperature in Celsius
    float voltage = (raw_value * ADC_REF_VOLTAGE) / (float)(1 << ADC_RESOLUTION);
    *temperature = 27.0f - (voltage - 0.706f) / 0.001721f;

    // Disable temperature sensor after use
    adc_set_temp_sensor_enabled(false);

    return ADC_OK;
}
