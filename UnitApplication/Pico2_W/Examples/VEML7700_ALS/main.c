/**
 * File: main.c - VEML7700 Ambient Light Sensor Example
 * Description: Demonstrates how to use the VEML7700 driver to read ambient light
 *              in raw counts and converted lux, and how to switch between gain and
 *              integration time settings.
 */

/* Library includes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "VEML7700.h"

int main(void)
{
    stdio_init_all();

    printf("\nVEML7700 Ambient Light Sensor Example\n");
    printf("--------------------------------------\n\n");

    /* Initialize the VEML7700. This configures I2C, verifies the device ID,
     * and powers the sensor on with default settings (gain x1, IT 100 ms). */
    printf("Initializing VEML7700...\n");
    VEML7700_Status status = VEML7700_Init();
    if (status == VEML7700_ERROR_NO_DEVICE)
    {
        printf("ERROR: VEML7700 not found on I2C bus. Check wiring.\n");
        return 1;
    }
    if (status != VEML7700_OK)
    {
        printf("ERROR: VEML7700 initialization failed (status=%d).\n", status);
        return 1;
    }
    printf("VEML7700 initialized successfully.\n\n");

    /* --- Part 1: Read with default settings (gain x1, IT 100 ms) --- */
    printf("Reading with default settings: gain x1, IT 100 ms\n");
    printf("Resolution: 0.0336 lx/count, max range: ~2200 lx\n");
    printf("--------------------------------------------------\n");

    /* Wait one full integration period for the first ADC result to be ready. */
    sleep_ms(110);

    for (int i = 0; i < 5; i++)
    {
        uint16_t als_raw, white_raw;
        float lux;

        status = VEML7700_ReadALS(&als_raw);
        if (status != VEML7700_OK)
        {
            printf("ERROR: Failed to read ALS (status=%d).\n", status);
        }

        status = VEML7700_ReadWhite(&white_raw);
        if (status != VEML7700_OK)
        {
            printf("ERROR: Failed to read WHITE (status=%d).\n", status);
        }

        status = VEML7700_GetLux(&lux);
        if (status != VEML7700_OK)
        {
            printf("ERROR: Failed to get lux (status=%d).\n", status);
        }

        printf("ALS: %5u counts  WHITE: %5u counts  Lux: %.4f lx\n",
               als_raw, white_raw, (double)lux);

        sleep_ms(200);
    }

    printf("\n");

    /* --- Part 2: Switch to high-sensitivity settings (gain x2, IT 800 ms) --- */
    printf("Switching to high-sensitivity: gain x2, IT 800 ms\n");
    printf("Resolution: 0.0042 lx/count, max range: ~275 lx\n");
    printf("--------------------------------------------------\n");

    status = VEML7700_Configure(VEML7700_GAIN_2, VEML7700_IT_800MS);
    if (status != VEML7700_OK)
    {
        printf("ERROR: Failed to configure sensor (status=%d).\n", status);
        return 1;
    }

    /* Wait one full integration period for the new settings to take effect. */
    sleep_ms(810);

    for (int i = 0; i < 5; i++)
    {
        float lux;
        status = VEML7700_GetLux(&lux);
        if (status != VEML7700_OK)
        {
            printf("ERROR: Failed to get lux (status=%d).\n", status);
        }
        else
        {
            printf("Lux: %.4f lx\n", (double)lux);
        }
        sleep_ms(900);
    }

    printf("\n");

    /* --- Part 3: Demonstrate shutdown and wake-up --- */
    printf("Entering shutdown mode...\n");
    VEML7700_Shutdown();
    sleep_ms(500);

    printf("Waking up from shutdown...\n");
    VEML7700_PowerOn();

    /* Restore default settings and wait for integration to complete. */
    VEML7700_Configure(VEML7700_GAIN_1, VEML7700_IT_100MS);
    sleep_ms(110);

    /* Switch to wide-range settings for the continuous loop to avoid saturation
     * outdoors (gain x1/8, IT 25 ms — max ~140 klx). */
    status = VEML7700_Configure(VEML7700_GAIN_1_8, VEML7700_IT_25MS);
    if (status != VEML7700_OK)
    {
        printf("ERROR: Failed to configure sensor (status=%d).\n", status);
        return 1;
    }
    sleep_ms(30);

    printf("\nContinuous reading with non-linearity correction (gain x1/8, IT 25 ms).\n");
    printf("Press reset to stop.\n");
    printf("------------------------------------------------------------------------\n");

    while (true)
    {
        float lux_linear, lux_corrected;

        status = VEML7700_GetLux(&lux_linear);
        if (status != VEML7700_OK)
        {
            printf("ERROR: Failed to get lux (status=%d).\n", status);
            sleep_ms(100);
            continue;
        }

        status = VEML7700_GetLuxCorrected(&lux_corrected);
        if (status != VEML7700_OK)
        {
            printf("ERROR: Failed to get corrected lux (status=%d).\n", status);
            sleep_ms(100);
            continue;
        }

        printf("Linear: %8.2f lx   Corrected: %8.2f lx\n",
               (double)lux_linear, (double)lux_corrected);

        sleep_ms(100);
    }

    return 0;
}
