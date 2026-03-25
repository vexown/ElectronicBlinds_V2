/**
 * File: main.c - DS3231 RTC Example
 * Description: Demonstrates how to use the DS3231 RTC driver to set and read time and date.
 */

/* Library includes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "DS3231_HAL.h"

/* Day-of-week names. Index matches the day field convention used in this example (1=Monday). */
static const char *day_names[] = { "", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday" };

int main(void)
{
    stdio_init_all();

    printf("\nDS3231 RTC Example\n");
    printf("------------------\n\n");

    /* Initialize the DS3231. This configures I2C and clears the Oscillator Stop Flag
     * (OSF) if set. OSF is set on first power-up and any time the oscillator was stopped,
     * indicating that the stored time may not be valid. */
    printf("Initializing DS3231...\n");
    DS3231_Status status = DS3231_Init();
    if (status == DS3231_ERROR_NO_DEVICE)
    {
        printf("ERROR: DS3231 not found on I2C bus. Check wiring.\n");
        return 1;
    }
    if (status != DS3231_OK)
    {
        printf("ERROR: DS3231 initialization failed (status=%d).\n", status);
        return 1;
    }
    printf("DS3231 initialized successfully.\n\n");

    /* Set the initial date and time. On a real application this would only be done
     * once (e.g. when OSF was set, or triggered by a user command), as the DS3231
     * keeps time independently on its backup battery. */
    DS3231_DateTime dt = {
        .seconds = 0,
        .minutes = 30,
        .hours   = 9,
        .day     = 1,  // 1 = Monday (user-defined convention, must be sequential)
        .date    = 22,
        .month   = 3,
        .year    = 26  // 2026
    };

    printf("Setting initial time: %02d:%02d:%02d  %s %02d/%02d/20%02d\n",
           dt.hours, dt.minutes, dt.seconds,
           day_names[dt.day], dt.date, dt.month, dt.year);

    status = DS3231_SetDateTime(&dt);
    if (status != DS3231_OK)
    {
        printf("ERROR: Failed to set date and time (status=%d).\n", status);
        return 1;
    }
    printf("Time set successfully.\n\n");

    /* Read and print the current time once per second. */
    printf("Reading time...\n");
    printf("------------------\n");

    while (true)
    {
        status = DS3231_GetDateTime(&dt);
        if (status != DS3231_OK)
        {
            printf("ERROR: Failed to read date and time (status=%d).\n", status);
        }
        else
        {
            printf("%s  %02d/%02d/20%02d  %02d:%02d:%02d\n",
                   day_names[dt.day],
                   dt.date, dt.month, dt.year,
                   dt.hours, dt.minutes, dt.seconds);
        }

        sleep_ms(1000);
    }

    return 0;
}
