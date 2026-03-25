
/**
 * File: main.c - UBLOX GPS Example
 * Description: Demonstrates how to use the UBLOX GPS driver
 */

/* Library includes. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "UBLOX_GPS_HAL.h"

int main() 
{
    stdio_init_all();
    gps_init();

    char buffer[128];
    size_t len;
    
    printf("GPS Test Application\n");
    printf("--------------------\n");

    while (true) {
        if (gps_read_nmea(buffer, sizeof(buffer), &len)) {
            buffer[len] = '\0'; // Null-terminate for safety
            printf("Received NMEA sentence: %s", buffer);
            
            if (gps_process_sentence(buffer)) {
                GPS_Data* data = gps_get_data();
                
                if (data->valid) {
                    printf("Parsed GPS Data:\n");
                    printf("  Time: %02d:%02d:%02d\n", data->hours, data->minutes, data->seconds);
                    printf("  Date: %02d/%02d/%04d\n", data->day, data->month, data->year);
                    if (data->position_valid) {
                        printf("  Position: %.6f° %c, %.6f° %c\n",
                               data->latitude >= 0 ? data->latitude : -data->latitude,
                               data->latitude >= 0 ? 'N' : 'S',
                               data->longitude >= 0 ? data->longitude : -data->longitude,
                               data->longitude >= 0 ? 'E' : 'W');
                        printf("  Altitude: %.1f m\n", data->altitude);
                    } else {
                        printf("  Position: Invalid\n");
                    }
                    printf("  Speed: %.3f knots\n", data->speed_knots);
                    printf("  Course: %.1f°\n", data->course_deg);
                    printf("  Fix Quality: %d\n", data->fix_quality);
                    printf("  Satellites: %d\n", data->satellites_used);
                    printf("--------------------\n");
                } else {
                    printf("No fix yet\n");
                    printf("--------------------\n");
                }
            }
        }
        sleep_ms(10);
    }
    return 0;
}
