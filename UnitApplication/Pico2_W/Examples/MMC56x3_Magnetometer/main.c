/**
 * File: main.c - MMC56x3 Magnetometer Example
 * Description: Demonstrates how to use the MMC56x3 magnetometer driver
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "MMC56x3_wrapper.h"

int main() {
    // Initialize standard I/O for printing messages
    stdio_init_all();
    
    printf("\nMMC56x3 Magnetometer Example\n");
    printf("----------------------------\n\n");
    
    // Initialize I2C (using i2c0 on GPIO pins 4 and 5)
    printf("Initializing I2C...\n");
    i2c_init(i2c0, 400000);  // 400 kHz clock
    gpio_set_function(4, GPIO_FUNC_I2C);  // SDA on GPIO 4
    gpio_set_function(5, GPIO_FUNC_I2C);  // SCL on GPIO 5
    gpio_pull_up(4);  // Enable internal pull-up resistors
    gpio_pull_up(5);  // (might not be needed if your board has external pull-ups)
    
    // Create magnetometer object using C wrapper
    printf("Creating MMC56x3 instance...\n");
    MMC56x3_Instance* magnetometer = MMC56x3_Create(i2c0, MMC56X3_I2C_ADDR);
    if (magnetometer == NULL) {
        printf("ERROR: Failed to create MMC56x3 instance\n");
        return 1;
    }
    
    // Initialize the sensor
    printf("Initializing MMC56x3 sensor...\n");
    if (!MMC56x3_Begin(magnetometer)) {
        printf("ERROR: Failed to initialize MMC56x3 magnetometer\n");
        MMC56x3_Destroy(magnetometer);
        return 1;
    }
    
    printf("MMC56x3 initialized successfully!\n\n");
    printf("Starting magnetic field readings...\n");
    
    // Optional: Perform a set/reset operation to ensure accurate readings
    MMC56x3_MagnetSetReset(magnetometer);
    
    // Read data in a loop
    while (true) {
        MMC56x3_MagData data;
        
        if (MMC56x3_ReadData(magnetometer, &data)) {
            // Print magnetic field values with better formatting
            printf("Magnetic field: X=%.2f, Y=%.2f, Z=%.2f µT\n", 
                   data.x, data.y, data.z);
                   
            // Optional: Also print the temperature
            printf("Temperature: %.1f °C\n", data.temperature);
        } else {
            printf("ERROR: Failed to read magnetometer data\n");
        }
        
        // Wait between readings
        sleep_ms(100);  // 10 Hz update rate
    }
    
    // This will never be reached in the infinite loop above
    MMC56x3_Destroy(magnetometer);
    return 0;
}