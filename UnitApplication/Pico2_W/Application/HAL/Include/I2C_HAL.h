#ifndef I2C_HAL_H
#define I2C_HAL_H

/* Key Features */
/*
    - Support for both I2C instances (I2C0 and I2C1)
    - Pin validation for valid I2C pin combinations
    - Simple read/write operations for single bytes
    - Support for multiple byte transfers
    - Bus scanning functionality
    - Device presence checking
    - Error handling for all operations
    - Configuration structure for easy initialization
*/

/* Example Use */
/*
void main() {
    // Configure I2C
    I2C_Config config = {
        .instance = I2C_INSTANCE_0,
        .sda_pin = 4,
        .scl_pin = 5,
        .speed_hz = 100000  // 100 kHz
    };
    
    // Initialize I2C
    I2C_Status status = I2C_Init(&config);
    if (status != I2C_OK) {
        // Handle error
        return;
    }
    
    // Example: Write to a device
    uint8_t device_addr = 0x50;  // Example device address
    status = I2C_WriteByte(I2C_INSTANCE_0, device_addr, 0x00, 0x42);
    
    // Example: Read from a device
    uint8_t data;
    status = I2C_ReadByte(I2C_INSTANCE_0, device_addr, 0x00, &data);
    
    // Example: Scan for devices
    static I2C_DeviceList devices;
    I2C_ScanBus(I2C_INSTANCE_0, &devices);
}
*/

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define MAX_I2C_DEVICES 112

/* I2C instance selection (on Pico there is I2C0 and I2C1) */
typedef enum 
{
    I2C_INSTANCE_0 = 0,
    I2C_INSTANCE_1
} I2C_Instance;

typedef struct 
{ 
    uint8_t addresses[MAX_I2C_DEVICES];
    size_t count; 
} I2C_DeviceList;

/* Error codes */
typedef enum 
{
    I2C_OK = 0,
    I2C_ERROR_INVALID_INSTANCE,
    I2C_ERROR_INVALID_PINS,
    I2C_ERROR_INIT_FAILED,
    I2C_ERROR_WRITE_FAILED,
    I2C_ERROR_WRITE_TIMEOUT,
    I2C_ERROR_READ_FAILED,
    I2C_ERROR_READ_TIMEOUT,
    I2C_ERROR_NO_DEVICE
} I2C_Status;

/* I2C configuration structure */
typedef struct 
{
    I2C_Instance instance;
    uint8_t sda_pin;
    uint8_t scl_pin;
    uint32_t speed_hz;
} I2C_Config;

/* Initialize I2C with provided configuration */
I2C_Status I2C_Init(const I2C_Config* config);

/* Write single byte to I2C device */
I2C_Status I2C_WriteByte(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/* Read single byte from I2C device */
I2C_Status I2C_ReadByte(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data);

/* Write multiple bytes to I2C device */
I2C_Status I2C_WriteMultiple(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t length);

/* Read multiple bytes from I2C device */
I2C_Status I2C_ReadMultiple(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t length);

/* Write multiple bytes to an I2C device that uses a 16-bit register address (e.g. EEPROM).
 * Sends [reg_addr_msb, reg_addr_lsb, data[0], ..., data[length-1]] in one transaction. */
I2C_Status I2C_WriteMultiple16(I2C_Instance instance, uint8_t dev_addr, uint16_t reg_addr, const uint8_t* data, size_t length);

/* Read multiple bytes from an I2C device that uses a 16-bit register address (e.g. EEPROM).
 * Performs a dummy write of [reg_addr_msb, reg_addr_lsb] to set the address pointer,
 * followed by a read of the requested number of bytes. */
I2C_Status I2C_ReadMultiple16(I2C_Instance instance, uint8_t dev_addr, uint16_t reg_addr, uint8_t* data, size_t length);

/* Check if device is present on the bus */
I2C_Status I2C_IsDeviceReady(I2C_Instance instance, uint8_t dev_addr);

/* Scan for devices on the I2C bus */
I2C_Status I2C_ScanBus(I2C_Instance instance, I2C_DeviceList *device_list);

#endif // I2C_HAL_H
