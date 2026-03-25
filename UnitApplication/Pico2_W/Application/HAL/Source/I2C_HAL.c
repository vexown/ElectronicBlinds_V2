/**
 * File: I2C_HAL.c
 * Description: High-level HAL for easier use of I2C on Raspberry Pico W
 */

/*******************************************************************************/
/*                                 INCLUDES                                    */
/*******************************************************************************/
#include "FreeRTOS.h"
#include "I2C_HAL.h"
#include <string.h>
#include "Common.h"

/*******************************************************************************/
/*                                  MACROS                                     */
/*******************************************************************************/
#define I2C_TIMEOUT_US      5000   //5ms
#define I2C_SEND_STOP       false  // Sends a Stop condition at the end of the transfer to release the bus
#define I2C_SEND_RESTART    true   // Retains control of the bus and sends a Restart instead of a Stop allowing to send more data right away

/*******************************************************************************/
/*                        STATIC FUNCTION DEFINITIONS                          */
/*******************************************************************************/

/* Description: Helper function to get i2c instance
 * Parameters:
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico 
 * 
 * Returns: the actual I2C instance in the data type defined in the pico-sdk 
 */
static i2c_inst_t* get_i2c_inst(I2C_Instance instance) 
{
    return (instance == I2C_INSTANCE_0) ? i2c0 : i2c1;
}

/* Description: Validate I2C pins
 * Parameters:
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico
 * 	- sda_pin: SDA pin of the I2C instance (default for I2C0 is GPIO4) 
 * 	- scl_pin: SCL pin of the I2C instance (default for I2C0 is GPIO5)
 * 
 * Returns: TRUE if the selected GPIO pins are available to be configured as SDA and SCL I2C pins. FALSE if not. */
static bool is_valid_i2c_pins(I2C_Instance instance, uint8_t sda_pin, uint8_t scl_pin) 
{
    if (instance == I2C_INSTANCE_0) 
    {
        /* I2C0 valid pin pairs: GPIO 0/1, 4/5, 8/9, 12/13, 16/17, 20/21 */
        const uint8_t valid_sda[] = {0, 4, 8, 12, 16, 20};
        const uint8_t valid_scl[] = {1, 5, 9, 13, 17, 21};
        
        for (int i = 0; i < 6; i++) 
	    {
            if (sda_pin == valid_sda[i] && scl_pin == valid_scl[i]) return true;
        }
    } 
    else 
    {
        /* I2C1 valid pin pairs: GPIO 2/3, 6/7, 10/11, 14/15, 18/19, 26/27 */
        const uint8_t valid_sda[] = {2, 6, 10, 14, 18, 26};
        const uint8_t valid_scl[] = {3, 7, 11, 15, 19, 27};
        
        for (int i = 0; i < 6; i++) 
	    {
            if (sda_pin == valid_sda[i] && scl_pin == valid_scl[i]) return true;
        }
    }
    return false;
}

/*******************************************************************************/
/*                        GLOBAL FUNCTION DEFINITIONS                          */
/*******************************************************************************/

/* Description: I2C initializtion function, uses pico-sdk I2C functions to:
 * 	- set the I2C instance 
 * 	- set the I2C pins 
 * 	- set the frequency of I2C communication 
 * 	- enables pull-up resistors on I2C lines
 * Parameters: 
 * 	- config: pointer to the filled out I2C configuration structure (see header file for the parameters that you need to provide) 
 * Returns: I2C_Status: I2C_ERROR_INSTANCE, I2C_ERROR_PINS or I2C_OK if all good
 */
I2C_Status I2C_Init(const I2C_Config* config)
{
    if (config->instance != I2C_INSTANCE_0 && config->instance != I2C_INSTANCE_1) return I2C_ERROR_INVALID_INSTANCE;

    if (!is_valid_i2c_pins(config->instance, config->sda_pin, config->scl_pin)) return I2C_ERROR_INVALID_PINS;

    i2c_inst_t* i2c = get_i2c_inst(config->instance);

    /* Initialize I2C peripheral */
    (void)i2c_init(i2c, config->speed_hz); //returns the actual set baudrate, idc
    
    /* Set up GPIO pins for I2C */
    gpio_set_function(config->sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(config->scl_pin, GPIO_FUNC_I2C);
    
    /* Enable pull-ups */
    gpio_pull_up(config->sda_pin);
    gpio_pull_up(config->scl_pin);

    return I2C_OK;
}

/* Description: Write a single byte to a specific register of the I2C device
 * Parameters: 
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico
 * 	- dev_addr: the address of the specific I2C device that you want to send the message to 
 * 		        (can be found in the datasheet or you can use I2C_ScanBus() to get addr of connected devices
 * 	- reg_addr: the address of the specific register of the selected I2C device that you want to write to 
 * 		        (register map should be available in the datasheet of the device) 
 * 	- data:     the actual value of the byte you want to write to the selected register of the selected I2C device 
 * Returns: I2C_Status: I2C_ERROR_WRITE_FAILED, I2C_ERROR_WRITE_TIMEOUT or I2C_OK
 */
I2C_Status I2C_WriteByte(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, uint8_t data) 
{
    i2c_inst_t* i2c = get_i2c_inst(instance);
    uint8_t buffer[2] = {reg_addr, data};
    
    int status = i2c_write_timeout_us(i2c, dev_addr, buffer, sizeof(buffer), I2C_SEND_STOP, I2C_TIMEOUT_US);

    switch (status) 
    {
        case PICO_ERROR_GENERIC:
            return I2C_ERROR_WRITE_FAILED;
        case PICO_ERROR_TIMEOUT:
            return I2C_ERROR_WRITE_TIMEOUT;
        default:
            return I2C_OK;
    }
}

/* Description: Read a single byte from a specific register of the I2C device
 * Parameters: 
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico
 * 	- dev_addr: the address of the specific I2C device that you want to ask for a readout (yes, in I2C you need to ask first heh) 
 * 		        (can be found in the datasheet or you can use I2C_ScanBus() to get addr of connected devices
 * 	- reg_addr: the address of the specific register of the selected I2C device that you want to read from 
 * 		        (register map should be available in the datasheet of the device) 
 * 	- *data:    the pointer to store the byte you want to read from the selected register of the selected I2C device 
 * Returns: I2C_Status: I2C_ERROR_WRITE_FAILED, I2C_ERROR_WRITE_TIMEOUT, I2C_ERROR_READ_FAILED, I2C_ERROR_READ_TIMEOUT or I2C_OK 
 */
I2C_Status I2C_ReadByte(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data) 
{
    i2c_inst_t* i2c = get_i2c_inst(instance);
    uint8_t buffer[1] = {reg_addr};
    
    /* Step 1: Write the register address to the device
       This step tells the slave device which register (or memory address) we want to read from.
       The master sends the register address to the slave, and the slave will know where to
       fetch the data from when the next read command is sent. */
    int status = i2c_write_timeout_us(i2c, dev_addr, buffer, sizeof(buffer), I2C_SEND_STOP, I2C_TIMEOUT_US);
    
    if (status == PICO_ERROR_GENERIC) {
        return I2C_ERROR_WRITE_FAILED;
    }
    else if (status == PICO_ERROR_TIMEOUT) {
        return I2C_ERROR_WRITE_TIMEOUT;
    }

    /* Step 2: Read the data from the device
       After the register address has been sent, we can now read the data from that register.
       The master sends a read command, and the slave responds by sending the requested data. */
    status = i2c_read_timeout_us(i2c, dev_addr, data, 1, I2C_SEND_STOP, I2C_TIMEOUT_US);
    
    if (status == PICO_ERROR_GENERIC) {
        return I2C_ERROR_READ_FAILED;
    }
    else if (status == PICO_ERROR_TIMEOUT) {
        return I2C_ERROR_READ_TIMEOUT;
    }

    return I2C_OK;
}


/* Description: Write multiple bytes to a specific register of the I2C device
 * Parameters: 
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico
 * 	- dev_addr: the address of the specific I2C device that you want to send the message to 
 * 		        (can be found in the datasheet or you can use I2C_ScanBus() to get addr of connected devices)
 * 	- reg_addr: the address of the specific register of the selected I2C device that you want to write to 
 * 		        (register map should be available in the datasheet of the device) 
 * 	- *data:    pointer to the data you want to write to the selected register of the selected I2C device
 * 	- length:   length of the data
 * Returns: I2C_Status: I2C_ERROR_WRITE_FAILED, I2C_ERROR_WRITE_TIMEOUT or I2C_OK
 */
I2C_Status I2C_WriteMultiple(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, const uint8_t* data, size_t length) 
{
    i2c_inst_t* i2c = get_i2c_inst(instance);

    uint8_t buffer[length + 1];  // One extra byte for the register address

    buffer[0] = reg_addr;  // In I2C, the first byte is the register address
    memcpy(buffer + 1, data, length);

    int status = i2c_write_timeout_us(i2c, dev_addr, buffer, length + 1, I2C_SEND_STOP, I2C_TIMEOUT_US);
    
    switch (status) 
    {
        case PICO_ERROR_GENERIC:
            return I2C_ERROR_WRITE_FAILED;
        
        case PICO_ERROR_TIMEOUT:
            return I2C_ERROR_WRITE_TIMEOUT;
        
        default:
            return I2C_OK;  // Success if no error
    }
}

/* Description: Read multiple bytes from a specific register of the I2C device
 * Parameters: 
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico
 * 	- dev_addr: the address of the specific I2C device that you want to ask for a readout (yes, in I2C you need to ask first heh) 
 * 		        (can be found in the datasheet or you can use I2C_ScanBus() to get addr of connected devices
 * 	- reg_addr: the address of the specific register of the selected I2C device that you want to read from 
 * 		        (register map should be available in the datasheet of the device) 
 * 	- *data:    the pointer to store the byte you want to read from the selected register of the selected I2C device
 * 	- length:   length of the data
 * Returns: I2C_Status: I2C_ERROR_WRITE_FAILED, I2C_ERROR_WRITE_TIMEOUT, I2C_ERROR_READ_FAILED, I2C_ERROR_READ_TIMEOUT or I2C_OK 
 */
I2C_Status I2C_ReadMultiple(I2C_Instance instance, uint8_t dev_addr, uint8_t reg_addr, uint8_t* data, size_t length)
{
    i2c_inst_t* i2c = get_i2c_inst(instance);
    uint8_t buffer[1] = {reg_addr};  // Buffer to hold the starting register address

    /* Step 1: Write the register address to the device
       This tells the slave which register to start reading from. 
       After this, the device will automatically increment the register address. */
    int status = i2c_write_timeout_us(i2c, dev_addr, buffer, sizeof(buffer), I2C_SEND_STOP, I2C_TIMEOUT_US);
    
    if (status == PICO_ERROR_GENERIC) {
        return I2C_ERROR_WRITE_FAILED;
    }
    else if (status == PICO_ERROR_TIMEOUT) {
        return I2C_ERROR_WRITE_TIMEOUT;
    }

    /* Step 2: Read the multiple consecutive registers
       The device will return the requested number of bytes starting from the register address provided. */
    status = i2c_read_timeout_us(i2c, dev_addr, data, length, I2C_SEND_STOP, I2C_TIMEOUT_US);
    
    if (status == PICO_ERROR_GENERIC) {
        return I2C_ERROR_READ_FAILED;
    }
    else if (status == PICO_ERROR_TIMEOUT) {
        return I2C_ERROR_READ_TIMEOUT;
    }

    return I2C_OK;
}

/* Description: Write multiple bytes to an I2C device that uses a 16-bit register address.
 *              Intended for devices such as EEPROMs where the internal address space exceeds
 *              8 bits and requires two address bytes to be sent before the data payload.
 * Parameters:
 *  - instance: I2C peripheral instance (I2C_INSTANCE_0 or I2C_INSTANCE_1)
 *  - dev_addr: 7-bit I2C device address
 *  - reg_addr: 16-bit register/memory address (sent MSB first, then LSB)
 *  - *data:    pointer to the data to write
 *  - length:   number of data bytes to write
 * Returns: I2C_Status: I2C_ERROR_WRITE_FAILED, I2C_ERROR_WRITE_TIMEOUT or I2C_OK
 */
I2C_Status I2C_WriteMultiple16(I2C_Instance instance, uint8_t dev_addr, uint16_t reg_addr, const uint8_t* data, size_t length)
{
    i2c_inst_t* i2c = get_i2c_inst(instance);

    uint8_t buffer[length + 2]; // Two extra bytes for the 16-bit register address
    buffer[0] = (uint8_t)(reg_addr >> 8);   // Address MSB
    buffer[1] = (uint8_t)(reg_addr & 0xFF); // Address LSB
    memcpy(buffer + 2, data, length);

    int status = i2c_write_timeout_us(i2c, dev_addr, buffer, length + 2, I2C_SEND_STOP, I2C_TIMEOUT_US);

    switch (status)
    {
        case PICO_ERROR_GENERIC:
            return I2C_ERROR_WRITE_FAILED;

        case PICO_ERROR_TIMEOUT:
            return I2C_ERROR_WRITE_TIMEOUT;

        default:
            return I2C_OK;
    }
}

/* Description: Read multiple bytes from an I2C device that uses a 16-bit register address.
 *              Performs a dummy write of the two-byte address to position the device's internal
 *              address counter, then issues a separate read transaction to fetch the data.
 * Parameters:
 *  - instance: I2C peripheral instance (I2C_INSTANCE_0 or I2C_INSTANCE_1)
 *  - dev_addr: 7-bit I2C device address
 *  - reg_addr: 16-bit register/memory address (sent MSB first, then LSB)
 *  - *data:    pointer to buffer where read bytes will be stored
 *  - length:   number of bytes to read
 * Returns: I2C_Status: I2C_ERROR_WRITE_FAILED, I2C_ERROR_WRITE_TIMEOUT, I2C_ERROR_READ_FAILED,
 *          I2C_ERROR_READ_TIMEOUT or I2C_OK
 */
I2C_Status I2C_ReadMultiple16(I2C_Instance instance, uint8_t dev_addr, uint16_t reg_addr, uint8_t* data, size_t length)
{
    i2c_inst_t* i2c = get_i2c_inst(instance);

    /* Step 1: Write the 16-bit address to position the device's internal address counter */
    uint8_t addr_buf[2] = {(uint8_t)(reg_addr >> 8), (uint8_t)(reg_addr & 0xFF)};
    int status = i2c_write_timeout_us(i2c, dev_addr, addr_buf, sizeof(addr_buf), I2C_SEND_STOP, I2C_TIMEOUT_US);

    if (status == PICO_ERROR_GENERIC)
    {
        return I2C_ERROR_WRITE_FAILED;
    }
    else if (status == PICO_ERROR_TIMEOUT)
    {
        return I2C_ERROR_WRITE_TIMEOUT;
    }

    /* Step 2: Read the requested bytes; the device auto-increments its address counter */
    status = i2c_read_timeout_us(i2c, dev_addr, data, length, I2C_SEND_STOP, I2C_TIMEOUT_US);

    if (status == PICO_ERROR_GENERIC)
    {
        return I2C_ERROR_READ_FAILED;
    }
    else if (status == PICO_ERROR_TIMEOUT)
    {
        return I2C_ERROR_READ_TIMEOUT;
    }

    return I2C_OK;
}

/* Description: Check if the I2C of the specified address exists on the bus and is ready to communicate
 * Parameters: 
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico
 * 	- dev_addr: the address of the I2C device to which we will attempt to communicate with to check if it's on the bus 
 * 		        (can be found in the datasheet or you can use I2C_ScanBus() to get addr of connected devices 
 * Returns: I2C_ERROR_NO_DEVICE or I2C_OK if device found 
 */
I2C_Status I2C_IsDeviceReady(I2C_Instance instance, uint8_t dev_addr) 
{
    i2c_inst_t* i2c = get_i2c_inst(instance);
    uint8_t dummy_data = 0;  // Dummy byte to send in the write operation

    /*
     In I2C communication, when a master sends data to a slave device at a given address, the slave:
      - is expected to acknowledge the receipt of the data by pulling the SDA line low during the acknowledge (ACK) clock pulse.
      - If the slave does not acknowledge, it indicates that the device is either not present or there is an issue on the bus.
     */
    int status = i2c_write_timeout_us(i2c, dev_addr, &dummy_data, 1, false, I2C_TIMEOUT_US);

    switch (status) 
    {
        case PICO_ERROR_GENERIC:
            // No ACK received, meaning the device is either not present or there's an error on the bus
            return I2C_ERROR_NO_DEVICE;

        case PICO_ERROR_TIMEOUT:
            // The write operation timed out (no ACK received within the timeout period)
            return I2C_ERROR_WRITE_TIMEOUT;

        default:
            // If we get here, the write was successful and an ACK was received
            return I2C_OK;
    }
}

/* Description: Scans the I2C bus and provides the list of devices that are on it and are able to communicate
 * Parameters: 
 * 	- instance: abstraction of the i2c_inst_t, you choose either I2C0 or I2C1 available on Pico
 * 	- *device_list: a structure to store the list of found devices (their addresses and total count) 
 * Returns: I2C_OK and the list of devices under the provided pointer 
 */
I2C_Status I2C_ScanBus(I2C_Instance instance, I2C_DeviceList *device_list)
{
    device_list->count = 0;
    
    /* Iterate through all the possible general purposed 7-bit I2C addresses */
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) 
    {
        /* If device found, save its address and increase the count */
        if (I2C_IsDeviceReady(instance, addr) == I2C_OK)
        {
            device_list->addresses[device_list->count] = addr;
            device_list->count++;

            LOG("Device found at address: 0x%02X\n", addr);
        }
    }

    return I2C_OK;
}
