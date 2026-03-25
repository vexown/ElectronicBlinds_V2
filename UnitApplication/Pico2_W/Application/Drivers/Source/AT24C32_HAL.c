#include "AT24C32_HAL.h"
#include <string.h>

/* Maximum number of acknowledge polling attempts after a write (one per ms, tWR budget + margin) */
#define WRITE_POLL_ATTEMPTS (AT24C32_WRITE_CYCLE_MS + 2)

/* I2C configuration for AT24C32 */
static const I2C_Config i2c_cfg =
{
    .instance = AT24C32_I2C_INSTANCE,
    .sda_pin  = AT24C32_I2C_SDA_PIN,
    .scl_pin  = AT24C32_I2C_SCL_PIN,
    .speed_hz = AT24C32_I2C_SPEED_HZ
};

/* Poll for acknowledgement after a write to detect when the internal write cycle has
 * completed. Returns AT24C32_OK as soon as the EEPROM ACKs its address, or
 * AT24C32_ERROR_WRITE_TIMEOUT if it does not respond within the polling budget. */
static AT24C32_Status wait_write_complete(void)
{
    for (uint8_t i = 0; i < WRITE_POLL_ATTEMPTS; i++)
    {
        if (I2C_IsDeviceReady(AT24C32_I2C_INSTANCE, AT24C32_I2C_ADDR) == I2C_OK)
        {
            return AT24C32_OK;
        }
        sleep_ms(1);
    }
    return AT24C32_ERROR_WRITE_TIMEOUT;
}

AT24C32_Status AT24C32_Init(void)
{
    if (I2C_Init(&i2c_cfg) != I2C_OK)
    {
        return AT24C32_ERROR_INIT;
    }

    if (I2C_IsDeviceReady(AT24C32_I2C_INSTANCE, AT24C32_I2C_ADDR) != I2C_OK)
    {
        return AT24C32_ERROR_NO_DEVICE;
    }

    return AT24C32_OK;
}

AT24C32_Status AT24C32_WriteByte(uint16_t mem_addr, uint8_t data)
{
    if (mem_addr >= AT24C32_MEM_SIZE)
    {
        return AT24C32_ERROR_INVALID_ADDR;
    }

    if (I2C_WriteMultiple16(AT24C32_I2C_INSTANCE, AT24C32_I2C_ADDR, mem_addr, &data, 1) != I2C_OK)
    {
        return AT24C32_ERROR_WRITE;
    }

    return wait_write_complete();
}

AT24C32_Status AT24C32_ReadByte(uint16_t mem_addr, uint8_t *data)
{
    if (mem_addr >= AT24C32_MEM_SIZE)
    {
        return AT24C32_ERROR_INVALID_ADDR;
    }

    return AT24C32_ReadSequential(mem_addr, data, 1);
}

AT24C32_Status AT24C32_WritePage(uint16_t mem_addr, const uint8_t *data, size_t length)
{
    /* Validate: address in range, length within page size, and write stays within
     * the current page (lower 5 bits of the address are the page offset). */
    if (mem_addr >= AT24C32_MEM_SIZE || length == 0 || length > AT24C32_PAGE_SIZE)
    {
        return AT24C32_ERROR_INVALID_ADDR;
    }

    uint16_t page_offset = mem_addr & (AT24C32_PAGE_SIZE - 1); // lower 5 bits
    if (page_offset + length > AT24C32_PAGE_SIZE)
    {
        return AT24C32_ERROR_INVALID_ADDR;
    }

    if (I2C_WriteMultiple16(AT24C32_I2C_INSTANCE, AT24C32_I2C_ADDR, mem_addr, data, length) != I2C_OK)
    {
        return AT24C32_ERROR_WRITE;
    }

    return wait_write_complete();
}

AT24C32_Status AT24C32_ReadSequential(uint16_t mem_addr, uint8_t *data, size_t length)
{
    if (mem_addr >= AT24C32_MEM_SIZE || length == 0)
    {
        return AT24C32_ERROR_INVALID_ADDR;
    }

    if (I2C_ReadMultiple16(AT24C32_I2C_INSTANCE, AT24C32_I2C_ADDR, mem_addr, data, length) != I2C_OK)
    {
        return AT24C32_ERROR_READ;
    }

    return AT24C32_OK;
}
