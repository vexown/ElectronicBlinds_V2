#include "DS3231_HAL.h"

/* I2C configuration for DS3231 */
static const I2C_Config i2c_cfg =
{
    .instance = DS3231_I2C_INSTANCE,
    .sda_pin  = DS3231_I2C_SDA_PIN,
    .scl_pin  = DS3231_I2C_SCL_PIN,
    .speed_hz = DS3231_I2C_SPEED_HZ
};

/* Convert BCD encoded byte to decimal */
static uint8_t bcd_to_dec(uint8_t bcd)
{
    return (uint8_t)(((bcd >> 4) * 10) + (bcd & 0x0F));
}

/* Convert decimal value to BCD encoded byte */
static uint8_t dec_to_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

DS3231_Status DS3231_Init(void)
{
    if (I2C_Init(&i2c_cfg) != I2C_OK)
    {
        return DS3231_ERROR_INIT;
    }

    if (I2C_IsDeviceReady(DS3231_I2C_INSTANCE, DS3231_I2C_ADDR) != I2C_OK)
    {
        return DS3231_ERROR_NO_DEVICE;
    }

    /* Read status register and clear OSF flag if set */
    uint8_t status_reg;
    if (I2C_ReadByte(DS3231_I2C_INSTANCE, DS3231_I2C_ADDR, DS3231_REG_STATUS, &status_reg) != I2C_OK)
    {
        return DS3231_ERROR_READ;
    }

    if (status_reg & DS3231_STATUS_OSF)
    {
        status_reg &= (uint8_t)(~DS3231_STATUS_OSF); // Clear OSF bit
        if (I2C_WriteByte(DS3231_I2C_INSTANCE, DS3231_I2C_ADDR, DS3231_REG_STATUS, status_reg) != I2C_OK)
        {
            return DS3231_ERROR_WRITE;
        }
    }

    return DS3231_OK;
}

DS3231_Status DS3231_GetDateTime(DS3231_DateTime *dt)
{
    /* Read all 7 timekeeping registers in a single burst starting from 0x00 */
    uint8_t buf[7];
    if (I2C_ReadMultiple(DS3231_I2C_INSTANCE, DS3231_I2C_ADDR, DS3231_REG_SECONDS, buf, 7) != I2C_OK)
    {
        return DS3231_ERROR_READ;
    }

    dt->seconds = bcd_to_dec(buf[0] & 0x7F); // Mask unused bit 7
    dt->minutes = bcd_to_dec(buf[1] & 0x7F); // Mask unused bit 7
    dt->hours   = bcd_to_dec(buf[2] & 0x3F); // Mask 12/24 and AM/PM bits, 24-hour mode assumed
    dt->day     = buf[3] & 0x07;              // Raw value 1-7, not BCD
    dt->date    = bcd_to_dec(buf[4] & 0x3F); // Mask unused upper bits
    dt->month   = bcd_to_dec(buf[5] & 0x1F); // Mask century bit 7
    dt->year    = bcd_to_dec(buf[6]);

    return DS3231_OK;
}

DS3231_Status DS3231_SetDateTime(const DS3231_DateTime *dt)
{
    /* Pack all 7 timekeeping registers and write them in a single burst starting from 0x00 */
    uint8_t buf[7];
    buf[0] = dec_to_bcd(dt->seconds);
    buf[1] = dec_to_bcd(dt->minutes);
    buf[2] = dec_to_bcd(dt->hours);  // Bit 6 stays 0: 24-hour mode
    buf[3] = dt->day & 0x07;         // Raw value 1-7, not BCD
    buf[4] = dec_to_bcd(dt->date);
    buf[5] = dec_to_bcd(dt->month);  // Century bit stays 0
    buf[6] = dec_to_bcd(dt->year);

    if (I2C_WriteMultiple(DS3231_I2C_INSTANCE, DS3231_I2C_ADDR, DS3231_REG_SECONDS, buf, 7) != I2C_OK)
    {
        return DS3231_ERROR_WRITE;
    }

    return DS3231_OK;
}
