#include "VEML7700.h"

/* I2C configuration for VEML7700 */
static const I2C_Config i2c_cfg =
{
    .instance = VEML7700_I2C_INSTANCE,
    .sda_pin  = VEML7700_I2C_SDA_PIN,
    .scl_pin  = VEML7700_I2C_SCL_PIN,
    .speed_hz = VEML7700_I2C_SPEED_HZ
};

/* Cached copy of ALS_CONF_0 — kept in sync with every write so gain/IT
 * are available for lux conversion without an extra I2C read. */
static uint16_t s_conf = 0x0000;

/* Write a 16-bit value to a VEML7700 command code register (LSB first). */
static VEML7700_Status write_register(uint8_t cmd, uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value & 0xFF), (uint8_t)(value >> 8) };
    if (I2C_WriteMultiple(VEML7700_I2C_INSTANCE, VEML7700_I2C_ADDR, cmd, buf, 2) != I2C_OK)
    {
        return VEML7700_ERROR_WRITE;
    }
    return VEML7700_OK;
}

/* Read a 16-bit value from a VEML7700 command code register (LSB first). */
static VEML7700_Status read_register(uint8_t cmd, uint16_t *value)
{
    uint8_t buf[2];
    if (I2C_ReadMultiple(VEML7700_I2C_INSTANCE, VEML7700_I2C_ADDR, cmd, buf, 2) != I2C_OK)
    {
        return VEML7700_ERROR_READ;
    }
    *value = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
    return VEML7700_OK;
}

/* Return lux-per-count resolution for the given gain and integration time.
 * Formula: 0.0042 * (800 / IT_ms) * (2 / gain_factor) */
static float get_resolution(VEML7700_Gain gain, VEML7700_IntegTime it)
{
    float it_ms;
    switch (it)
    {
        case VEML7700_IT_25MS:  it_ms = 25.0f;  break;
        case VEML7700_IT_50MS:  it_ms = 50.0f;  break;
        case VEML7700_IT_100MS: it_ms = 100.0f; break;
        case VEML7700_IT_200MS: it_ms = 200.0f; break;
        case VEML7700_IT_400MS: it_ms = 400.0f; break;
        case VEML7700_IT_800MS: it_ms = 800.0f; break;
        default:                it_ms = 100.0f; break;
    }

    float gain_factor;
    switch (gain)
    {
        case VEML7700_GAIN_1:   gain_factor = 1.0f;   break;
        case VEML7700_GAIN_2:   gain_factor = 2.0f;   break;
        case VEML7700_GAIN_1_8: gain_factor = 0.125f; break;
        case VEML7700_GAIN_1_4: gain_factor = 0.25f;  break;
        default:                gain_factor = 1.0f;   break;
    }

    return 0.0042f * (800.0f / it_ms) * (2.0f / gain_factor);
}

VEML7700_Status VEML7700_Init(void)
{
    if (I2C_Init(&i2c_cfg) != I2C_OK)
    {
        return VEML7700_ERROR_INIT;
    }

    if (I2C_IsDeviceReady(VEML7700_I2C_INSTANCE, VEML7700_I2C_ADDR) != I2C_OK)
    {
        return VEML7700_ERROR_NO_DEVICE;
    }

    /* Verify device identity */
    uint16_t id;
    if (read_register(VEML7700_REG_ID, &id) != VEML7700_OK)
    {
        return VEML7700_ERROR_READ;
    }
    if ((id & 0xFF) != VEML7700_DEVICE_ID)
    {
        return VEML7700_ERROR_WRONG_ID;
    }

    /* Power on with default settings: gain x1, IT 100 ms, persistence 1, interrupts off */
    s_conf = ((uint16_t)VEML7700_GAIN_1   << VEML7700_CONF_GAIN_SHIFT) |
             ((uint16_t)VEML7700_IT_100MS << VEML7700_CONF_IT_SHIFT);

    return write_register(VEML7700_REG_ALS_CONF, s_conf);
}

VEML7700_Status VEML7700_Configure(VEML7700_Gain gain, VEML7700_IntegTime it)
{
    s_conf &= (uint16_t)(~(VEML7700_CONF_GAIN_MASK | VEML7700_CONF_IT_MASK));
    s_conf |= ((uint16_t)gain << VEML7700_CONF_GAIN_SHIFT) |
              ((uint16_t)it   << VEML7700_CONF_IT_SHIFT);

    return write_register(VEML7700_REG_ALS_CONF, s_conf);
}

VEML7700_Status VEML7700_ReadALS(uint16_t *raw)
{
    return read_register(VEML7700_REG_ALS, raw);
}

VEML7700_Status VEML7700_ReadWhite(uint16_t *raw)
{
    return read_register(VEML7700_REG_WHITE, raw);
}

static VEML7700_Status read_lux_linear(float *lux)
{
    uint16_t raw;
    VEML7700_Status status = read_register(VEML7700_REG_ALS, &raw);
    if (status != VEML7700_OK)
    {
        return status;
    }

    VEML7700_Gain      gain = (VEML7700_Gain)     ((s_conf & VEML7700_CONF_GAIN_MASK) >> VEML7700_CONF_GAIN_SHIFT);
    VEML7700_IntegTime it   = (VEML7700_IntegTime)((s_conf & VEML7700_CONF_IT_MASK)   >> VEML7700_CONF_IT_SHIFT);

    *lux = (float)raw * get_resolution(gain, it);
    return VEML7700_OK;
}

VEML7700_Status VEML7700_GetLux(float *lux)
{
    return read_lux_linear(lux);
}

VEML7700_Status VEML7700_GetLuxCorrected(float *lux)
{
    float l;
    VEML7700_Status status = read_lux_linear(&l);
    if (status != VEML7700_OK)
    {
        return status;
    }

    /* Vishay 4th-order polynomial correction for non-linearity at high illuminance.
     * Source: Vishay application note "Designing the VEML7700 Into an Application". */
    *lux = (6.0135e-13f * l * l * l * l)
         - (9.3924e-9f  * l * l * l)
         + (8.1488e-5f  * l * l)
         + (1.0023f     * l);

    return VEML7700_OK;
}

VEML7700_Status VEML7700_Shutdown(void)
{
    s_conf |= VEML7700_CONF_SD;
    return write_register(VEML7700_REG_ALS_CONF, s_conf);
}

VEML7700_Status VEML7700_PowerOn(void)
{
    s_conf &= (uint16_t)(~VEML7700_CONF_SD);
    return write_register(VEML7700_REG_ALS_CONF, s_conf);
}
