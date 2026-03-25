#include "MMC56x3.hpp"
#include "hardware/i2c.h"
#include "Common.h"
#include <cmath>
#include <cstdio>

/* Constructor definition */
MMC56x3::MMC56x3(i2c_inst_t* i2c_instance, uint8_t address) :
    /* Initialize members (runs before constructor body) */
    _i2c(i2c_instance), 
    _addr(address),
    _ctrl2_cache(0) 
{/* Do nothing */}

bool MMC56x3::begin() 
{
    /* Check communication by reading product ID */
    uint8_t id;
    
    if (!readRegister(MMC56X3_REG_PRODUCT_ID, &id)) 
    {
        LOG("Failed to read MMC56x3 product ID\n");
        return false;
    }
    
    LOG("Detected MMC56x3 product ID: 0x%02x\n", id);
    
    if (id != MMC56X3_PRODUCT_ID) 
    {
        LOG("Unexpected MMC56x3 product ID: 0x%02x (expected 0x%02x)\n", id, MMC56X3_PRODUCT_ID);
        return false;
    }
    
    /* Reset the device */
    reset();
    
    /* Initialize by performing a set/reset of the magnetometer */
    magnetSetReset();
    
    /* Default to non-continuous mode */
    setContinuousMode(false);
    
    return true;
}

void MMC56x3::reset() 
{
    /* Perform software reset */
    writeRegister(MMC56X3_REG_CTRL1, MMC56X3_CTRL1_SW_RST);
    sleep_ms(20); // Wait for reset to complete 
    
    /* Reset cache values */
    _ctrl2_cache = 0;
}

void MMC56x3::magnetSetReset() 
{
    /* Set magnetization */
    writeRegister(MMC56X3_REG_CTRL0, MMC56X3_CTRL_SET);
    sleep_ms(1);
    
    /* Reset magnetization */
    writeRegister(MMC56X3_REG_CTRL0, MMC56X3_CTRL_RESET);
    sleep_ms(1);
}

void MMC56x3::setContinuousMode(bool mode) 
{
    if (mode) 
    {
        writeRegister(MMC56X3_REG_CTRL0, MMC56X3_CTRL_CMM_FREQ);
        _ctrl2_cache |= MMC56X3_CTRL2_CMM_EN;
    } 
    else 
    {
        _ctrl2_cache &= static_cast<uint8_t>(~MMC56X3_CTRL2_CMM_EN);
    }

    writeRegister(MMC56X3_REG_CTRL2, _ctrl2_cache);
}

bool MMC56x3::isContinuousMode() 
{
    return (_ctrl2_cache & MMC56X3_CTRL2_CMM_EN) != 0;
}

void MMC56x3::setDataRate(uint16_t rate) 
{
    /* Rates can be 0-255 or 1000 (high power) */
    if (rate > 255) 
    {
        rate = 255;
        _ctrl2_cache |= MMC56X3_CTRL2_HPOWER; // High power mode
    } 
    else 
    {
        _ctrl2_cache &= static_cast<uint8_t>(~MMC56X3_CTRL2_HPOWER); // Normal power mode
    }
    
    writeRegister(MMC56X3_REG_ODR, (uint8_t)rate);
    writeRegister(MMC56X3_REG_CTRL2, _ctrl2_cache);
}

float MMC56x3::readTemperature() 
{
    /* Cannot read temperature in continuous mode */
    if (isContinuousMode()) 
    {
        return NAN;
    }
    
    /* Trigger temperature measurement */
    writeRegister(MMC56X3_REG_CTRL0, MMC56X3_CTRL_TM_T);
    
    /* Wait for measurement completion */
    uint8_t status;
    uint32_t startTime = time_us_32();
    do {
        if (!readRegister(MMC56X3_REG_STATUS, &status)) 
        {
            return NAN;
        }
        
        if (status & MMC56X3_STATUS_MEAS_T_DONE) 
        {
            break;
        }
        
        sleep_ms(5);
        
        /* Timeout after 100ms */
        if (time_us_32() - startTime > 100000) 
        {
            LOG("MMC56x3 temperature measurement timeout\n");
            return NAN;
        }

    } while (true);
    
    /* Read temperature data */
    uint8_t temp_data;
    if (!readRegister(MMC56X3_REG_TEMP, &temp_data)) 
    {
        return NAN;
    }
    
    /* Convert to Celsius: 0.8°C/LSB offset -75°C */
    float temp = temp_data * 0.8f - 75.0f;
    
    return temp;
}

bool MMC56x3::readData(MagData& data) 
{
    int32_t x_raw, y_raw, z_raw;
    
    /* If not in continuous mode, trigger a measurement */
    if (!isContinuousMode()) 
    {
        if (!writeRegister(MMC56X3_REG_CTRL0, MMC56X3_CTRL_TM_M)) 
        {
            return false;
        }
        
        /* Wait for measurement to complete */
        uint8_t status;
        uint32_t startTime = time_us_32();
        do {
            if (!readRegister(MMC56X3_REG_STATUS, &status)) 
            {
                return false;
            }
            
            if (status & MMC56X3_STATUS_MEAS_M_DONE) 
            {
                break;
            }
            
            sleep_ms(5);
            
            /* Timeout after 100ms */
            if (time_us_32() - startTime > 100000) 
            {
                LOG("MMC56x3 measurement timeout\n");
                return false;
            }

        } while (true);
    }
    
    /* Read all measurement data (9 bytes for X, Y, Z including extra bits) */
    uint8_t buffer[9];
    if (!readRegisters(MMC56X3_REG_XOUT_L, buffer, 9)) 
    {
        return false;
    }
    
    /* Combine bytes to make 20-bit values for X, Y, Z */
    x_raw = ((int32_t)buffer[0] << 12) | ((int32_t)buffer[1] << 4) | ((buffer[6] & 0xF0) >> 4);
    y_raw = ((int32_t)buffer[2] << 12) | ((int32_t)buffer[3] << 4) | ((buffer[7] & 0xF0) >> 4);
    z_raw = ((int32_t)buffer[4] << 12) | ((int32_t)buffer[5] << 4) | ((buffer[8] & 0xF0) >> 4);
    
    /* Fix center offsets */
    x_raw -= (int32_t)1 << 19;
    y_raw -= (int32_t)1 << 19;
    z_raw -= (int32_t)1 << 19;
    
    /* Convert to microtesla (scaling factor 0.00625 µT/LSB per datasheet) */
    const float scaling = 0.00625f;
    data.x = (float)x_raw * scaling;
    data.y = (float)y_raw * scaling;
    data.z = (float)z_raw * scaling;

    /* Safety check for unreasonable values */
    const float MAX_REASONABLE_FIELD = 3277.0f;  // ±3277 µT is the actual sensor range
    if (fabsf(data.x) > MAX_REASONABLE_FIELD) data.x = 0.0f;
    if (fabsf(data.y) > MAX_REASONABLE_FIELD) data.y = 0.0f;
    if (fabsf(data.z) > MAX_REASONABLE_FIELD) data.z = 0.0f;
    
    /* Read temperature too */
    data.temperature = readTemperature();
    
    return true;
}

bool MMC56x3::readRegister(uint8_t reg, uint8_t* data) 
{
    return readRegisters(reg, data, 1);
}

bool MMC56x3::writeRegister(uint8_t reg, uint8_t data) 
{
    uint8_t buffer[2] = {reg, data};
    int result = i2c_write_blocking(_i2c, _addr, buffer, 2, false);
    return result == 2;
}

bool MMC56x3::readRegisters(uint8_t reg, uint8_t* data, size_t length) 
{
    int result = i2c_write_blocking(_i2c, _addr, &reg, 1, true);  // true to keep master control of bus
    if (result != 1) 
    {
        return false;
    }
    
    result = i2c_read_blocking(_i2c, _addr, data, length, false);
    
    return result == static_cast<int>(length);
}