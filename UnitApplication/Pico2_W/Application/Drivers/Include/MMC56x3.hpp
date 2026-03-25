#ifndef MMC56X3_HPP
#define MMC56X3_HPP

#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* I2C address for MMC56x3 */
#define MMC56X3_I2C_ADDR 0x30  // Default address

/* MMC56x3 registers */
#define MMC56X3_REG_XOUT_L     0x00
#define MMC56X3_REG_XOUT_H     0x01
#define MMC56X3_REG_YOUT_L     0x02
#define MMC56X3_REG_YOUT_H     0x03
#define MMC56X3_REG_ZOUT_L     0x04
#define MMC56X3_REG_ZOUT_H     0x05
#define MMC56X3_REG_TEMP       0x09
#define MMC56X3_REG_STATUS     0x18
#define MMC56X3_REG_ODR        0x1A
#define MMC56X3_REG_CTRL0      0x1B
#define MMC56X3_REG_CTRL1      0x1C
#define MMC56X3_REG_CTRL2      0x1D
#define MMC56X3_REG_PRODUCT_ID 0x39 

/* Control register bits */
#define MMC56X3_CTRL_TM_M      0x01  // Take measurement
#define MMC56X3_CTRL_TM_T      0x02  // Take temperature measurement
#define MMC56X3_CTRL_SET       0x08  // Set magnetization
#define MMC56X3_CTRL_RESET     0x10  // Reset magnetization
#define MMC56X3_CTRL_CMM_FREQ  0x80  // Enable continuous mode frequency

/* CTRL2 register bits */
#define MMC56X3_CTRL2_CMM_EN   0x10  // Enable continuous mode
#define MMC56X3_CTRL2_HPOWER   0x80  // High power mode

/* CTRL1 register bits */
#define MMC56X3_CTRL1_SW_RST   0x80  // Software reset

/* Status register bits */
#define MMC56X3_STATUS_MEAS_M_DONE 0x40  // Measurement done
#define MMC56X3_STATUS_MEAS_T_DONE 0x80  // Temperature measurement done

/* Expected product ID */
#define MMC56X3_PRODUCT_ID 0x10

/* Class definition (defines a cohesive unit composed of data (member variables) and functions (member methods)) */
class MMC56x3 
{
    /* Public section contains members that can be accessed from outside the class 
       This is part of C++ encapsulation - controlling access to class members */
    public:

        /*******************************  Nested structure declaration  *******************************/
        /* Structure to hold magnetometer data 
        In C++, you can define types within classes to logically group them together */
        struct MagData 
        {
            float x;
            float y;
            float z;
            float temperature;
        };

        /*******************************  Constructor declaration  *******************************/

        /* Constructor declaration 
           It's a special function that gets called when creating objects (defined outside the class) 
           The function name must match the class name exactly. Constructor has no return type (not even void) */
        MMC56x3(i2c_inst_t* i2c_instance, uint8_t address = MMC56X3_I2C_ADDR);

        /*******************************  Member function declarations  *******************************/
        /* Initialize the sensor */
        bool begin();

        /* Read magnetometer data */
        bool readData(MagData& data);
        
        /* Read temperature data */
        float readTemperature();

        /* Software reset */
        void reset();
        
        /* Set/reset magnetization */
        void magnetSetReset();

        /* Set output data rate */
        void setDataRate(uint16_t rate);
        
        /* Continuous mode control */
        void setContinuousMode(bool mode);
        bool isContinuousMode();

    /* Private section contains members that can only be accessed from within the class
       This hides implementation details and protects data from accidental modification */
    private:
        /* Member variables (attributes/fields) that store the object's state
           The underscore prefix (_) is a common convention to distinguish member variables */
        i2c_inst_t* _i2c;
        uint8_t _addr;
        uint8_t _ctrl2_cache;
        
        /* Private helper functions  - internal implementation that users don't need to see */
        bool readRegister(uint8_t reg, uint8_t* data);
        bool writeRegister(uint8_t reg, uint8_t data);
        bool readRegisters(uint8_t reg, uint8_t* data, size_t length);
};

#endif /* MMC56X3_HPP */