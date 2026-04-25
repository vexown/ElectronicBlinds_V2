/**
 * @file VEML7700.h
 * @brief HAL driver for the Vishay VEML7700 high accuracy ambient light sensor.
 *
 * The VEML7700 is a 16-bit ambient light sensor with an I2C interface. It includes
 * an ALS (ambient light) photodiode and a WHITE broadband photodiode, each backed by
 * a 16-bit integrating ADC. The sensor supports configurable gain and integration time
 * settings to cover a dynamic range from 0 lx to ~140 klx.
 *
 * ---
 *
 * @section registers Register Map Overview
 *
 * The VEML7700 exposes six 16-bit command code registers over I2C (addresses 0x00–0x07,
 * address 0x03 is reserved). All registers are 16-bit, transferred LSB-first.
 *
 * | Command Code | Register   | Access |
 * |--------------|------------|--------|
 * | 0x00         | ALS_CONF_0 | R/W    |
 * | 0x01         | ALS_WH     | R/W    |
 * | 0x02         | ALS_WL     | R/W    |
 * | 0x03         | PSM        | R/W    |
 * | 0x04         | ALS        | R      |
 * | 0x05         | WHITE      | R      |
 * | 0x06         | ALS_INT    | R      |
 * | 0x07         | ID         | R      |
 *
 * @section conf Configuration Register (0x00)
 *
 * | Bits  | Name       | Description                                      |
 * |-------|------------|--------------------------------------------------|
 * | 15:13 | Reserved   | Must be written as 000                           |
 * | 12:11 | ALS_GAIN   | Gain: 00=x1, 01=x2, 10=x1/8, 11=x1/4            |
 * | 10    | Reserved   | Must be written as 0                             |
 * | 9:6   | ALS_IT     | Integration time (see @ref VEML7700_IntegTime)   |
 * | 5:4   | ALS_PERS   | Interrupt persistence: 00=1, 01=2, 10=4, 11=8   |
 * | 3:2   | Reserved   | Must be written as 00                            |
 * | 1     | ALS_INT_EN | Interrupt enable: 0=off, 1=on                    |
 * | 0     | ALS_SD     | Shutdown: 0=power on, 1=shutdown                 |
 *
 * @note The power-on default value of ALS_CONF_0 is 0x0001 (shutdown mode).
 *       @ref VEML7700_Init clears this bit and powers the device on.
 *
 * @section lux Lux Conversion
 *
 * The raw 16-bit ALS count is converted to lux using:
 *
 *   resolution = 0.0042 * (800 / IT_ms) * (2 / gain_factor)
 *   lux = raw_count * resolution
 *
 * Where gain_factor is 1, 2, 0.25, or 0.125 for x1, x2, x1/4, x1/8 respectively.
 * This linear formula is accurate for illuminance below ~1000 lx. For higher values,
 * refer to the Vishay application note for the optional polynomial correction.
 *
 * @note The interrupt pin is not available on the VEML7700. The threshold window
 *       registers (ALS_WH, ALS_WL) and interrupt status register are present in
 *       the register map but cannot trigger a physical interrupt signal.
 *
 * @section i2c I2C Interface
 *
 * The VEML7700 has a fixed 7-bit I2C address of 0x10. It supports standard mode
 * (10–100 kHz) and fast mode (up to 400 kHz). The bus pins, instance, and speed
 * are configurable via the macros below.
 */

#ifndef VEML7700_H
#define VEML7700_H

#include "pico/stdlib.h"
#include "I2C_HAL.h"

/** @defgroup VEML7700_I2C_Config I2C Configuration
 *  @brief Bus settings for the VEML7700. Modify these to match your hardware wiring.
 * @{
 */

/** @brief Fixed 7-bit I2C address of the VEML7700. */
#define VEML7700_I2C_ADDR       0x10

/** @brief I2C peripheral instance to use (I2C_INSTANCE_0 or I2C_INSTANCE_1). */
#define VEML7700_I2C_INSTANCE   I2C_INSTANCE_0

/** @brief GPIO pin number for I2C SDA. */
#define VEML7700_I2C_SDA_PIN    4

/** @brief GPIO pin number for I2C SCL. */
#define VEML7700_I2C_SCL_PIN    5

/** @brief I2C clock speed in Hz. The VEML7700 supports up to 400 kHz (fast mode). */
#define VEML7700_I2C_SPEED_HZ   400000

/** @} */

/** @defgroup VEML7700_Registers Command Code Register Addresses
 * @{
 */

/** @brief Configuration register. Controls gain, integration time, shutdown, and interrupts. */
#define VEML7700_REG_ALS_CONF   0x00

/** @brief ALS high threshold window register (16-bit). */
#define VEML7700_REG_ALS_WH     0x01

/** @brief ALS low threshold window register (16-bit). */
#define VEML7700_REG_ALS_WL     0x02

/** @brief Power saving mode register. */
#define VEML7700_REG_PSM        0x03

/** @brief ALS 16-bit output data register. Read-only. */
#define VEML7700_REG_ALS        0x04

/** @brief WHITE channel 16-bit output data register. Read-only. */
#define VEML7700_REG_WHITE      0x05

/** @brief ALS interrupt status register. Read-only. */
#define VEML7700_REG_ALS_INT    0x06

/** @brief Device ID register. Read-only. Low byte is 0x81; high byte is 0xC4 for address 0x10. */
#define VEML7700_REG_ID         0x07

/** @} */

/** @defgroup VEML7700_ConfBits ALS_CONF_0 Register Bit Masks and Shifts
 * @{
 */

#define VEML7700_CONF_GAIN_SHIFT    11
#define VEML7700_CONF_GAIN_MASK     (0x03u << VEML7700_CONF_GAIN_SHIFT)

#define VEML7700_CONF_IT_SHIFT      6
#define VEML7700_CONF_IT_MASK       (0x0Fu << VEML7700_CONF_IT_SHIFT)

#define VEML7700_CONF_PERS_SHIFT    4
#define VEML7700_CONF_PERS_MASK     (0x03u << VEML7700_CONF_PERS_SHIFT)

#define VEML7700_CONF_INT_EN        (1u << 1)
#define VEML7700_CONF_SD            (1u << 0)

/** @} */

/** @brief Expected low byte of the Device ID register (command code 0x07). */
#define VEML7700_DEVICE_ID          0x81

/**
 * @brief ALS gain selection options.
 *
 * The gain multiplier affects the sensitivity of the photodiode. Higher gain increases
 * sensitivity at low light but reduces the maximum measurable illuminance.
 */
typedef enum
{
    VEML7700_GAIN_1   = 0x00, /**< ALS gain x1.   Maximum ~36 klx at IT=100ms. */
    VEML7700_GAIN_2   = 0x01, /**< ALS gain x2.   Maximum ~18 klx at IT=100ms. */
    VEML7700_GAIN_1_8 = 0x02, /**< ALS gain x1/8. Maximum ~140 klx at IT=25ms. */
    VEML7700_GAIN_1_4 = 0x03  /**< ALS gain x1/4. Maximum ~70 klx at IT=25ms.  */
} VEML7700_Gain;

/**
 * @brief ALS integration time selection options.
 *
 * Longer integration times yield higher resolution and sensitivity but reduce the
 * maximum measurable illuminance and increase the refresh time.
 */
typedef enum
{
    VEML7700_IT_25MS  = 0x0C, /**< Integration time 25 ms.  */
    VEML7700_IT_50MS  = 0x08, /**< Integration time 50 ms.  */
    VEML7700_IT_100MS = 0x00, /**< Integration time 100 ms. */
    VEML7700_IT_200MS = 0x01, /**< Integration time 200 ms. */
    VEML7700_IT_400MS = 0x02, /**< Integration time 400 ms. */
    VEML7700_IT_800MS = 0x03  /**< Integration time 800 ms. */
} VEML7700_IntegTime;

/**
 * @brief Return codes for all VEML7700 driver functions.
 */
typedef enum
{
    VEML7700_OK = 0,          /**< Operation completed successfully. */
    VEML7700_ERROR_INIT,      /**< I2C peripheral initialisation failed. */
    VEML7700_ERROR_READ,      /**< I2C read transaction failed. */
    VEML7700_ERROR_WRITE,     /**< I2C write transaction failed. */
    VEML7700_ERROR_NO_DEVICE  /**< VEML7700 did not acknowledge its I2C address. */
} VEML7700_Status;

/**
 * @brief Initialise the VEML7700 driver and power the sensor on.
 *
 * Performs the following steps:
 * 1. Initialises the I2C peripheral with the configuration defined by the
 *    VEML7700_I2C_* macros.
 * 2. Verifies that the VEML7700 acknowledges its I2C address.
 * 3. Verifies the device ID register.
 * 4. Configures the sensor with default settings (gain x1, IT 100 ms) and
 *    clears the shutdown bit to enable measurements.
 *
 * @note After powering on, allow at least one full integration period to elapse
 *       before reading the ALS output to ensure the first result is valid.
 *
 * @return VEML7700_OK on success, or the relevant VEML7700_ERROR_* code on failure.
 */
VEML7700_Status VEML7700_Init(void);

/**
 * @brief Configure the ALS gain and integration time.
 *
 * Reads the current configuration register, updates the gain and integration time
 * fields, and writes the new configuration back. Other fields (persistence,
 * interrupt enable, shutdown) are preserved.
 *
 * @note After changing settings, allow one full integration period before reading
 *       ALS data to allow the ADC to complete a fresh conversion.
 *
 * @param[in] gain  Gain setting. See @ref VEML7700_Gain.
 * @param[in] it    Integration time setting. See @ref VEML7700_IntegTime.
 *
 * @return VEML7700_OK on success, or VEML7700_ERROR_READ / VEML7700_ERROR_WRITE on failure.
 */
VEML7700_Status VEML7700_Configure(VEML7700_Gain gain, VEML7700_IntegTime it);

/**
 * @brief Read the raw 16-bit ALS output count.
 *
 * @param[out] raw  Pointer to store the raw ADC count (0–65535). Must not be NULL.
 *
 * @return VEML7700_OK on success, or VEML7700_ERROR_READ on I2C failure.
 */
VEML7700_Status VEML7700_ReadALS(uint16_t *raw);

/**
 * @brief Read the raw 16-bit WHITE channel output count.
 *
 * The WHITE channel covers a broader spectral range than the ALS channel.
 *
 * @param[out] raw  Pointer to store the raw WHITE count (0–65535). Must not be NULL.
 *
 * @return VEML7700_OK on success, or VEML7700_ERROR_READ on I2C failure.
 */
VEML7700_Status VEML7700_ReadWhite(uint16_t *raw);

/**
 * @brief Read the ALS channel and convert the result to lux.
 *
 * Reads the raw ALS count and multiplies by the resolution factor derived from
 * the currently active gain and integration time settings.
 *
 * Resolution (lx/count) = 0.0042 * (800 / IT_ms) * (2 / gain_factor)
 *
 * @param[out] lux  Pointer to store the computed illuminance in lux. Must not be NULL.
 *
 * @return VEML7700_OK on success, or VEML7700_ERROR_READ on I2C failure.
 */
VEML7700_Status VEML7700_GetLux(float *lux);

/**
 * @brief Read the ALS channel and return lux with Vishay non-linearity correction applied.
 *
 * Internally calls @ref VEML7700_GetLux to obtain the linear lux value, then applies
 * the 4th-order polynomial correction from the Vishay application note
 * "Designing the VEML7700 Into an Application" (document 84323):
 *
 *   lux_c = 6.0135e-13 * lux^4
 *         - 9.3924e-9  * lux^3
 *         + 8.1488e-5  * lux^2
 *         + 1.0023     * lux
 *
 * This correction compensates for the sensor's non-linear response above ~1000 lx.
 * At low illuminance the higher-order terms vanish and the result is effectively
 * identical to @ref VEML7700_GetLux.
 *
 * @note This correction is derived for gain x1/8 and x1/4 only. Per the Vishay
 *       application note, gain x1 and x2 become non-linear at high illuminance in
 *       a way this formula does not compensate for — use @ref VEML7700_GAIN_1_8 or
 *       @ref VEML7700_GAIN_1_4 when calling this function outdoors or in bright
 *       conditions.
 *
 * @note Once the raw count saturates at 65535, no correction can recover the true
 *       value. Combine gain x1/8 with IT 25 ms for the widest range (~140 klx).
 *
 * @param[out] lux  Pointer to store the corrected illuminance in lux. Must not be NULL.
 *
 * @return VEML7700_OK on success, or VEML7700_ERROR_READ on I2C failure.
 */
VEML7700_Status VEML7700_GetLuxCorrected(float *lux);

/**
 * @brief Enter shutdown mode to minimise power consumption.
 *
 * Sets the ALS_SD bit in the configuration register. The sensor draws typically
 * 0.5 µA in shutdown. The last measured ALS value is retained in the output
 * register and can still be read via I2C while in shutdown.
 *
 * @return VEML7700_OK on success, or VEML7700_ERROR_READ / VEML7700_ERROR_WRITE on failure.
 */
VEML7700_Status VEML7700_Shutdown(void);

/**
 * @brief Exit shutdown mode and resume continuous measurements.
 *
 * Clears the ALS_SD bit in the configuration register. Allow one full integration
 * period before reading ALS data after waking up.
 *
 * @return VEML7700_OK on success, or VEML7700_ERROR_READ / VEML7700_ERROR_WRITE on failure.
 */
VEML7700_Status VEML7700_PowerOn(void);

#endif /* VEML7700_H */
