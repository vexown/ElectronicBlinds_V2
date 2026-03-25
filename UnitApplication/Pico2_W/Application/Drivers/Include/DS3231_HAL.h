/**
 * @file DS3231_HAL.h
 * @brief HAL driver for the Maxim DS3231 extremely accurate I2C RTC.
 *
 * The DS3231 is a low-cost, extremely accurate real-time clock with an integrated
 * temperature-compensated crystal oscillator (TCXO). The TCXO automatically corrects
 * the oscillator frequency against temperature drift, achieving ±2ppm accuracy from
 * 0°C to +40°C and ±3.5ppm across the full industrial range (-40°C to +85°C).
 *
 * ---
 *
 * @section registers Register Map Overview
 *
 * The DS3231 exposes 19 registers over I2C (addresses 0x00–0x12). All timekeeping
 * registers use Binary-Coded Decimal (BCD) encoding. Registers 0x00–0x06 hold the
 * current time and date. On a multi-byte access, the address pointer auto-increments,
 * so all 7 timekeeping registers can be read or written in a single I2C transaction.
 *
 * | Address | Register        | Range           |
 * |---------|-----------------|-----------------|
 * | 0x00    | Seconds         | 00–59           |
 * | 0x01    | Minutes         | 00–59           |
 * | 0x02    | Hours           | 00–23 (24h)     |
 * | 0x03    | Day of week     | 1–7             |
 * | 0x04    | Date            | 01–31           |
 * | 0x05    | Month/Century   | 01–12           |
 * | 0x06    | Year            | 00–99           |
 * | 0x0E    | Control         | —               |
 * | 0x0F    | Status          | —               |
 *
 * @section bcd BCD Encoding
 *
 * Each timekeeping register stores its value in BCD (Binary-Coded Decimal). For example, the value 59
 * is stored as 0x59 (upper nibble = 5 tens, lower nibble = 9 ones), not 0x3B.
 * This driver handles BCD conversion internally; all values in @ref DS3231_DateTime
 * are plain decimal integers.
 *
 * @section hours Hours Register (0x02)
 *
 * Bit 6 of the hours register selects the clock format:
 * - Bit 6 = 0: 24-hour mode. Bits [5:4] hold the 10-hour digit (0–2),
 *              bits [3:0] hold the 1-hour digit (0–9).
 * - Bit 6 = 1: 12-hour mode. Bit 5 is the AM/PM indicator (1 = PM).
 *
 * This driver operates exclusively in 24-hour mode. When writing hours,
 * bit 6 is always kept at 0.
 *
 * @section day Day-of-Week Register (0x03)
 *
 * The day-of-week value (1–7) is user-defined. The DS3231 does not assign
 * meaning to specific values; it only requires that values are sequential.
 * The register increments at midnight. Illogical entries cause undefined behaviour.
 *
 * @section osf Oscillator Stop Flag (OSF)
 *
 * Bit 7 of the Status register (0x0F) is the Oscillator Stop Flag. It is set to 1:
 * - On the very first application of power.
 * - Whenever VCC and VBAT are both insufficient to sustain oscillation.
 * - When the EOSC bit in the Control register is set (oscillator disabled).
 *
 * When OSF is set, the timekeeping data cannot be trusted. @ref DS3231_Init
 * detects and clears this flag. The caller should treat a fresh power-up
 * as requiring a call to @ref DS3231_SetDateTime before relying on @ref DS3231_GetDateTime.
 *
 * @section i2c I2C Interface
 *
 * The DS3231 communicates over I2C with a 7-bit address of 0x68 (all address pins
 * tied to GND). It supports both standard mode (100kHz) and fast mode (400kHz).
 * The bus pins, instance, and speed are configurable via the macros below.
 */

#ifndef DS3231_HAL_H
#define DS3231_HAL_H

#include "pico/stdlib.h"
#include "I2C_HAL.h"

/** @defgroup DS3231_I2C_Config I2C Configuration
 *  @brief Bus settings for the DS3231. Modify these to match your hardware wiring.
 * @{
 */

/** @brief 7-bit I2C address of the DS3231. Fixed when all address pins are tied to GND. */
#define DS3231_I2C_ADDR     0x68

/** @brief I2C peripheral instance to use (I2C_INSTANCE_0 or I2C_INSTANCE_1). */
#define DS3231_I2C_INSTANCE I2C_INSTANCE_0

/** @brief GPIO pin number for I2C SDA. */
#define DS3231_I2C_SDA_PIN  4

/** @brief GPIO pin number for I2C SCL. */
#define DS3231_I2C_SCL_PIN  5

/** @brief I2C clock speed in Hz. The DS3231 supports up to 400kHz (fast mode). */
#define DS3231_I2C_SPEED_HZ 400000

/** @} */

/** @defgroup DS3231_Registers Register Addresses
 *  @brief Internal register map of the DS3231 (see datasheet Figure 1).
 * @{
 */

/** @brief Seconds register. BCD, range 00–59. Bit 7 is unused (always 0). */
#define DS3231_REG_SECONDS  0x00

/** @brief Minutes register. BCD, range 00–59. Bit 7 is unused (always 0). */
#define DS3231_REG_MINUTES  0x01

/**
 * @brief Hours register. BCD, range 00–23 in 24-hour mode.
 *
 * Bit 6 = 0 selects 24-hour mode (used by this driver). Bits [5:4] = 10-hour
 * digit, bits [3:0] = 1-hour digit. Bit 6 = 1 enables 12-hour mode, where
 * bit 5 becomes the AM/PM indicator; this mode is not supported by this driver.
 */
#define DS3231_REG_HOURS    0x02

/**
 * @brief Day-of-week register. Raw integer, range 1–7.
 *
 * The mapping of numeric values to day names is user-defined. Values must be
 * sequential and non-zero. This register increments at midnight. Note: this
 * register is NOT BCD encoded.
 */
#define DS3231_REG_DAY      0x03

/** @brief Date register. BCD, range 01–31. The DS3231 automatically adjusts
 *         for months with fewer than 31 days, including leap year corrections
 *         valid through the year 2100. */
#define DS3231_REG_DATE     0x04

/**
 * @brief Month register. BCD, range 01–12.
 *
 * Bit 7 is the century bit. It toggles when the years register overflows from
 * 99 to 00. This driver does not expose the century bit; years are treated as
 * offsets within the range 2000–2099.
 */
#define DS3231_REG_MONTH    0x05

/** @brief Year register. BCD, range 00–99 (representing 2000–2099). */
#define DS3231_REG_YEAR     0x06

/**
 * @brief Control register (0x0E).
 *
 * Key bits:
 * - Bit 7 (EOSC): Enable Oscillator (active low). Writing 1 stops the oscillator
 *   when running on battery. Set to 0 (oscillator enabled) on power-up.
 * - Bit 2 (INTCN): Interrupt Control. 1 = INT/SQW pin reflects alarm state.
 * - Bits [4:3] (RS2:RS1): Square-wave output frequency select.
 */
#define DS3231_REG_CONTROL  0x0E

/**
 * @brief Status register (0x0F).
 *
 * Key bits:
 * - Bit 7 (OSF): Oscillator Stop Flag. Set when the oscillator has stopped or
 *   was stopped. Indicates that timekeeping data may be invalid.
 * - Bit 2 (BSY): Busy flag. Set during a TCXO temperature conversion.
 * - Bit 1 (A2F): Alarm 2 matched flag.
 * - Bit 0 (A1F): Alarm 1 matched flag.
 */
#define DS3231_REG_STATUS   0x0F

/** @} */

/** @defgroup DS3231_StatusBits Status Register Bit Masks
 * @{
 */

/**
 * @brief Oscillator Stop Flag (OSF) mask for the Status register.
 *
 * When this bit is 1, the oscillator has stopped at some point since the flag
 * was last cleared. This bit is set on the first application of power and must
 * be explicitly cleared by writing 0. @ref DS3231_Init handles this automatically.
 */
#define DS3231_STATUS_OSF   0x80

/** @} */

/**
 * @brief Return codes for all DS3231 driver functions.
 */
typedef enum
{
    DS3231_OK = 0,        /**< Operation completed successfully. */
    DS3231_ERROR_INIT,    /**< I2C peripheral initialisation failed. */
    DS3231_ERROR_READ,    /**< I2C read transaction failed. */
    DS3231_ERROR_WRITE,   /**< I2C write transaction failed. */
    DS3231_ERROR_NO_DEVICE /**< DS3231 did not acknowledge its I2C address. */
} DS3231_Status;

/**
 * @brief Holds a complete date and time snapshot from the DS3231.
 *
 * All fields are plain decimal integers. BCD encoding and decoding is handled
 * internally by the driver. All values must be within the ranges documented
 * below; out-of-range values written via @ref DS3231_SetDateTime result in
 * undefined behaviour as per the datasheet.
 */
typedef struct
{
    uint8_t seconds; /**< Seconds, 0–59. */
    uint8_t minutes; /**< Minutes, 0–59. */
    uint8_t hours;   /**< Hours, 0–23. 24-hour format only. */
    uint8_t day;     /**< Day of week, 1–7. Mapping to day names is user-defined. */
    uint8_t date;    /**< Day of month, 1–31. */
    uint8_t month;   /**< Month, 1–12. */
    uint8_t year;    /**< Year within century, 0–99 (i.e., 2000–2099). */
} DS3231_DateTime;

/**
 * @brief Initialise the DS3231 driver and prepare the device for use.
 *
 * Performs the following steps:
 * 1. Initialises the I2C peripheral with the configuration defined by the
 *    DS3231_I2C_* macros.
 * 2. Verifies that the DS3231 acknowledges its I2C address.
 * 3. Reads the Status register and clears the Oscillator Stop Flag (OSF) if set.
 *
 * @note If OSF was set (first power-up or oscillator failure), the timekeeping
 *       data is unreliable. The caller should call @ref DS3231_SetDateTime to
 *       load a known-good time before calling @ref DS3231_GetDateTime.
 *
 * @return DS3231_OK on success, or the relevant DS3231_ERROR_* code on failure.
 */
DS3231_Status DS3231_Init(void);

/**
 * @brief Read the current date and time from the DS3231.
 *
 * All 7 timekeeping registers (0x00–0x06) are fetched in a single I2C burst
 * read. The DS3231 internally uses a shadow buffer for this purpose, so the
 * values returned represent a coherent snapshot even if the clock ticks mid-read.
 *
 * @param[out] dt  Pointer to a @ref DS3231_DateTime struct to populate.
 *                 Must not be NULL.
 *
 * @return DS3231_OK on success, or DS3231_ERROR_READ on I2C failure.
 */
DS3231_Status DS3231_GetDateTime(DS3231_DateTime *dt);

/**
 * @brief Write a new date and time to the DS3231.
 *
 * All 7 timekeeping registers (0x00–0x06) are written in a single I2C burst.
 * Per the datasheet, writing the seconds register resets the countdown chain,
 * so all remaining registers must be written within 1 second of the seconds
 * write — the burst write satisfies this requirement.
 *
 * The hours register is always written in 24-hour mode (bit 6 = 0).
 *
 * @param[in] dt  Pointer to a @ref DS3231_DateTime struct with the desired time.
 *                All fields must be within their documented ranges. Must not be NULL.
 *
 * @return DS3231_OK on success, or DS3231_ERROR_WRITE on I2C failure.
 */
DS3231_Status DS3231_SetDateTime(const DS3231_DateTime *dt);

#endif /* DS3231_HAL_H */
