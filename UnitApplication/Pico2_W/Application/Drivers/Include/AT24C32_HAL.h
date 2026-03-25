/**
 * @file AT24C32_HAL.h
 * @brief HAL driver for the Atmel AT24C32 I2C serial EEPROM.
 *
 * The AT24C32 is a 32Kbit (4096 x 8-bit) non-volatile EEPROM accessed over
 * a 2-wire (I2C) serial interface. It retains data for a minimum of 100 years
 * and supports up to 1,000,000 write cycles per byte. It is commonly paired
 * with the DS3231 RTC on breakout modules, sharing the same I2C bus.
 *
 * ---
 *
 * @section mem Memory Organisation
 *
 * The AT24C32 is organised as 128 pages of 32 bytes each, giving 4096 bytes total.
 * A 12-bit word address (0x000–0xFFF) selects any individual byte. The address is
 * sent as two bytes over I2C: the upper byte first (bits [11:8]), followed by the
 * lower byte (bits [7:0]).
 *
 * @section writes Write Operations
 *
 * Two write modes are supported:
 *
 * - **Byte write:** Writes a single byte to any address. Sequence:
 *   START → device_addr(W) → addr_msb → addr_lsb → data → STOP.
 *
 * - **Page write:** Writes 1–32 bytes starting at a 32-byte page-aligned address.
 *   The data must not cross a page boundary. If more than 32 bytes are sent, the
 *   lower 5 bits of the internal address counter wrap within the current page,
 *   overwriting previously written bytes. Sequence:
 *   START → device_addr(W) → addr_msb → addr_lsb → data[0..n] → STOP.
 *
 * After either write type, the EEPROM enters an internal self-timed write cycle
 * (tWR, 10 ms maximum). During this time the device does not acknowledge its I2C
 * address. @ref AT24C32_WriteByte and @ref AT24C32_WritePage both poll for
 * acknowledgement after writing (acknowledge polling) and return only once the
 * write cycle is confirmed complete, or when the polling timeout is exceeded.
 *
 * @section reads Read Operations
 *
 * - **Random read:** Reads one or more bytes starting from an arbitrary address.
 *   A dummy write (START → device_addr(W) → addr_msb → addr_lsb → STOP) sets the
 *   internal address counter, followed by a read transaction
 *   (START → device_addr(R) → data[0..n] → STOP). The internal counter
 *   auto-increments after each byte and wraps from the last byte of the array back
 *   to address 0x000.
 *
 * @section addressing Device Addressing
 *
 * The 7-bit I2C device address is 1010 A2 A1 A0, where A2, A1, A0 correspond to
 * the physical address pins on the chip. With all pins tied to GND the address is
 * 0x50. Up to 8 AT24C32 devices can share one bus by wiring the pins differently.
 *
 * @section wp Write Protect Pin
 *
 * The WP pin, when tied to VCC, protects the upper quadrant (2048 bytes,
 * addresses 0x800–0xFFF) from being written. When tied to GND or left unconnected
 * (internally pulled down), the full memory is writable. This driver does not
 * control the WP pin; its state is the caller's responsibility.
 *
 * @section i2c I2C Interface
 *
 * The AT24C32 operates at up to 400 kHz (fast mode) when powered from 2.7–5.5V,
 * or up to 100 kHz when powered from 1.8V or 2.5V. The bus pins, instance, and
 * speed are configurable via the macros below.
 */

#ifndef AT24C32_HAL_H
#define AT24C32_HAL_H

#include "pico/stdlib.h"
#include "I2C_HAL.h"

/** @defgroup AT24C32_I2C_Config I2C Configuration
 *  @brief Bus settings for the AT24C32. Modify these to match your hardware wiring.
 * @{
 */

/**
 * @brief 7-bit I2C device address of the AT24C32.
 *
 * Formed as 1010 A2 A1 A0. Evaluates to 0x50 when all address pins are tied to GND.
 * Modify the lower 3 bits to match your hardware if multiple devices share the bus.
 */
#define AT24C32_I2C_ADDR      0x57

/** @brief I2C peripheral instance to use (I2C_INSTANCE_0 or I2C_INSTANCE_1). */
#define AT24C32_I2C_INSTANCE  I2C_INSTANCE_0

/** @brief GPIO pin number for I2C SDA. */
#define AT24C32_I2C_SDA_PIN   4

/** @brief GPIO pin number for I2C SCL. */
#define AT24C32_I2C_SCL_PIN   5

/** @brief I2C clock speed in Hz. The AT24C32 supports up to 400kHz at 2.7–5.5V. */
#define AT24C32_I2C_SPEED_HZ  400000

/** @} */

/** @defgroup AT24C32_MemoryLayout Memory Layout
 *  @brief Constants describing the AT24C32 memory organisation.
 * @{
 */

/** @brief Total usable memory in bytes (4096 x 8 bits = 32 Kbits). */
#define AT24C32_MEM_SIZE      4096

/**
 * @brief Page size in bytes.
 *
 * The AT24C32 is internally organised as 128 pages of 32 bytes. A page write
 * must not cross a page boundary; if it does, the lower 5 bits of the internal
 * address counter wrap within the current page, overwriting earlier bytes.
 */
#define AT24C32_PAGE_SIZE     32

/**
 * @brief Maximum write cycle time in milliseconds (tWR).
 *
 * After a byte or page write, the EEPROM performs an internal clear/write cycle
 * for up to 10 ms. The device does not acknowledge its I2C address during this
 * period. @ref AT24C32_WriteByte and @ref AT24C32_WritePage poll for completion
 * using acknowledge polling rather than a fixed delay.
 */
#define AT24C32_WRITE_CYCLE_MS 10

/** @} */

/**
 * @brief Return codes for all AT24C32 driver functions.
 */
typedef enum
{
    AT24C32_OK = 0,           /**< Operation completed successfully. */
    AT24C32_ERROR_INIT,       /**< I2C peripheral initialisation failed. */
    AT24C32_ERROR_READ,       /**< I2C read transaction failed. */
    AT24C32_ERROR_WRITE,      /**< I2C write transaction failed. */
    AT24C32_ERROR_WRITE_TIMEOUT, /**< EEPROM did not complete the internal write cycle in time. */
    AT24C32_ERROR_INVALID_ADDR,  /**< Address out of range or write would cross a page boundary. */
    AT24C32_ERROR_NO_DEVICE   /**< AT24C32 did not acknowledge its I2C address. */
} AT24C32_Status;

/**
 * @brief Initialise the AT24C32 driver and verify the device is present on the bus.
 *
 * Initialises the I2C peripheral with the configuration defined by the
 * AT24C32_I2C_* macros and confirms that the EEPROM acknowledges its address.
 *
 * @note This function does not alter the EEPROM contents.
 *
 * @return AT24C32_OK on success, or the relevant AT24C32_ERROR_* code on failure.
 */
AT24C32_Status AT24C32_Init(void);

/**
 * @brief Write a single byte to the EEPROM.
 *
 * Sends a byte write sequence and then polls the EEPROM until the internal
 * write cycle (tWR ≤ 10 ms) completes before returning.
 *
 * @param[in] mem_addr  Word address to write to (0x000–0xFFF). Must be within
 *                      @ref AT24C32_MEM_SIZE.
 * @param[in] data      Byte value to write.
 *
 * @return AT24C32_OK on success, AT24C32_ERROR_INVALID_ADDR if the address is
 *         out of range, AT24C32_ERROR_WRITE on I2C failure, or
 *         AT24C32_ERROR_WRITE_TIMEOUT if the write cycle did not complete in time.
 */
AT24C32_Status AT24C32_WriteByte(uint16_t mem_addr, uint8_t data);

/**
 * @brief Read a single byte from the EEPROM.
 *
 * Performs a random read: sets the internal address counter via a dummy write,
 * then reads one byte.
 *
 * @param[in]  mem_addr  Word address to read from (0x000–0xFFF).
 * @param[out] data      Pointer to store the read byte. Must not be NULL.
 *
 * @return AT24C32_OK on success, AT24C32_ERROR_INVALID_ADDR if the address is
 *         out of range, or AT24C32_ERROR_READ on I2C failure.
 */
AT24C32_Status AT24C32_ReadByte(uint16_t mem_addr, uint8_t *data);

/**
 * @brief Write up to 32 bytes to the EEPROM in a single page write transaction.
 *
 * All bytes in the write must reside within the same 32-byte page. The starting
 * address and length are validated before any I2C transaction is started. After
 * writing, the function polls for acknowledgement until the internal write cycle
 * completes.
 *
 * @param[in] mem_addr  Starting word address (0x000–0xFFF). Must be within
 *                      @ref AT24C32_MEM_SIZE.
 * @param[in] data      Pointer to the data buffer to write. Must not be NULL.
 * @param[in] length    Number of bytes to write. Must satisfy:
 *                      - length > 0
 *                      - length ≤ @ref AT24C32_PAGE_SIZE
 *                      - (mem_addr % PAGE_SIZE) + length ≤ @ref AT24C32_PAGE_SIZE
 *                        (write must not cross a page boundary)
 *
 * @return AT24C32_OK on success, AT24C32_ERROR_INVALID_ADDR if any constraint
 *         is violated, AT24C32_ERROR_WRITE on I2C failure, or
 *         AT24C32_ERROR_WRITE_TIMEOUT if the write cycle did not complete in time.
 */
AT24C32_Status AT24C32_WritePage(uint16_t mem_addr, const uint8_t *data, size_t length);

/**
 * @brief Read one or more bytes sequentially from the EEPROM.
 *
 * Performs a random read followed by sequential reads. After the first byte
 * is read, the EEPROM automatically increments the internal address counter for
 * each subsequent byte. The counter wraps from the last address (0xFFF) back to
 * 0x000. The caller is responsible for ensuring that @p mem_addr + @p length
 * does not exceed @ref AT24C32_MEM_SIZE if wrap-around behaviour is undesired.
 *
 * @param[in]  mem_addr  Starting word address (0x000–0xFFF).
 * @param[out] data      Pointer to the buffer to store the read bytes. Must not be NULL.
 * @param[in]  length    Number of bytes to read. Must be > 0.
 *
 * @return AT24C32_OK on success, AT24C32_ERROR_INVALID_ADDR if the address is
 *         out of range, or AT24C32_ERROR_READ on I2C failure.
 */
AT24C32_Status AT24C32_ReadSequential(uint16_t mem_addr, uint8_t *data, size_t length);

#endif /* AT24C32_HAL_H */
