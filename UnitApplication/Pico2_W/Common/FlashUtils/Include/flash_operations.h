#ifndef FLASH_OPERATIONS_H
#define FLASH_OPERATIONS_H

/*
 * Flash Memory Summary – W25Q32RVXHJQ (see https://datasheets.raspberrypi.com/picow/pico-2-w-schematic.pdf)
 *
 * The RP2350 board uses a Winbond W25Q32RVXHJQ external NOR flash memory.
 * - 32 Mbit (4 MB) serial NOR flash
 * - Connected via QSPI (Quad SPI) interface
 * - Supports Execute-In-Place (XIP) operation
 * - Sector size: 4 KB (smallest erasable unit)
 * - Page size: 256 bytes (write operations must fit within a page)
 * - Erase-before-write: required due to NOR flash limitations (can only change bits from 1 → 0)
 * - Default erase value: 0xFF
 *
 * NOTE: Flash operations must be carefully managed to avoid executing from flash
 * while it is being programmed or erased.
 */

/*********************** WARNING ***********************/
/* FLASH ADDRESSING QUICK REFERENCE
 * ------------------------------
 * SDK flash functions (flash_range_erase, flash_range_program) expect PHYSICAL addresses:
 * - Physical addresses start at 0x00000000
 * - Must convert from XIP/virtual addresses by subtracting 0x10000000
 * 
 * Example:
 * XIP address:    0x10040000 (where code runs from)
 * Physical addr:  0x00040000 (what to pass to flash functions)
 * 
 * Usage:
 * flash_range_erase(0x40000, FLASH_SECTOR_SIZE);    // ✓ Correct   (physical address)
 * flash_range_erase(0x10040000, FLASH_SECTOR_SIZE); // ✗ Incorrect (XIP address)
 */
/******************************************************/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "metadata.h"

/**
 * @brief Reads bootloader metadata from flash memory into RAM
 *
 * This function reads the boot metadata structure from the flash memory's boot config sector
 * into a RAM copy. It validates the metadata by checking for a magic number.
 *
 * @param ram_metadata Pointer to a boot_metadata_t structure in RAM (to store the read metadata)
 * 
 * @return true if valid metadata was read successfully
 * @return false if metadata was invalid (magic number mismatch)
 */
bool read_metadata_from_flash(boot_metadata_t *ram_metadata);

/**
 * @brief Writes the current RAM metadata to flash memory
 * 
 * This function writes the boot metadata from RAM to the flash memory's boot config sector.
 * It performs the following steps:
 * 1. Erases the metadata sector (must be 4096-byte aligned)
 * 2. Programs the new metadata (must be 256-byte aligned)
 * 
 * @param ram_metadata Pointer to a boot_metadata_t structure in RAM (to write to flash)
 *
 * @return true if metadata was written successfully
 * @return false if alignment requirements were not met
 */
bool write_metadata_to_flash(const boot_metadata_t *ram_metadata);

/**
 * Write data to flash memory
 * 
 * This function writes data to the RP2350's on-board (external) flash memory.
 * Because the flash is mapped into the address space via an XIP (Execute-In-Place) interface,
 * special precautions are needed to safely perform write and erase operations.
 * 
 * @param flash_offset Flash address offset where data will be written
 * @param data Pointer to the data buffer that will be written to flash
 * @param length Number of bytes to write
 * @return true if successful, false otherwise
 */
bool write_to_flash(uint32_t flash_offset, const uint8_t *data, size_t length);

/* 
 * Function: check_active_bank
 * 
 * Description: Check which bank the application is running from based on the metadata from flash
 * 
 * Parameters:
 *   - none
 * 
 * Returns: uint8_t
 *   - BANK_A (0) if running from Bank A
 *   - BANK_B (1) if running from Bank B
 *   - 0xFF if invalid bank (error)
 */
uint8_t check_active_bank(void);

/**
 * Function: check_current_fw_version
 * 
 * Description: Check the current firmware version from the metadata in flash
 * 
 * Parameters:
 *   - none
 * 
 * Returns: uint32_t
 *   - Current firmware version (currently just incremented on each update. TODO - come up with a better versioning scheme)
 */
uint32_t check_current_fw_version(void);

bool validate_app_image(uint32_t addr);

/**
 * @brief Read the persisted blinds travel-time calibration.
 *
 * Reads boot metadata and, if the blinds calibration magic is valid, returns
 * the stored UP and DOWN travel times. Either output pointer may be NULL if
 * only one direction is needed.
 *
 * @param t_up_ms   Out: full travel time UP in milliseconds.
 * @param t_down_ms Out: full travel time DOWN in milliseconds.
 *
 * @return true if a valid calibration was loaded, false if no calibration
 *         exists (first boot, post-upgrade, or corrupted metadata).
 */
bool read_blinds_calibration(uint32_t *t_up_ms, uint32_t *t_down_ms);

/**
 * @brief Persist the blinds travel-time calibration.
 *
 * Reads existing boot metadata, updates the calibration fields, and writes
 * it back. Other metadata (active bank, version, etc.) is preserved.
 *
 * @param t_up_ms   Full travel time UP in milliseconds.
 * @param t_down_ms Full travel time DOWN in milliseconds.
 *
 * @return true on success, false if the underlying metadata read or write
 *         failed.
 */
bool write_blinds_calibration(uint32_t t_up_ms, uint32_t t_down_ms);

#endif // FLASH_OPERATIONS_H