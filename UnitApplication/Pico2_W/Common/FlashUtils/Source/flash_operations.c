
#include <string.h>
#include "flash_operations.h"
#include "flash_layout.h"
#include "metadata.h"

bool read_metadata_from_flash(boot_metadata_t *ram_metadata)
{
    /* Point to the Flash Memory Location of the Boot Config sector which contains the metadata */
    const boot_metadata_t* flash_metadata = (boot_metadata_t*)BOOT_CONFIG_START;
    
    /* First check if we have valid metadata */
    if (flash_metadata->magic != BOOT_METADATA_MAGIC) 
    {
        /* No valid metadata found */
        return false;
    }
    
    /* Copy metadata to RAM */
    memcpy(ram_metadata, flash_metadata, sizeof(boot_metadata_t));

    return true;
}

bool write_metadata_to_flash(const boot_metadata_t *ram_metadata)
{
    uint32_t flash_offset = BOOT_CONFIG_START - FLASH_BASE; // Account for the XIP base address (0x10000000)
    uint32_t size_in_bytes = BOOT_CONFIG_SIZE;

    /*******************  Erase metadata sector *****************/

    /* Validate sizes for erase operation */
    if ((flash_offset & 0xFFF) != 0 || (size_in_bytes & 0xFFF) != 0) 
    {
        return false;  /* Must be aligned to 4096-byte sectors */
    }

    /* Disable interrupts during flash operations */
    uint32_t ints = save_and_disable_interrupts();

    /* flash_offset - Offset into flash, in bytes, to start the erase. Must be aligned to a 4096-byte flash sector.
       size_in_bytes - How many bytes to erase. Must be a multiple of 4096 bytes (one sector) */
    flash_range_erase(flash_offset, size_in_bytes);
    
    /*******************  Program metadata sector *****************/
    /* Calculate aligned size for programming - due to flash_range_program() requirement it must be a multiple of 256 bytes (one page) */
    uint32_t aligned_size = ((sizeof(boot_metadata_t) + 255u) / 256u) * 256u; // Per C11 §6.3.1.4p1: floating-to-integer conversion truncates toward zero

    /* Prepare aligned buffer */
    uint8_t aligned_buffer[aligned_size];
    memset(aligned_buffer, 0xFF, aligned_size);  // Fill with 0xFF (erased flash state)
    memcpy(aligned_buffer, ram_metadata, sizeof(boot_metadata_t));

    /* Validate offset alignment */
    if ((flash_offset & 0xFF) != 0) 
    {
        restore_interrupts(ints);
        return false;  /* Must be aligned to 256-byte pages */
    }

    /* flash_offset - Offset into flash, in bytes, to start to program. Must be aligned to a 256-byte flash page.
       data - Pointer to the data to be programmed.
       size_in_bytes - Must be a multiple of 256 bytes (one page) */
    flash_range_program(BOOT_CONFIG_START - FLASH_BASE, 
                       (const uint8_t*)aligned_buffer,
                       aligned_size); 

    /* Re-enable interrupts */
    restore_interrupts(ints);

    return true;
}

bool write_to_flash(uint32_t flash_offset, const uint8_t *data, size_t length) 
{
    /* Save and disable interrupts during flash operations.
     * This is important on the RP2350 because:
     * 1. Code executes directly from external flash via XIP (Execute In Place).
     *    During write or erase operations, XIP access is stalled, which can lead to faults or hangs.
     * 2. The QSPI flash interface is temporarily repurposed for programming,
     *    making code execution from flash unsafe during this time.
     * 3. On a dual-core system (Cortex-M33), both cores must avoid flash access
     *    during flash operations to prevent contention and potential corruption.
     */
    uint32_t ints = save_and_disable_interrupts();
    
    /* Align the erase range to 4KB sector boundaries.
     * The W25Q32 flash used with the RP2350 supports erasing only in fixed 4KB sectors.
     * This alignment ensures proper erase granularity.
     */
    uint32_t aligned_offset = flash_offset & ~(FLASH_SECTOR_SIZE - 1);  // Round down to sector boundary
    uint32_t end_offset = (flash_offset + length + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);  // Round up
    uint32_t sectors = (end_offset - aligned_offset) / FLASH_SECTOR_SIZE;
    
    /* Erase the required flash sectors.
     * NOR flash only allows changing bits from 1 to 0 (when programming).
     * Erasing sets all bits in a sector to 1 (0xFF), enabling them to be programmed again.
     * Erasing does NOT work on the level of individual bytes or bits - only whole sectors (due to several fundemental
     * physical and engineering constraints, such as: floating-gate transistor structure, high-voltage injection, electrical complexity etc.)
     * This means that if any bits in the sector need to be changed from 0 to 1, the entire sector must be erased first
     * and only then programmed to get the desired values in the flash.
     */
    flash_range_erase(aligned_offset, sectors * FLASH_SECTOR_SIZE);
    
    /* Program the data to flash.
     * The flash_range_program function handles 256-byte page programming internally.
     * Ensure the region being written was previously erased to avoid data corruption.
     */
    flash_range_program(flash_offset, data, length);
    
    /* Restore the previous interrupt state,
     * re-enabling interrupts if they were enabled before.
     */
    restore_interrupts(ints);
    
    return true; // All the operations are void functions, so no error checking is done, we always return true (TODO - maybe add readback verification?)
}

uint8_t check_active_bank(void)
{
    uint8_t active_bank;

    boot_metadata_t current_metadata;
    if (read_metadata_from_flash(&current_metadata)) 
    {
        if (current_metadata.active_bank == BANK_A)
        {
            active_bank = BANK_A;
        }
        else if(current_metadata.active_bank == BANK_B)
        {
            active_bank = BANK_B;
        }
        else
        {
            active_bank = INVALID_BANK; // Invalid active bank value in metadata, set to error value
        }
    }
    else
    {
        active_bank = INVALID_BANK; // Failed to read metadata from flash: invalid magic number or corrupted data, set to error value
    }

    /* If active_bank is set to 0xFF, it indicates an error state.
       Downstream code should handle this scenario appropriately. */
    return active_bank;
}

uint32_t check_current_fw_version(void)
{
    uint32_t current_version = 0;

    boot_metadata_t current_metadata;
    if (read_metadata_from_flash(&current_metadata)) 
    {
        current_version = current_metadata.version;
    }
    else
    {
        current_version = 0xFFFFFFFF; // Failed to read metadata from flash: invalid magic number or corrupted data, set to error value
    }

    return current_version;
}