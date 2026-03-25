#ifndef FLASH_LAYOUT_H
#define FLASH_LAYOUT_H

/*
 * Flash Layout for RP2350 4MB Flash (XIP addressing) in OTA use case:
 * 
 * TODO - design the layout
 */

/* TODO - values below are not fully correct, just a placeholder to give the idea for OTA, correct it before removing this TODO */
/* Flash Configuration */
#define FLASH_BASE                  0x10000000   // XIP mapped base address
#define FLASH_TOTAL_SIZE            0x400000     // 4MB

/* Region Definitions (XIP addresses) */
#define BOOTLOADER_START            (FLASH_BASE)    
#define BOOTLOADER_SIZE             0x3F000      // 252KB

#define BOOT_CONFIG_START           (BOOTLOADER_START + BOOTLOADER_SIZE)
#define BOOT_CONFIG_SIZE            0x1000       // 4KB

#define APP_BANK_A_START            (FLASH_BASE + BOOTLOADER_SIZE + BOOT_CONFIG_SIZE)   // 0x10040000
#define APP_BANK_A_OFFSET           (APP_BANK_A_START - FLASH_BASE)                     // 0x00040000
#define APP_BANK_SIZE               0x1E0000                                            // 1920KB (so 2 banks take up 3840KB)
#define APP_BANK_B_START            (APP_BANK_A_START + APP_BANK_SIZE)                  // 0x10220000
#define APP_BANK_B_OFFSET           (APP_BANK_B_START - FLASH_BASE)                     // 0x00220000

#endif // FLASH_LAYOUT_H