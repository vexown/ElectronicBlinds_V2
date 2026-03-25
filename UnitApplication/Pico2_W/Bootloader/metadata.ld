/* Simplified linker script for the initial metadata binary */

MEMORY
{
    /* Configuration sector (4KB) */
    BOOT_CONFIG_FLASH(rx)      : ORIGIN = 0x1003F000, LENGTH = 4K
}

SECTIONS
{
    .boot_config : {
        . = ALIGN(4);
        *(.boot_config*)   /* Match any section names starting with .boot_config */
        . = ALIGN(4);
    } > BOOT_CONFIG_FLASH
}

    
