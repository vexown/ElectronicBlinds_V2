#include "metadata.h"

__attribute__((section(".boot_config"))) 
const boot_metadata_t initial_metadata = 
{
    .magic = BOOT_METADATA_MAGIC, 
    .active_bank = BANK_A,  // Default to bank A
    .version = 0x010000,    // v1.0.0 (major: 0x01, minor: 0x00, patch: 0x00)
    .app_size = 0,
    .app_crc = 0,
    .update_pending = false,
    .boot_attempts = 0
};