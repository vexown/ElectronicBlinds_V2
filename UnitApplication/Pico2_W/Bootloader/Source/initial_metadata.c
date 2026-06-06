#include "metadata.h"
#include "fw_version.h"

__attribute__((section(".boot_config")))
const boot_metadata_t initial_metadata =
{
    .magic = BOOT_METADATA_MAGIC,
    .active_bank = BANK_A,  // Default to bank A
    .version = FW_VERSION,  // Compiled-in version (see fw_version.h)
    .app_size = 0,
    .app_crc = 0,
    .update_pending = false,
    .boot_attempts = 0
};