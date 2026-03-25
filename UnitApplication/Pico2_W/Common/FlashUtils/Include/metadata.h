#ifndef METADATA_H
#define METADATA_H

#include <stdint.h>
#include <stdbool.h>

/* Magic number for metadata validation */
#define BOOT_METADATA_MAGIC 0xB007B007

/* Bank definitions */
#define BANK_A 0
#define BANK_B 1
#define INVALID_BANK 0xFF

/* Metadata structure */
typedef struct 
{
    uint32_t magic;           // Magic number for validation
    uint8_t active_bank;      // 0 = Bank A, 1 = Bank B
    uint32_t version;         // Firmware version
    uint32_t app_size;        // Size of application
    uint32_t app_crc;         // CRC of application
    bool update_pending;      // Update requested flag
    uint8_t boot_attempts;    // Number of boot attempts
} boot_metadata_t;

#endif // METADATA_H