#ifndef METADATA_H
#define METADATA_H

#include <stdint.h>
#include <stdbool.h>

/* Magic number for metadata validation */
#define BOOT_METADATA_MAGIC 0xB007B007

/* Magic distinguishing valid blinds calibration from erased flash (0xFFFFFFFF)
 * or older firmware that didn't write these fields. */
#define BLINDS_CALIB_MAGIC  0xB11DCA11

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

    /* Blinds travel-time calibration (ms). Piggybacked on the boot config
     * sector to avoid carving a new flash region. blinds_calib_magic reads as
     * 0xFFFFFFFF on uninitialised flash or after a struct-layout upgrade —
     * either way, the blinds module treats the calibration as absent. */
    uint32_t blinds_calib_magic;
    uint32_t blinds_t_up_ms;
    uint32_t blinds_t_down_ms;
} boot_metadata_t;

#endif // METADATA_H