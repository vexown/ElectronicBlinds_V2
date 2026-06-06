#ifndef FW_VERSION_H
#define FW_VERSION_H

#include <stdint.h>

/* Firmware version — the single source of truth.
 *
 * Bump these three numbers by hand for each release (semantic versioning:
 * MAJOR.MINOR.PATCH). The value is baked into the image at build time */
#define FW_VERSION_MAJOR  1
#define FW_VERSION_MINOR  2
#define FW_VERSION_PATCH  0

/* Packed 0x00MMmmpp encoding stored in boot metadata. */
#define FW_VERSION_ENCODE(maj, min, pat) \
    (((uint32_t)(maj) << 16) | ((uint32_t)(min) << 8) | ((uint32_t)(pat) & 0xFFu))

#define FW_VERSION  FW_VERSION_ENCODE(FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH)

/* Decode helpers for a packed version word (e.g. one read back from metadata). */
#define FW_VERSION_GET_MAJOR(v)  (((uint32_t)(v) >> 16) & 0xFFu)
#define FW_VERSION_GET_MINOR(v)  (((uint32_t)(v) >> 8)  & 0xFFu)
#define FW_VERSION_GET_PATCH(v)  ((uint32_t)(v) & 0xFFu)

/* "1.0.0" string literal for logging / reporting. */
#define FW_VERSION_STR_HELPER(maj, min, pat)  #maj "." #min "." #pat
#define FW_VERSION_STR_EXPAND(maj, min, pat)  FW_VERSION_STR_HELPER(maj, min, pat)
#define FW_VERSION_STR \
    FW_VERSION_STR_EXPAND(FW_VERSION_MAJOR, FW_VERSION_MINOR, FW_VERSION_PATCH)

#endif // FW_VERSION_H
