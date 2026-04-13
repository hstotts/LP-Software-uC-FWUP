#ifndef MEMORY_MAP_H
#define MEMORY_MAP_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================

// ******************** FLASH AND SRAM MEMORY MAP ********************

// ============================================================================
// STM32F7 Dual-bank flash layout 
// Bank1: 0x0800_0000 .. 0x080F_FFFF  (Sectors 0..11)
// Bank2: 0x0810_0000 .. 0x081F_FFFF  (Sectors 12..23)
// Sector sizes:
//   0-3:  16KB
//   4:    64KB
//   5-11: 128KB
//   12-15:16KB
//   16:   64KB
//   17-23:128KB
// ============================================================================

#define FLASH_BANK1_BASE   (0x08000000u)
#define FLASH_BANK2_BASE   (0x08100000u)

#define FLASH_BANK1_END    (0x080FFFFFu)
#define FLASH_BANK2_END    (0x081FFFFFu)

// ---- Sector sizes ----
#define FLASH_SECTOR_SIZE_16KB   (0x00004000u)  //  16 * 1024
#define FLASH_SECTOR_SIZE_64KB   (0x00010000u)  //  64 * 1024
#define FLASH_SECTOR_SIZE_128KB  (0x00020000u)  // 128 * 1024

// ---- Sector numbers (keep names) ----
typedef enum
{
    SECTOR_0  = 0,
    SECTOR_1  = 1,
    SECTOR_2  = 2,
    SECTOR_3  = 3,
    SECTOR_4  = 4,
    SECTOR_5  = 5,
    SECTOR_6  = 6,
    SECTOR_7  = 7,
    SECTOR_8  = 8,
    SECTOR_9  = 9,
    SECTOR_10 = 10,
    SECTOR_11 = 11,

    SECTOR_12 = 12,
    SECTOR_13 = 13,
    SECTOR_14 = 14,
    SECTOR_15 = 15,
    SECTOR_16 = 16,
    SECTOR_17 = 17,
    SECTOR_18 = 18,
    SECTOR_19 = 19,
    SECTOR_20 = 20,
    SECTOR_21 = 21,
    SECTOR_22 = 22,
    SECTOR_23 = 23,

    SECTOR_INVALID = 0xFF
} flash_sector_t;

// ---- Sector start addresses ----
// Bank 1
#define SECTOR_0_START    (0x08000000u)
#define SECTOR_1_START    (0x08004000u)
#define SECTOR_2_START    (0x08008000u)
#define SECTOR_3_START    (0x0800C000u)
#define SECTOR_4_START    (0x08010000u)
#define SECTOR_5_START    (0x08020000u)
#define SECTOR_6_START    (0x08040000u)
#define SECTOR_7_START    (0x08060000u)
#define SECTOR_8_START    (0x08080000u)
#define SECTOR_9_START    (0x080A0000u)
#define SECTOR_10_START   (0x080C0000u)
#define SECTOR_11_START   (0x080E0000u)

// Bank 2
#define SECTOR_12_START   (0x08100000u)
#define SECTOR_13_START   (0x08104000u)
#define SECTOR_14_START   (0x08108000u)
#define SECTOR_15_START   (0x0810C000u)
#define SECTOR_16_START   (0x08110000u)
#define SECTOR_17_START   (0x08120000u)
#define SECTOR_18_START   (0x08140000u)
#define SECTOR_19_START   (0x08160000u)
#define SECTOR_20_START   (0x08180000u)
#define SECTOR_21_START   (0x081A0000u)
#define SECTOR_22_START   (0x081C0000u)
#define SECTOR_23_START   (0x081E0000u)

// ---- Sector sizes by number ----
static inline uint32_t flash_sector_size(flash_sector_t s)
{
    switch (s)
    {
        case SECTOR_0:
        case SECTOR_1:
        case SECTOR_2:
        case SECTOR_3:
        case SECTOR_12:
        case SECTOR_13:
        case SECTOR_14:
        case SECTOR_15:
            return FLASH_SECTOR_SIZE_16KB;

        case SECTOR_4:
        case SECTOR_16:
            return FLASH_SECTOR_SIZE_64KB;

        case SECTOR_5:
        case SECTOR_6:
        case SECTOR_7:
        case SECTOR_8:
        case SECTOR_9:
        case SECTOR_10:
        case SECTOR_11:
        case SECTOR_17:
        case SECTOR_18:
        case SECTOR_19:
        case SECTOR_20:
        case SECTOR_21:
        case SECTOR_22:
        case SECTOR_23:
            return FLASH_SECTOR_SIZE_128KB;

        default:
            return 0u;
    }
}

// ---- Sector start address by number ----
static inline uint32_t flash_sector_start(flash_sector_t s)
{
    switch (s)
    {
        case SECTOR_0:  return SECTOR_0_START;
        case SECTOR_1:  return SECTOR_1_START;
        case SECTOR_2:  return SECTOR_2_START;
        case SECTOR_3:  return SECTOR_3_START;
        case SECTOR_4:  return SECTOR_4_START;
        case SECTOR_5:  return SECTOR_5_START;
        case SECTOR_6:  return SECTOR_6_START;
        case SECTOR_7:  return SECTOR_7_START;
        case SECTOR_8:  return SECTOR_8_START;
        case SECTOR_9:  return SECTOR_9_START;
        case SECTOR_10: return SECTOR_10_START;
        case SECTOR_11: return SECTOR_11_START;

        case SECTOR_12: return SECTOR_12_START;
        case SECTOR_13: return SECTOR_13_START;
        case SECTOR_14: return SECTOR_14_START;
        case SECTOR_15: return SECTOR_15_START;
        case SECTOR_16: return SECTOR_16_START;
        case SECTOR_17: return SECTOR_17_START;
        case SECTOR_18: return SECTOR_18_START;
        case SECTOR_19: return SECTOR_19_START;
        case SECTOR_20: return SECTOR_20_START;
        case SECTOR_21: return SECTOR_21_START;
        case SECTOR_22: return SECTOR_22_START;
        case SECTOR_23: return SECTOR_23_START;

        default:        return 0u;
    }
}

// ---- Sector end address (inclusive) ----
static inline uint32_t flash_sector_end(flash_sector_t s)
{
    uint32_t start = flash_sector_start(s);
    uint32_t size  = flash_sector_size(s);
    if (start == 0u || size == 0u) return 0u;
    return (start + size - 1u);
}

// ---- Find sector containing an address ----
static inline flash_sector_t flash_sector_from_addr(uint32_t addr)
{
    // Bank 1
    if (addr >= SECTOR_0_START && addr <= flash_sector_end(SECTOR_11)) {
        if (addr < SECTOR_4_START) {
            // 16KB sectors 0..3
            uint32_t off = addr - SECTOR_0_START;
            return (flash_sector_t)(off / FLASH_SECTOR_SIZE_16KB);
        }
        if (addr < SECTOR_5_START) return SECTOR_4;
        // 128KB sectors 5..11
        if (addr >= SECTOR_5_START && addr <= flash_sector_end(SECTOR_11)) {
            uint32_t off = addr - SECTOR_5_START;
            return (flash_sector_t)(SECTOR_5 + (off / FLASH_SECTOR_SIZE_128KB));
        }
    }

    // Bank 2
    if (addr >= SECTOR_12_START && addr <= flash_sector_end(SECTOR_23)) {
        if (addr < SECTOR_16_START) {
            // 16KB sectors 12..15
            uint32_t off = addr - SECTOR_12_START;
            return (flash_sector_t)(SECTOR_12 + (off / FLASH_SECTOR_SIZE_16KB));
        }
        if (addr < SECTOR_17_START) return SECTOR_16;
        // 128KB sectors 17..23
        if (addr >= SECTOR_17_START && addr <= flash_sector_end(SECTOR_23)) {
            uint32_t off = addr - SECTOR_17_START;
            return (flash_sector_t)(SECTOR_17 + (off / FLASH_SECTOR_SIZE_128KB));
        }
    }

    return SECTOR_INVALID;
}

// ---- Simple range check ----
static inline int flash_range_is_within_flash(uint32_t base, uint32_t size)
{
    if (size == 0u) return 0;
    uint32_t end = base + size - 1u;
    // avoid overflow
    if (end < base) return 0;

    // must be fully within one of the banks
    if ((base >= FLASH_BANK1_BASE && end <= FLASH_BANK1_END) ||
        (base >= FLASH_BANK2_BASE && end <= FLASH_BANK2_END)) {
        return 1;
    }
    return 0;
}


// ============================================================================
// SRAM layout – STM32F767VIT
// DTCM  128KB @ 0x20000000  – application stack/heap only
// SRAM1 368KB @ 0x20020000  – general use; top 44KB reserved below
// SRAM2  16KB @ 0x2007C000  – application use
// ============================================================================

#define SRAM1_BASE             0x20020000u
#define SRAM1_SIZE             0x0005C000u   // 368KB

// Firmware staging buffer – holds one incoming OTA image
#define SRAM_FW_STAGING_BASE   0x20071000u
#define SRAM_FW_STAGING_SIZE   0x0000A000u   // 40KB

// Sweep table working buffer – 8 tables × 256 steps × 2 bytes
#define SRAM_SWEEP_BUF_BASE    0x2007B000u
#define SRAM_SWEEP_BUF_SIZE    0x00001000u   // 4KB

// Sanity: staging + sweep = 44KB, fits within top of SRAM1
// 0x20071000 + 0xA000 + 0x1000 = 0x2007C000 == SRAM1 top


#ifdef __cplusplus
}
#endif

#endif // MEMORY_MAP_H
