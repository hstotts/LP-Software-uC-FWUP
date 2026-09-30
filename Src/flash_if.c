/*
 * flash_if.c
 *
 *  Created on: Oct 9, 2025
 *      Author: haydenstotts
 */

#include "flash_if.h"
#include "memory_map.h"
#include "General_Functions.h"

typedef struct {
    uint32_t base;
    uint32_t size;
    uint32_t hal_id;
} flash_sector_desc_t;

#define KB(x) ((uint32_t)(x) * 1024u)
#define FLASH_SECTOR_COUNT 24

/* STM32F767 2 MiB dual-bank sector map. HAL sector identifiers are kept here
 * because memory_map.h intentionally has no HAL dependency. */
static const flash_sector_desc_t g_sectors[FLASH_SECTOR_COUNT] = {
    /* Bank 1 */
    {0x08000000u, KB(16), FLASH_SECTOR_0},
    {0x08004000u, KB(16), FLASH_SECTOR_1},
    {0x08008000u, KB(16), FLASH_SECTOR_2},
    {0x0800C000u, KB(16), FLASH_SECTOR_3},
    {0x08010000u, KB(64), FLASH_SECTOR_4},
    {0x08020000u, KB(128), FLASH_SECTOR_5},
    {0x08040000u, KB(128), FLASH_SECTOR_6},
    {0x08060000u, KB(128), FLASH_SECTOR_7},
    {0x08080000u, KB(128), FLASH_SECTOR_8},
    {0x080A0000u, KB(128), FLASH_SECTOR_9},
    {0x080C0000u, KB(128), FLASH_SECTOR_10},
    {0x080E0000u, KB(128), FLASH_SECTOR_11},

    /* Bank 2 */
    {0x08100000u, KB(16), FLASH_SECTOR_12},
    {0x08104000u, KB(16), FLASH_SECTOR_13},
    {0x08108000u, KB(16), FLASH_SECTOR_14},
    {0x0810C000u, KB(16), FLASH_SECTOR_15},
    {0x08110000u, KB(64), FLASH_SECTOR_16},
    {0x08120000u, KB(128), FLASH_SECTOR_17},
    {0x08140000u, KB(128), FLASH_SECTOR_18},
    {0x08160000u, KB(128), FLASH_SECTOR_19},
    {0x08180000u, KB(128), FLASH_SECTOR_20},
    {0x081A0000u, KB(128), FLASH_SECTOR_21},
    {0x081C0000u, KB(128), FLASH_SECTOR_22},
    {0x081E0000u, KB(128), FLASH_SECTOR_23},
};

static uint32_t sector_end(int sector)
{
    return g_sectors[sector].base + g_sectors[sector].size;
}

int FLASHIF_SectorIndexForAddress(uint32_t addr)
{
    for (int sector = 0; sector < FLASH_SECTOR_COUNT; sector++) {
        if (addr >= g_sectors[sector].base && addr < sector_end(sector)) {
            return sector;
        }
    }
    return -1;
}

uint32_t FLASHIF_SectorBase(int sector)
{
    return (sector >= 0 && sector < FLASH_SECTOR_COUNT)
           ? g_sectors[sector].base
           : 0u;
}

uint32_t FLASHIF_SectorSize(int sector)
{
    return (sector >= 0 && sector < FLASH_SECTOR_COUNT)
           ? g_sectors[sector].size
           : 0u;
}

static void find_sector_span(uint32_t base, uint32_t size,
                             int* first, int* last_inclusive)
{
    uint32_t end = base + size;
    int first_sector = FLASHIF_SectorIndexForAddress(base);

    if (first_sector < 0) {
        *first = -1;
        *last_inclusive = -1;
        return;
    }

    int last_sector = first_sector;
    while (last_sector < FLASH_SECTOR_COUNT &&
           sector_end(last_sector) < end) {
        last_sector++;
    }

    if (last_sector >= FLASH_SECTOR_COUNT) {
        *first = -1;
        *last_inclusive = -1;
        return;
    }

    *first = first_sector;
    *last_inclusive = last_sector;
}

static bool range_uses_only_ota_sectors(uint32_t base, uint32_t size,
                                        int* first_out, int* last_out)
{
    if (!flash_range_is_within_flash(base, size)) {
        return false;
    }

    int first = -1;
    int last = -1;
    find_sector_span(base, size, &first, &last);
    if (first < 0 || last < first) {
        return false;
    }

    for (int sector = first; sector <= last; sector++) {
        if (!((sector >= 5 && sector <= 11) ||
              (sector >= 17 && sector <= 23))) {
            return false;
        }
    }

    if (first_out != NULL) {
        *first_out = first;
    }
    if (last_out != NULL) {
        *last_out = last;
    }
    return true;
}

bool FLASHIF_IsBlank(uint32_t base, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        if (*(volatile const uint8_t*)(base + i) != 0xFFu) {
            return false;
        }
    }
    return true;
}

HAL_StatusTypeDef FLASHIF_EraseRange(uint32_t base, uint32_t size)
{
    if (size == 0u) {
        return HAL_OK;
    }

    int first = -1;
    int last = -1;
    if (!range_uses_only_ota_sectors(base, size, &first, &last)) {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    for (int sector = first; sector <= last; sector++) {
        FLASH_EraseInitTypeDef erase = {0};
        uint32_t sector_error = 0u;

        erase.TypeErase = FLASH_TYPEERASE_SECTORS;
        erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
        erase.Sector = g_sectors[sector].hal_id;
        erase.NbSectors = 1u;

        status = HAL_FLASHEx_Erase(&erase, &sector_error);
        if (status != HAL_OK) {
            break;
        }
        (void)BootHealth_RefreshIWDG();
    }

    (void)HAL_FLASH_Lock();
    return status;
}

FLASHIF_StatusTypedef FLASHIF_ProgramBuffer(uint32_t* dst,
                                            const uint8_t* src,
                                            uint32_t byte_count)
{
    if (dst == NULL || src == NULL ||
        !range_uses_only_ota_sectors((uint32_t)dst, byte_count, NULL, NULL)) {
        return FLASHIF_ERROR;
    }

    HAL_StatusTypeDef status = HAL_FLASH_Unlock();
    if (status != HAL_OK) {
        return status;
    }

    uint32_t dst_addr = (uint32_t)dst;
    const uint8_t* src_bytes = src;
    uint32_t bytes_remaining = byte_count;
    uint32_t bytes_since_refresh = 0u;

    while (bytes_remaining > 0u) {
        /* FLASH_TYPEPROGRAM_WORD (x32) requires only Vdd >= 2.7 V — no external
         * Vpp pin is needed. FLASH_TYPEPROGRAM_DOUBLEWORD (x64) requires the
         * external Vpp supply (STM32F767 RM0410, section 3.6). */
        uint32_t word = 0xFFFFFFFFu;
        uint32_t bytes_this_word = (bytes_remaining >= 4u)
                                   ? 4u
                                   : bytes_remaining;

        for (uint32_t i = 0u; i < bytes_this_word; i++) {
            ((uint8_t*)&word)[i] = src_bytes[i];
        }

        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                   dst_addr,
                                   (uint64_t)word);
        if (status != HAL_OK) {
            break;
        }

        dst_addr += 4u;
        src_bytes += bytes_this_word;
        bytes_remaining -= bytes_this_word;
        bytes_since_refresh += bytes_this_word;

        if (bytes_since_refresh >= 1024u) {
            (void)BootHealth_RefreshIWDG();
            bytes_since_refresh = 0u;
        }
    }

    (void)HAL_FLASH_Lock();
    return status;
}
