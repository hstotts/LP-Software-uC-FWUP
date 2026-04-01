/*
 * flash_if.c
 *
 *  Created on: Oct 9, 2025
 *      Author: haydenstotts
 */

#include "flash_if.h"
#include "memory_map.h"
extern IWDG_HandleTypeDef hiwdg;

// -------------------- Sector map: STM32F767, Dual-Bank, 2MB --------------------
// Bank1: 0x0800_0000 .. 0x080F_FFFF (S0..S11)
//  S0..S3:  16KB, S4:  64KB, S5..S11: 128KB
// Bank2: 0x0810_0000 .. 0x081F_FFFF (S12..S23)
//  S12..S15: 16KB, S16: 64KB, S17..S23: 128KB

typedef struct { uint32_t base, size; uint32_t hal_id; } sector_t;

#define KB(x) ((uint32_t)(x) * 1024u)

// --------------------- REMOVE AND REPLACE WITH MEMORY MAP.h ----------------------
static const sector_t g_sectors[24] = {
    // Bank 1
    {0x08000000u, KB(16), FLASH_SECTOR_0},
    {0x08004000u, KB(16), FLASH_SECTOR_1},
    {0x08008000u, KB(16), FLASH_SECTOR_2},
    {0x0800C000u, KB(16), FLASH_SECTOR_3},
    {0x08010000u, KB(64), FLASH_SECTOR_4},
    {0x08020000u, KB(128),FLASH_SECTOR_5},
    {0x08040000u, KB(128),FLASH_SECTOR_6},
    {0x08060000u, KB(128),FLASH_SECTOR_7},
    {0x08080000u, KB(128),FLASH_SECTOR_8},
    {0x080A0000u, KB(128),FLASH_SECTOR_9},
    {0x080C0000u, KB(128),FLASH_SECTOR_10},
    {0x080E0000u, KB(128),FLASH_SECTOR_11},

    // Bank 2
    {0x08100000u, KB(16), FLASH_SECTOR_12},
    {0x08104000u, KB(16), FLASH_SECTOR_13},
    {0x08108000u, KB(16), FLASH_SECTOR_14},
    {0x0810C000u, KB(16), FLASH_SECTOR_15},
    {0x08110000u, KB(64), FLASH_SECTOR_16},
    {0x08120000u, KB(128),FLASH_SECTOR_17},
    {0x08140000u, KB(128),FLASH_SECTOR_18},
    {0x08160000u, KB(128),FLASH_SECTOR_19},
    {0x08180000u, KB(128),FLASH_SECTOR_20},
    {0x081A0000u, KB(128),FLASH_SECTOR_21},
    {0x081C0000u, KB(128),FLASH_SECTOR_22},
    {0x081E0000u, KB(128),FLASH_SECTOR_23},
};

// --------------------------------------------------------------------------------------

static inline uint32_t sector_end(int i){ return g_sectors[i].base + g_sectors[i].size; }

int FLASHIF_SectorIndexForAddress(uint32_t addr)
{
    for (int i=0;i<24;i++){
        if (addr >= g_sectors[i].base && addr < sector_end(i)) return i;
    }
    return -1;
}

uint32_t FLASHIF_SectorBase(int sector)
{
    return (sector>=0 && sector<24) ? g_sectors[sector].base : 0;
}

uint32_t FLASHIF_SectorSize(int sector)
{
    return (sector>=0 && sector<24) ? g_sectors[sector].size : 0;
}

// Round an arbitrary [base,size) to whole sectors
static void span_align_to_sectors(uint32_t base, uint32_t size, int* first, int* last_inclusive)
{
    uint32_t end = base + size;
    int s_first = FLASHIF_SectorIndexForAddress(base);
    if (s_first < 0) { *first = -1; *last_inclusive = -1; return; }

    // If base is exactly at a sector boundary, use that; otherwise we still start at s_first
    int s = s_first;
    while (s < 24 && sector_end(s) < end) s++;
    if (s >= 24) { *first = -1; *last_inclusive = -1; return; }
    *first = s_first;
    *last_inclusive = s;
}

bool FLASHIF_IsBlank(uint32_t base, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++) {
        if (*(volatile const uint8_t *)(base + i) != 0xFFu) {
            return false;
        }
    }
    return true;
}

// -------------------- Public erase/program --------------------

HAL_StatusTypeDef FLASHIF_EraseRange(uint32_t base, uint32_t size)
{
    if (size == 0) return HAL_OK;

    int first=-1, last=-1;
    span_align_to_sectors(base, size, &first, &last);
    if (first < 0 || last < first) return HAL_ERROR;

    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) return st;

    for (int s = first; s <= last; s++){
        FLASH_EraseInitTypeDef ei = {0};
        uint32_t sector_err = 0;
        ei.TypeErase    = FLASH_TYPEERASE_SECTORS;
        ei.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7–3.6V
        ei.Sector       = g_sectors[s].hal_id;
        ei.NbSectors    = 1;
        st = HAL_FLASHEx_Erase(&ei, &sector_err);
        if (st != HAL_OK) break;
        HAL_IWDG_Refresh(&hiwdg);
    }

    HAL_FLASH_Lock();
    return st;
}

FLASHIF_StatusTypedef FLASHIF_ProgramBuffer(uint32_t *dst, uint32_t *src, uint32_t word_count)
{
    HAL_StatusTypeDef st = HAL_FLASH_Unlock();
    if (st != HAL_OK) return st;

    uint32_t dst_addr = (uint32_t)dst;
    const uint8_t *src8 = (const uint8_t *)src;
    uint32_t len = word_count * 4u;

    uint32_t acc = 0;
    while (len) {
        uint64_t q = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t n = (len >= 8u) ? 8u : len;

        for (uint32_t i = 0; i < n; i++) {
            ((uint8_t*)&q)[i] = src8[i];
        }

        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dst_addr, q);
        if (st != HAL_OK) break;

        if (*(volatile uint64_t*)dst_addr != q) {
            st = HAL_ERROR;
            break;
        }

        dst_addr += 8u;
        src8 += n;
        len -= n;
        acc += n;

        if (acc >= 1024u) {
            HAL_IWDG_Refresh(&hiwdg);
            acc = 0;
        }
    }

    HAL_FLASH_Lock();
    return st;
}

