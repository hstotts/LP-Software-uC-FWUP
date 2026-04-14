/*
 * fram_meta.c
 *
 *  Created on: Oct 9, 2025
 *      Author: haydenstotts
 */


#include "fram_meta.h"
#include "FRAM.h"
#include <string.h>
#include <stddef.h>
#include "stm32f7xx_hal.h"

/* ---------- FRAM A/B block ---------- */

#define FRAM_BLK_A_ADDR  ((uint16_t)(FRAM_META_BLK_A_ADDR))
#define FRAM_BLK_B_ADDR  ((uint16_t)(FRAM_META_BLK_B_ADDR))

/* ---------- Constants ---------- */
#define META_MAGIC 0x4D455441u  /* 'META' */
#define META_VER   1
#define META_WIP   0xFFu
#define META_OK    0xA5u

/* ---------- Internal working copy ---------- */
static fram_meta_block_t g_work;


static uint16_t block_crc(const fram_meta_block_t* b)
{
    // CRC over all fields preceding crc16 — one contiguous region, no chaining needed
    return Calc_CRC16((uint8_t*)b, offsetof(fram_meta_block_t, crc16));
}

static bool valid_blk(const fram_meta_block_t* b)
{
    if (b->magic   != META_MAGIC) return false;
    if (b->version != META_VER)   return false;
    if (b->commit  != META_OK)    return false;
    return block_crc(b) == b->crc16;
}

static bool read_blk(uint16_t addr, fram_meta_block_t* out)
{
    return readFRAM(addr, (uint8_t*)out, sizeof(*out)) == HAL_OK;
}

static bool write_blk(uint16_t addr, const fram_meta_block_t* in)
{
    return writeFRAM(addr, (uint8_t*)in, sizeof(*in)) == HAL_OK;
}

/* ---------- Public API ---------- */

bool FRAMMETA_Load(fram_meta_block_t* out, uint32_t* cur_addr)
{
    fram_meta_block_t A, B;
    bool va = false, vb = false;

    if (read_blk(FRAM_BLK_A_ADDR, &A)) va = valid_blk(&A);
    if (read_blk(FRAM_BLK_B_ADDR, &B)) vb = valid_blk(&B);

    if (va && vb) {
        // pick newest by seq
        if (B.seq > A.seq) { if (out) *out = B; if (cur_addr) *cur_addr = FRAM_BLK_B_ADDR; g_work = B; }
        else               { if (out) *out = A; if (cur_addr) *cur_addr = FRAM_BLK_A_ADDR; g_work = A; }
        return true;
    } else if (va) {
        if (out) *out = A; if (cur_addr) *cur_addr = FRAM_BLK_A_ADDR; g_work = A; return true;
    } else if (vb) {
        if (out) *out = B; if (cur_addr) *cur_addr = FRAM_BLK_B_ADDR; g_work = B; return true;
    }
    return false;
}

bool FRAMMETA_InitDefaults(uint8_t active_idx)
{
    memset(&g_work, 0xFF, sizeof(g_work));
    g_work.magic      = META_MAGIC;
    g_work.version    = META_VER;
    g_work.seq        = 1;
    g_work.active_idx = active_idx;
    g_work.commit     = META_WIP;   // set last

    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        uint8_t* raw = g_work.rec[i];
        memset(raw, 0, 20);

        // flash_addr, image_size, image_crc32 default to 0 (unknown/empty)
        raw[SLOT_OFF_BANK_ID]      = 0;
        raw[SLOT_OFF_IMAGE_INDEX]  = (uint8_t)(i + 1);
        raw[SLOT_OFF_BOOT_COUNTER] = 3;
        raw[SLOT_OFF_BOOT_FB]      = 1;   // BOOT_NEW_IMAGE placeholder
        raw[SLOT_OFF_NEW_META]     = (i == (uint8_t)(active_idx - 1)) ? 0 : 1;
        raw[SLOT_OFF_ERROR_CODE]   = 0;

        uint16_t rc = Calc_CRC16(&raw[2], SLOT_RECORD_DATA_LEN);
        raw[SLOT_OFF_CRC16_HI] = (uint8_t)((rc >> 8) & 0xFF);
        raw[SLOT_OFF_CRC16_LO] = (uint8_t)(rc & 0xFF);
    }

    g_work.crc16 = block_crc(&g_work);

    // Write to A, then set commit byte LAST
    if (!write_blk(FRAM_BLK_A_ADDR, &g_work)) return false;
    uint8_t ok = META_OK;
    if (writeFRAM(FRAM_BLK_A_ADDR + offsetof(fram_meta_block_t, commit), &ok, 1) != HAL_OK) return false;

    // Cache as current
    return true;
}

bool FRAMMETA_CommitNext(const fram_meta_block_t* next_in, uint32_t cur_addr)
{
	fram_meta_block_t tmp = next_in ? *next_in : g_work;  /* commit the in-RAM working copy */
    tmp.commit = META_WIP;                 // ensure not committed yet
    tmp.seq    = (uint16_t)(tmp.seq + 1);  // next generation
    tmp.crc16  = block_crc(&tmp);

    uint16_t dst = (cur_addr == FRAM_BLK_A_ADDR) ? FRAM_BLK_B_ADDR : FRAM_BLK_A_ADDR;

    if (!write_blk(dst, &tmp)) return false;

    // Write commit byte LAST (single-byte write → "atomic" for our purposes)
    uint8_t ok = META_OK;
    if (writeFRAM(dst + offsetof(fram_meta_block_t, commit), &ok, 1) != HAL_OK) return false;

    // Verify by re-reading
    fram_meta_block_t check;
    if (!read_blk(dst, &check)) return false;
    if (!valid_blk(&check))     return false;

    g_work = check;
    return true;
}

bool FRAMMETA_SetImageInfo(uint8_t img_id,
                           uint32_t flash_addr,
                           uint32_t image_size,
                           uint32_t image_crc,
                           uint8_t bank_id)
{
    uint32_t cur_addr = 0;
    fram_meta_block_t blk;

    // Load current metadata, or initialize defaults if none exist yet
    if (!FRAMMETA_Load(&blk, &cur_addr)) {
        if (!FRAMMETA_InitDefaults(1)) {
            return false;
        }
        if (!FRAMMETA_Load(&blk, &cur_addr)) {
            return false;
        }
    }

    // img_id is used as slot index in the current metadata model
    if (img_id < 1 || img_id > NUM_SLOTS) {
        return false;
    }

    // Mark slot as having a newly written image
    // Suggested mapping with current 7-byte scheme:
    // boot_feedback = 1
    // boot_counter  = 3
    // new_metadata  = 1
    // error_code    = 0
    FRAMMETA_SetSlot(img_id,
                     flash_addr,
                     image_size,
                     image_crc,
                     bank_id,
                     1,   // boot_feedback: new image pending
                     3,   // boot_counter
                     1,   // new_metadata/pending
                     0);  // error_code

    // Activate new image
    FRAMMETA_SetActiveIndex(img_id);

    return FRAMMETA_CommitNext(NULL, cur_addr);  // NULL causes CommitNext to use g_work

}

void FRAMMETA_SetActiveIndex(uint8_t idx)
{
    g_work.active_idx = idx;
}

void FRAMMETA_SetSlot(uint8_t slot_idx,
                      uint32_t base_addr, uint32_t image_size, uint32_t image_crc, uint8_t bank_id,
                      uint8_t boot_feedback, uint8_t boot_counter, uint8_t new_metadata, uint8_t error_code)
{
    if (slot_idx < 1 || slot_idx > NUM_SLOTS) return;
    uint8_t* raw = g_work.rec[slot_idx - 1];

    // [2..5]  flash_addr (LE)
    raw[SLOT_OFF_FLASH_ADDR + 0] = (uint8_t)(base_addr        & 0xFF);
    raw[SLOT_OFF_FLASH_ADDR + 1] = (uint8_t)((base_addr >> 8) & 0xFF);
    raw[SLOT_OFF_FLASH_ADDR + 2] = (uint8_t)((base_addr >>16) & 0xFF);
    raw[SLOT_OFF_FLASH_ADDR + 3] = (uint8_t)((base_addr >>24) & 0xFF);

    // [6..9]  image_size (LE)
    raw[SLOT_OFF_IMAGE_SIZE + 0] = (uint8_t)(image_size        & 0xFF);
    raw[SLOT_OFF_IMAGE_SIZE + 1] = (uint8_t)((image_size >> 8) & 0xFF);
    raw[SLOT_OFF_IMAGE_SIZE + 2] = (uint8_t)((image_size >>16) & 0xFF);
    raw[SLOT_OFF_IMAGE_SIZE + 3] = (uint8_t)((image_size >>24) & 0xFF);

    // [10..13] image_crc32 (LE)
    raw[SLOT_OFF_IMAGE_CRC32 + 0] = (uint8_t)(image_crc        & 0xFF);
    raw[SLOT_OFF_IMAGE_CRC32 + 1] = (uint8_t)((image_crc >> 8) & 0xFF);
    raw[SLOT_OFF_IMAGE_CRC32 + 2] = (uint8_t)((image_crc >>16) & 0xFF);
    raw[SLOT_OFF_IMAGE_CRC32 + 3] = (uint8_t)((image_crc >>24) & 0xFF);

    raw[SLOT_OFF_BANK_ID]      = bank_id;
    raw[SLOT_OFF_IMAGE_INDEX]  = (uint8_t)slot_idx;
    raw[SLOT_OFF_BOOT_COUNTER] = boot_counter;
    raw[SLOT_OFF_BOOT_FB]      = boot_feedback;
    raw[SLOT_OFF_NEW_META]     = new_metadata;
    raw[SLOT_OFF_ERROR_CODE]   = error_code;

    uint16_t rc = Calc_CRC16(&raw[2], SLOT_RECORD_DATA_LEN);  // 18 bytes
    raw[SLOT_OFF_CRC16_HI] = (uint8_t)((rc >> 8) & 0xFF);
    raw[SLOT_OFF_CRC16_LO] = (uint8_t)(rc & 0xFF);
}

uint8_t FRAMMETA_GetActiveIndex(void)
{
    return g_work.active_idx;
}

bool FRAMMETA_GetSlotRaw(uint8_t slot_idx, uint8_t out20[20])
{
    if (slot_idx < 1 || slot_idx > NUM_SLOTS) return false;
    if (out20) memcpy(out20, g_work.rec[slot_idx - 1], 20);
    return true;
}

void FRAMMETA_RecalcSlotCRC(uint8_t slot_idx)
{
    if (slot_idx < 1 || slot_idx > NUM_SLOTS) return;
    uint8_t* raw = g_work.rec[slot_idx-1];
    uint16_t rc = Calc_CRC16(&raw[2], SLOT_RECORD_DATA_LEN);
    raw[0] = (uint8_t)((rc >> 8) & 0xFF);
    raw[1] = (uint8_t)(rc & 0xFF);
}

void ConfirmBoot(void)
{
    uint32_t cur_addr = 0;

    // Load the current metadata — this also populates g_work
    if (!FRAMMETA_Load(NULL, &cur_addr)) return;

    uint8_t idx = g_work.active_idx;
    if (idx < 1 || idx > NUM_SLOTS) return;

    uint8_t* rec = g_work.rec[idx - 1];
    rec[SLOT_OFF_BOOT_FB]      = 1;   // BOOTED_OK
    rec[SLOT_OFF_BOOT_COUNTER] = 3;   // reset counter for next OTA cycle
    rec[SLOT_OFF_ERROR_CODE]   = 0;   // NO_BOOT_ERROR

    uint16_t rc = Calc_CRC16(&rec[2], SLOT_RECORD_DATA_LEN);
    rec[SLOT_OFF_CRC16_HI] = (uint8_t)((rc >> 8) & 0xFF);
    rec[SLOT_OFF_CRC16_LO] = (uint8_t)(rc & 0xFF);

    FRAMMETA_CommitNext(NULL, cur_addr);
}

