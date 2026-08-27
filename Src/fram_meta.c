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
#include "General_Functions.h"

/* ---------- FRAM A/B block ---------- */

#define FRAM_BLK_A_ADDR  ((uint16_t)(FRAM_META_BLK_A_ADDR))
#define FRAM_BLK_B_ADDR  ((uint16_t)(FRAM_META_BLK_B_ADDR))

/* ---------- Constants ---------- */
#define META_MAGIC 0x4D455441u  /* 'META' */
#define META_VER   1
#define META_WIP   0xFFu
#define META_OK    0xA5u
#define FRAMMETA_ATTEMPTS 2u

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

static void recover_fram_bus(void)
{
    (void)HAL_I2C_DeInit(&hi2c4);
    HAL_Delay(2u);
    (void)HAL_I2C_Init(&hi2c4);
}

static bool read_blk(uint16_t addr, fram_meta_block_t* out)
{
    for (uint32_t attempt = 0u; attempt < FRAMMETA_ATTEMPTS; attempt++) {
        (void)BootHealth_RefreshIWDG();
        if (readFRAM(addr, (uint8_t*)out, sizeof(*out)) == HAL_OK) {
            return true;
        }
        if (attempt + 1u < FRAMMETA_ATTEMPTS) {
            recover_fram_bus();
        }
    }
    return false;
}

static bool commit_copy(uint16_t addr, const fram_meta_block_t* in)
{
    for (uint32_t attempt = 0u; attempt < FRAMMETA_ATTEMPTS; attempt++) {
        fram_meta_block_t tmp = *in;
        tmp.commit = META_WIP;

        (void)BootHealth_RefreshIWDG();

        bool ok = writeFRAM(addr, (uint8_t*)&tmp, sizeof(tmp)) == HAL_OK;
        uint8_t commit = META_OK;
        if (ok) {
            (void)BootHealth_RefreshIWDG();
            ok = writeFRAM(
                (uint16_t)(addr + offsetof(fram_meta_block_t, commit)),
                &commit, 1u) == HAL_OK;
        }

        fram_meta_block_t check;
        if (ok) {
            (void)BootHealth_RefreshIWDG();
            ok = readFRAM(addr, (uint8_t*)&check, sizeof(check)) == HAL_OK &&
                 valid_blk(&check);
        }

        if (ok) {
            g_work = check;
            return true;
        }
        if (attempt + 1u < FRAMMETA_ATTEMPTS) {
            recover_fram_bus();
        }
    }
    return false;
}

/* ---------- Public API ---------- */

bool FRAMMETA_ReadSnapshot(fram_meta_snapshot_t* out)
{
    if (out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->selected_copy = FRAMMETA_SELECTED_NONE;

    bool read_a_ok = read_blk(FRAM_BLK_A_ADDR, &out->copy_a);
    bool read_b_ok = read_blk(FRAM_BLK_B_ADDR, &out->copy_b);

    out->copy_a_status = !read_a_ok
                       ? FRAMMETA_COPY_IO_ERROR
                       : (valid_blk(&out->copy_a)
                          ? FRAMMETA_COPY_VALID
                          : FRAMMETA_COPY_INVALID);
    out->copy_b_status = !read_b_ok
                       ? FRAMMETA_COPY_IO_ERROR
                       : (valid_blk(&out->copy_b)
                          ? FRAMMETA_COPY_VALID
                          : FRAMMETA_COPY_INVALID);

    bool valid_a = out->copy_a_status == FRAMMETA_COPY_VALID;
    bool valid_b = out->copy_b_status == FRAMMETA_COPY_VALID;

    if (valid_a && valid_b) {
        out->selected_copy =
            ((int16_t)(out->copy_b.seq - out->copy_a.seq) > 0)
            ? FRAMMETA_SELECTED_B
            : FRAMMETA_SELECTED_A;
    } else if (valid_a) {
        out->selected_copy = FRAMMETA_SELECTED_A;
    } else if (valid_b) {
        out->selected_copy = FRAMMETA_SELECTED_B;
    }

    return out->selected_copy != FRAMMETA_SELECTED_NONE;
}

bool FRAMMETA_Load(fram_meta_block_t* out, uint32_t* cur_addr)
{
    fram_meta_block_t A, B;
    bool va = false, vb = false;

    if (read_blk(FRAM_BLK_A_ADDR, &A)) va = valid_blk(&A);
    if (read_blk(FRAM_BLK_B_ADDR, &B)) vb = valid_blk(&B);

    if (va && vb) {
        // pick newest by seq
        if ((int16_t)(B.seq - A.seq) > 0) { if (out) *out = B; if (cur_addr) *cur_addr = FRAM_BLK_B_ADDR; g_work = B; }
        else               { if (out) *out = A; if (cur_addr) *cur_addr = FRAM_BLK_A_ADDR; g_work = A; }
        return true;
    } else if (va) {
        if (out) *out = A;
        if (cur_addr) *cur_addr = FRAM_BLK_A_ADDR;
        g_work = A;
        return true;
    } else if (vb) {
        if (out) *out = B;
        if (cur_addr) *cur_addr = FRAM_BLK_B_ADDR;
        g_work = B;
        return true;
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

    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        uint8_t* raw = g_work.rec[i];
        memset(raw, 0, 20);

        // flash_addr, image_size, image_crc32 default to 0 (unknown/empty)
        raw[SLOT_OFF_BANK_ID]      = 0;
        raw[SLOT_OFF_IMAGE_INDEX]  = (uint8_t)(i + 1);
        raw[SLOT_OFF_BOOT_COUNTER] = 3;
        raw[SLOT_OFF_BOOT_FB]      = 0;   // BOOT_NEW_IMAGE (matches bootloader FRAM.h)
        raw[SLOT_OFF_NEW_META]     = (i == (uint8_t)(active_idx - 1)) ? 1 : 0;
        raw[SLOT_OFF_ERROR_CODE]   = 0;

        uint16_t rc = Calc_CRC16(&raw[2], SLOT_RECORD_DATA_LEN);
        raw[SLOT_OFF_CRC16_HI] = (uint8_t)((rc >> 8) & 0xFF);
        raw[SLOT_OFF_CRC16_LO] = (uint8_t)(rc & 0xFF);
    }

    // Compute CRC with commit = META_OK so it matches the final committed state
    g_work.commit = META_OK;
    g_work.crc16  = block_crc(&g_work);

    return commit_copy(FRAM_BLK_A_ADDR, &g_work);
}

bool FRAMMETA_CommitNext(const fram_meta_block_t* next_in, uint32_t cur_addr)
{
	fram_meta_block_t tmp = next_in ? *next_in : g_work;  /* commit the in-RAM working copy */
    tmp.seq    = (uint16_t)(tmp.seq + 1);  // next generation

    // Compute CRC with commit = META_OK so it matches the final committed state
    tmp.commit = META_OK;
    tmp.crc16  = block_crc(&tmp);

    uint16_t dst = (cur_addr == FRAM_BLK_A_ADDR) ? FRAM_BLK_B_ADDR : FRAM_BLK_A_ADDR;
    return commit_copy(dst, &tmp);
}

bool FRAMMETA_SetImageInfo(uint8_t img_id,
                           uint32_t flash_addr,
                           uint32_t image_size,
                           uint32_t image_crc,
                           uint8_t bank_id)
{
    uint32_t cur_addr = 0;
    fram_meta_block_t blk;

    // Require valid metadata to already exist. The bootloader provisions the
    // metadata block (including real golden-slot address/size/CRC) via
    // FRAMMETA_BL_InitDefaults before the app ever runs, so a missing block here
    // is a genuine FRAM fault — report it (surfaces as FRAM_META_FAIL at the
    // FWUP_FLASH caller) rather than fabricating an unverifiable golden default.
    if (!FRAMMETA_Load(&blk, &cur_addr)) {
        return false;
    }

    // img_id is used as slot index in the current metadata model
    if (img_id < 1 || img_id > NUM_SLOTS) {
        return false;
    }

    // Mark the slot as a newly installed image.
    // It remains pending until the application successfully calls ConfirmBoot().
    FRAMMETA_SetSlot(img_id,
                     flash_addr,
                     image_size,
                     image_crc,
                     bank_id,
                     0,   // BOOT_NEW_IMAGE
                     3,   // boot attempts remaining
                     1,   // META_PENDING
                     0);  // NO_BOOT_ERROR

    // Activate new image
    FRAMMETA_SetActiveIndex(img_id);

    return FRAMMETA_CommitNext(NULL, cur_addr);

}

bool FRAMMETA_ActivateImage(uint8_t img_id)
{
    uint32_t cur_addr = 0;

    if (!FRAMMETA_Load(NULL, &cur_addr)) {
        return false;
    }

    if (img_id < 1u || img_id > NUM_SLOTS) {
        return false;
    }

    uint8_t* rec = g_work.rec[img_id - 1u];
    uint16_t stored_crc = ((uint16_t)rec[SLOT_OFF_CRC16_HI] << 8)
                        | rec[SLOT_OFF_CRC16_LO];

    if (stored_crc != Calc_CRC16(&rec[2], SLOT_RECORD_DATA_LEN)) {
        return false;
    }

    bool already_selected =
        g_work.active_idx == img_id &&
        rec[SLOT_OFF_BOOT_FB] == 0u &&
        rec[SLOT_OFF_NEW_META] == 1u &&
        rec[SLOT_OFF_BOOT_COUNTER] == 3u &&
        rec[SLOT_OFF_ERROR_CODE] == 0u;

    if (already_selected) {
        return true;
    }

    g_work.active_idx = img_id;
    rec[SLOT_OFF_BOOT_FB] = 0u;       // BOOT_NEW_IMAGE
    rec[SLOT_OFF_NEW_META] = 1u;      // META_PENDING
    rec[SLOT_OFF_BOOT_COUNTER] = 3u;
    rec[SLOT_OFF_ERROR_CODE] = 0u;
    FRAMMETA_RecalcSlotCRC(img_id);

    return FRAMMETA_CommitNext(NULL, cur_addr);
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

bool ConfirmBoot(void)
{
    uint32_t cur_addr = 0;

    // Load the current metadata — this also populates g_work
    if (!FRAMMETA_Load(NULL, &cur_addr)) return false;

    uint8_t idx = g_work.active_idx;
    if (idx < 1 || idx > NUM_SLOTS) return false;

    uint8_t* rec = g_work.rec[idx - 1];
    rec[SLOT_OFF_BOOT_FB]      = 1;   // BOOTED_OK
    rec[SLOT_OFF_NEW_META]     = 0;   // META_CONFIRMED
    rec[SLOT_OFF_BOOT_COUNTER] = 3;   // reset counter for next OTA cycle
    rec[SLOT_OFF_ERROR_CODE]   = 0;   // NO_BOOT_ERROR
    FRAMMETA_RecalcSlotCRC(idx);

    return FRAMMETA_CommitNext(NULL, cur_addr);
}
