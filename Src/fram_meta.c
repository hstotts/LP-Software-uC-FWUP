/*
 * fram_meta.c
 *
 *  Created on: Oct 9, 2025
 *      Author: haydenstotts
 */
#include "fram_meta.h"
#include "FRAM.h"
#include "General_Functions.h"
#include "stm32f7xx_hal.h"

#include <stddef.h>
#include <string.h>

#define FRAM_BLK_A_ADDR ((uint16_t)FRAM_META_BLK_A_ADDR)
#define FRAM_BLK_B_ADDR ((uint16_t)FRAM_META_BLK_B_ADDR)

#define META_MAGIC 0x4D455441u  /* 'META' */
#define META_VER   1u
#define META_WIP   0xFFu
#define META_OK    0xA5u
#define FRAMMETA_ATTEMPTS 2u

static fram_meta_block_t g_work;

static uint16_t calculate_block_crc(const fram_meta_block_t* block)
{
    return Calc_CRC16((uint8_t*)block,
                      offsetof(fram_meta_block_t, crc16));
}

static bool block_is_valid(const fram_meta_block_t* block)
{
    if (block->magic != META_MAGIC ||
        block->version != META_VER ||
        block->commit != META_OK) {
        return false;
    }

    return calculate_block_crc(block) == block->crc16;
}

static void recover_fram_bus(void)
{
    (void)HAL_I2C_DeInit(&hi2c4);
    HAL_Delay(2u);
    (void)HAL_I2C_Init(&hi2c4);
}

static bool read_block(uint16_t addr, fram_meta_block_t* out)
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

static bool commit_block_copy(uint16_t addr, const fram_meta_block_t* source)
{
    for (uint32_t attempt = 0u; attempt < FRAMMETA_ATTEMPTS; attempt++) {
        fram_meta_block_t pending = *source;
        pending.commit = META_WIP;

        (void)BootHealth_RefreshIWDG();

        bool write_ok =
            writeFRAM(addr, (uint8_t*)&pending, sizeof(pending)) == HAL_OK;
        uint8_t commit = META_OK;
        if (write_ok) {
            (void)BootHealth_RefreshIWDG();
            write_ok = writeFRAM(
                           (uint16_t)(addr +
                                      offsetof(fram_meta_block_t, commit)),
                           &commit,
                           1u) == HAL_OK;
        }

        fram_meta_block_t verified;
        if (write_ok) {
            (void)BootHealth_RefreshIWDG();
            write_ok = readFRAM(addr,
                                (uint8_t*)&verified,
                                sizeof(verified)) == HAL_OK &&
                       block_is_valid(&verified);
        }

        if (write_ok) {
            g_work = verified;
            return true;
        }

        if (attempt + 1u < FRAMMETA_ATTEMPTS) {
            recover_fram_bus();
        }
    }
    return false;
}

static fram_meta_copy_status_t classify_copy(bool read_succeeded,
                                              const fram_meta_block_t* copy)
{
    if (!read_succeeded) {
        return FRAMMETA_COPY_IO_ERROR;
    }

    return block_is_valid(copy)
           ? FRAMMETA_COPY_VALID
           : FRAMMETA_COPY_INVALID;
}

bool FRAMMETA_ReadSnapshot(fram_meta_snapshot_t* out)
{
    if (out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    out->selected_copy = FRAMMETA_SELECTED_NONE;

    bool read_a_ok = read_block(FRAM_BLK_A_ADDR, &out->copy_a);
    bool read_b_ok = read_block(FRAM_BLK_B_ADDR, &out->copy_b);

    out->copy_a_status = classify_copy(read_a_ok, &out->copy_a);
    out->copy_b_status = classify_copy(read_b_ok, &out->copy_b);

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
    fram_meta_block_t copy_a;
    fram_meta_block_t copy_b;
    bool valid_a = read_block(FRAM_BLK_A_ADDR, &copy_a) &&
                   block_is_valid(&copy_a);
    bool valid_b = read_block(FRAM_BLK_B_ADDR, &copy_b) &&
                   block_is_valid(&copy_b);
    const fram_meta_block_t* selected;
    uint32_t selected_addr;

    if (valid_a && valid_b) {
        if ((int16_t)(copy_b.seq - copy_a.seq) > 0) {
            selected = &copy_b;
            selected_addr = FRAM_BLK_B_ADDR;
        } else {
            selected = &copy_a;
            selected_addr = FRAM_BLK_A_ADDR;
        }
    } else if (valid_a) {
        selected = &copy_a;
        selected_addr = FRAM_BLK_A_ADDR;
    } else if (valid_b) {
        selected = &copy_b;
        selected_addr = FRAM_BLK_B_ADDR;
    } else {
        return false;
    }

    if (out != NULL) {
        *out = *selected;
    }
    if (cur_addr != NULL) {
        *cur_addr = selected_addr;
    }
    g_work = *selected;
    return true;
}

bool FRAMMETA_InitDefaults(uint8_t active_idx)
{
    memset(&g_work, 0xFF, sizeof(g_work));
    g_work.magic = META_MAGIC;
    g_work.version = META_VER;
    g_work.seq = 1u;
    g_work.active_idx = active_idx;

    for (uint8_t i = 0u; i < NUM_SLOTS; i++) {
        uint8_t* raw = g_work.rec[i];
        memset(raw, 0, sizeof(g_work.rec[i]));

        raw[SLOT_OFF_BANK_ID] = 0u;
        raw[SLOT_OFF_IMAGE_INDEX] = (uint8_t)(i + 1u);
        raw[SLOT_OFF_BOOT_COUNTER] = 3u;
        raw[SLOT_OFF_BOOT_FB] = 0u; /* BOOT_NEW_IMAGE */
        raw[SLOT_OFF_NEW_META] =
            (i == (uint8_t)(active_idx - 1u)) ? 1u : 0u;
        raw[SLOT_OFF_ERROR_CODE] = 0u;

        uint16_t record_crc = Calc_CRC16(&raw[2], SLOT_RECORD_DATA_LEN);
        raw[SLOT_OFF_CRC16_HI] = (uint8_t)((record_crc >> 8) & 0xFFu);
        raw[SLOT_OFF_CRC16_LO] = (uint8_t)(record_crc & 0xFFu);
    }

    /* The block CRC covers the final committed value, not META_WIP. */
    g_work.commit = META_OK;
    g_work.crc16 = calculate_block_crc(&g_work);

    return commit_block_copy(FRAM_BLK_A_ADDR, &g_work);
}

bool FRAMMETA_CommitNext(const fram_meta_block_t* next_in, uint32_t cur_addr)
{
    fram_meta_block_t next = (next_in != NULL) ? *next_in : g_work;
    next.seq = (uint16_t)(next.seq + 1u);

    /* The block CRC covers the final committed value, not META_WIP. */
    next.commit = META_OK;
    next.crc16 = calculate_block_crc(&next);

    uint16_t destination = (cur_addr == FRAM_BLK_A_ADDR)
                           ? FRAM_BLK_B_ADDR
                           : FRAM_BLK_A_ADDR;
    return commit_block_copy(destination, &next);
}

bool FRAMMETA_SetImageInfo(uint8_t img_id,
                           uint32_t flash_addr,
                           uint32_t image_size,
                           uint32_t image_crc,
                           uint8_t bank_id)
{
    uint32_t cur_addr = 0u;

    /* The bootloader provisions the golden image metadata. Do not fabricate
     * defaults here if both copies are unavailable or invalid. */
    if (!FRAMMETA_Load(NULL, &cur_addr)) {
        return false;
    }

    if (img_id < 1u || img_id > NUM_SLOTS) {
        return false;
    }

    FRAMMETA_SetSlot(img_id,
                     flash_addr,
                     image_size,
                     image_crc,
                     bank_id,
                     0u,  /* BOOT_NEW_IMAGE */
                     3u,  /* boot attempts remaining */
                     1u,  /* META_PENDING */
                     0u); /* NO_BOOT_ERROR */

    FRAMMETA_SetActiveIndex(img_id);

    return FRAMMETA_CommitNext(NULL, cur_addr);
}

bool FRAMMETA_ActivateImage(uint8_t img_id)
{
    uint32_t cur_addr = 0u;

    if (!FRAMMETA_Load(NULL, &cur_addr)) {
        return false;
    }

    if (img_id < 1u || img_id > NUM_SLOTS) {
        return false;
    }

    uint8_t* record = g_work.rec[img_id - 1u];
    uint16_t stored_crc = ((uint16_t)record[SLOT_OFF_CRC16_HI] << 8)
                        | record[SLOT_OFF_CRC16_LO];

    if (stored_crc != Calc_CRC16(&record[2], SLOT_RECORD_DATA_LEN)) {
        return false;
    }

    bool already_pending =
        g_work.active_idx == img_id &&
        record[SLOT_OFF_BOOT_FB] == 0u &&
        record[SLOT_OFF_NEW_META] == 1u &&
        record[SLOT_OFF_BOOT_COUNTER] == 3u &&
        record[SLOT_OFF_ERROR_CODE] == 0u;

    if (already_pending) {
        return true;
    }

    g_work.active_idx = img_id;
    record[SLOT_OFF_BOOT_FB] = 0u;  /* BOOT_NEW_IMAGE */
    record[SLOT_OFF_NEW_META] = 1u; /* META_PENDING */
    record[SLOT_OFF_BOOT_COUNTER] = 3u;
    record[SLOT_OFF_ERROR_CODE] = 0u;
    FRAMMETA_RecalcSlotCRC(img_id);

    return FRAMMETA_CommitNext(NULL, cur_addr);
}

void FRAMMETA_SetActiveIndex(uint8_t idx)
{
    g_work.active_idx = idx;
}

void FRAMMETA_SetSlot(uint8_t slot_idx,
                      uint32_t base_addr,
                      uint32_t image_size,
                      uint32_t image_crc,
                      uint8_t bank_id,
                      uint8_t boot_feedback,
                      uint8_t boot_counter,
                      uint8_t new_metadata,
                      uint8_t error_code)
{
    if (slot_idx < 1u || slot_idx > NUM_SLOTS) {
        return;
    }
    uint8_t* raw = g_work.rec[slot_idx - 1u];

    raw[SLOT_OFF_FLASH_ADDR + 0u] = (uint8_t)(base_addr & 0xFFu);
    raw[SLOT_OFF_FLASH_ADDR + 1u] = (uint8_t)((base_addr >> 8) & 0xFFu);
    raw[SLOT_OFF_FLASH_ADDR + 2u] = (uint8_t)((base_addr >> 16) & 0xFFu);
    raw[SLOT_OFF_FLASH_ADDR + 3u] = (uint8_t)((base_addr >> 24) & 0xFFu);

    raw[SLOT_OFF_IMAGE_SIZE + 0u] = (uint8_t)(image_size & 0xFFu);
    raw[SLOT_OFF_IMAGE_SIZE + 1u] = (uint8_t)((image_size >> 8) & 0xFFu);
    raw[SLOT_OFF_IMAGE_SIZE + 2u] = (uint8_t)((image_size >> 16) & 0xFFu);
    raw[SLOT_OFF_IMAGE_SIZE + 3u] = (uint8_t)((image_size >> 24) & 0xFFu);

    raw[SLOT_OFF_IMAGE_CRC32 + 0u] = (uint8_t)(image_crc & 0xFFu);
    raw[SLOT_OFF_IMAGE_CRC32 + 1u] = (uint8_t)((image_crc >> 8) & 0xFFu);
    raw[SLOT_OFF_IMAGE_CRC32 + 2u] = (uint8_t)((image_crc >> 16) & 0xFFu);
    raw[SLOT_OFF_IMAGE_CRC32 + 3u] = (uint8_t)((image_crc >> 24) & 0xFFu);

    raw[SLOT_OFF_BANK_ID] = bank_id;
    raw[SLOT_OFF_IMAGE_INDEX] = slot_idx;
    raw[SLOT_OFF_BOOT_COUNTER] = boot_counter;
    raw[SLOT_OFF_BOOT_FB] = boot_feedback;
    raw[SLOT_OFF_NEW_META] = new_metadata;
    raw[SLOT_OFF_ERROR_CODE] = error_code;

    uint16_t record_crc = Calc_CRC16(&raw[2], SLOT_RECORD_DATA_LEN);
    raw[SLOT_OFF_CRC16_HI] = (uint8_t)((record_crc >> 8) & 0xFFu);
    raw[SLOT_OFF_CRC16_LO] = (uint8_t)(record_crc & 0xFFu);
}

uint8_t FRAMMETA_GetActiveIndex(void)
{
    return g_work.active_idx;
}

bool FRAMMETA_GetSlotRaw(uint8_t slot_idx, uint8_t out20[20])
{
    if (slot_idx < 1u || slot_idx > NUM_SLOTS) {
        return false;
    }
    if (out20 != NULL) {
        memcpy(out20, g_work.rec[slot_idx - 1u], 20u);
    }
    return true;
}

void FRAMMETA_RecalcSlotCRC(uint8_t slot_idx)
{
    if (slot_idx < 1u || slot_idx > NUM_SLOTS) {
        return;
    }

    uint8_t* raw = g_work.rec[slot_idx - 1u];
    uint16_t record_crc = Calc_CRC16(&raw[2], SLOT_RECORD_DATA_LEN);
    raw[SLOT_OFF_CRC16_HI] = (uint8_t)((record_crc >> 8) & 0xFFu);
    raw[SLOT_OFF_CRC16_LO] = (uint8_t)(record_crc & 0xFFu);
}

bool ConfirmBoot(void)
{
    uint32_t cur_addr = 0u;

    if (!FRAMMETA_Load(NULL, &cur_addr)) {
        return false;
    }

    uint8_t active_idx = g_work.active_idx;
    if (active_idx < 1u || active_idx > NUM_SLOTS) {
        return false;
    }

    uint8_t* record = g_work.rec[active_idx - 1u];
    record[SLOT_OFF_BOOT_FB] = 1u;      /* BOOTED_OK */
    record[SLOT_OFF_NEW_META] = 0u;     /* META_CONFIRMED */
    record[SLOT_OFF_BOOT_COUNTER] = 3u; /* reset for the next OTA cycle */
    record[SLOT_OFF_ERROR_CODE] = 0u;   /* NO_BOOT_ERROR */
    FRAMMETA_RecalcSlotCRC(active_idx);

    return FRAMMETA_CommitNext(NULL, cur_addr);
}
