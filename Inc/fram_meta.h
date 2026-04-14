/*
 * fram_meta.h
 *
 *  Created on: Oct 9, 2025
 *      Author: haydenstotts
 */

#ifndef INC_FRAM_META_H_
#define INC_FRAM_META_H_

#pragma once
#include <stdint.h>
#include <stdbool.h>

// Reuse built FRAM.h CRC + read/write:
#include <FRAM.h>

#ifndef NUM_SLOTS
#define NUM_SLOTS 24
#endif

/* ----- Public block type (exposed for reading/debug) ----- */

typedef struct __attribute__((packed)) {
    uint32_t magic;        // 'META' = 0x4D455441
    uint16_t version;      // 1
    uint16_t seq;          // generation counter (monotonic)
    uint8_t  active_idx;   // selected image index (1..NUM_SLOTS)
    uint8_t  commit;       // 0xFF = WIP, 0xA5 = committed (write LAST)
    uint8_t  _rsv[2];      // reserved (set to 0xFF)
    uint8_t  rec[NUM_SLOTS][20]; // packed 20-byte per-slot records
    uint16_t crc16;        // CRC16-CCITT over [magic.._rsv] + rec[] (excl. this field)
} fram_meta_block_t;


/* Per-slot record byte offsets (within each rec[i][20] array) */
#define SLOT_OFF_CRC16_HI     0   // CRC16 high byte (over bytes [2..19])
#define SLOT_OFF_CRC16_LO     1   // CRC16 low byte
#define SLOT_OFF_FLASH_ADDR   2   // u32 LE, bytes [2..5]
#define SLOT_OFF_IMAGE_SIZE   6   // u32 LE, bytes [6..9]
#define SLOT_OFF_IMAGE_CRC32  10  // u32 LE, bytes [10..13]
#define SLOT_OFF_BANK_ID      14
#define SLOT_OFF_IMAGE_INDEX  15
#define SLOT_OFF_BOOT_COUNTER 16
#define SLOT_OFF_BOOT_FB      17
#define SLOT_OFF_NEW_META     18
#define SLOT_OFF_ERROR_CODE   19
#define SLOT_RECORD_DATA_LEN  18  // bytes [2..19], covered by CRC


/* ----- API ----- */

// Load the current A/B metadata block into *out and return which FRAM copy was used.
// Returns true if a valid committed block is found.
bool FRAMMETA_Load(fram_meta_block_t* out, uint32_t* cur_addr);

// Initialize FRAM with a default block (first-time or full repair).
// active_idx is set to the image you want to boot by default (e.g., 4 for S5 current app).
bool FRAMMETA_InitDefaults(uint8_t active_idx);

// Commit a new generation (write to the other A/B copy, commit byte last, verify).
bool FRAMMETA_CommitNext(const fram_meta_block_t* next_in, uint32_t cur_addr);


bool FRAMMETA_SetImageInfo(uint8_t img_id,
                           uint32_t flash_addr,
                           uint32_t image_size,
                           uint32_t image_crc,
                           uint8_t bank_id);

// Convenience mutators that modify an in-RAM working copy (stored internally).
// After calling setters, call FRAMMETA_CommitNext() to persist.
void FRAMMETA_SetActiveIndex(uint8_t idx);

// Update one slot's 20-byte record. Packs all fields: flash_addr, image_size,
// image_crc32, bank_id, and boot state fields. Call FRAMMETA_CommitNext() to persist.
void FRAMMETA_SetSlot(uint8_t slot_idx,
                      uint32_t base_addr,    // flash address of image
                      uint32_t image_size,   // image size in bytes
                      uint32_t image_crc,    // CRC32 of image
                      uint8_t  bank_id,      // flash bank (0=bank1, 1=bank2)
                      uint8_t boot_feedback, // e.g. BOOT_NEW_IMAGE or BOOTED_OK
                      uint8_t boot_counter,  // decremented by bootloader on each attempt
                      uint8_t new_metadata,  // 1=pending confirm, 0=confirmed healthy
                      uint8_t error_code);

// Accessors (from the internal working copy loaded by FRAMMETA_Load / InitDefaults)
uint8_t FRAMMETA_GetActiveIndex(void);
bool FRAMMETA_GetSlotRaw(uint8_t slot_idx, uint8_t out20[20]);

/* Optional: helper to force-recompute per-slot record CRC (2 bytes on [2..6]) */
void FRAMMETA_RecalcSlotCRC(uint8_t slot_idx);

// Call once at startup after peripherals are ready.
// Tells the bootloader this image booted successfully so it does not
// decrement the boot counter or fall back to the golden image on next reset.
void ConfirmBoot(void);

#endif /* INC_FRAM_META_H_ */
