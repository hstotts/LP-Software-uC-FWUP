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

// Reuse your FRAM.h CRC + read/write:
#include <FRAM.h>

#ifndef NUM_SLOTS
#define NUM_SLOTS 5
#endif

/* ----- Public block type (exposed for reading/debug) ----- */

typedef struct __attribute__((packed)) {
    uint32_t magic;        // 'META' = 0x4D455441
    uint16_t version;      // 1
    uint16_t seq;          // generation counter (monotonic)
    uint8_t  active_idx;   // selected image index (1..NUM_SLOTS)
    uint8_t  commit;       // 0xFF = WIP, 0xA5 = committed (write LAST)
    uint8_t  _rsv[2];      // reserved (set to 0xFF)
    uint16_t crc16;        // CRC16-CCITT over [magic.._rsv] + rec[] (excl. this field)
    uint8_t  rec[NUM_SLOTS][7]; // packed 7-byte per-slot records
} fram_meta_block_t;

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

// Update one slot's 7-byte record using your existing minimal fields.
// NOTE: since 7-byte layout doesn't hold base addresses, size, etc.,
// those params are accepted for future use but only minimal fields are packed.
void FRAMMETA_SetSlot(uint8_t slot_idx,
                      /* optional informational fields for future expansion */
                      uint32_t base_addr, uint32_t image_size, uint32_t image_crc, uint8_t bank_id,
                      /* state mapping to your 7-byte semantics */
                      uint8_t boot_feedback, uint8_t boot_counter, uint8_t new_metadata, uint8_t error_code);

// Accessors (from the internal working copy loaded by FRAMMETA_Load / InitDefaults)
uint8_t FRAMMETA_GetActiveIndex(void);
bool FRAMMETA_GetSlotRaw(uint8_t slot_idx, uint8_t out7[7]);

/* Optional: helper to force-recompute per-slot record CRC (2 bytes on [2..6]) */
void FRAMMETA_RecalcSlotCRC(uint8_t slot_idx);


#endif /* INC_FRAM_META_H_ */
