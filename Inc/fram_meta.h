/*
 * fram_meta.h
 *
 *  Created on: Oct 9, 2025
 *      Author: haydenstotts
 */

#ifndef INC_FRAM_META_H_
#define INC_FRAM_META_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "FRAM.h"

#ifndef NUM_SLOTS
#define NUM_SLOTS 24u
#endif

typedef struct __attribute__((packed)) {
    uint32_t magic;              /* 'META' = 0x4D455441 */
    uint16_t version;            /* metadata format version */
    uint16_t seq;                /* wrapping generation counter */
    uint8_t active_idx;          /* selected slot, 1..NUM_SLOTS */
    uint8_t commit;              /* 0xFF = WIP, 0xA5 = committed */
    uint8_t _rsv[2];             /* reserved, initialized to 0xFF */
    uint8_t rec[NUM_SLOTS][20];  /* packed 20-byte slot records */
    uint16_t crc16;              /* CRC16 over every preceding byte */
} fram_meta_block_t;

_Static_assert(sizeof(fram_meta_block_t) == 494u,
               "FRAM metadata block layout changed");
_Static_assert(offsetof(fram_meta_block_t, rec) == 12u,
               "FRAM metadata record offset changed");
_Static_assert(offsetof(fram_meta_block_t, crc16) == 492u,
               "FRAM metadata CRC offset changed");
_Static_assert(sizeof(((fram_meta_block_t*)0)->rec[0]) == 20u,
               "FRAM slot record layout changed");

/* Read-only health of each redundant FRAM metadata copy.  These numeric
 * values are also used by the GET_BOOT_METADATA wire report. */
typedef enum {
    FRAMMETA_COPY_VALID    = 0,
    FRAMMETA_COPY_INVALID  = 1,
    FRAMMETA_COPY_IO_ERROR = 2,
} fram_meta_copy_status_t;

typedef enum {
    FRAMMETA_SELECTED_NONE = 0,
    FRAMMETA_SELECTED_A    = 1,
    FRAMMETA_SELECTED_B    = 2,
} fram_meta_selected_copy_t;

typedef struct {
    fram_meta_block_t copy_a;
    fram_meta_block_t copy_b;
    fram_meta_copy_status_t copy_a_status;
    fram_meta_copy_status_t copy_b_status;
    fram_meta_selected_copy_t selected_copy;
} fram_meta_snapshot_t;

/* Per-slot record byte offsets (within each rec[i][20] array) */
#define SLOT_OFF_CRC16_HI     0   /* CRC16 high byte (over bytes 2..19) */
#define SLOT_OFF_CRC16_LO     1   /* CRC16 low byte */
#define SLOT_OFF_FLASH_ADDR   2   /* u32 LE, bytes 2..5 */
#define SLOT_OFF_IMAGE_SIZE   6   /* u32 LE, bytes 6..9 */
#define SLOT_OFF_IMAGE_CRC32  10  /* u32 LE, bytes 10..13 */
#define SLOT_OFF_BANK_ID      14
#define SLOT_OFF_IMAGE_INDEX  15
#define SLOT_OFF_BOOT_COUNTER 16
#define SLOT_OFF_BOOT_FB      17
#define SLOT_OFF_NEW_META     18
#define SLOT_OFF_ERROR_CODE   19
#define SLOT_RECORD_DATA_LEN  18  /* bytes 2..19, covered by CRC */

/* Load the newest valid committed copy and update the internal working copy. */
bool FRAMMETA_Load(fram_meta_block_t* out, uint32_t* cur_addr);

/* Read and classify both copies without changing the internal working copy. */
bool FRAMMETA_ReadSnapshot(fram_meta_snapshot_t* out);

/* Initialize copy A with default records and the supplied active slot. */
bool FRAMMETA_InitDefaults(uint8_t active_idx);

/* Commit the next generation to the inactive copy, commit byte last, and verify. */
bool FRAMMETA_CommitNext(const fram_meta_block_t* next_in, uint32_t cur_addr);

bool FRAMMETA_SetImageInfo(uint8_t img_id,
                           uint32_t flash_addr,
                           uint32_t image_size,
                           uint32_t image_crc,
                           uint8_t bank_id);

/* Select an existing image as a fresh pending boot and commit the change. */
bool FRAMMETA_ActivateImage(uint8_t img_id);

/* Working-copy mutators; call FRAMMETA_CommitNext() to persist changes. */
void FRAMMETA_SetActiveIndex(uint8_t idx);

void FRAMMETA_SetSlot(uint8_t slot_idx,
                      uint32_t base_addr,
                      uint32_t image_size,
                      uint32_t image_crc,
                      uint8_t bank_id,
                      uint8_t boot_feedback,
                      uint8_t boot_counter,
                      uint8_t new_metadata,
                      uint8_t error_code);

/* Accessors for the internal working copy. */
uint8_t FRAMMETA_GetActiveIndex(void);
bool FRAMMETA_GetSlotRaw(uint8_t slot_idx, uint8_t out20[20]);

/* Recompute the selected record's CRC16 over bytes 2..19. */
void FRAMMETA_RecalcSlotCRC(uint8_t slot_idx);

/* Persist BOOTED_OK for the active slot after startup health checks pass. */
bool ConfirmBoot(void);

#endif /* INC_FRAM_META_H_ */
