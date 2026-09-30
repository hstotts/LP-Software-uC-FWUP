/*
 * PUS_8_service.c
 *
 *  Created on: 2024. gada 11. jūl.
 *      Author: Rūdolfs Arvīds Kalniņš <rakal@kth.se>
 */

#include <Device_State.h>
#include "cmsis_os.h"
#include "Space_Packet_Protocol.h"
#include "PUS.h"
#include "General_Functions.h"
#include "PUS_1_service.h"
#include "PUS_8_service.h"
#include "FRAM.h"
#include "flash_if.h"
#include "memory_map.h"
#include "fram_meta.h"

#define FPGA_MSG_PREAMBLE_0     0xB5
#define FPGA_MSG_PREAMBLE_1     0x43
#define FPGA_MSG_POSTAMBLE      0x0A

#define LANGMUIR_READBACK_PREAMBLE_0    FPGA_MSG_PREAMBLE_0
#define LANGMUIR_READBACK_PREAMBLE_1    FPGA_MSG_PREAMBLE_1
#define LANGMUIR_READBACK_POSTAMBLE     FPGA_MSG_POSTAMBLE

#define SCIENTIFIC_DATA_PREAMBLE        0x83
#define SC_DATA_MAX_SIZE                10000
#define SC_CB_PACKET_RAW_DATA_LEN       6 // 2 sequence counter bytes and 2 data bytes each probe.
#define SC_CB_PACKET_FULL_DATA_LEN      1 + SC_CB_PACKET_RAW_DATA_LEN  // 1 byte header

/* GET_BOOT_METADATA report format (all multibyte fields are little-endian). */
#define MD_REPORT_FORMAT_VERSION  1u
#define MD_REPORT_SUMMARY         0u
#define MD_REPORT_SLOT_DETAIL     1u

#define MD_OVERALL_OK             0u
#define MD_OVERALL_DEGRADED       1u
#define MD_OVERALL_UNAVAILABLE    2u

#define MD_FLAG_INSTALLED         (1u << 0)
#define MD_FLAG_ACTIVE            (1u << 1)
#define MD_FLAG_RECORD_CRC_OK     (1u << 2)
#define MD_FLAG_PENDING           (1u << 3)
#define MD_FLAG_BOOTED_OK         (1u << 4)
#define MD_FLAG_GOLDEN            (1u << 5)
#define MD_FLAG_OTA               (1u << 6)
#define MD_FLAG_PROTECTED         (1u << 7)

#define MD_BOOTED_OK              1u
#define MD_META_CONFIRMED         0u
#define MD_META_PENDING           1u

uint8_t g_fw_staging[SRAM_FW_STAGING_SIZE]
	__attribute__((section(".fw_staging"), aligned(32), used));

_Static_assert(sizeof(g_fw_staging) == 0x00020000u,
			   "FWUP staging buffer must be 128 KiB");

extern uint32_t g_pfnVectors[];

static int fwup_get_executing_bank_id(uint8_t* bank_id)
{
	if (bank_id == NULL) {
		return 0;
	}

	uintptr_t linked_image_base = (uintptr_t)&g_pfnVectors[0];

	if (linked_image_base >= FLASH_BANK1_BASE &&
		linked_image_base <= FLASH_BANK1_END) {
		*bank_id = 0u;
		return 1;
	}

	if (linked_image_base >= FLASH_BANK2_BASE &&
		linked_image_base <= FLASH_BANK2_END) {
		*bank_id = 1u;
		return 1;
	}

	return 0;
}

static uint8_t fwup_get_executing_slot(void)
{
	uintptr_t linked_image_base = (uintptr_t)&g_pfnVectors[0];

	for (uint8_t slot_id = 1u; slot_id <= NUM_SLOTS; slot_id++) {
		fw_slot_desc_t slot;
		if (fw_slot_get(slot_id, &slot) && slot.base == linked_image_base) {
			return slot_id;
		}
	}

	return 0u;
}

typedef enum {
	FWUP_STATE_IDLE = 0,
	FWUP_STATE_STAGING,
} fwup_state_t;

/* Erase/program/readback/metadata failures deliberately remain STAGING so
 * ground may retry FWUP_FLASH. Only a successful metadata commit returns IDLE. */
static fwup_state_t fwup_state = FWUP_STATE_IDLE;
static uint8_t fwup_target_slot = 0u;
static uint32_t fwup_expected_size = 0u;
static uint32_t fwup_expected_crc32 = 0u;
static uint32_t fwup_staged_extent = 0u;

/* Placement frozen from the trusted slot descriptor at FWUP_BEGIN. */
static uint32_t fwup_target_address = 0u;
static uint8_t fwup_target_bank = 0u;

static uint32_t read_u32_le(const uint8_t* raw, uint8_t offset)
{
	return (uint32_t)raw[offset] |
		   ((uint32_t)raw[offset + 1u] << 8) |
		   ((uint32_t)raw[offset + 2u] << 16) |
		   ((uint32_t)raw[offset + 3u] << 24);
}

static bool fwup_staged_vectors_valid(void)
{
	uint32_t app_sp = read_u32_le(g_fw_staging, 0u);
	uint32_t app_pc = read_u32_le(g_fw_staging, 4u);
	uint32_t reset_addr = app_pc & ~1u;
	uint32_t image_end = fwup_target_address + fwup_expected_size;

	if (image_end < fwup_target_address) {
		return false;
	}
	if (app_sp <= 0x20000000u || app_sp > 0x20080000u ||
		(app_sp & 0x7u) != 0u) {
		return false;
	}
	return (app_pc & 1u) != 0u &&
		   reset_addr >= fwup_target_address && reset_addr < image_end;
}

static bool fwup_boot_candidate_valid(uint8_t img_id,
                                      uint32_t requested_addr,
                                      const uint8_t record[20])
{
	fw_slot_desc_t slot;

	if (record == NULL || !fw_slot_get(img_id, &slot)) {
		return false;
	}

	if (slot.role != FW_SLOT_ROLE_GOLDEN &&
		slot.role != FW_SLOT_ROLE_OTA) {
		return false;
	}

	if (requested_addr != slot.base) {
		return false;
	}

	uint16_t stored_record_crc =
		((uint16_t)record[SLOT_OFF_CRC16_HI] << 8) |
		record[SLOT_OFF_CRC16_LO];
	uint16_t calculated_record_crc =
		Calc_CRC16((uint8_t*)&record[2], SLOT_RECORD_DATA_LEN);

	if (stored_record_crc != calculated_record_crc) {
		return false;
	}

	uint32_t flash_addr = read_u32_le(record, SLOT_OFF_FLASH_ADDR);
	uint32_t image_size = read_u32_le(record, SLOT_OFF_IMAGE_SIZE);
	uint32_t stored_image_crc =
		read_u32_le(record, SLOT_OFF_IMAGE_CRC32);

	if (record[SLOT_OFF_IMAGE_INDEX] != img_id ||
		record[SLOT_OFF_BANK_ID] != slot.bank_id ||
		flash_addr != slot.base ||
		image_size < 8u || image_size > slot.capacity) {
		return false;
	}

	uint32_t image_end = flash_addr + image_size;
	if (image_end < flash_addr ||
		image_end > (slot.base + slot.capacity)) {
		return false;
	}

	if (crc32_calc((const uint8_t*)flash_addr, image_size) !=
		stored_image_crc) {
		return false;
	}

	uint32_t app_sp = *(volatile const uint32_t*)flash_addr;
	uint32_t app_pc = *(volatile const uint32_t*)(flash_addr + 4u);
	uint32_t reset_addr = app_pc & ~1u;

	if (app_sp <= 0x20000000u || app_sp > 0x20080000u ||
		(app_sp & 0x7u) != 0u) {
		return false;
	}

	if ((app_pc & 1u) == 0u ||
		reset_addr < flash_addr || reset_addr >= image_end) {
		return false;
	}

	return true;
}

extern QueueHandle_t UART_OBC_Out_Queue;
extern UART_HandleTypeDef huart5;

extern volatile uint8_t g_boot_confirmed;

extern osThreadId PUS_3_TaskHandle;
extern osThreadId Watchdog_TaskHandle;
extern osThreadId UART_FPGA_INHandle;

extern volatile uint8_t Sweep_Bias_Mode_Data[3072];
extern volatile uint16_t Sweep_Bias_Data_counter;
extern volatile uint16_t Old_Sweep_Bias_Data_counter;

// This queue is used to receive info from the UART handler task
QueueHandle_t PUS_8_Queue;

uint8_t UART_FPGA_Rx_Buffer[100];
uint8_t UART_FPGA_OBC_Tx_Buffer[100];

volatile uint8_t uart_tx_FPGA_done = 1;

static const fram_meta_block_t* metadata_selected_block(
	const fram_meta_snapshot_t* snapshot)
{
	if (snapshot->selected_copy == FRAMMETA_SELECTED_A) {
		return &snapshot->copy_a;
	}
	if (snapshot->selected_copy == FRAMMETA_SELECTED_B) {
		return &snapshot->copy_b;
	}
	return NULL;
}

static bool metadata_record_crc_valid(const uint8_t record[20])
{
	if (record == NULL) {
		return false;
	}

	uint16_t stored_crc =
		((uint16_t)record[SLOT_OFF_CRC16_HI] << 8) |
		record[SLOT_OFF_CRC16_LO];
	uint16_t calculated_crc =
		Calc_CRC16((uint8_t*)&record[2], SLOT_RECORD_DATA_LEN);
	return stored_crc == calculated_crc;
}

static bool metadata_record_matches_slot(uint8_t slot_id,
                                         const uint8_t record[20])
{
	fw_slot_desc_t slot;
	if (record == NULL ||
		!fw_slot_get(slot_id, &slot) ||
		(slot.role != FW_SLOT_ROLE_GOLDEN && slot.role != FW_SLOT_ROLE_OTA)) {
		return false;
	}

	uint32_t flash_addr = read_u32_le(record, SLOT_OFF_FLASH_ADDR);
	uint32_t image_size = read_u32_le(record, SLOT_OFF_IMAGE_SIZE);

	return record[SLOT_OFF_IMAGE_INDEX] == slot_id &&
		   record[SLOT_OFF_BANK_ID] == slot.bank_id &&
		   flash_addr == slot.base &&
		   image_size >= 8u && image_size <= slot.capacity;
}

static uint8_t metadata_slot_flags(uint8_t slot_id,
                                   const uint8_t record[20],
                                   uint8_t active_idx)
{
	uint8_t flags = 0u;
	fw_slot_desc_t slot;

	if (fw_slot_get(slot_id, &slot)) {
		if (slot.role == FW_SLOT_ROLE_GOLDEN) {
			flags |= MD_FLAG_GOLDEN;
		}
		if (slot.role == FW_SLOT_ROLE_OTA) {
			flags |= MD_FLAG_OTA;
		} else {
			flags |= MD_FLAG_PROTECTED;
		}
	}

	if (slot_id == active_idx) {
		flags |= MD_FLAG_ACTIVE;
	}

	if (metadata_record_crc_valid(record)) {
		flags |= MD_FLAG_RECORD_CRC_OK;

		if (metadata_record_matches_slot(slot_id, record)) {
			flags |= MD_FLAG_INSTALLED;
		}
		if (record[SLOT_OFF_NEW_META] == MD_META_PENDING) {
			flags |= MD_FLAG_PENDING;
		}
		if (record[SLOT_OFF_NEW_META] == MD_META_CONFIRMED &&
			record[SLOT_OFF_BOOT_FB] == MD_BOOTED_OK) {
			flags |= MD_FLAG_BOOTED_OK;
		}
	}

	return flags;
}

static void report_put_u8(uint8_t* data, uint16_t* offset, uint8_t value)
{
	data[(*offset)++] = value;
}

static void report_put_u16_le(uint8_t* data, uint16_t* offset, uint16_t value)
{
	data[(*offset)++] = (uint8_t)(value & 0xFFu);
	data[(*offset)++] = (uint8_t)((value >> 8) & 0xFFu);
}

static void report_put_u32_le(uint8_t* data, uint16_t* offset, uint32_t value)
{
	data[(*offset)++] = (uint8_t)(value & 0xFFu);
	data[(*offset)++] = (uint8_t)((value >> 8) & 0xFFu);
	data[(*offset)++] = (uint8_t)((value >> 16) & 0xFFu);
	data[(*offset)++] = (uint8_t)((value >> 24) & 0xFFu);
}

static void init_function_report(UART_OUT_OBC_msg* msg,
                                 const PUS_TC_header_t* PUS_TC_h)
{
	*msg = (UART_OUT_OBC_msg){0};
	msg->PUS_HEADER_PRESENT = 1u;
	msg->PUS_SOURCE_ID = PUS_TC_h->source_id;
	msg->SERVICE_ID = FUNCTION_MANAGEMNET_ID;
	msg->SUBTYPE_ID = FM_FUNCTION_REPORT;
}

static void send_version_report(const PUS_TC_header_t* PUS_TC_h)
{
	UART_OUT_OBC_msg msg;
	init_function_report(&msg, PUS_TC_h);

	msg.TM_data[0] = GET_VERSION;
	msg.TM_data[1] = FW_VERSION_MAJOR;
	msg.TM_data[2] = FW_VERSION_MINOR;
	msg.TM_data[3] = FW_VERSION_PATCH;
	msg.TM_data[4] = g_boot_confirmed;
	msg.TM_data_len = 5u;

	xQueueSend(UART_OBC_Out_Queue, &msg, portMAX_DELAY);
}

static void send_metadata_summary(const SPP_header_t* SPP_h,
                                  const PUS_TC_header_t* PUS_TC_h,
                                  const fram_meta_snapshot_t* snapshot)
{
	UART_OUT_OBC_msg msg;
	init_function_report(&msg, PUS_TC_h);

	const fram_meta_block_t* selected = metadata_selected_block(snapshot);
	uint8_t active_idx = selected != NULL ? selected->active_idx : 0u;
	bool valid_a = snapshot->copy_a_status == FRAMMETA_COPY_VALID;
	bool valid_b = snapshot->copy_b_status == FRAMMETA_COPY_VALID;
	uint8_t overall_status = MD_OVERALL_UNAVAILABLE;
	if (valid_a && valid_b) {
		overall_status = MD_OVERALL_OK;
	} else if (valid_a || valid_b) {
		overall_status = MD_OVERALL_DEGRADED;
	}

	uint16_t offset = 0u;
	report_put_u8(msg.TM_data, &offset, GET_BOOT_METADATA);
	report_put_u8(msg.TM_data, &offset, MD_REPORT_FORMAT_VERSION);
	report_put_u8(msg.TM_data, &offset, MD_REPORT_SUMMARY);
	report_put_u16_le(msg.TM_data, &offset, SPP_h->packet_sequence_count);
	report_put_u8(msg.TM_data, &offset, overall_status);
	report_put_u8(msg.TM_data, &offset, (uint8_t)snapshot->copy_a_status);
	report_put_u8(msg.TM_data, &offset, (uint8_t)snapshot->copy_b_status);
	report_put_u8(msg.TM_data, &offset, (uint8_t)snapshot->selected_copy);
	report_put_u16_le(msg.TM_data, &offset, snapshot->copy_a.seq);
	report_put_u16_le(msg.TM_data, &offset, snapshot->copy_b.seq);
	report_put_u16_le(msg.TM_data, &offset,
					 selected != NULL ? selected->version : 0u);
	report_put_u8(msg.TM_data, &offset, active_idx);
	report_put_u8(msg.TM_data, &offset, fwup_get_executing_slot());
	report_put_u8(msg.TM_data, &offset, g_boot_confirmed);
	report_put_u8(msg.TM_data, &offset, NUM_SLOTS);

	for (uint8_t slot_id = 1u; slot_id <= NUM_SLOTS; slot_id++) {
		const uint8_t* record = selected != NULL
			? selected->rec[slot_id - 1u]
			: NULL;
		report_put_u8(msg.TM_data, &offset, slot_id);
		report_put_u8(msg.TM_data, &offset,
					  metadata_slot_flags(slot_id, record, active_idx));
		report_put_u8(msg.TM_data, &offset,
					  record != NULL
					  ? record[SLOT_OFF_BOOT_COUNTER]
					  : 0u);
		report_put_u8(msg.TM_data, &offset,
					  record != NULL ? record[SLOT_OFF_ERROR_CODE] : 0u);
	}

	msg.TM_data_len = offset;
	xQueueSend(UART_OBC_Out_Queue, &msg, portMAX_DELAY);
}

static void send_metadata_slot_detail(const PUS_TC_header_t* PUS_TC_h,
                                      const fram_meta_snapshot_t* snapshot,
                                      uint8_t slot_id)
{
	UART_OUT_OBC_msg msg;
	init_function_report(&msg, PUS_TC_h);

	const fram_meta_block_t* selected = metadata_selected_block(snapshot);
	const uint8_t* record = selected->rec[slot_id - 1u];
	fw_slot_desc_t slot = {0};
	(void)fw_slot_get(slot_id, &slot);

	uint16_t stored_record_crc =
		((uint16_t)record[SLOT_OFF_CRC16_HI] << 8) |
		record[SLOT_OFF_CRC16_LO];
	bool record_crc_valid = metadata_record_crc_valid(record);

	uint16_t offset = 0u;
	report_put_u8(msg.TM_data, &offset, GET_BOOT_METADATA);
	report_put_u8(msg.TM_data, &offset, MD_REPORT_FORMAT_VERSION);
	report_put_u8(msg.TM_data, &offset, MD_REPORT_SLOT_DETAIL);
	report_put_u8(msg.TM_data, &offset, slot_id);
	report_put_u8(msg.TM_data, &offset, (uint8_t)slot.role);
	report_put_u8(msg.TM_data, &offset,
				  metadata_slot_flags(slot_id, record, selected->active_idx));
	report_put_u8(msg.TM_data, &offset, record[SLOT_OFF_BANK_ID]);
	report_put_u32_le(msg.TM_data, &offset,
				   read_u32_le(record, SLOT_OFF_FLASH_ADDR));
	report_put_u32_le(msg.TM_data, &offset,
				   read_u32_le(record, SLOT_OFF_IMAGE_SIZE));
	report_put_u32_le(msg.TM_data, &offset,
				   read_u32_le(record, SLOT_OFF_IMAGE_CRC32));
	report_put_u8(msg.TM_data, &offset, record[SLOT_OFF_BOOT_COUNTER]);
	report_put_u8(msg.TM_data, &offset, record[SLOT_OFF_BOOT_FB]);
	report_put_u8(msg.TM_data, &offset, record[SLOT_OFF_NEW_META]);
	report_put_u8(msg.TM_data, &offset, record[SLOT_OFF_ERROR_CODE]);
	report_put_u16_le(msg.TM_data, &offset, stored_record_crc);
	report_put_u8(msg.TM_data, &offset, record_crc_valid ? 1u : 0u);
	report_put_u8(msg.TM_data, &offset,
				  selected->active_idx == slot_id ? 1u : 0u);

	msg.TM_data_len = offset;
	xQueueSend(UART_OBC_Out_Queue, &msg, portMAX_DELAY);
}

static TM_Err_Codes send_boot_metadata_report(
	const SPP_header_t* SPP_h,
	const PUS_TC_header_t* PUS_TC_h,
	const PUS_8_msg_unpacked* request)
{
	if (request->N_args > 1u) {
		return INVALID_PLENGTH;
	}

	if (request->N_args == 1u &&
		(request->img_id < 1u || request->img_id > NUM_SLOTS)) {
		return UNDEFINED_ID;
	}

	fram_meta_snapshot_t snapshot;
	bool has_selected_copy = FRAMMETA_ReadSnapshot(&snapshot);

	if (request->N_args == 0u) {
		send_metadata_summary(SPP_h, PUS_TC_h, &snapshot);
		return NO_ERROR;
	}

	if (!has_selected_copy) {
		return FRAM_META_FAIL;
	}

	send_metadata_slot_detail(PUS_TC_h, &snapshot, request->img_id);
	return NO_ERROR;
}
 
bool PUS_8_check_FPGA_msg_format(uint8_t* msg, uint8_t msg_len) {
    bool result = false;
    if (msg[0] == LANGMUIR_READBACK_PREAMBLE_0) {
        if (msg[1] == LANGMUIR_READBACK_PREAMBLE_1) {
            if (msg[(msg_len - 1)] == LANGMUIR_READBACK_POSTAMBLE) {
                result = true;
            }
        }
    }
    return result;
}

TM_Err_Codes PUS_8_unpack_msg(PUS_8_msg *pus8_msg_received, PUS_8_msg_unpacked* pus8_msg_unpacked)
{
	uint8_t* data_interator = pus8_msg_received->data;
	uint8_t* data_end = pus8_msg_received->data + pus8_msg_received->data_size;

	// Check at least 2 bytes available: func_id and N_args
	if ((data_end - data_interator) < 2)
		return INVALID_PLENGTH;

	pus8_msg_unpacked->func_id = *data_interator++;
	pus8_msg_unpacked->N_args = *data_interator++;

	for(int i = 0; i < pus8_msg_unpacked->N_args; i++) {

		if ((data_end - data_interator) < 1)
			return INVALID_PLENGTH; // Need at least arg_ID

		uint8_t arg_ID = *data_interator++;

		switch(arg_ID) {
			case TABLE_ID_ARG_ID:
				if ((data_end - data_interator) < 1)
					return INVALID_PLENGTH;
					
				uint8_t fpga_probe_id = *data_interator & 0x0F;
				uint8_t fram_table_id = (*data_interator >> 4) & 0x0F;
				pus8_msg_unpacked->FRAM_Table_ID = fram_table_id;
				pus8_msg_unpacked->FPGA_Probe_ID = fpga_probe_id;
				data_interator++;
				break;
			case STEP_ID_ARG_ID:
				if ((data_end - data_interator) < 1)
					return INVALID_PLENGTH;
				pus8_msg_unpacked->step_ID = *data_interator++;
				break;
			case VOL_LVL_ARG_ID:
				if ((data_end - data_interator) < sizeof(pus8_msg_unpacked->voltage_level))
					return INVALID_PLENGTH;
				memcpy((uint8_t*)&pus8_msg_unpacked->voltage_level, data_interator, sizeof(pus8_msg_unpacked->voltage_level));
				data_interator += sizeof(pus8_msg_unpacked->voltage_level);
				break;
			case N_STEPS_ARG_ID:
				if ((data_end - data_interator) < 1)
					return INVALID_PLENGTH;
				pus8_msg_unpacked->N_steps = *data_interator++;
				break;
			case N_SKIP_ARG_ID:
				if ((data_end - data_interator) < sizeof(pus8_msg_unpacked->N_skip))
					return INVALID_PLENGTH;
				memcpy((uint8_t*)&pus8_msg_unpacked->N_skip, data_interator, sizeof(pus8_msg_unpacked->N_skip));
				data_interator += sizeof(pus8_msg_unpacked->N_skip);
				break;
			case N_F_ARG_ID:
				if ((data_end - data_interator) < sizeof(pus8_msg_unpacked->N_f))
					return INVALID_PLENGTH;
				memcpy((uint8_t*)&pus8_msg_unpacked->N_f, data_interator, sizeof(pus8_msg_unpacked->N_f));
				data_interator += sizeof(pus8_msg_unpacked->N_f);
				break;
			case N_POINTS_ARG_ID:
				if ((data_end - data_interator) < sizeof(pus8_msg_unpacked->N_points))
					return INVALID_PLENGTH;
				memcpy((uint8_t*)&pus8_msg_unpacked->N_points, data_interator, sizeof(pus8_msg_unpacked->N_points));
				data_interator += sizeof(pus8_msg_unpacked->N_points);
				break;
			case N_SAMPLES_PER_STEP_ARG_ID:
				if ((data_end - data_interator) < sizeof(pus8_msg_unpacked->N_samples_per_step))
					return INVALID_PLENGTH;
				memcpy((uint8_t*)&pus8_msg_unpacked->N_samples_per_step, data_interator, sizeof(pus8_msg_unpacked->N_samples_per_step));
				data_interator += sizeof(pus8_msg_unpacked->N_samples_per_step);
				break;
			case IMG_ID_ARG_ID:
				if ((data_end - data_interator) < 1) {
					return INVALID_PLENGTH;
				}
				pus8_msg_unpacked->img_id = *data_interator++;
				break;

			case IMG_SIZE_ARG_ID:
				if ((data_end - data_interator) < 4) {
					return INVALID_PLENGTH;
				}
				memcpy(&pus8_msg_unpacked->img_size, data_interator, 4u);
				data_interator += 4u;
				break;

			case IMG_CRC32_ARG_ID:
				if ((data_end - data_interator) < 4) {
					return INVALID_PLENGTH;
				}
				memcpy(&pus8_msg_unpacked->img_crc32, data_interator, 4u);
				data_interator += 4u;
				break;

			case IMG_ADDR_ARG_ID:
				if ((data_end - data_interator) < 4) {
					return INVALID_PLENGTH;
				}
				memcpy(&pus8_msg_unpacked->img_addr, data_interator, 4u);
				data_interator += 4u;
				break;

			case SRAM_DEST_ADDR_ARG_ID:
				if ((data_end - data_interator) < 4) {
					return INVALID_PLENGTH;
				}
				memcpy(&pus8_msg_unpacked->sram_dest_addr,
					   data_interator,
					   4u);
				data_interator += 4u;
				break;

			case BANK_ID_ARG_ID:
				if ((data_end - data_interator) < 1) {
					return INVALID_PLENGTH;
				}
				pus8_msg_unpacked->bank_id = *data_interator++;
				break;

			case IMG_DATA_ARG_ID:
			{
				uint16_t remaining = (uint16_t)(data_end - data_interator);
				if (remaining > PUS_8_MAX_DATA_LEN) {
					return INVALID_PLENGTH;
				}
				pus8_msg_unpacked->img_data_len = remaining;
				memcpy(pus8_msg_unpacked->img_data,
					   data_interator,
					   remaining);
				data_interator = data_end;
				break;
			}

			default:
				return UNDEFINED_PARAM_ID;
				break;
		}
	}
	return NO_ERROR;
}

void PUS_8_copy_table_FRAM_to_FPGA(uint8_t fram_table_id, uint8_t fpga_probe_id) {

    uint8_t msg[64] = {0};
	msg[0] = FPGA_MSG_PREAMBLE_0;
	msg[1] = FPGA_MSG_PREAMBLE_1;
	msg[2] = FPGA_SET_SWT_VOL_LVL;
	msg[3] = fpga_probe_id;
	msg[7] = FPGA_MSG_POSTAMBLE;

    for(int i = 0; i < 256; i++) {
        uint8_t step_id = i;
        uint16_t value = read_sweep_table_value_FRAM(fram_table_id, step_id);

		msg[4] = step_id;
		msg[5] = ((uint8_t*)(&value))[0]; // MSB
		msg[6] = ((uint8_t*)(&value))[1]; // LSB
		if (HAL_UART_Transmit(&huart5, msg, 8, 100)!= HAL_OK) {
			HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
		}
		osDelay(5);
    }
}


TM_Err_Codes PUS_8_perform_function(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_TC_h , PUS_8_msg_unpacked* pus8_msg_unpacked)
{

	switch(pus8_msg_unpacked->func_id)
	{
		case FPGA_SET_SWT_VOL_LVL:
		{
			// if target is MCU
			if(pus8_msg_unpacked->FPGA_Probe_ID == 0 && pus8_msg_unpacked->FRAM_Table_ID != 0)
			{
				if(pus8_msg_unpacked->FRAM_Table_ID < 1 || pus8_msg_unpacked->FRAM_Table_ID > 8)
					return UNDEFINED_PARAM_ID; // There are only 8 tables available in the FRAM

				save_sweep_table_value_FRAM(pus8_msg_unpacked->FRAM_Table_ID,
											pus8_msg_unpacked->step_ID,
											pus8_msg_unpacked->voltage_level);
			}
			// if target is FPGA
			else if (pus8_msg_unpacked->FRAM_Table_ID == 0 && pus8_msg_unpacked->FPGA_Probe_ID != 0)
			{
				if(pus8_msg_unpacked->FPGA_Probe_ID < 1 || pus8_msg_unpacked->FPGA_Probe_ID > 2)
					return UNDEFINED_PARAM_ID; // There are only 2 tables available in the FPGA 

				uint8_t msg[64] = {0};
				uint8_t msg_cnt = 0;

				msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
				msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
				msg[msg_cnt++] = FPGA_SET_SWT_VOL_LVL;
				msg[msg_cnt++] = pus8_msg_unpacked->FPGA_Probe_ID;
				msg[msg_cnt++] = pus8_msg_unpacked->step_ID;
				msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->voltage_level))[0]; // MSB
				msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->voltage_level))[1]; // LSB 

				msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

				if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
					HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
					return DEV_CPDU_EXEC_FAIL;
				}

			}
			// target is both -- copy table from FRAM to FPGA command --
			else if (pus8_msg_unpacked->FRAM_Table_ID != 0 && pus8_msg_unpacked->FPGA_Probe_ID != 0)
			{
				if(pus8_msg_unpacked->FRAM_Table_ID < 1 || pus8_msg_unpacked->FRAM_Table_ID > 8)
					return UNDEFINED_ID;

				if(pus8_msg_unpacked->FPGA_Probe_ID < 1 || pus8_msg_unpacked->FPGA_Probe_ID > 2)
					return UNDEFINED_ID;

				uint8_t FRAM_Table_ID = pus8_msg_unpacked->FRAM_Table_ID;
				uint8_t FPGA_Probe_ID = pus8_msg_unpacked->FPGA_Probe_ID;
				PUS_8_copy_table_FRAM_to_FPGA(FRAM_Table_ID, FPGA_Probe_ID);

				break;
			}

			break;
		}

		case FPGA_GET_SWT_VOL_LVL:
		{
			// target is MUC
			if(pus8_msg_unpacked->FPGA_Probe_ID == 0 || pus8_msg_unpacked->FRAM_Table_ID != 0)
			{
				if(pus8_msg_unpacked->FRAM_Table_ID < 1 || pus8_msg_unpacked->FRAM_Table_ID > 8)
					return UNDEFINED_PARAM_ID; // There are only 8 sweep tables available in the FRAM

				uint16_t step_voltage = read_sweep_table_value_FRAM(pus8_msg_unpacked->FRAM_Table_ID,
																	pus8_msg_unpacked->step_ID);
				UART_OUT_OBC_msg msg = {0};

				msg.PUS_HEADER_PRESENT	= 0;
				
				msg.TM_data[0] = FPGA_GET_SWT_VOL_LVL;
				msg.TM_data[1] = pus8_msg_unpacked->FRAM_Table_ID+0xF;
				msg.TM_data[2] = pus8_msg_unpacked->step_ID;
				msg.TM_data[3] = (uint8_t)(step_voltage & 0xFF); // MSB
				msg.TM_data[4] = (uint8_t)((step_voltage >> 8) & 0xFF);        // LSB	
				msg.TM_data_len			= 5;

				xQueueSend(UART_OBC_Out_Queue, &msg, portMAX_DELAY);
			}
			// target is FPGA
			else if(pus8_msg_unpacked->FRAM_Table_ID == 0 || pus8_msg_unpacked->FPGA_Probe_ID != 0)
			{
				if(pus8_msg_unpacked->FPGA_Probe_ID < 1 || pus8_msg_unpacked->FPGA_Probe_ID > 2)
					return UNDEFINED_PARAM_ID; // There are only 2 sweep tables available in the FPGA RAM

				uint8_t msg[64] = {0};
				uint8_t msg_cnt = 0;

				msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
				msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
				msg[msg_cnt++] = FPGA_GET_SWT_VOL_LVL;
				msg[msg_cnt++] = pus8_msg_unpacked->FPGA_Probe_ID;
				msg[msg_cnt++] = pus8_msg_unpacked->step_ID;
				msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

				memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
				memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

				UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_VOL_LVL;
				UART_FPGA_OBC_Tx_Buffer[1] = pus8_msg_unpacked->FPGA_Probe_ID;
				UART_FPGA_OBC_Tx_Buffer[2] = pus8_msg_unpacked->step_ID;

				if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
					HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
					return DEV_CPDU_EXEC_FAIL;
				}
			}
			break;
		}

		case FPGA_SWT_ACTIVATE_SWEEP:
		{
//			Current_Global_Device_State = CB_MODE;
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_SWT_ACTIVATE_SWEEP;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			Sweep_Bias_Data_counter = 0;
			Old_Sweep_Bias_Data_counter = 1;

			memset((uint8_t*)Sweep_Bias_Mode_Data, 0, sizeof(Sweep_Bias_Mode_Data));

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			// VERY IMPORTANT TO CLEAR THE INTERRUPTS
			__HAL_GPIO_EXTI_CLEAR_IT(FPGA_BUF_INT_Pin);
			NVIC_ClearPendingIRQ(EXTI9_5_IRQn);

			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

			break;
		}

		case FPGA_EN_CB_MODE:
		{
			Current_Global_Device_State = CB_MODE;

			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_EN_CB_MODE;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
			memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

			UART_FPGA_OBC_Tx_Buffer[0] = FPGA_EN_CB_MODE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			break;
		}
		case FPGA_DIS_CB_MODE:
		{
			Current_Global_Device_State = NORMAL_MODE;

			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_DIS_CB_MODE;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			// CB flush
			osSignalSet(UART_FPGA_INHandle, 0x04);
			break;
		}

		case FPGA_SET_CB_VOL_LVL:
		{
			if(pus8_msg_unpacked->FPGA_Probe_ID < 1 || pus8_msg_unpacked->FPGA_Probe_ID > 2)
				return UNDEFINED_PARAM_ID; // There are only 2 probes

			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_SET_CB_VOL_LVL;
			msg[msg_cnt++] = pus8_msg_unpacked->FPGA_Probe_ID;
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->voltage_level))[0]; // MSB
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->voltage_level))[1]; // LSB
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}
			break;
		}

		case FPGA_GET_CB_VOL_LVL:
		{
			if(pus8_msg_unpacked->FPGA_Probe_ID < 1 || pus8_msg_unpacked->FPGA_Probe_ID > 2)
				return UNDEFINED_PARAM_ID; // There are only 2 probes

			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_GET_CB_VOL_LVL;
			msg[msg_cnt++] = pus8_msg_unpacked->FPGA_Probe_ID;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
			memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

			//UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_CB_VOL_LVL;
			//UART_FPGA_OBC_Tx_Buffer[1] = pus8_msg_unpacked->probe_ID;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			break;
		}

		case FPGA_SET_SWT_STEPS:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_SET_SWT_STEPS;
			msg[msg_cnt++] = pus8_msg_unpacked->N_steps;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}
			break;
		}

		case FPGA_GET_SWT_STEPS:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_GET_SWT_STEPS;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
			memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

			UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_STEPS;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			break;
		}

		case FPGA_SET_SWT_SAMPLES_PER_STEP:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_SET_SWT_SAMPLES_PER_STEP;
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_samples_per_step))[0]; // MSB
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_samples_per_step))[1]; // LSB
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}
			break;
		}

		case FPGA_GET_SWT_SAMPLES_PER_STEP:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_GET_SWT_SAMPLES_PER_STEP;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
			memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

			UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLES_PER_STEP;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			break;
		}

		case FPGA_SET_SWT_SAMPLE_SKIP:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_SET_SWT_SAMPLE_SKIP;
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_skip))[0]; // MSB
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_skip))[1]; // LSB
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}
			break;
		}

		case FPGA_GET_SWT_SAMPLE_SKIP:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_GET_SWT_SAMPLE_SKIP;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
			memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

			UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLE_SKIP;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			break;
		}

		case FPGA_SET_SWT_SAMPLES_PER_POINT:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_SET_SWT_SAMPLES_PER_POINT;
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_f))[0]; // MSB
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_f))[1]; // LSB
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}
			break;
		}

		case FPGA_GET_SWT_SAMPLES_PER_POINT:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_GET_SWT_SAMPLES_PER_POINT;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
			memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

			UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLES_PER_POINT;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			break;
		}

		case FPGA_SET_SWT_NPOINTS:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_SET_SWT_NPOINTS;
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_points))[0]; // MSB
			msg[msg_cnt++] = ((uint8_t*)(&pus8_msg_unpacked->N_points))[1]; // LSB
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}
			break;
		}

		case FPGA_GET_SWT_NPOINTS:
		{
			uint8_t msg[64] = {0};
			uint8_t msg_cnt = 0;

			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_0;
			msg[msg_cnt++] = FPGA_MSG_PREAMBLE_1;
			msg[msg_cnt++] = FPGA_GET_SWT_NPOINTS;
			msg[msg_cnt++] = FPGA_MSG_POSTAMBLE;

			memset(UART_FPGA_Rx_Buffer, 0, sizeof(UART_FPGA_Rx_Buffer));
			memset(UART_FPGA_OBC_Tx_Buffer, 0, sizeof(UART_FPGA_OBC_Tx_Buffer));

			UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_NPOINTS;

			if (HAL_UART_Transmit(&huart5, msg, msg_cnt, 100)!= HAL_OK) {
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
				return DEV_CPDU_EXEC_FAIL;
			}

			break;
		}


		case REBOOT_DEVICE:
		{
			/* A software reset preserves a confirmed image; an IWDG reset can
			 * trigger golden-image recovery. This path sends its own completion
			 * acknowledgement because it never returns to the PUS-8 task. */
			PUS_1_send_succ_comp(SPP_h, PUS_TC_h);
			osDelay(100u);
			NVIC_SystemReset();
			for (;;) {
				/* Defensive: reset should never return. */
			}
		}

		case JUMP_TO_IMAGE:
		{
			fram_meta_block_t blk;
			uint8_t img_id = pus8_msg_unpacked->img_id;

			if (!FRAMMETA_Load(&blk, NULL)) {
				return FRAM_META_FAIL;
			}

			if (img_id < 1u || img_id > NUM_SLOTS) {
				return IMAGE_NOT_BOOTABLE;
			}

			const uint8_t* record = blk.rec[img_id - 1u];
			if (!fwup_boot_candidate_valid(img_id,
										 pus8_msg_unpacked->img_addr,
										 record)) {
				return IMAGE_NOT_BOOTABLE;
			}

			if (!FRAMMETA_ActivateImage(img_id)) {
				return FRAM_META_FAIL;
			}

			PUS_1_send_succ_comp(SPP_h, PUS_TC_h);
			osDelay(100u);
			NVIC_SystemReset();

			for (;;) {
				/* Defensive: reset should never return. */
			}
		}
		case FWUP_BEGIN:
		{
			fw_slot_desc_t target;

			if (!fw_slot_get(pus8_msg_unpacked->img_id, &target) ||
				target.role != FW_SLOT_ROLE_OTA) {
				return FWUP_SLOT_NOT_WRITABLE;
			}

			if (pus8_msg_unpacked->img_size < 8u ||
				pus8_msg_unpacked->img_size > target.capacity) {
				return IMG_SIZE_DISCREP;
			}

			if (pus8_msg_unpacked->img_size > SRAM_FW_STAGING_SIZE) {
				return SRAM_IMG_DISCREP;
			}

			uint8_t executing_bank;
			if (!fwup_get_executing_bank_id(&executing_bank)) {
				return BAD_STATE;
			}

			if (target.bank_id == executing_bank) {
				return BAD_STATE;
			}

			/* Do not replace an active session until every request field passes. */
			fwup_state = FWUP_STATE_STAGING;
			fwup_target_slot = target.slot_id;
			fwup_expected_size = pus8_msg_unpacked->img_size;
			fwup_expected_crc32 = pus8_msg_unpacked->img_crc32;
			fwup_staged_extent = 0u;
			fwup_target_address = target.base;
			fwup_target_bank = target.bank_id;

			break;
		}

		case FWUP_SRAM_WRITE:
		{
			if (fwup_state != FWUP_STATE_STAGING) {
				return UPDATE_INACTIVE;
			}

			uint32_t destination = pus8_msg_unpacked->sram_dest_addr;
			uint16_t chunk_length = pus8_msg_unpacked->img_data_len;

			if (chunk_length == 0u) {
				return INVALID_PLENGTH;
			}

			if (destination < SRAM_FW_STAGING_BASE) {
				return SRAM_BUFFER_FAIL;
			}

			uint32_t staging_offset = destination - SRAM_FW_STAGING_BASE;
			if (staging_offset > SRAM_FW_STAGING_SIZE ||
				(uint32_t)chunk_length >
					SRAM_FW_STAGING_SIZE - staging_offset) {
				return SRAM_BUFFER_FAIL;
			}

			if (staging_offset > fwup_expected_size ||
				(uint32_t)chunk_length >
					fwup_expected_size - staging_offset) {
				return SRAM_IMG_DISCREP;
			}

			memcpy(&g_fw_staging[staging_offset],
				   pus8_msg_unpacked->img_data,
				   chunk_length);

			uint32_t staged_end = staging_offset + chunk_length;
			if (staged_end > fwup_staged_extent) {
				fwup_staged_extent = staged_end;
			}

			break;
		}

		case FWUP_FLASH:
		{
			if (fwup_state != FWUP_STATE_STAGING) {
				return UPDATE_INACTIVE;
			}

			if (fwup_staged_extent != fwup_expected_size) {
				return IMG_INCOMPLETE;
			}

			uint32_t staged_crc =
				crc32_calc(g_fw_staging, fwup_expected_size);
			if (staged_crc != fwup_expected_crc32) {
				return CS_DISCREP;
			}

			/* Placement/vector preflight happens before any destructive flash
			 * operation and uses the real image extent, not sector capacity. */
			if (!fwup_staged_vectors_valid()) {
				return IMAGE_NOT_BOOTABLE;
			}

			/* Preserve the existing command fields for protocol compatibility,
			 * but require them to match the placement accepted by FWUP_BEGIN. */
			if (pus8_msg_unpacked->img_id != fwup_target_slot ||
				pus8_msg_unpacked->img_addr != fwup_target_address ||
				pus8_msg_unpacked->bank_id != fwup_target_bank) {
				return FWUP_SLOT_NOT_WRITABLE;
			}

			uint32_t flash_addr = fwup_target_address;

			/* Retain an explicit range check before the destructive erase. */
			if (!flash_range_is_within_flash(flash_addr, fwup_expected_size)) {
				return DEV_CPDU_EXEC_FAIL;
			}

			if (FLASHIF_EraseRange(flash_addr, fwup_expected_size) != HAL_OK) {
				return DEV_CPDU_EXEC_FAIL;
			}

			/* Make the staged bytes visible to flash programming. CMSIS cache
			 * maintenance requires a 32-byte-aligned extent. */
			uint32_t aligned_size = (fwup_expected_size + 31u) & ~31u;
			SCB_CleanDCache_by_Addr((uint32_t*)g_fw_staging, (int32_t)aligned_size);

			if (FLASHIF_ProgramBuffer((uint32_t*)flash_addr,
								  g_fw_staging,
								  fwup_expected_size) != FLASHIF_OK) {
				return DEV_CPDU_EXEC_FAIL;
			}

			/* Discard every cache layer that could hide programmed flash data. */
			SCB_InvalidateICache();
			SCB_InvalidateDCache_by_Addr((uint32_t*)flash_addr, (int32_t)aligned_size);
			__HAL_FLASH_ART_DISABLE();
			__HAL_FLASH_ART_RESET();
			__HAL_FLASH_ART_ENABLE();

			uint32_t flash_crc =
				crc32_calc((const uint8_t*)flash_addr, fwup_expected_size);
			if (flash_crc != fwup_expected_crc32) {
				return FLASH_CS_DISCREP;
			}

			/* Publish the image only after flash readback succeeds. A failure
			 * deliberately leaves the staging session active for retry. */
			if (!FRAMMETA_SetImageInfo(fwup_target_slot,
									flash_addr,
									fwup_expected_size,
									fwup_expected_crc32,
									fwup_target_bank)) {
				return FRAM_META_FAIL;
			}
			fwup_state = FWUP_STATE_IDLE;
			break;
		}

		case GET_BOOT_METADATA:
		{
			TM_Err_Codes report_result =
				send_boot_metadata_report(SPP_h, PUS_TC_h, pus8_msg_unpacked);
			if (report_result != NO_ERROR) {
				return report_result;
			}
			break;
		}

		case GET_VERSION:
		{
			if (pus8_msg_unpacked->N_args != 0u) {
				return INVALID_PLENGTH;
			}
			send_version_report(PUS_TC_h);
			break;
		}
		//-----------------------------------------------------------------------------------------


		default:
			return UNKNOWN_FUNCTION_ID; 
	}

	return NO_ERROR;
}


// Function Management PUS service 8
TM_Err_Codes PUS_8_handle_FM_TC(SPP_header_t* SPP_header , PUS_TC_header_t* PUS_TC_header, uint8_t* data, uint8_t data_size) {

	if(data_size < 2)
	{
		return INVALID_PLENGTH;
	}
	if (data_size > PUS_8_MAX_DATA_LEN)
	{
		return INVALID_PLENGTH;   // guard the memcpy into PUS_8_msg.data below
	}
	if (Current_Global_Device_State != NORMAL_MODE)
	{
		if(Current_Global_Device_State != CB_MODE)
			return BAD_STATE;
		else if(Current_Global_Device_State == CB_MODE && *data != FPGA_DIS_CB_MODE)
			return BAD_STATE;
	}
	if (SPP_header == NULL || PUS_TC_header == NULL) {
		return ILLEGAL_VERSION;
	}

	switch (PUS_TC_header->message_subtype_id) {
		case FM_PERFORM_FUNCTION:
			break;
		default:
			return UNKNOWN_TYPE_SUBTYPE;  
	}

	PUS_1_send_succ_acc(SPP_header, PUS_TC_header);

	PUS_8_msg pus8_msg_to_send;
	pus8_msg_to_send.SPP_header = *SPP_header;
	pus8_msg_to_send.PUS_TC_header = *PUS_TC_header;
	memcpy(pus8_msg_to_send.data, data, data_size);
	pus8_msg_to_send.data_size = data_size;

	if (xQueueSend(PUS_8_Queue, &pus8_msg_to_send, 0) != pdPASS) {
		PUS_1_send_fail_comp(SPP_header, PUS_TC_header, BAD_STATE);
	}

    return NO_ERROR;
}
