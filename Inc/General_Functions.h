/*
 * General_Functions.h
 *
 *  Created on: Feb 11, 2025
 *      Author: sergi
 */
#include "Space_Packet_Protocol.h"
#include "PUS.h"

#ifndef GENERAL_FUNCTIONS_H_
#define GENERAL_FUNCTIONS_H_

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"

#define CB_MODE_BUFFERED_MEASUREMENTS 40
#define FPGA_FRAME_LEN 12

extern uint8_t Constant_Bias_Mode_Buffer[2][CB_MODE_BUFFERED_MEASUREMENTS*8];
extern uint8_t Buffer_Index;
extern uint8_t Measurement_Index;
extern uint8_t send_buffered_data;
extern uint16_t cb_expected_fpga_cnt;
extern uint8_t cb_cnt_valid;


// TODO: enum could save them as 32 bits, maybe better to clarify they are 16 bits. (Now it works even without)
// typedef enum: uint16_t {
typedef enum {


	NO_ERROR                    = 0x0000,   

    // --- Generic CCSDS / PUS validation ---
    ILLEGAL_VERSION             = 0x0100, // Invalid CCSDS packet version
    INVALID_PLENGTH             = 0x0106, // TC length out of allowed range
    UNKNOWN_TYPE_SUBTYPE        = 0x0109, // Unknown service type/subtype
	COBS_FRAME_ERROR			= 0x010A, // COBS frame not valid
	SPP_HEADER_ERROR			= 0x010B, // SPP header not valid
    CS_DISCREP                  = 0x010C, // CRC mismatch


    // --- System state and execution ---
    BAD_STATE                   = 0x0147, // Command cannot be executed in current mode
    DEV_CPDU_EXEC_FAIL          = 0x0206, // Command execution or device transmission failed
    UNKNOWN_FUNCTION_ID         = 0x0809, // Unknown Function Management ID

    // --- Housekeeping service ---
    HK_INVALID_SID              = 0x0300, // SID out of allowed range
    HK_INVALID_COLL_INT         = 0x0301, // Invalid collection interval 
    HK_MAX_SID_NB_EXCEEDED      = 0x0304, // Too many SIDs defined
	HKD_TM_SIZE_EXCEEDED        = 0x0303, // HK packet size exceeds TM buffer

    // --- Memory / FRAM access ---
    MEM_INVALID_ADDRESS         = 0x0600, // Invalid memory address
    MEM_HW_ERROR                = 0x0605, // HW/FRAM access failure

    // --- Distribution / undefined parameter ---
    UNDEFINED_PARAM_ID          = 0x0142, // Undefined parameter ID
    UNDEFINED_ID                = 0x0132  // Undefined general identifier

// UNUSED
/* 
	HK_MAX_SID_NB_EXCEEDED
	HKD_TM_SIZE_EXCEEDED
	MEM_INVALID_ADDRESS
	MEM_HW_ERROR
*/

} TM_Err_Codes;

typedef struct {
    //uint8_t len;       
    uint8_t data[FPGA_FRAME_LEN]; 
} FPGA_IN_Msg_t;


typedef struct {
	uint8_t PUS_HEADER_PRESENT;
	uint16_t PUS_SOURCE_ID;
	uint8_t SERVICE_ID;
	uint8_t SUBTYPE_ID;
	uint8_t TM_data[SPP_MAX_PACKET_LEN];
	uint16_t TM_data_len;
} UART_OUT_OBC_msg;


void Add_SPP_PUS_and_send_TM(UART_OUT_OBC_msg* UART_OUT_msg_received);

void Send_TM(SPP_header_t* resp_SPP_header,
				PUS_TM_header_t* resp_PUS_header,
				uint8_t* data,
				uint16_t data_len);

void Prepare_full_msg(SPP_header_t* resp_SPP_header,
						PUS_TM_header_t* resp_PUS_header,
						uint8_t* data,
						uint16_t data_len,
						uint8_t* OUT_full_msg,
						uint16_t* OUT_full_msg_len );

void FPGA_process_frame(const uint8_t *frame);
void Handle_incoming_TC();

#endif /* GENERAL_FUNCTIONS_H_ */
