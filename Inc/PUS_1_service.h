/*
 * PUS_1_service.h
 *
 *  Created on: Feb 11, 2025
 *      Author: sergi
 */

#ifndef PUS_1_SERVICE_H_
#define PUS_1_SERVICE_H_

#include "Space_Packet_Protocol.h"

// Request Verification service [1] subtype IDs
typedef enum { // ALL TM
    RV_SUCC_ACCEPTANCE_VERIFICATION_ID     = 1,
    RV_FAIL_ACCEPTANCE_VERIFICATION_ID     = 2,
    RV_SUCC_START_OF_EXEC_VERIFICATION_ID  = 3,
    RV_FAIL_START_OF_EXEC_VERIFICATION_ID  = 4,
    RV_SUCC_PROG_OF_EXEC_VERIFICATION_ID   = 5,
    RV_FAIL_PROG_OF_EXEC_VERIFICATION_ID   = 6,
    RV_SUCC_COMPL_OF_EXEC_VERIFICATION_ID  = 7,
    RV_FAIL_COMPL_OF_EXEC_VERIFICATION_ID  = 8,
} PUS_RV_Subtype_ID;

typedef struct {
	uint16_t TC_Pcklen;
	uint16_t TC_ReceivedBytes;
	uint16_t TC_ReceivedCRC;
	uint16_t TC_CalcCRC;
} PUS_1_Fail_Acc_Data_t;

/* PUS_1_service */
void PUS_1_send_succ_acc  (SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h);
void PUS_1_send_fail_acc  (SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, PUS_1_Fail_Acc_Data_t* PUS_1_Fail_Acc_Data, uint16_t err_code);
void PUS_1_send_succ_start(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h);
void PUS_1_send_fail_start(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, uint16_t err_code);
void PUS_1_send_succ_prog (SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h);
void PUS_1_send_fail_prog (SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, uint16_t err_code);
void PUS_1_send_succ_comp (SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h);
void PUS_1_send_fail_comp (SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, uint16_t err_code);
void PUS_1_debug(uint8_t *buffer);
void PUS_1_empty_debug();
void PUS_1_debug_byte(uint8_t b);
void PUS_1_debug_uint32(uint32_t value, uint8_t id);

#endif /* PUS_1_SERVICE_H_ */
