/*
 * PUS_1_service.c
 *
 *  Created on: 2024. gada 12. jūn.
 *      Author: Rūdolfs Arvīds Kalniņš <rakal@kth.se>
 */
#include "Space_Packet_Protocol.h"
#include "General_Functions.h"
#include "PUS_1_service.h"
#include "Device_State.h"

extern QueueHandle_t UART_OBC_Out_Queue;

// Flags denoting if an ACK TM message is requested for
// Success of request acceptence, start , progress and completion of execution
static inline uint8_t succ_acceptence_req(PUS_TC_header_t* secondary_header) {
    return secondary_header->ACK_flags & 0x08;
}
static inline uint8_t succ_start_req     (PUS_TC_header_t* secondary_header) {
    return secondary_header->ACK_flags & 0x04;
}
static inline uint8_t succ_progress_req  (PUS_TC_header_t* secondary_header) {
    return secondary_header->ACK_flags & 0x02;
}
static inline uint8_t succ_completion_req(PUS_TC_header_t* secondary_header) {
    return secondary_header->ACK_flags & 0x01;
}

void PUS_1_send_succ_acc(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h) {
    if (succ_acceptence_req(PUS_h)) {

    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
    	msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
    	msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
    	msg_to_send.SUBTYPE_ID			= RV_SUCC_ACCEPTANCE_VERIFICATION_ID;
		// The ACK message contains as data the SPP header of the incoming request (without the data length field)
    	SPP_encode_header(SPP_h, msg_to_send.TM_data);
    	msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2;

    	xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}
void PUS_1_send_fail_acc(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, PUS_1_Fail_Acc_Data_t* PUS_1_Fail_Acc_Data, uint16_t err_code)
{
    if (succ_acceptence_req(PUS_h)) {
    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
    	msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
    	msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
    	msg_to_send.SUBTYPE_ID			= RV_FAIL_ACCEPTANCE_VERIFICATION_ID;
		// The ACK FAIL message contains as data
    	// -> 4 bytes = the SPP header of the incoming request (without the data length field)
    	SPP_encode_header(SPP_h, msg_to_send.TM_data);

    	// -> 12 bytes = information about the failure
    	// here we overwrite the last 2 bytes written before because they store the data length
    	// of the SPP packet and that should not be included
    	msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 0] 	= (err_code >> 8) & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 1] 	= err_code & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 2] 	= PUS_h->service_type_id;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 3] 	= PUS_h->message_subtype_id;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 4] 	= (PUS_1_Fail_Acc_Data->TC_Pcklen >> 8)& 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 5] 	= PUS_1_Fail_Acc_Data->TC_Pcklen & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 6] 	= (PUS_1_Fail_Acc_Data->TC_ReceivedBytes >> 8)& 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 7] 	= PUS_1_Fail_Acc_Data->TC_ReceivedBytes & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 8] 	= (PUS_1_Fail_Acc_Data->TC_ReceivedCRC >> 8)& 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 9] 	= PUS_1_Fail_Acc_Data->TC_ReceivedCRC & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 10] 	= (PUS_1_Fail_Acc_Data->TC_CalcCRC >> 8)& 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 11] 	= PUS_1_Fail_Acc_Data->TC_CalcCRC & 0xFF;

		msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2 + 12;

    	xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}

void PUS_1_send_succ_start(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h) {
    if (succ_start_req(PUS_h)) {

    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
		msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
		msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
		msg_to_send.SUBTYPE_ID			= RV_SUCC_START_OF_EXEC_VERIFICATION_ID;
		// The ACK message contains as data the SPP header of the incoming request (without the data length field)
		SPP_encode_header(SPP_h, msg_to_send.TM_data);
		msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2;

		xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}
void PUS_1_send_fail_start(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, uint16_t err_code) {
    if (succ_start_req(PUS_h)) {
    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
    	msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
    	msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
    	msg_to_send.SUBTYPE_ID			= RV_FAIL_START_OF_EXEC_VERIFICATION_ID;
		// The ACK FAIL message contains as data
    	// -> 4 bytes = the SPP header of the incoming request (without the data length field)
    	SPP_encode_header(SPP_h, msg_to_send.TM_data);

    	// -> 4 bytes = information about the failure
    	// here we overwrite the last 2 bytes written before because they store the data length
    	// of the SPP packet and that should not be included
    	msg_to_send.TM_data[SPP_HEADER_LEN - 2] 	= (err_code >> 8) & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 1] = err_code & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 2] = PUS_h->service_type_id;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 3] = PUS_h->message_subtype_id;

		msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2 + 4;

    	xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}

void PUS_1_send_succ_prog(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h) {
    if (succ_progress_req(PUS_h)) {

    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
		msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
		msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
		msg_to_send.SUBTYPE_ID			= RV_SUCC_PROG_OF_EXEC_VERIFICATION_ID;
		// The ACK message contains as data the SPP header of the incoming request (without the data length field)
		SPP_encode_header(SPP_h, msg_to_send.TM_data);
		msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2;

		xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}
void PUS_1_send_fail_prog(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, uint16_t err_code) {
    if (succ_progress_req(PUS_h)) {
    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
    	msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
    	msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
    	msg_to_send.SUBTYPE_ID			= RV_FAIL_PROG_OF_EXEC_VERIFICATION_ID;
		// The ACK FAIL message contains as data
    	// -> 4 bytes = the SPP header of the incoming request (without the data length field)
    	SPP_encode_header(SPP_h, msg_to_send.TM_data);

    	// -> 4 bytes = information about the failure
    	// here we overwrite the last 2 bytes written before because they store the data length of the SPP packet
    	// and that should not be included
    	msg_to_send.TM_data[SPP_HEADER_LEN - 2] 	= (err_code >> 8) & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 1] = err_code & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 2] = PUS_h->service_type_id;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 3] = PUS_h->message_subtype_id;

		msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2 + 4;

    	xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}

void PUS_1_send_succ_comp(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h) {
    if (succ_completion_req(PUS_h)) {

    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
		msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
		msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
		msg_to_send.SUBTYPE_ID			= RV_SUCC_COMPL_OF_EXEC_VERIFICATION_ID;
		// The ACK message contains as data the SPP header of the incoming request (without the data length field)
		SPP_encode_header(SPP_h, msg_to_send.TM_data);
		msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2;

		xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}
void PUS_1_send_fail_comp(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_h, uint16_t err_code) {
    if (succ_completion_req(PUS_h)) {
    	UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
    	msg_to_send.PUS_SOURCE_ID 		= PUS_h->source_id;
    	msg_to_send.SERVICE_ID			= REQUEST_VERIFICATION_SERVICE_ID;
    	msg_to_send.SUBTYPE_ID			= RV_FAIL_COMPL_OF_EXEC_VERIFICATION_ID;
		// The ACK FAIL message contains as data
    	// -> 4 bytes = the SPP header of the incoming request (without the data length field)
    	SPP_encode_header(SPP_h, msg_to_send.TM_data);

    	// -> 4 bytes = information about the failure
    	// here we overwrite the last 2 bytes written before because they store the data length of the SPP packet
    	// and that should not be included
    	msg_to_send.TM_data[SPP_HEADER_LEN - 2] 	= (err_code >> 8) & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 1] = err_code & 0xFF;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 2] = PUS_h->service_type_id;
		msg_to_send.TM_data[SPP_HEADER_LEN - 2 + 3] = PUS_h->message_subtype_id;

		msg_to_send.TM_data_len			= SPP_HEADER_LEN - 2 + 4;

    	xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}

