/*
 * PUS_17_service.c
 *
 *  Created on: 2024. gada 12. jūn.
 *      Author: Rūdolfs Arvīds Kalniņš <rakal@kth.se>
 */
#include "Space_Packet_Protocol.h"
#include "General_Functions.h"
#include "PUS_1_service.h"
#include "PUS_17_service.h"
#include "Device_State.h"

extern QueueHandle_t UART_OBC_Out_Queue;

TM_Err_Codes PUS_17_handle_TEST_TC(SPP_header_t* SPP_header, PUS_TC_header_t* PUS_TC_header) {

	if (Current_Global_Device_State != NORMAL_MODE) {
        return BAD_STATE;
    }
    if (SPP_header == NULL || PUS_TC_header == NULL) {
        return ILLEGAL_VERSION;
    }

    if (PUS_TC_header->message_subtype_id == T_ARE_YOU_ALIVE_TEST_ID) {

    	PUS_1_send_succ_acc(SPP_header, PUS_TC_header);

        UART_OUT_OBC_msg msg_to_send = {0};

		msg_to_send.PUS_HEADER_PRESENT	= 1;
		msg_to_send.PUS_SOURCE_ID 		= PUS_TC_header->source_id;
		msg_to_send.SERVICE_ID			= TEST_SERVICE_ID;
		msg_to_send.SUBTYPE_ID			= T_ARE_YOU_ALIVE_TEST_REPORT_ID;
		msg_to_send.TM_data[0] 			= 0;
		msg_to_send.TM_data_len			= 0;

		xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);

		PUS_1_send_succ_comp(SPP_header, PUS_TC_header);
    }
    else
    {
    	return UNKNOWN_TYPE_SUBTYPE;
    }

    return NO_ERROR;
}


