/*
 * General_Functions.c
 *
 *  Created on: Feb 11, 2025
 *      Author: sergi
 */

#include "General_Functions.h"
#include "Space_Packet_Protocol.h"
#include "PUS_1_service.h"
#include "PUS_3_service.h"
#include "PUS_8_service.h"
#include "PUS_17_service.h"
#include "cmsis_os.h"
#include "device_state.h"

volatile uint8_t uart_tx_OBC_done = 1;

extern uint16_t HK_SPP_APP_ID;
extern uint16_t HK_PUS_SOURCE_ID;

extern UART_Rx_OBC_Msg UART_RxBuffer;
extern uint8_t UART_TxBuffer[MAX_COBS_FRAME_LEN];
extern uint8_t UART_FPGA_OBC_Tx_Buffer[100];

extern osThreadId Watchdog_TaskHandle;


uint16_t SPP_SEQUENCE_COUNTER = 0;

QueueHandle_t UART_OBC_Out_Queue;
QueueHandle_t FPGA_IN_Queue;

uint8_t Constant_Bias_Mode_Buffer[2][CB_MODE_BUFFERED_MEASUREMENTS*8];
uint8_t Buffer_Index = 0;
uint8_t Measurement_Index = 0;
uint8_t send_buffered_data;
uint16_t cb_expected_fpga_cnt = 0;
uint8_t  cb_cnt_valid = 0; 

// These are used as empty headers for when calling the PUS_1_send_fail_acc,
// as the system might not have the ones from the actual message (ex: COBS fail, CRC fail, etc)
SPP_header_t Error_SPP_Header;
PUS_TC_header_t Error_PUS_TC_Header;
PUS_1_Fail_Acc_Data_t PUS_1_Fail_Acc_Data;

void Prepare_full_msg(SPP_header_t* resp_SPP_header,
						PUS_TM_header_t* resp_PUS_header,
						uint8_t* data,
						uint16_t data_len,
						uint8_t* OUT_full_msg,
						uint16_t* OUT_full_msg_len ) {

    uint8_t* current_pointer = OUT_full_msg;

    SPP_encode_header(resp_SPP_header, current_pointer);
    current_pointer += SPP_HEADER_LEN;

    if (resp_PUS_header != NULL) {
        PUS_encode_TM_header(resp_PUS_header, current_pointer);
        current_pointer += SPP_PUS_TM_HEADER_LEN_WO_SPARE;
    }

    if (data != NULL) {
        SPP_add_data_to_packet(data, data_len, current_pointer);
        current_pointer += data_len;
    }

    SPP_add_CRC_to_msg(OUT_full_msg, current_pointer - OUT_full_msg, current_pointer);
    current_pointer += CRC_BYTE_LEN;
    *OUT_full_msg_len = current_pointer - OUT_full_msg;
}

void Send_TM(SPP_header_t* resp_SPP_header,
				PUS_TM_header_t* resp_PUS_header,
				uint8_t* data,
				uint16_t data_len) {

    uint8_t response_TM_packet[SPP_MAX_PACKET_LEN] = {0};
    uint8_t response_TM_packet_COBS[SPP_MAX_PACKET_LEN] = {0};
    uint16_t packet_total_len = 0;

    Prepare_full_msg(resp_SPP_header,
						resp_PUS_header,
						data, data_len,
						response_TM_packet,
						&packet_total_len);

    uint16_t cobs_packet_total_len = COBS_encode(response_TM_packet,
												packet_total_len,
												response_TM_packet_COBS);

    // Wait until the previous DMA transfer has finished
    while (!uart_tx_OBC_done)
	{
		osDelay(1);
	}

    // Mark the DMA pipeline busy
    uart_tx_OBC_done = 0;

    memcpy(UART_TxBuffer, response_TM_packet_COBS, cobs_packet_total_len);
    UART_TxBuffer[cobs_packet_total_len] = 0x00; // Adding sentinel value.
    cobs_packet_total_len +=1;

	if (HAL_UART_Transmit_DMA(&DEBUG_UART, UART_TxBuffer, cobs_packet_total_len) != HAL_OK) {
		HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);
		uart_tx_OBC_done = 1;  // Reset flag on failure
		vTaskSuspend(Watchdog_TaskHandle);
	}
}


void Add_SPP_PUS_and_send_TM(UART_OUT_OBC_msg* UART_OUT_msg_received) {

		if(SPP_SEQUENCE_COUNTER >= 65535)
			SPP_SEQUENCE_COUNTER = 0;
		else
			SPP_SEQUENCE_COUNTER++;

        if(UART_OUT_msg_received->PUS_HEADER_PRESENT == 1){
        	SPP_header_t TM_SPP_header = SPP_make_header(
        				SPP_VERSION,
        				SPP_PACKET_TYPE_TM,
        				UART_OUT_msg_received->PUS_HEADER_PRESENT,
        				SPP_APP_ID,
        				SPP_SEQUENCE_SEG_UNSEG,
        				SPP_SEQUENCE_COUNTER,
        				SPP_PUS_TM_HEADER_LEN_WO_SPARE + UART_OUT_msg_received->TM_data_len + CRC_BYTE_LEN - 1
        	        );

			PUS_TM_header_t TM_PUS_header  = PUS_make_TM_header(
				PUS_VERSION,
				0,
				UART_OUT_msg_received->SERVICE_ID,
				UART_OUT_msg_received->SUBTYPE_ID,
				0,
				UART_OUT_msg_received->PUS_SOURCE_ID,
				0
			);
			Send_TM(&TM_SPP_header, &TM_PUS_header, UART_OUT_msg_received->TM_data, UART_OUT_msg_received->TM_data_len);
        }
        else{
        	SPP_header_t TM_SPP_header = SPP_make_header(
				SPP_VERSION,
				SPP_PACKET_TYPE_TM,
				UART_OUT_msg_received->PUS_HEADER_PRESENT,
				SPP_APP_ID,
				SPP_SEQUENCE_SEG_UNSEG,
				SPP_SEQUENCE_COUNTER,
				UART_OUT_msg_received->TM_data_len + CRC_BYTE_LEN - 1
			);

        	Send_TM(&TM_SPP_header, NULL, UART_OUT_msg_received->TM_data, UART_OUT_msg_received->TM_data_len);
        }
}


void FPGA_process_frame(const uint8_t *frame)
{
    UART_OUT_OBC_msg msg_to_send = (UART_OUT_OBC_msg){0};
    msg_to_send.PUS_HEADER_PRESENT = 0;

	switch(frame[2])
	{
		case FPGA_GET_CB_VOL_LVL:
		{

            UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_CB_VOL_LVL;
            UART_FPGA_OBC_Tx_Buffer[1] = frame[3];
            UART_FPGA_OBC_Tx_Buffer[2] = frame[4];
            UART_FPGA_OBC_Tx_Buffer[3] = frame[5];
            memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 4);
            msg_to_send.TM_data_len			= 4;

			break;
		}

		case FPGA_GET_SWT_VOL_LVL:
		{

            UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_VOL_LVL;
            UART_FPGA_OBC_Tx_Buffer[1] = frame[3];
            UART_FPGA_OBC_Tx_Buffer[2] = frame[4];
            UART_FPGA_OBC_Tx_Buffer[3] = frame[5];
            UART_FPGA_OBC_Tx_Buffer[4] = frame[6];
            memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 5);
            msg_to_send.TM_data_len			= 5;
          
			break;
		}

		case FPGA_GET_SWT_STEPS:
		{

            UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_STEPS;
            UART_FPGA_OBC_Tx_Buffer[1] = frame[3];
            memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 2);
            msg_to_send.TM_data_len			= 2;
          
			break;
		}

		case FPGA_GET_SWT_SAMPLES_PER_STEP:
		{
            UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLES_PER_STEP;
            UART_FPGA_OBC_Tx_Buffer[1] = frame[3];
            UART_FPGA_OBC_Tx_Buffer[2] = frame[4];
            memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
            msg_to_send.TM_data_len			= 3;

			break;
		}

		case FPGA_GET_SWT_SAMPLE_SKIP:
		{

            UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLE_SKIP;
            UART_FPGA_OBC_Tx_Buffer[1] = frame[3];
            UART_FPGA_OBC_Tx_Buffer[2] = frame[4];
            memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
            msg_to_send.TM_data_len			= 3;

			break;
		}

		case FPGA_GET_SWT_SAMPLES_PER_POINT:
		{

            UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLES_PER_POINT;
            UART_FPGA_OBC_Tx_Buffer[1] = frame[3];
            UART_FPGA_OBC_Tx_Buffer[2] = frame[4];
            memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
            msg_to_send.TM_data_len			= 3;
          
			break;
		}

		case FPGA_GET_SWT_NPOINTS:
		{

            UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_NPOINTS;
            UART_FPGA_OBC_Tx_Buffer[1] = frame[3];
            UART_FPGA_OBC_Tx_Buffer[2] = frame[4];
            memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
            msg_to_send.TM_data_len			= 3;
          
			break;
		}

		case SC_BIAS:
		{
            if (Current_Global_Device_State != CB_MODE)
            {
                break;
            }

            send_buffered_data = 0;

            uint16_t fpga_cnt = ((uint16_t)frame[3] << 8) | (uint16_t)frame[4];
            uint8_t *buf = Constant_Bias_Mode_Buffer[Buffer_Index];

            if (!cb_cnt_valid)
            {
              buf[0] = frame[3];
              buf[1] = frame[4];
              cb_expected_fpga_cnt = fpga_cnt;
              cb_cnt_valid = 1;
              Measurement_Index = 0;
            }
            else
            {
              uint16_t expected_next = (uint16_t)(cb_expected_fpga_cnt + Measurement_Index);

              if (fpga_cnt != expected_next)
              {
                msg_to_send.TM_data[0] = SC_BIAS;
                memcpy(msg_to_send.TM_data + 1, buf, 2 + 6 * Measurement_Index);
                msg_to_send.TM_data_len = 1 + 2 + 6 * Measurement_Index;

                // send data and switch buffer
                send_buffered_data = 1;
                Buffer_Index ^= 1;
                Measurement_Index = 0;

                // save counter in new buffer
                buf = Constant_Bias_Mode_Buffer[Buffer_Index];
                buf[0] = frame[3];
                buf[1] = frame[4];
                cb_expected_fpga_cnt = fpga_cnt;
                cb_cnt_valid = 1;
              }
            }
            
            // save data in buffer
            uint8_t *dest = buf + 2 + (Measurement_Index * 6);
            memcpy(dest, frame + 5, 6);
            Measurement_Index++;
      
            if (Measurement_Index == CB_MODE_BUFFERED_MEASUREMENTS)
            {
              msg_to_send.TM_data[0] = SC_BIAS;
              memcpy(msg_to_send.TM_data + 1, buf, 2 + 6 * CB_MODE_BUFFERED_MEASUREMENTS);
              msg_to_send.TM_data_len = 1 + 2 + 6 * CB_MODE_BUFFERED_MEASUREMENTS;

              send_buffered_data = 1;
              Measurement_Index = 0;
              cb_cnt_valid = 0;    
              Buffer_Index ^= 1;
            }
            
			break;
		}

		case FPGA_GET_SENSOR_DATA:
		{

            msg_to_send.PUS_HEADER_PRESENT = 1;
            msg_to_send.SERVICE_ID = HOUSEKEEPING_SERVICE_ID;   
            msg_to_send.SUBTYPE_ID = HK_PARAMETER_REPORT;       

            msg_to_send.TM_data[0] = frame[3]; // HK ID 

            memcpy(&msg_to_send.TM_data[1], &frame[4], 7);

            msg_to_send.TM_data_len = 8;
          
			break;
		}

		case FPGA_GET_PERIOD:
		{
            msg_to_send.PUS_HEADER_PRESENT = 1;
            msg_to_send.SERVICE_ID = HOUSEKEEPING_SERVICE_ID;   
            msg_to_send.SUBTYPE_ID = HK_PARAMETER_REPORT;       

            msg_to_send.TM_data[0] = frame[3]; // HK ID 

            memcpy(&msg_to_send.TM_data[1], &frame[4], 1);

            msg_to_send.TM_data_len = 2;
          
            break;
		}

		default:
			break;
	}

    if (frame[2] != SC_BIAS ||
        (frame[2] == SC_BIAS && send_buffered_data == 1 && Current_Global_Device_State == CB_MODE))
    {
        xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
    }
}


// Function that processes incoming Telecommands
void Handle_incoming_TC() {
	memset(&Error_SPP_Header, 0, sizeof(Error_SPP_Header));
	memset(&Error_PUS_TC_Header, 0, sizeof(Error_PUS_TC_Header));
	memset(&PUS_1_Fail_Acc_Data, 0, sizeof(PUS_1_Fail_Acc_Data));

    // Decode COBS frame if valid
    if(!COBS_is_valid(UART_RxBuffer.RxBuffer, UART_RxBuffer.frame_size))
    {
    	PUS_1_send_fail_acc(&Error_SPP_Header, &Error_PUS_TC_Header, &PUS_1_Fail_Acc_Data, COBS_FRAME_ERROR);
		return;
    }
    uint8_t 		decoded_msg[UART_RxBuffer.frame_size];
    COBS_decode(UART_RxBuffer.RxBuffer, UART_RxBuffer.frame_size, decoded_msg);

    // Decode SPP header if possible and verify its checksum
    SPP_header_t 	SPP_header;
    uint8_t 		decoded_msg_size = UART_RxBuffer.frame_size - 2; //After COBS decoding, the first and last byte are removed.
    PUS_1_Fail_Acc_Data.TC_ReceivedBytes = decoded_msg_size;

    if(!SPP_decode_header(decoded_msg, decoded_msg_size, &SPP_header))
    {
    	PUS_1_send_fail_acc(&Error_SPP_Header, &Error_PUS_TC_Header, &PUS_1_Fail_Acc_Data, SPP_HEADER_ERROR);
    	return;
    }

    // CCSDS SPP defines the packet data length as the number of octets - 1 of the packet data field (including the CRC)
    uint16_t  		decoded_msg_expected_size = SPP_HEADER_LEN + SPP_header.packet_data_length + 1; // length = SPP + PUS + data + CRC;
    PUS_1_Fail_Acc_Data.TC_Pcklen = decoded_msg_expected_size;

    // The computed data length has to be the same as the length of the message received
    if(decoded_msg_size != decoded_msg_expected_size)
    {
    	PUS_1_send_fail_acc(&SPP_header, &Error_PUS_TC_Header, &PUS_1_Fail_Acc_Data, INVALID_PLENGTH);
    	return;
    }

	if (SPP_validate_checksum(decoded_msg, decoded_msg_size, &PUS_1_Fail_Acc_Data) != SPP_OK) {
		PUS_1_send_fail_acc(&SPP_header, &Error_PUS_TC_Header, &PUS_1_Fail_Acc_Data, CS_DISCREP);
		return;
	}


    // Decode PUS header if present
    if (SPP_header.secondary_header_flag) {

        PUS_TC_header_t PUS_TC_header;

        if(!PUS_decode_TC_header(decoded_msg + SPP_HEADER_LEN, &PUS_TC_header, SPP_header.packet_data_length - 1))
        {
        	PUS_1_send_fail_acc(&SPP_header, &Error_PUS_TC_Header, &PUS_1_Fail_Acc_Data, UNKNOWN_TYPE_SUBTYPE);
        	return;
        }

		uint8_t* data = decoded_msg + SPP_HEADER_LEN + PUS_TC_HEADER_LEN_WO_SPARE;
		uint8_t data_size = SPP_header.packet_data_length - PUS_TC_HEADER_LEN_WO_SPARE - 1;

		uint8_t result = NO_ERROR;

		if (PUS_TC_header.service_type_id == HOUSEKEEPING_SERVICE_ID) {
			result = PUS_3_handle_HK_TC(&SPP_header, &PUS_TC_header, data, data_size);
			if(result != NO_ERROR)
				PUS_1_send_fail_acc(&SPP_header, &PUS_TC_header, &PUS_1_Fail_Acc_Data, result);
		}
		else if (PUS_TC_header.service_type_id == FUNCTION_MANAGEMNET_ID && data_size > 0) {
			result = PUS_8_handle_FM_TC(&SPP_header, &PUS_TC_header, data, data_size);
			if(result != NO_ERROR)
				PUS_1_send_fail_acc(&SPP_header, &PUS_TC_header, &PUS_1_Fail_Acc_Data, result);
		}
		else if (PUS_TC_header.service_type_id == TEST_SERVICE_ID) {
			result = PUS_17_handle_TEST_TC(&SPP_header, &PUS_TC_header);
			if(result != NO_ERROR)
				PUS_1_send_fail_acc(&SPP_header, &PUS_TC_header, &PUS_1_Fail_Acc_Data, result);
		} else {
			PUS_1_send_fail_acc(&SPP_header, &PUS_TC_header, &PUS_1_Fail_Acc_Data, UNKNOWN_TYPE_SUBTYPE);
		}
    }
}

