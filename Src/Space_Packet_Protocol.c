 /*
 * Space_Packet_Protocol.c
 *
 *  Created on: 2024. gada 23. apr.
 *      Author: Rūdolfs Arvīds Kalniņš <rakal@kth.se>
 */

#include "General_Functions.h"
#include "Space_Packet_Protocol.h"
#include "PUS.h"
#include "PUS_1_service.h"
#include "PUS_3_service.h"
#include "PUS_8_service.h"
#include "PUS_17_service.h"
#include <stdio.h>

uint8_t DEBUG_Space_Packet_Data_Buffer[256];
uint8_t OBC_Space_Packet_Data_Buffer[1024];

volatile UART_Rx_OBC_Msg UART_RxBuffer;
volatile uint16_t UART_recv_count = 0;
volatile uint8_t UART_recv_char = 0xff;
uint8_t UART_TxBuffer[MAX_COBS_FRAME_LEN];


// CRC16-CCITT
static uint16_t SPP_CRC16_byte(uint16_t crcValue, uint8_t newByte) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (((crcValue & 0x8000) >> 8) ^ (newByte & 0x80)){
			crcValue = (crcValue << 1)  ^ 0x1021;
		}else{
			crcValue = (crcValue << 1);
		}
		newByte <<= 1;
	}
  
	return crcValue;
}

static uint16_t SPP_calc_CRC16(uint8_t* data, uint16_t length) {
    uint16_t CRCvalue = 0xFFFF;
    for(uint16_t i = 0; i < length; i++) {
        CRCvalue = SPP_CRC16_byte(CRCvalue, data[i]);
    }
   
    return CRCvalue;
}


// CRC is calculated over all of the packet. The last two bytes are the received CRC.
SPP_error SPP_validate_checksum(uint8_t* packet, uint16_t packet_length, void* PUS_1_Fail_Acc_Data_ptr) {
    uint16_t received_CRC = 0x0000;
    received_CRC |= (packet[packet_length - 2] << 8) | packet[packet_length - 1];

    uint16_t calculated_CRC = SPP_calc_CRC16(packet, packet_length - 2);

    PUS_1_Fail_Acc_Data_t* PUS_1_Fail_Acc_Data = (PUS_1_Fail_Acc_Data_t*) PUS_1_Fail_Acc_Data_ptr;
    PUS_1_Fail_Acc_Data->TC_ReceivedCRC = received_CRC;
    PUS_1_Fail_Acc_Data->TC_CalcCRC 	= calculated_CRC;

    if (received_CRC != calculated_CRC) {
        return SPP_PACKET_CRC_MISMATCH;
    } else {
        return SPP_OK;
    }
}



SPP_error SPP_decode_header(uint8_t* input_msg, uint8_t input_msg_size, SPP_header_t* primary_header) {

	if (input_msg == NULL || primary_header == NULL || input_msg_size < 6) {
	        return 0;
	}

	primary_header->packet_version_number	= (input_msg[0] & 0xE0) >> 5;
	primary_header->packet_type 			= (input_msg[0] & 0x10) >> 4;
	primary_header->secondary_header_flag	= (input_msg[0] & 0x08) >> 3;
	primary_header->application_process_id	=((input_msg[0] & 0x03) << 8) | (input_msg[1]);
	primary_header->sequence_flags			= (input_msg[2] & 0xC0) >> 6;
	primary_header->packet_sequence_count	=((input_msg[2] & 0x3F) << 8) | (input_msg[3]);
	primary_header->packet_data_length		= (input_msg[4] << 8) 		   | (input_msg[5]);

    return 1;
}



SPP_error SPP_encode_header(SPP_header_t* primary_header, uint8_t* result_buffer) {
    for(int i = 0; i < SPP_HEADER_LEN; i++) {
        result_buffer[i] ^= result_buffer[i];    // Clear result buffer.
    }

    result_buffer[0] |=  primary_header->packet_version_number  << 5;
    result_buffer[0] |=  primary_header->packet_type            << 4;
    result_buffer[0] |=  primary_header->secondary_header_flag  << 3;
    result_buffer[0] |= (primary_header->application_process_id & 0x300) >> 8;
    result_buffer[1] |=  primary_header->application_process_id & 0x0FF;
    result_buffer[2] |=  primary_header->sequence_flags         << 6;
    result_buffer[2] |= (primary_header->packet_sequence_count  & 0x3F00) >> 8;
    result_buffer[3] |=  primary_header->packet_sequence_count  & 0x00FF;
    result_buffer[4] |= (primary_header->packet_data_length     & 0xFF00) >> 8;
    result_buffer[5] |=  primary_header->packet_data_length     & 0x00FF;
    return SPP_OK;
}



SPP_header_t SPP_make_header(uint8_t packet_version_number, uint8_t packet_type, uint8_t secondary_header_flag, uint16_t application_process_id, uint8_t sequence_flags, uint16_t packet_sequence_count, uint16_t packet_data_length) {
    SPP_header_t primary_header;
    primary_header.packet_version_number    = packet_version_number;
    primary_header.packet_type              = packet_type;
    primary_header.secondary_header_flag    = secondary_header_flag;
    primary_header.application_process_id   = application_process_id;
    primary_header.sequence_flags           = sequence_flags;
    primary_header.packet_sequence_count    = packet_sequence_count;
    primary_header.packet_data_length       = packet_data_length;
    return primary_header;
}



static inline uint16_t byte_swap16(uint16_t value) {
    return ((value << 8) | (value >> 8)) & 0xFFFF;
}


SPP_error SPP_add_CRC_to_msg(uint8_t* packet, uint16_t length, uint8_t* output) {
    uint16_t calculated_CRC = SPP_calc_CRC16(packet, length);
    uint16_t bs_CRC = byte_swap16(calculated_CRC); // (byteswapped CRC) - memcpy copies starting from LSB thus we swap.
    memcpy(output, &bs_CRC, CRC_BYTE_LEN);
    return SPP_OK;
}

SPP_error SPP_add_data_to_packet(uint8_t* data, uint16_t data_len, uint8_t* packet) {
    memcpy(packet, data, data_len);
    return SPP_OK;
}
