/*
 * PUS.h
 *
 *  Created on: Feb 11, 2025
 *      Author: sergi
 */

#ifndef PUS_H_
#define PUS_H_


typedef enum {
    REQUEST_VERIFICATION_SERVICE_ID      = 1,
    HOUSEKEEPING_SERVICE_ID              = 3,
    FUNCTION_MANAGEMNET_ID               = 8,
    TEST_SERVICE_ID                      = 17,
} PUS_Service_ID;

// House Keeping service [3] subtype IDs
typedef enum {
    HK_CREATE_HK_PAR_REPORT_STRUCT         = 1,  // TC
    HK_DELETE_HK_PAR_REPORT_STRUCT         = 3,  // TC
    HK_EN_PERIODIC_REPORTS                 = 5,  // TC
    HK_DIS_PERIODIC_REPORTS                = 6,  // TC
    HK_REPORT_HK_PAR_REPORT_STRUCT         = 9,  // TC
    HK_REPORT_HK_PAR_REPORT_STRUCT_REPORT  = 10, // TM (response to 9)
    HK_PARAMETER_REPORT                    = 25, // TM
    HK_ONE_SHOT                            = 27, // TC
    HK_SET_PERIOD                          = 31, // TC
} PUS_HK_Subtype_ID;

typedef struct {
    uint8_t  PUS_version_number;    // 4
    uint8_t  ACK_flags;             // 4
    uint8_t  service_type_id;       // 8
    uint8_t  message_subtype_id;    // 8
    uint16_t source_id;             // 16
    uint32_t spare;
} PUS_TC_header_t;

typedef struct {
    uint8_t  PUS_version_number;    // 4
    uint8_t  sc_time_ref_status;    // 4
    uint8_t  service_type_id;       // 8
    uint8_t  message_subtype_id;    // 8
    uint16_t message_type_counter;  // 16
    uint16_t destination_id;        // 16
    uint16_t time;                  // 16
    uint32_t spare;
} PUS_TM_header_t;

typedef enum {
    SC_BIAS          		= 0x09,
} SC_ID;

/* PUS */
PUS_TM_header_t PUS_make_TM_header(uint8_t PUS_version_number,
								uint8_t sc_time_ref_status,
								uint8_t service_type_id,
								uint8_t message_subtype_id,
								uint16_t message_type_counter,
								uint16_t destination_id,
								uint16_t time);

SPP_error PUS_decode_TC_header(uint8_t* raw_header, PUS_TC_header_t* secondary_header, uint8_t available_data_size);
SPP_error PUS_encode_TC_header(PUS_TC_header_t* secondary_header, uint8_t* result_buffer);
SPP_error PUS_decode_TM_header(uint8_t* raw_header, PUS_TM_header_t* secondary_header);
SPP_error PUS_encode_TM_header(PUS_TM_header_t* secondary_header, uint8_t* result_buffer);

#endif /* PUS_H_ */
