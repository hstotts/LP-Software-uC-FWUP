/*
 * PUS_8_servie.h
 *
 *  Created on: Feb 12, 2025
 *      Author: sergi
 */
#include "Space_Packet_Protocol.h"
#include "General_Functions.h"
#include "PUS.h"

#ifndef PUS_8_SERVIE_H_
#define PUS_8_SERVIE_H_

#define PUS_8_MAX_DATA_LEN 	20




// Function Management [8] subtype IDs
typedef enum {
    FM_PERFORM_FUNCTION                    = 1,  // TC
} PUS_FM_Subtype_ID;

typedef struct {
	SPP_header_t 	SPP_header;
	PUS_TC_header_t PUS_TC_header;
	uint8_t data[PUS_8_MAX_DATA_LEN];
	uint8_t data_size;
} PUS_8_msg;


typedef struct {
	uint8_t  func_id;
	uint8_t  N_args;
    uint8_t  step_ID;
    uint16_t voltage_level;
    uint16_t N_skip;
    uint8_t  N_steps;
    uint16_t N_f; // Samples per points
    uint16_t N_points;
    uint16_t N_samples_per_step;
    uint8_t FRAM_Table_ID;
    uint8_t FPGA_Probe_ID;
    uint8_t HK_ID; //ADDED
    uint8_t HK_PERIODIC_ID; //ADDED
    uint8_t HK_PERIOD_ID;

} PUS_8_msg_unpacked;

typedef enum {
    FPGA_EN_CB_MODE          		= 0x08,
    FPGA_DIS_CB_MODE         		= 0x10,

    FPGA_SET_CB_VOL_LVL      		= 0x1B,
    FPGA_GET_CB_VOL_LVL      		= 0x21,

    FPGA_SWT_ACTIVATE_SWEEP         = 0x50,
    FPGA_SET_SWT_VOL_LVL            = 0xB4,
    FPGA_SET_SWT_STEPS              = 0x61,
    FPGA_SET_SWT_SAMPLES_PER_STEP   = 0x72,
    FPGA_SET_SWT_SAMPLE_SKIP        = 0x82,
    FPGA_SET_SWT_SAMPLES_PER_POINT  = 0x92,
    FPGA_SET_SWT_NPOINTS            = 0xA2,

    FPGA_GET_SWT_SWEEP_CNT          = 0x58,
    FPGA_GET_SWT_VOL_LVL            = 0xBA,
    FPGA_GET_SWT_STEPS              = 0x68,
    FPGA_GET_SWT_SAMPLES_PER_STEP   = 0x78,
    FPGA_GET_SWT_SAMPLE_SKIP        = 0x88,
    FPGA_GET_SWT_SAMPLES_PER_POINT  = 0x98,
    FPGA_GET_SWT_NPOINTS            = 0xA8,

	REBOOT_DEVICE 					= 0xF3,
	JUMP_TO_IMAGE					= 0xF4,

} PUS_8_Func_ID;

typedef enum {
    TABLE_ID_ARG_ID             = 0x01,
    STEP_ID_ARG_ID              = 0x02,
    VOL_LVL_ARG_ID              = 0x03,
    N_STEPS_ARG_ID              = 0x04,
    N_SKIP_ARG_ID               = 0x05,
    N_F_ARG_ID                  = 0x06,
    N_POINTS_ARG_ID             = 0x07,
    N_SAMPLES_PER_STEP_ARG_ID   = 0x08,
} FPGA_Arg_ID_t;

/* PUS_8_service */
bool PUS_8_check_FPGA_msg_format(uint8_t* msg, uint8_t msg_len);
TM_Err_Codes PUS_8_unpack_msg(PUS_8_msg *pus8_msg_received, PUS_8_msg_unpacked* pus8_msg_unpacked);
TM_Err_Codes PUS_8_perform_function(SPP_header_t* SPP_h, PUS_TC_header_t* PUS_TC_h , PUS_8_msg_unpacked* pus8_msg_unpacked);
TM_Err_Codes PUS_8_handle_FM_TC(SPP_header_t* SPP_header , PUS_TC_header_t* secondary_header, uint8_t* data, uint8_t data_size);

#endif /* PUS_8_SERVIE_H_ */
