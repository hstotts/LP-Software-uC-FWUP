#include "FRAM.h"
#include "Device_State.h"

HAL_StatusTypeDef writeFRAM_DMA(uint16_t addr, uint8_t* data, uint32_t size) {
	return HAL_I2C_Mem_Write_DMA(&hi2c4, FRAM_I2C_ADDR, addr, 2, data, size);
}

HAL_StatusTypeDef readFRAM_DMA(uint16_t addr, uint8_t* buf, uint32_t size) {
	return HAL_I2C_Mem_Read_DMA(&hi2c4, FRAM_I2C_ADDR, addr, 2, buf, size);
}

HAL_StatusTypeDef writeFRAM(uint16_t addr, uint8_t* data, uint32_t size) {
	return HAL_I2C_Mem_Write(&hi2c4, FRAM_I2C_ADDR, addr, 2, data, size, 50);
}

HAL_StatusTypeDef readFRAM(uint16_t addr, uint8_t* buf, uint32_t size) {
	return HAL_I2C_Mem_Read(&hi2c4, FRAM_I2C_ADDR_READ, addr, 2, buf, size, 50);
}

static uint16_t get_sweep_table_address(uint8_t save_id) {
    uint16_t addr = 0x2000; // Invalid
    if (save_id == 1) {addr = FRAM_SWEEP_TABLE_0;}
    if (save_id == 2) {addr = FRAM_SWEEP_TABLE_1;}
    if (save_id == 3) {addr = FRAM_SWEEP_TABLE_2;}
    if (save_id == 4) {addr = FRAM_SWEEP_TABLE_3;}
    if (save_id == 5) {addr = FRAM_SWEEP_TABLE_4;}
    if (save_id == 6) {addr = FRAM_SWEEP_TABLE_5;}
    if (save_id == 7) {addr = FRAM_SWEEP_TABLE_6;}
    if (save_id == 8) {addr = FRAM_SWEEP_TABLE_7;}
    return addr;
}

static uint16_t CRC16_byte(uint16_t crc, uint8_t byte)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (((crc & 0x8000u) >> 8) ^ (byte & 0x80u))
            crc = (crc << 1) ^ 0x1021u;
        else
            crc = (crc << 1);
        byte <<= 1;
    }
    return crc;
}

uint16_t Calc_CRC16(uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < length; i++)
        crc = CRC16_byte(crc, data[i]);
    return crc;
}

SPP_error save_sweep_table_value_FRAM(uint8_t table_id, uint8_t step_id, uint16_t value) {

    uint16_t sweep_table_address = get_sweep_table_address(table_id);
    uint16_t FRAM_address = sweep_table_address + (step_id * 2); // step ID is 0x00 to 0xFF, but each value is 16 bits.

    writeFRAM(FRAM_address, (uint8_t*) &value, 2);
    return SPP_OK;
}


uint16_t read_sweep_table_value_FRAM(uint8_t table_id, uint8_t step_id) {

    uint16_t value = {0x0000};
    uint16_t sweep_table_address = get_sweep_table_address(table_id);

    uint16_t FRAM_address = sweep_table_address + (step_id * 2);
    readFRAM(FRAM_address, (uint8_t*) &value, 2);

    return value;
}
