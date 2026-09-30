/*
 * flash_if.h
 *
 *  Created on: Oct 9, 2025
 *      Author: haydenstotts
 */

#ifndef INC_FLASH_IF_H_
#define INC_FLASH_IF_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f7xx_hal.h"

typedef HAL_StatusTypeDef FLASHIF_StatusTypedef;

#define FLASHIF_OK     HAL_OK
#define FLASHIF_ERROR  HAL_ERROR

/* Flash operations are restricted internally to writable OTA sectors. */
bool FLASHIF_IsBlank(uint32_t base, uint32_t size);
HAL_StatusTypeDef FLASHIF_EraseRange(uint32_t base, uint32_t size);
FLASHIF_StatusTypedef FLASHIF_ProgramBuffer(uint32_t* dst,
                                            const uint8_t* src,
                                            uint32_t byte_count);

int FLASHIF_SectorIndexForAddress(uint32_t addr);
uint32_t FLASHIF_SectorBase(int sector);
uint32_t FLASHIF_SectorSize(int sector);

#endif /* INC_FLASH_IF_H_ */
