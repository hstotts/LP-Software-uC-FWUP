/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UC_CONSOLE_EN_Pin GPIO_PIN_0
#define UC_CONSOLE_EN_GPIO_Port GPIOH
#define SD_SW_B_Pin GPIO_PIN_6
#define SD_SW_B_GPIO_Port GPIOC
#define SD_SW_A_Pin GPIO_PIN_7
#define SD_SW_A_GPIO_Port GPIOC
#define FPGA_BUF_INT_Pin GPIO_PIN_6
#define FPGA_BUF_INT_GPIO_Port GPIOD
#define FPGA_BUF_INT_EXTI_IRQn EXTI9_5_IRQn
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOB

/* Software version — increment per the scheme in the ICD.
 * MAJOR: breaking change (hardware rev, boot scheme).
 * MINOR: new commands or OTA features, backward compatible.
 * PATCH: bug fixes, no interface change.
 * Reset lower fields to 0 on any upper-field bump.          */
#define FW_VERSION_MAJOR  1u
#define FW_VERSION_MINOR  0u
#define FW_VERSION_PATCH  0u

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
