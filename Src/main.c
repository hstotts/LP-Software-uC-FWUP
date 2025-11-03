/* USER CODE BEGIN Header */
/**
    ******************************************************************************
    * @file                     : main.c
    * @brief                    : Main program body
    ******************************************************************************
    * @attention
    *
    * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
    * All rights reserved.</center></h2>
    *
    * This software component is licensed by ST under Ultimate Liberty license
    * SLA0044, the "License"; You may not use this file except in compliance with
    * the License. You may obtain a copy of the License at:
    *                                                         www.st.com/SLA0044
    *
    ******************************************************************************
    */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "FRAM.h"
#include "COBS.h"
#include "General_Functions.h"
#include "Space_Packet_Protocol.h"
#include "PUS.h"
#include "PUS_1_service.h"
#include "PUS_3_service.h"
#include "PUS_8_service.h"
#include "PUS_17_service.h"
#include "Device_State.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c4;
DMA_HandleTypeDef hdma_i2c4_rx;
DMA_HandleTypeDef hdma_i2c4_tx;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart5_rx;

DMA_HandleTypeDef hdma_memtomem_dma2_stream1;
SRAM_HandleTypeDef hsram1;

osThreadId PUS_3_TaskHandle;
osThreadId UART_OBC_INHandle;
osThreadId PUS_8_TaskHandle;
osThreadId UART_OBC_OUTHandle;
osThreadId UART_FPGA_INHandle;
osThreadId Watchdog_TaskHandle;
/* USER CODE BEGIN PV */

#define UNIT_ID_CU 0x1A
#define UNIT_ID_EMUCONTROL 0x1B
#define UNIT_ID_EMUSCIENCE 0x1C
#define UNIT_ID_SMILE 0x1D

#define SWEEP_MODE_MEASUREMENT_BYTE_SIZE 8 // size in bytes of a measurement stored inside the FIFO
#define SWEEP_MODE_ROWS_TO_READ 64 // nr of rows that have to be read when the AFULL interrupt is generated
#define SWEEP_MODE_DATA_SIZE_TO_READ SWEEP_MODE_ROWS_TO_READ * SWEEP_MODE_MEASUREMENT_BYTE_SIZE * sizeof(uint8_t)

#define UART_MAX_RETRIES 3
#define CB_MODE_BUFFERED_MEASUREMENTS 4


uint16_t HK_SPP_APP_ID = 0;  
uint16_t HK_PUS_SOURCE_ID = 0;

extern QueueHandle_t UART_OBC_Out_Queue;
extern QueueHandle_t PUS_3_Queue;
extern QueueHandle_t PUS_8_Queue;

extern volatile uint8_t uart_tx_OBC_done;
extern volatile uint8_t uart_tx_FPGA_done;

extern uint8_t UART_FPGA_Rx_Buffer[100];
extern uint8_t UART_FPGA_OBC_Tx_Buffer[100];

extern volatile UART_Rx_OBC_Msg UART_RxBuffer;
extern volatile uint16_t UART_recv_count;
extern volatile uint8_t UART_recv_char;
extern volatile uint8_t UART_TxBuffer[MAX_COBS_FRAME_LEN];

DeviceState Current_Global_Device_State = NORMAL_MODE;

volatile uint8_t Sweep_Bias_Mode_Data[3072];
uint8_t Constant_Bias_Mode_Buffer[2][CB_MODE_BUFFERED_MEASUREMENTS*8];
uint8_t Buffer_Index = 0;
uint8_t Measurement_Index = 0;
uint8_t send_buffered_data;

volatile uint16_t Sweep_Bias_Data_counter = 0;
volatile uint16_t Old_Sweep_Bias_Data_counter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FMC_Init(void);
static void MX_I2C4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_IWDG_Init(void);
void handle_PUS_3_Service(void const * argument);
void handle_UART_IN_OBC(void const * argument);
void handle_PUS_8_Service(void const * argument);
void handle_UART_OUT_OBC(void const * argument);
void handle_UART_IN_FPGA(void const * argument);
void handle_Watchdog(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  // This condition checks if the system has reset because the IWDG was not refreshed in time
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
      __HAL_RCC_CLEAR_RESET_FLAGS();  // Clear reset flags
  }

  // This line is very IMPORTANT because it also pauses the hardware watchdog when in debug mode,
  // thus allowing for a proper analysis of the system state (ex: science data buffer)
  __HAL_DBGMCU_FREEZE_IWDG();

  UART_OBC_Out_Queue = xQueueCreate(1, sizeof(UART_OUT_OBC_msg));
  PUS_3_Queue = xQueueCreate(1, sizeof(PUS_3_msg));
  PUS_8_Queue = xQueueCreate(1, sizeof(PUS_8_msg));
  Current_Global_Device_State = NORMAL_MODE;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FMC_Init();
  MX_I2C4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_IWDG_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

  hsram1.hdma = &hdma_memtomem_dma2_stream1;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of PUS_3_Task */
  osThreadDef(PUS_3_Task, handle_PUS_3_Service, osPriorityNormal, 0, 1024);
  PUS_3_TaskHandle = osThreadCreate(osThread(PUS_3_Task), NULL);

  /* definition and creation of UART_OBC_IN */
  osThreadDef(UART_OBC_IN, handle_UART_IN_OBC, osPriorityHigh, 0, 1024);
  UART_OBC_INHandle = osThreadCreate(osThread(UART_OBC_IN), NULL);

  /* definition and creation of PUS_8_Task */
  osThreadDef(PUS_8_Task, handle_PUS_8_Service, osPriorityNormal, 0, 1024);
  PUS_8_TaskHandle = osThreadCreate(osThread(PUS_8_Task), NULL);

  /* definition and creation of UART_OBC_OUT */
  osThreadDef(UART_OBC_OUT, handle_UART_OUT_OBC, osPriorityAboveNormal, 0, 1024);
  UART_OBC_OUTHandle = osThreadCreate(osThread(UART_OBC_OUT), NULL);

  /* definition and creation of UART_FPGA_IN */
  osThreadDef(UART_FPGA_IN, handle_UART_IN_FPGA, osPriorityHigh, 0, 1024);
  UART_FPGA_INHandle = osThreadCreate(osThread(UART_FPGA_IN), NULL);

  /* definition and creation of Watchdog_Task */
  osThreadDef(Watchdog_Task, handle_Watchdog, osPriorityLow, 0, 128);
  Watchdog_TaskHandle = osThreadCreate(osThread(Watchdog_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4|RCC_PERIPHCLK_UART5
                              |RCC_PERIPHCLK_I2C4;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c4ClockSelection = RCC_I2C4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
}

/**
  * @brief I2C4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C4_Init(void)
{

  /* USER CODE BEGIN I2C4_Init 0 */

  /* USER CODE END I2C4_Init 0 */

  /* USER CODE BEGIN I2C4_Init 1 */

  /* USER CODE END I2C4_Init 1 */
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x20404768;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C4_Init 2 */

  /* USER CODE END I2C4_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  * Configure DMA for memory to memory transfers
  *   hdma_memtomem_dma2_stream1
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma2_stream1 on DMA2_Stream1 */
  hdma_memtomem_dma2_stream1.Instance = DMA2_Stream1;
  hdma_memtomem_dma2_stream1.Init.Channel = DMA_CHANNEL_0;
  hdma_memtomem_dma2_stream1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma2_stream1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma2_stream1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma2_stream1.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  hdma_memtomem_dma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_memtomem_dma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_memtomem_dma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_memtomem_dma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_memtomem_dma2_stream1) != HAL_OK)
  {
    Error_Handler( );
  }

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}
/* FMC initialization function */
static void MX_FMC_Init(void)
{
  FMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FMC_NORSRAM_DEVICE;
  hsram1.Extended = FMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FMC_DATA_ADDRESS_MUX_ENABLE;
  hsram1.Init.MemoryType = FMC_MEMORY_TYPE_PSRAM;
  hsram1.Init.MemoryDataWidth = FMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FMC_BURST_ACCESS_MODE_ENABLE;
  hsram1.Init.WaitSignalPolarity = FMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WaitSignalActive = FMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FMC_WRITE_OPERATION_DISABLE;
  hsram1.Init.WaitSignal = FMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FMC_WRITE_BURST_DISABLE;
  hsram1.Init.ContinuousClock = FMC_CONTINUOUS_CLOCK_SYNC_ONLY;
  hsram1.Init.WriteFifo = FMC_WRITE_FIFO_DISABLE;
  hsram1.Init.PageSize = FMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 15;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 255;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 6;
  Timing.DataLatency = 2;
  Timing.AccessMode = FMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UC_CONSOLE_EN_GPIO_Port, UC_CONSOLE_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SW_A_GPIO_Port, SD_SW_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : UC_CONSOLE_EN_Pin */
  GPIO_InitStruct.Pin = UC_CONSOLE_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UC_CONSOLE_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SW_B_Pin */
  GPIO_InitStruct.Pin = SD_SW_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_SW_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SW_A_Pin */
  GPIO_InitStruct.Pin = SD_SW_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_SW_A_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FPGA_BUF_INT_Pin */
  GPIO_InitStruct.Pin = FPGA_BUF_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FPGA_BUF_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
    Sweep_Bias_Data_counter += SWEEP_MODE_DATA_SIZE_TO_READ;
    if (__HAL_GPIO_EXTI_GET_IT(FPGA_BUF_INT_Pin) != RESET)
    {
    	// VERY IMPORTANT TO CLEAR THE INTERRUPTS
    	__HAL_GPIO_EXTI_CLEAR_IT(FPGA_BUF_INT_Pin);
    	NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    }

    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == FPGA_BUF_INT_Pin && (Old_Sweep_Bias_Data_counter != Sweep_Bias_Data_counter))  // Adjust to your pin
	{
		Old_Sweep_Bias_Data_counter = Sweep_Bias_Data_counter;
		HAL_GPIO_TogglePin(GPIOB, LED4_Pin | LED3_Pin);
		if(Sweep_Bias_Data_counter <= sizeof(Sweep_Bias_Mode_Data) - SWEEP_MODE_DATA_SIZE_TO_READ)
		{
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
			HAL_SRAM_Read_DMA(&hsram1,
					(uint32_t *)0x60000000,
					(uint32_t *)(Sweep_Bias_Mode_Data + Sweep_Bias_Data_counter),
					SWEEP_MODE_DATA_SIZE_TO_READ);

		}
		else
		{
			HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

			// VERY IMPORTANT TO CLEAR THE INTERRUPTS
			__HAL_GPIO_EXTI_CLEAR_IT(FPGA_BUF_INT_Pin);
			NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
		}
	}
}

void Clear_UART_Errors(UART_HandleTypeDef *huart)
{
	// Clear Overrun Error, Noise Error, and Framing Error flags
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE);
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);
	__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart5) {
		osSignalSet(UART_FPGA_INHandle, 0x02);
	}

	else if(huart == &DEBUG_UART)
    {
		// Store the received character in the buffer
		UART_RxBuffer.RxBuffer[UART_recv_count] = UART_recv_char;

		// Check if this is the end of frame (0x00 terminator) or the buffer is full
		if (UART_recv_char == 0x00 || UART_recv_count >= MAX_COBS_FRAME_LEN - 1)
		{
			// Mark the frame size
			UART_RxBuffer.frame_size = UART_recv_count + 1;

			UART_recv_count = 0;
			UART_recv_char = 0xff;

			// Signal the task that a complete frame is ready to be processed
			osSignalSet(UART_OBC_INHandle, 0x01);

			// DO NOT RE-ARM THE ISR, it will be done after the task processes the buffer,
			// thus ensuring no race condition (the buffer is not modified while being used)
		}
		else
		{
			// Continue accumulating characters
			UART_recv_count++;

			Clear_UART_Errors(&DEBUG_UART);

			if (HAL_UART_Receive_IT(&DEBUG_UART,(uint8_t*) &UART_recv_char, 1) != HAL_OK)
			{
				HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);

				int retry_count = 0;
				HAL_StatusTypeDef uart_status;

				do {
					Clear_UART_Errors(&DEBUG_UART);

					uart_status = HAL_UART_Receive_IT(&DEBUG_UART,(uint8_t*) &UART_recv_char, 1);
					retry_count++;

					if (uart_status != HAL_OK) {
						osDelay(10); // Small delay between retries
					}
				} while (uart_status != HAL_OK && retry_count < UART_MAX_RETRIES);

				if (uart_status != HAL_OK) {
					vTaskSuspend(Watchdog_TaskHandle);  // Let watchdog trigger reset
				}
			}
		}

    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &DEBUG_UART) {
    	uart_tx_OBC_done = 1;  // Mark transmission as complete
    }
    else if (huart == &huart5) {
    	uart_tx_FPGA_done = 1;  // Mark transmission as complete
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_handle_PUS_3_Service */
/**
  * @brief  Function implementing the PUS_3_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_handle_PUS_3_Service */
void handle_PUS_3_Service(void const * argument)
{

  /* USER CODE BEGIN 5 */

    uint32_t current_ticks = 0;
    // uint8_t periodic_report = 0;
    PUS_3_msg pus3_msg_received;
    TM_Err_Codes result;

    /* Infinite loop */
    for(;;)
    {
    		if (xQueueReceive(PUS_3_Queue, &pus3_msg_received, portMAX_DELAY) == pdPASS)
    		{
    			result = PUS_3_set_report_frequency(pus3_msg_received.data, &pus3_msg_received);
          
    			if(result == NO_ERROR)
    			{
    				current_ticks = xTaskGetTickCount();
            PUS_1_send_succ_comp(&pus3_msg_received.SPP_header, &pus3_msg_received.PUS_TC_header);
    			}
    			else
    			{
    				PUS_1_send_fail_comp(&pus3_msg_received.SPP_header, &pus3_msg_received.PUS_TC_header, result);
    			}
    		}

    	osDelay(1);
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_handle_UART_IN_OBC */
/**
* @brief Function implementing the UART_OBC_IN_Tas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_handle_UART_IN_OBC */
void handle_UART_IN_OBC(void const * argument)
{
  /* USER CODE BEGIN handle_UART_IN_OBC */

	// Clear Overrun Error, Noise Error, and Framing Error flags
	Clear_UART_Errors(&DEBUG_UART);

	HAL_UART_Receive_IT(&DEBUG_UART,(uint8_t*) &UART_recv_char, 1);

	/* Infinite loop */
	for(;;)
	{
		osEvent evt = osSignalWait(0x01, osWaitForever);

		if (evt.status == osEventSignal)
		{
			if (evt.value.signals & 0x01)
			{
				Handle_incoming_TC();

				memset((void*)UART_RxBuffer.RxBuffer, 0, sizeof(UART_RxBuffer.RxBuffer));

				// Read out any pending data in the RX buffer that might have been received while ISR was disabled
				while (__HAL_UART_GET_FLAG(&DEBUG_UART, UART_FLAG_RXNE)) {
					volatile uint8_t dummy = (uint8_t)(DEBUG_UART.Instance->RDR);  // Read and discard
					(void)dummy;  // Avoid compiler warnings
				}

				Clear_UART_Errors(&DEBUG_UART);

				if (HAL_UART_Receive_IT(&DEBUG_UART,(uint8_t*) &UART_recv_char, 1) != HAL_OK)
				{
					// if rearming fails, try several times
					HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED3_Pin, GPIO_PIN_SET);

					int retry_count = 0;
					HAL_StatusTypeDef uart_status;

					do {
						Clear_UART_Errors(&DEBUG_UART);

					    uart_status = HAL_UART_Receive_IT(&DEBUG_UART, (uint8_t*)&UART_recv_char, 1);
					    retry_count++;

					    if (uart_status != HAL_OK) {
					        osDelay(10); // Small delay between retries
					    }
					} while (uart_status != HAL_OK && retry_count < UART_MAX_RETRIES);

					if (uart_status != HAL_OK) {
					    vTaskSuspend(Watchdog_TaskHandle);  // Let watchdog trigger reset
					}
				}
			}
		}
		osDelay(1);
	}
  /* USER CODE END handle_UART_IN_OBC */
}

/* USER CODE BEGIN Header_handle_PUS_8_Service */
/**
* @brief Function implementing the PUS_8_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_handle_PUS_8_Service */
void handle_PUS_8_Service(void const * argument)
{
  /* USER CODE BEGIN handle_PUS_8_Service */
  /* Infinite loop */
	PUS_8_msg pus8_msg_received;
	PUS_8_msg_unpacked pus8_msg_unpacked;
	TM_Err_Codes result;

	for(;;)
	{
		if (xQueueReceive(PUS_8_Queue, &pus8_msg_received, portMAX_DELAY) == pdPASS)
		{
			pus8_msg_unpacked = (PUS_8_msg_unpacked){0};

			result = PUS_8_unpack_msg(&pus8_msg_received, &pus8_msg_unpacked);

			if(result == NO_ERROR)
			{
				result = PUS_8_perform_function(&pus8_msg_received.SPP_header, &pus8_msg_received.PUS_TC_header, &pus8_msg_unpacked);

				if(result == NO_ERROR)
				{
					PUS_1_send_succ_comp(&pus8_msg_received.SPP_header, &pus8_msg_received.PUS_TC_header);
				}
				else
				{
					PUS_1_send_fail_comp(&pus8_msg_received.SPP_header, &pus8_msg_received.PUS_TC_header, result);
				}
			}
			else
			{
				PUS_1_send_fail_comp(&pus8_msg_received.SPP_header, &pus8_msg_received.PUS_TC_header, result);
			}

			pus8_msg_received = (PUS_8_msg){0};
		}
		osDelay(1);
	}
  /* USER CODE END handle_PUS_8_Service */
}

/* USER CODE BEGIN Header_handle_UART_OUT_OBC */
/**
* @brief Function implementing the UART_OBC_OUT_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_handle_UART_OUT_OBC */
void handle_UART_OUT_OBC(void const * argument)
{
  /* USER CODE BEGIN handle_UART_OUT_OBC */

	UART_OUT_OBC_msg UART_OUT_msg_received;

  /* Infinite loop */
  for(;;)
  {
	  if (xQueueReceive(UART_OBC_Out_Queue, &UART_OUT_msg_received, portMAX_DELAY) == pdPASS)
	  {
		  Add_SPP_PUS_and_send_TM(&UART_OUT_msg_received);
	  }
	  osDelay(1);
  }
  /* USER CODE END handle_UART_OUT_OBC */
}

/* USER CODE BEGIN Header_handle_UART_IN_FPGA */
/**
* @brief Function implementing the UART_FPGA_IN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_handle_UART_IN_FPGA */
void handle_UART_IN_FPGA(void const * argument)
{
  /* USER CODE BEGIN handle_UART_IN_FPGA */

	HAL_UART_Receive_DMA(&huart5, UART_FPGA_Rx_Buffer, 2 + 9 + 1);

  /* Infinite loop */
  for(;;)
  {
	osEvent evt = osSignalWait(0x02, osWaitForever);

		if (evt.status == osEventSignal)
		{
			if (evt.value.signals & 0x02)
			{
				UART_OUT_OBC_msg msg_to_send= {0};

				msg_to_send.PUS_HEADER_PRESENT	= 0;

				switch(UART_FPGA_Rx_Buffer[2])
				{
					case FPGA_GET_CB_VOL_LVL:
					{
						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_CB_VOL_LVL;
							// UART_FPGA_OBC_Tx_Buffer[0] & [1] is set when first sending the msg to the FPGA
							// TO DO: change FPGA implementation to also send the probe ID
							UART_FPGA_OBC_Tx_Buffer[2] = UART_FPGA_Rx_Buffer[3];
							UART_FPGA_OBC_Tx_Buffer[3] = UART_FPGA_Rx_Buffer[4];
							memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 4);
							msg_to_send.TM_data_len			= 4;
						}
						break;
					}

					case FPGA_GET_SWT_VOL_LVL:
					{
						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_VOL_LVL;
							// UART_FPGA_OBC_Tx_Buffer[0] & [1] are set when first sending the msg to the FPGA
							// TO DO: change FPGA implementation to also send the probe ID and step ID back with the voltage level
							UART_FPGA_OBC_Tx_Buffer[3] = UART_FPGA_Rx_Buffer[3];
							UART_FPGA_OBC_Tx_Buffer[4] = UART_FPGA_Rx_Buffer[4];
							memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 5);
							msg_to_send.TM_data_len			= 5;
						}
						break;
					}

					case FPGA_GET_SWT_STEPS:
					{
						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_STEPS;
							UART_FPGA_OBC_Tx_Buffer[1] = UART_FPGA_Rx_Buffer[3];
							memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 2);
							msg_to_send.TM_data_len			= 2;
						}
						break;
					}

					case FPGA_GET_SWT_SAMPLES_PER_STEP:
					{
						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLES_PER_STEP;
							UART_FPGA_OBC_Tx_Buffer[1] = UART_FPGA_Rx_Buffer[3];
							UART_FPGA_OBC_Tx_Buffer[2] = UART_FPGA_Rx_Buffer[4];
							memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
							msg_to_send.TM_data_len			= 3;
						}

						break;
					}

					case FPGA_GET_SWT_SAMPLE_SKIP:
					{
						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLE_SKIP;
							UART_FPGA_OBC_Tx_Buffer[1] = UART_FPGA_Rx_Buffer[3];
							UART_FPGA_OBC_Tx_Buffer[2] = UART_FPGA_Rx_Buffer[4];
							memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
							msg_to_send.TM_data_len			= 3;
						}
						break;
					}

					case FPGA_GET_SWT_SAMPLES_PER_POINT:
					{
						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_SAMPLES_PER_POINT;
							UART_FPGA_OBC_Tx_Buffer[1] = UART_FPGA_Rx_Buffer[3];
							UART_FPGA_OBC_Tx_Buffer[2] = UART_FPGA_Rx_Buffer[4];
							memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
							msg_to_send.TM_data_len			= 3;
						}
						break;
					}

					case FPGA_GET_SWT_NPOINTS:
					{
						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							UART_FPGA_OBC_Tx_Buffer[0] = FPGA_GET_SWT_NPOINTS;
							UART_FPGA_OBC_Tx_Buffer[1] = UART_FPGA_Rx_Buffer[3];
							UART_FPGA_OBC_Tx_Buffer[2] = UART_FPGA_Rx_Buffer[4];
							memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 3);
							msg_to_send.TM_data_len			= 3;
						}

						break;
					}

					case FPGA_EN_CB_MODE:
					{
						send_buffered_data = 0;

						if(PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12))
						{
							uint8_t *dest = Constant_Bias_Mode_Buffer[Buffer_Index] + Measurement_Index * 8;
							memcpy(dest, UART_FPGA_Rx_Buffer + 3, 8);
							Measurement_Index++;

							if(Measurement_Index == CB_MODE_BUFFERED_MEASUREMENTS)
							{
								msg_to_send.TM_data[0] = FPGA_EN_CB_MODE;
								memcpy(msg_to_send.TM_data + 1,
										Constant_Bias_Mode_Buffer[Buffer_Index],
										8 * CB_MODE_BUFFERED_MEASUREMENTS
								);
								msg_to_send.TM_data_len	= 8 * CB_MODE_BUFFERED_MEASUREMENTS + 1;

								send_buffered_data = 1;
								Measurement_Index = 0;
								Buffer_Index ^= 1;
							}
						}
						break;
					}

          case SC_BIAS:
            {
              if (PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12)) {

                UART_FPGA_OBC_Tx_Buffer[0] = UART_FPGA_Rx_Buffer[3];
                UART_FPGA_OBC_Tx_Buffer[1] = UART_FPGA_Rx_Buffer[4];
                UART_FPGA_OBC_Tx_Buffer[2] = UART_FPGA_Rx_Buffer[5];
                UART_FPGA_OBC_Tx_Buffer[3] = UART_FPGA_Rx_Buffer[6];
                UART_FPGA_OBC_Tx_Buffer[4] = UART_FPGA_Rx_Buffer[7];
                UART_FPGA_OBC_Tx_Buffer[5] = UART_FPGA_Rx_Buffer[8];
                UART_FPGA_OBC_Tx_Buffer[6] = UART_FPGA_Rx_Buffer[9];
                UART_FPGA_OBC_Tx_Buffer[7] = UART_FPGA_Rx_Buffer[10];

                memcpy(msg_to_send.TM_data, UART_FPGA_OBC_Tx_Buffer, 8);

                msg_to_send.TM_data_len = 8;
              }
              break;
            }

					case FPGA_GET_SENSOR_DATA:
          {
            if (PUS_8_check_FPGA_msg_format(UART_FPGA_Rx_Buffer, 12)) {

              msg_to_send.PUS_HEADER_PRESENT = 1;
              msg_to_send.SERVICE_ID = HOUSEKEEPING_SERVICE_ID;   
              msg_to_send.SUBTYPE_ID = HK_PARAMETER_REPORT;       

              msg_to_send.TM_data[0] = UART_FPGA_Rx_Buffer[3]; // HK ID 

              memcpy(&msg_to_send.TM_data[1], &UART_FPGA_Rx_Buffer[4], 7);

              msg_to_send.TM_data_len = 8;
            }
            break;
          }

					default:
						break;
				}

				if(UART_FPGA_Rx_Buffer[2] != FPGA_EN_CB_MODE ||
					(UART_FPGA_Rx_Buffer[2] == FPGA_EN_CB_MODE && send_buffered_data == 1))
				{
          xQueueSend(UART_OBC_Out_Queue, &msg_to_send, portMAX_DELAY);
        }
				HAL_UART_Receive_DMA(&huart5, UART_FPGA_Rx_Buffer, 2 + 9 + 1);
			}
		}
	osDelay(1);
  }
  /* USER CODE END handle_UART_IN_FPGA */
}

/* USER CODE BEGIN Header_handle_Watchdog */
/**
* @brief Function implementing the Watchdog_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_handle_Watchdog */
void handle_Watchdog(void const * argument)
{
  /* USER CODE BEGIN handle_Watchdog */
  /* Infinite loop */
	for(;;)
	{
		// This task has the lowest priority and if available to run, will reset the internal hardware watchdog
	  HAL_IWDG_Refresh(&hiwdg);  // Refresh watchdog
	  osDelay(1000);
	}
  /* USER CODE END handle_Watchdog */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
         tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
