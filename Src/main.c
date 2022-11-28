/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "MorseConversionLayer.h"
#include "stm32l4s5i_iot01_qspi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAITTIME 1
#define THRES_DISTANCE 100
#define DURATION_THRES 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_up;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId receiveTerminalHandle;
osThreadId sideTaskHandle;
/* USER CODE BEGIN PV */
char output[50] = {0};
VL53L0X_Dev_t MyDevice;
VL53L0X_Dev_t* pMyDevice = &MyDevice;
VL53L0X_Error status;
uint8_t VhvSettings = 0;
uint8_t PhaseCal = 0;
uint32_t refSpadCount;
uint8_t isApertureSpads;
VL53L0X_RangingMeasurementData_t rangeData;
VL53L0X_RangingMeasurementData_t* pRangeData = &rangeData;
uint16_t distance_output = 8190;

int sleepcounter = 0;
char morse[6];
char morseBuffer[6];
char* dummy[1];
char* i;
int interruptFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_DAC1_Init(void);
static void MX_OCTOSPI1_Init(void);
void StartReceivingTerminal(void const * argument);
void StartSideTask(void const * argument);

/* USER CODE BEGIN PFP */
void print_error(VL53L0X_Error status);
void print_to_terminal(char* output);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int32_t arrayOne[50000];
uint16_t forDac[50000];

uint16_t dacValue = 0;
int32_t micValue = 0;
void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin);
void HAL_DFSDM_FilterRegConvCpltCallback (DFSDM_Filter_HandleTypeDef * hdfsdm_filter);



uint8_t buff[1000];
uint8_t rxData[4];
uint8_t UART2_rxBuffer[1] = {0};
int flag = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_DFSDM1_Init();
  MX_DAC1_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  HAL_DMA_Init(&hdma_dac1_ch1);
  HAL_DMA_Init(&hdma_dfsdm1_flt0);
  HAL_DMA_Init(&hdma_tim2_up);
  BSP_QSPI_Init();
  pMyDevice->I2cHandle = &hi2c2;
  pMyDevice->I2cDevAddr      = 0x52;
  pMyDevice->comms_type      =  1;
  pMyDevice->comms_speed_khz =  400;
  VL53L0X_ResetDevice(&MyDevice);
  status = VL53L0X_DataInit(&MyDevice);
  status = VL53L0X_StaticInit(pMyDevice);
  status = VL53L0X_PerformRefCalibration(pMyDevice,
          		&VhvSettings, &PhaseCal);
  status = VL53L0X_PerformRefSpadManagement(pMyDevice,
          		&refSpadCount, &isApertureSpads);
  status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  char ascii_char[2]; // null-terminated string of length 1.

  HAL_UART_Receive_IT(&huart1, UART2_rxBuffer, 1);



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
  /* definition and creation of receiveTerminal */
  osThreadDef(receiveTerminal, StartReceivingTerminal, osPriorityNormal, 0, 128);
  receiveTerminalHandle = osThreadCreate(osThread(receiveTerminal), NULL);

  /* definition and creation of sideTask */
  osThreadDef(sideTask, StartSideTask, osPriorityNormal, 0, 128);
  sideTaskHandle = osThreadCreate(osThread(sideTask), NULL);

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
	char* array[1];
	read_char_morse();
	array[0] = morse;
	convertMorseToText(array, ascii_char, 1);
	ascii_char[1]='\0';
	snprintf(output, sizeof(output), "%s\n\r", ascii_char);
	HAL_UART_Transmit(&huart1, output, strlen(output), 100);
	snprintf(output, sizeof(output), "%s\n\r", morse);
	HAL_UART_Transmit(&huart1, output, strlen(output), 100);



//	  status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice, &rangeData);
//	  distance_output = pRangeData->RangeMilliMeter;
//	  if (distance_output>THRES_DISTANCE && mode==1)
//	  {
//		  mode = 0;
//	  snprintf(output, sizeof(output), "time: %d\n", counter);
//	  HAL_UART_Transmit(&huart1, output, strlen(output), 100);
//		  counter = 0;
//	  } else  if (distance_output<THRES_DISTANCE && mode==0) {
//		  counter++;
//
//		  mode = 1;
//	  } else if (distance_output<THRES_DISTANCE) {
//		  counter++;
//	  }
//	  //snprintf(output, sizeof(output), "Distance: %d\n", distance_output);
//
//	  HAL_Delay(WAITTIME);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 55;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 34;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1814;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BLUEBUTTON_Pin */
  GPIO_InitStruct.Pin = BLUEBUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BLUEBUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void print_error(VL53L0X_Error status) {
    char buffer[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(status, buffer);
    printf("API Status: %i : %s\n", status, buffer);
}

void read_char_morse() // reads a single ascii character (multiple morse codes)
{
	int mode = 0; // 0=not started reading. 1=reading(waiting for hand to leave sensor). 2=waiting for additional morse code
	int i = 0;
	int counter = 0;
	while(1)
	{
		  status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice, &rangeData);
		  distance_output = pRangeData->RangeMilliMeter;
		  if (distance_output>THRES_DISTANCE)
		  {
			  if (mode==1){
				  mode = 2;
				  morse[i] = (counter>=DURATION_THRES) ? '-' : '.';
				  i++;
				  counter = 0;
			  } else if (mode==2){
				  counter++;
				  if (counter>14){
					  morse[i]='\0';
					  break;
				  }
			  }


		  } else  if (distance_output<THRES_DISTANCE && (mode==0||mode==2)) {
			  counter++;
			  mode = 1;
		  } else if (distance_output<THRES_DISTANCE && mode==1) {
			  counter++;
		  }
		  HAL_Delay(WAITTIME);
	}



}


void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
	// put to sleep once pushbutton is pressed
//	if (sleepcounter == 0){
//		HAL_SuspendTick();
//		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,PWR_SLEEPENTRY_WFI);
//		HAL_ResumeTick();
//
//	}
//	sleepcounter = (sleepcounter + 1) % 2;
//

//    if(HAL_GPIO_ReadPin(GPIOB, LED_Pin) == GPIO_PIN_SET) {
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
//    } else {
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
//	}
//    sprintf(buff, "Board\n\r");
//    HAL_UART_Transmit_DMA(&huart1, &buff, strlen(buff));

//
//	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
//	HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
//	for (int i = 0; i < 50000 ; i++){
//		arrayOne[i] = (int32_t)0;
//	}
//	HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, &arrayOne, 50000);
//
//	if(HAL_GPIO_ReadPin(GPIOB, LED_Pin) == GPIO_PIN_SET) {
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
//	} else {
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
//	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	interruptFlag = 1;
	HAL_UART_Receive_IT(&huart1, UART2_rxBuffer, 1);

}
//
void HAL_DFSDM_FilterRegConvCpltCallback (DFSDM_Filter_HandleTypeDef * hdfsdm_filter) {

	for (int i = 0; i < 50000 ; i++){

		micValue = arrayOne[i];

		int32_t temp = arrayOne[i] >> 8;
//		if(temp < -30000) { temp = -30000; }
//		else if(temp > 30000) { temp = 30000; }
		temp = (temp/256)/*max here is 32768*/ + 32768; // total max is 65536 <= 2^16 for 16 bits.

		forDac[i] = (uint16_t)((int16_t)(arrayOne[i] >> 8) + 32748);

		//dacValue = (uint16_t) temp;
	}


//	BSP_QSPI_Erase_Block(0x330000);
//	if (BSP_QSPI_Write(&forDac, 0x330000, 100000) != QSPI_OK) Error_Handler();

//	if (BSP_QSPI_Read(&forDac, 0x210000, 100000) != 0) Error_Handler();
//	HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

}




/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReceivingTerminal */
/**
  * @brief  Function implementing the receiveTerminal thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReceivingTerminal */
void StartReceivingTerminal(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
    if (interruptFlag == 1) {
    	HAL_UART_Transmit(&huart1, UART2_rxBuffer, 1, 100);
    	char* morse[1];
    	morse[0] = morseBuffer;
    	//convertMorseToText(array, ascii_char, 1);
    	convertTextToMorse(UART2_rxBuffer, morse, 1);
    	i = morse[0];
    	//HAL_UART_Transmit(&huart1, morse, strlen(morse), 100);

    	for(int x = 0; x < strlen(*(morse)); x++) {
    		char m = (*(morse))[x];

    		if(strcmp(m, '-') == 0) {
    			HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
    			osDelay(1500);
    		} else if(strcmp(m, '.') == 0) {
    			HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
    			osDelay(700);
    		}
    		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
    		osDelay(200);
    	}

    	// read out loud the passed letter to DAC
    	char toRead = tolower(UART2_rxBuffer[0]);
//    	int offset = toRead - 'a';
    	if (toRead == 'a'){
			if (BSP_QSPI_Read(&forDac, 0x000000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);
    	}
    	if (toRead == 'b'){
			if (BSP_QSPI_Read(&forDac, 0x020000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'c'){
			if (BSP_QSPI_Read(&forDac, 0x040000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'd'){
			if (BSP_QSPI_Read(&forDac, 0x060000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'e'){
			if (BSP_QSPI_Read(&forDac, 0x080000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'f'){
			if (BSP_QSPI_Read(&forDac, 0x0A0000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'g'){
			if (BSP_QSPI_Read(&forDac, 0x0C0000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'h'){
			if (BSP_QSPI_Read(&forDac, 0x0E0000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'i'){
			if (BSP_QSPI_Read(&forDac, 0x100000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'j'){
			if (BSP_QSPI_Read(&forDac, 0x120000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'k'){
			if (BSP_QSPI_Read(&forDac, 0x140000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'l'){
			if (BSP_QSPI_Read(&forDac, 0x160000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'm'){
			if (BSP_QSPI_Read(&forDac, 0x180000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'n'){
			if (BSP_QSPI_Read(&forDac, 0x1A0000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'o'){
			if (BSP_QSPI_Read(&forDac, 0x1C0000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'p'){
			if (BSP_QSPI_Read(&forDac, 0x1E0000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'q'){
			if (BSP_QSPI_Read(&forDac, 0x210000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'r'){
			if (BSP_QSPI_Read(&forDac, 0x330000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 's'){
			if (BSP_QSPI_Read(&forDac, 0x230000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 't'){
			if (BSP_QSPI_Read(&forDac, 0x250000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'u'){
			if (BSP_QSPI_Read(&forDac, 0x270000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
    	if (toRead == 'v'){
			if (BSP_QSPI_Read(&forDac, 0x290000, 100000) != 0) Error_Handler();
			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

    	}
       	if (toRead == 'w'){
    			if (BSP_QSPI_Read(&forDac, 0x2B0000, 100000) != 0) Error_Handler();
    			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

		}
       	if (toRead == 'x'){
    			if (BSP_QSPI_Read(&forDac, 0x2D0000, 100000) != 0) Error_Handler();
    			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

		}
       	if (toRead == 'y'){
    			if (BSP_QSPI_Read(&forDac, 0x2F0000, 100000) != 0) Error_Handler();
    			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

		}
       	if (toRead == 'z'){
    			if (BSP_QSPI_Read(&forDac, 0x310000, 100000) != 0) Error_Handler();
    			HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, &forDac, 100000, DAC_ALIGN_12B_R);

		}


    	interruptFlag = 0;
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSideTask */
/**
* @brief Function implementing the sideTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSideTask */
void StartSideTask(void const * argument)
{
  /* USER CODE BEGIN StartSideTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
    char Message[] = "Write anything on Serial Terminal\r\n";
    HAL_UART_Transmit(&huart1, (uint8_t *)Message, strlen(Message), 10);
  }
  /* USER CODE END StartSideTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
