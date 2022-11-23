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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define WAITTIME 1
#define THRES_DISTANCE 100
#define DURATION_THRES 8

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId readDistanceHandle;
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
char morse[6];
char morseBuffer[6];
char* dummy[1];
char* i;

// ******************************************************** from main
char ascii_char[2]; // null-terminated string of length 1.
// ********************************************************


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void StartReadingDistance(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	char Message[] = "Write anything on Serial Terminal\r\n";

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
  /* USER CODE BEGIN 2 */

//  HAL_UART_Receive_DMA (&huart1, rxData, 4);
//  flag = 1;

  //HAL_UART_Transmit(&huart1, (uint8_t *)Message, strlen(Message), 10);
  //HAL_UART_Receive_IT(&huart1, UART2_rxBuffer, 1);

  //********************************************
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
//  char ascii_char[2]; // null-terminated string of length 1.
  //********************************************

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
  /* definition and creation of readDistance */
  osThreadDef(readDistance, StartReadingDistance, osPriorityNormal, 0, 128);
  readDistanceHandle = osThreadCreate(osThread(readDistance), NULL);

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
//	  if(strcmp(i, "-..-") == 0) {
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
//		HAL_Delay(2000);
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
//		HAL_Delay(1000);
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
//		HAL_Delay(2000);
//		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
//		HAL_Delay(500);
//		i = morse;
//	  }



//	  char* array[1];
//	  read_char_morse();
//	  array[0] = morse;
//	  convertMorseToText(array, ascii_char, 1);
//	  ascii_char[1]='\0';
//	  snprintf(output, sizeof(output), "%s\n\r", ascii_char);
//	  HAL_UART_Transmit(&huart1, output, strlen(output), 100);
//	  snprintf(output, sizeof(output), "%s\n\r", morse);
//	  HAL_UART_Transmit(&huart1, output, strlen(output), 100);
//
//	  int i = 0;


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
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//*******************************************************
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
//*******************************************************

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
    if(HAL_GPIO_ReadPin(GPIOB, LED_Pin) == GPIO_PIN_SET) {
		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
    } else {
		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
	}
    sprintf(buff, "Board\n\r");
    HAL_UART_Transmit_DMA(&huart1, &buff, strlen(buff));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Transmit(&huart1, UART2_rxBuffer, 1, 100);
	char* character[1];
	character[0] = morseBuffer;
	convertTextToMorse(UART2_rxBuffer, character, 1);

	i= character[0];


	HAL_UART_Transmit(&huart1, character, strlen(character), 100);
	HAL_UART_Receive_IT(&huart1, UART2_rxBuffer, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	//HAL_UART_Receive_DMA (&huart1, rxData, 4);
}


//void EXTI0_IRQHandler(void)
//{
//  __asm("CPSIE i"); // depending on your compiler's inline asm syntax
//
//}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartReadingDistance */
/**
  * @brief  Function implementing the readDistance thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartReadingDistance */
void StartReadingDistance(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
	char* array[1];
	read_char_morse();
	array[0] = morse;
	convertMorseToText(array, ascii_char, 1);
	ascii_char[1]='\0';
	snprintf(output, sizeof(output), "%s\n\r", ascii_char);
	HAL_UART_Transmit(&huart1, output, strlen(output), 100);
	snprintf(output, sizeof(output), "%s\n\r", morse);
	HAL_UART_Transmit(&huart1, output, strlen(output), 100);

	int i = 0;
  }
  /* USER CODE END 5 */
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
