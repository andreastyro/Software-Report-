/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdbool.h> // Add this include for boolean support
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define size 10
#define queue_length 10
#define sensor_queue_item_size sizeof(int)
#define configUSE_QUEUE_SETS

QueueHandle_t xSensorQueue;
QueueHandle_t xRoundQueue;
SemaphoreHandle_t xBinarySemaphore;
SemaphoreHandle_t xRoundSemaphore;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId roundTaskHandle;
osThreadId DisplayTaskHandle;
osThreadId zombie_followHandle;
osThreadId shootTaskHandle;
osThreadId moveHandle;
osThreadId fetch_input_datHandle;
osThreadId switch_modeTaskHandle;
osThreadId switch_modeHandle;
osThreadId deathTaskHandle;
osThreadId healTaskHandle;
/* USER CODE BEGIN PV */
uint16_t Xaxis = 0;
uint16_t Yaxis = 0;
uint16_t rawValues[2];

uint8_t screenstatus[11];
uint8_t data[2];

static const uint8_t LEDMAT_ADD = 0x75 << 1;
static const uint8_t PAGE_1 = 0x00;
//static const uint8_t PAGE_2 = 0x01;
//static const uint8_t PAGE_3 = 0x02;
//static const uint8_t PAGE_4 = 0x03;
//static const uint8_t PAGE_5 = 0x04;
//static const uint8_t PAGE_6 = 0x05;
//static const uint8_t PAGE_7 = 0x06;
//static const uint8_t PAGE_8 = 0x07;
static const uint8_t FUN_REG = 0x0B;
static const uint8_t COM_REG = 0xFD;
static const uint8_t MAT_ROW[11] = {0x00, 0x02, 0x04, 0x06, 0x08, 0x0A, 0x01, 0x03, 0x05, 0x07, 0x09};
static const uint8_t MAT_COL[7] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};
char msg[100];
uint16_t c[2] = {5,3};

int mode = 0;
int count = 0;
int bullet_pos_x = 0;
int bullet_pos_y = 0;
//int zombie_counter = 0;
int zombie_x = 0;
int zombie_y = 0;
int zombie_state = 0;
int zombie_count = 5;



#define DWT_CTRL (*(volatile uint32_t*) 0xE0001000)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
void StartRoundTask(void const * argument);
void StartDisplayTask(void const * argument);
void Start_Zombie_Follow(void const * argument);
void StartShootTask(void const * argument);
void StartMoveTask(void const * argument);
void StartFetchTask(void const * argument);
void StartSwitchModeTask(void const * argument);
void StartDeathTask(void const * argument);
void StartHealTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

////the code below is an interrupt that is triggered on an Update event (i.e. when the timer reaches its ARR value)
////if you look at TIM2 in the .ioc file (the GUI tab) you will see it has a source of 84MHz, an ARR of 84000-1 and
////a prescaler of 10, meaning this interrupt will occur every 10 ms.

void clearscreen(){
		for(uint8_t r = 0; r <11; r++){
				  data[0] = COM_REG;
				  data[1] = PAGE_1;
				  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
				  //HAL_Delay(10);
				  data[0] = MAT_ROW[r];
				  data[1] = 0x00;
				  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
				  //HAL_Delay(10);
				  for(uint8_t r = 0; r <11; r++){
				  		screenstatus[r] = 0;
				  }
		  }
}
void turnoffscreen(){
	  data[0] = COM_REG;
	  data[1] = FUN_REG;
	  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
	  //HAL_Delay(10);
	  data[0] = 0x0A; //shutdown on/off
	  data[1] = 0x00; //off
	  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}
void turnonscreen(){
	  data[0] = COM_REG;
	  data[1] = FUN_REG;
	  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
	 // HAL_Delay(10);
	  data[0] = 0x0A; //shutdown on/off
	  data[1] = 0x01; //on
	  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}
void setpixel(uint8_t r, uint8_t c){
	  data[0] = COM_REG;
	  data[1] = PAGE_1;
	  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
	  data[0] = MAT_ROW[r];
	  data[1] = MAT_COL[c];
	  HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}
void addpixel(uint8_t r, uint8_t c){
	screenstatus[r] |= MAT_COL[c];
	data[0] = COM_REG;
	data[1] = PAGE_1;
	HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
	data[0] = MAT_ROW[r];
	data[1] = screenstatus[r];
	HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}
void removepixel(uint8_t r, uint8_t c){
	screenstatus[r] &= ~MAT_COL[c];
	data[0] = COM_REG;
	data[1] = PAGE_1;
	HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
	data[0] = MAT_ROW[r];
	data[1] = screenstatus[r];
	HAL_I2C_Master_Transmit(&hi2c1, LEDMAT_ADD, data, 2, HAL_MAX_DELAY);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Instance == TIM1) {
	    HAL_IncTick();
	  }

}

void SetBacklightBrightness(uint8_t brightness)
{
    if (brightness > 100) brightness = 100; // Limit brightness to 0-100%
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, brightness); // Adjust PWM duty cycle
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  SetBacklightBrightness(50); // Set brightness to 50%
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Start PWM on TIM2 Channel 1

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  //SEGGER_UART_init(500000);

  DWT_CTRL |= (1<<0);
  NVIC_SetPriorityGrouping(0);
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

  ////This begins the process of storing our ADC readings into the DMA. The DMA can be thought of a temporary storage location.
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *) rawValues, 2);
  ////This begins our timer 2
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */

  // Create the binary semaphore

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of roundTask */
  osThreadDef(roundTask, StartRoundTask, osPriorityHigh, 0, 128);
  roundTaskHandle = osThreadCreate(osThread(roundTask), NULL);

  /* definition and creation of DisplayTask */
  osThreadDef(DisplayTask, StartDisplayTask, osPriorityHigh, 0, 256);
  DisplayTaskHandle = osThreadCreate(osThread(DisplayTask), NULL);

  /* definition and creation of zombie_follow */
  osThreadDef(zombie_follow, Start_Zombie_Follow, osPriorityIdle, 0, 128);
  zombie_followHandle = osThreadCreate(osThread(zombie_follow), NULL);

  /* definition and creation of shootTask */
  osThreadDef(shootTask, StartShootTask, osPriorityIdle, 0, 128);
  shootTaskHandle = osThreadCreate(osThread(shootTask), NULL);

  /* definition and creation of move */
  osThreadDef(move, StartMoveTask, osPriorityIdle, 0, 128);
  moveHandle = osThreadCreate(osThread(move), NULL);

  /* definition and creation of fetch_input_dat */
  osThreadDef(fetch_input_dat, StartFetchTask, osPriorityIdle, 0, 128);
  fetch_input_datHandle = osThreadCreate(osThread(fetch_input_dat), NULL);

  /* definition and creation of switch_modeTask */
  osThreadDef(switch_modeTask, StartSwitchModeTask, osPriorityIdle, 0, 128);
  switch_modeTaskHandle = osThreadCreate(osThread(switch_modeTask), NULL);

  /* definition and creation of switch_mode */
  osThreadDef(switch_mode, StartSwitchModeTask, osPriorityHigh, 0, 128);
  switch_modeHandle = osThreadCreate(osThread(switch_mode), NULL);

  /* definition and creation of deathTask */
  osThreadDef(deathTask, StartDeathTask, osPriorityIdle, 0, 128);
  deathTaskHandle = osThreadCreate(osThread(deathTask), NULL);

  /* definition and creation of healTask */
  osThreadDef(healTask, StartHealTask, osPriorityIdle, 0, 128);
  healTaskHandle = osThreadCreate(osThread(healTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  xBinarySemaphore = xSemaphoreCreateBinary();
  xRoundSemaphore  = xSemaphoreCreateBinary();

  xSensorQueue = xQueueCreate(queue_length, sensor_queue_item_size);
  xRoundQueue = xQueueCreate(queue_length, sensor_queue_item_size);

  //xMutex = xSemaphoreCreateMutex();

  // Create FetchTask (common to all modes)
  xTaskCreate(StartFetchTask, "FetchTask", 1000, NULL, 1, NULL);
  xTaskCreate(StartMoveTask, "MoveTask", 1000, NULL, 1, NULL);
  xTaskCreate(StartShootTask, "ShootTask", 1000, NULL, 1, NULL);
  //xTaskCreate(Start_Zombie_Follow, "ZombieFollowTask", 1000, NULL, 2, NULL);
  xTaskCreate(StartRoundTask, "RoundTask", 1000, NULL, 2, NULL);

  vTaskResume(moveHandle);
  vTaskSuspend(shootTaskHandle);

  if (xBinarySemaphore != NULL){
	  xTaskCreate(StartSwitchModeTask, "SwitchModeTask", 1000, NULL, 2, NULL);
  }

  if (xRoundSemaphore != NULL){

  }

  //if (xMutex != NULL){
	  //xTaskCreate(StartRoundTask, "RoundTask", 1000, NULL, 2, NULL);
  //}

  //vTaskPrioritySet(moveHandle, 2);
  //vTaskPrioritySet(shootTaskHandle, 1);

  vTaskStartScheduler();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_interrupt_time = 0; // Timestamp of the last valid interrupt
    uint32_t current_time = HAL_GetTick();  // Current time in milliseconds

    if(GPIO_Pin == GPIO_PIN_4) // If The INT Source Is EXTI Line9 (A9 Pin)
    {

        if ((current_time - last_interrupt_time) > 300)
        {
            last_interrupt_time = current_time; // Debouncing Mechanism

            // Give the semaphore to unblock the shoot task
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;

            // Give the semaphore to unblock the task
            xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);

            // Request a context switch if necessary
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

        }

    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartRoundTask */
/**
  * @brief  Function implementing the roundTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartRoundTask */
void StartRoundTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  int round = 1;

  /* Infinite loop */
  for(;;)
  {

	if (xSemaphoreTake(xRoundSemaphore, portMAX_DELAY) == pdTRUE){
		  round = round + 1;
	      sprintf(msg, "WELL DONE SOLDIER! TIME FOR ROUND %d\r\n", round);
	      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

	      zombie_count = round * 5;
	}


    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_Start_Zombie_Follow */
/**
* @brief Function implementing the zombie_follow thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Zombie_Follow */
void Start_Zombie_Follow(void const * argument)
{
  /* USER CODE BEGIN Start_Zombie_Follow */

  int zombie_direction = 0;
  int edge_spawn = 0;
  int zombie_speed = 1000;

    /* Infinite loop */
  for(;;)
  {
	  if (zombie_state == 0){

		  zombie_direction = rand()%2;
		  edge_spawn = rand()%2;

		  if (zombie_direction == 0){
			  zombie_x = (rand() % 11) + 1;

			  if (edge_spawn == 0){
				  zombie_y = 0;
			  } else {
				  zombie_y = 7;
			  }

		  } else {
			  if (edge_spawn == 0){
				  zombie_x = 0;
			  } else {
				  zombie_x = 11;
			  }
			  zombie_y = (rand() % 7) + 1;
		  }

		  zombie_state = 1;

	  }

	  if (zombie_x < c[0]){
		  zombie_x++;

	  }else if(zombie_x > c[0]){
		  zombie_x--;
	  }

	  if (zombie_y < c[1]){
		  zombie_y++;
	  }
	  else if (zombie_y > c[1]){
		  zombie_y--;
	  }

	  //sprintf(msg, "Zombie X: %d\r\n", zombie_x);
	 // HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

	osDelay(800);

  }
  /* USER CODE END Start_Zombie_Follow */
}

/* USER CODE BEGIN Header_StartShootTask */
/**
* @brief Function implementing the shootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShootTask */
void StartShootTask(void const * argument)
{
  /* USER CODE BEGIN StartShootTask */

  /* Infinite loop */
  for(;;)
  {
      	  //sprintf(msg, "SHOOT TASK \r\n"); // Initialize the message
    	  //HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

		  bool receivedDirections[4]; // Array to store the received values

		  memset(receivedDirections, 0, sizeof(receivedDirections));

		  xQueueReceive(xSensorQueue, &receivedDirections[0], portMAX_DELAY);
		  xQueueReceive(xSensorQueue, &receivedDirections[1], portMAX_DELAY);
		  xQueueReceive(xSensorQueue, &receivedDirections[2], portMAX_DELAY);
		  xQueueReceive(xSensorQueue, &receivedDirections[3], portMAX_DELAY);

		  bool updated = false;

		  if(receivedDirections[2] == true){
			  //HAL_UART_Transmit(&huart2, (uint8_t *) west, strlen(west), HAL_MAX_DELAY);
			  for (int i = c[0]; i >= 0; i--){

				static uint32_t last_time = 0;

					osDelay(20);

					bullet_pos_x = i;
					bullet_pos_y = c[1];

					addpixel(bullet_pos_x, bullet_pos_y);

					if(bullet_pos_x + 1 != c[0]){
						removepixel(bullet_pos_x+1, c[1]);
					}

					if(bullet_pos_x == zombie_x){
			            zombie_state = 0;
			            zombie_count--;
					}

				}
			  updated = true;
		  }

		  if(receivedDirections[3] == true){
			  //HAL_UART_Transmit(&huart2, (uint8_t *) east, strlen(east), HAL_MAX_DELAY);
			  for (int i = c[0]; i <= 11; i++){
				static uint32_t last_time = 0;

				osDelay(20);

				  bullet_pos_x = i;
				  bullet_pos_y = c[1];
				  addpixel(bullet_pos_x, bullet_pos_y);

					if(bullet_pos_x - 1 != c[0]){
						removepixel(bullet_pos_x-1, c[1]);
					}

					if(bullet_pos_x == zombie_x){
			            zombie_state = 0;
			            zombie_count--;
					}
			  }

			  updated = true;
		  }

	      sprintf(msg, "COUNT %d\r\n", zombie_count);
	      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

		  if (zombie_count <= 0){
	            BaseType_t xRoundPriorityTaskWoken = pdFALSE;

	            // Give the semaphore to unblock the task
	            xSemaphoreGiveFromISR(xRoundSemaphore, xRoundPriorityTaskWoken);

	            // Request a context switch if necessary
	            portYIELD_FROM_ISR(xRoundPriorityTaskWoken);

		  }

		osDelay(1);
  }
  /* USER CODE END StartShootTask */
}

/* USER CODE BEGIN Header_StartMoveTask */
/**
* @brief Function implementing the move thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMoveTask */
void StartMoveTask(void const * argument)
{
  /* USER CODE BEGIN StartMoveTask */

	bool receivedDirections[4]; // Array to store the received values


  /* Infinite loop */
  for(;;)
  {

	  if(mode == 0){

		  //turnoffscreen();
		  //turnonscreen();
		  //clearscreen();

		  memset(receivedDirections, 0, sizeof(receivedDirections));

		  xQueueReceive(xSensorQueue, &receivedDirections[0], portMAX_DELAY);
		  xQueueReceive(xSensorQueue, &receivedDirections[1], portMAX_DELAY);
		  xQueueReceive(xSensorQueue, &receivedDirections[2], portMAX_DELAY);
		  xQueueReceive(xSensorQueue, &receivedDirections[3], portMAX_DELAY);

		  bool updated = false;

		  if(receivedDirections[0] == true){
			  //HAL_UART_Transmit(&huart2, (uint8_t *) north, strlen(north), HAL_MAX_DELAY);
	          //sprintf(msg, "C: %d\r\n", c[1]);
	          //HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
			  if(c[1] >= 6){
				  c[1] = 0;
			  }else{
				  c[1] = c[1] + 1;
			  }
			  updated = true;
		  }

		  if(receivedDirections[1] == true){
			 // HAL_UART_Transmit(&huart2, (uint8_t *) south, strlen(south), HAL_MAX_DELAY);
			  if(c[1] <= 0){
				  c[1] = 6;
			  }else{
				  c[1] = c[1] - 1;
			  }
			  updated = true;

		  }

			if(receivedDirections[2] == true){
				  //HAL_UART_Transmit(&huart2, (uint8_t *) west, strlen(west), HAL_MAX_DELAY);
				  if(c[0]<=0){
					  c[0] = 10;
				  }else{
					  c[0] = c[0] - 1;
				  }
				  updated = true;
			  }

			  if(receivedDirections[3] == true){
				  //HAL_UART_Transmit(&huart2, (uint8_t *) east, strlen(east), HAL_MAX_DELAY);
				  if(c[0] >= 10){
					  c[0] = 0;
				  }else{
					  c[0] = c[0] + 1;

				  }
				  updated = true;
			  }

			  if (updated) {
				  osDelay(200);  // Add a small delay to avoid processing consecutive moves too quickly
			  }

			  //sprintf(msg, "MOVE TASK \r\n"); // Initialize the message
			  //HAL_UART_Transmit(&huart2, (uint8_t *) msg, strlen(msg), HAL_MAX_DELAY);

			  osDelay(1);
	  }
  }
  /* USER CODE END StartMoveTask */
}

/* USER CODE BEGIN Header_StartFetchTask */
/**
* @brief Function implementing the fetch_input_dat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFetchTask */
void StartFetchTask(void const * argument)
{
  /* USER CODE BEGIN StartFetchTask */

	bool up = false;
	bool down = false;
	bool left = false;
	bool right = false;

	int count = 0;

  /* Infinite loop */
  for(;;)
  {
	////this updates the X and Y axes of my joystick
	  for(uint8_t i = 0; i<hadc1.Init.NbrOfConversion; i++){
		  Xaxis = (uint16_t) rawValues[0];
		  Yaxis = (uint16_t) rawValues[1];
	  }

	 up = (Yaxis < 10);     // High value means pushed up
	 down = (Yaxis > 3250);   // Low value means pushed down
	 left = (Xaxis < 10);   // Low value means pushed left
	 right = (Xaxis > 3200);  // High value means pushed right

	 //if stops immediate values

	 count = count+1;
	 static bool executed = false;

	 if (!executed) { // to avoid double execution of sending
	     executed = true;

		 xQueueSend(xSensorQueue, &up, portMAX_DELAY);
		 xQueueSend(xSensorQueue, &down, portMAX_DELAY);
		 xQueueSend(xSensorQueue, &left, portMAX_DELAY);
		 xQueueSend(xSensorQueue, &right, portMAX_DELAY);


         //sprintf(msg, "UP: %d\r\n", up);
         //HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);


	     // Reset flag after delay
	     osDelay(100);
	     executed = false;

	 }


    osDelay(400);
  }
  /* USER CODE END StartFetchTask */
}

/* USER CODE BEGIN Header_StartSwitchModeTask */
/**
* @brief Function implementing the switch_modeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSwitchModeTask */
void StartSwitchModeTask(void const * argument)
{
  /* USER CODE BEGIN StartSwitchModeTask */
  /* Infinite loop */
    for (;;)
    {
        // Wait for the semaphore
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE)
        {
            mode = !mode; // Toggle mode

            if (mode == 0){
                vTaskResume(moveHandle);
                vTaskSuspend(shootTaskHandle);
            }

            else {
                vTaskResume(shootTaskHandle);
                vTaskSuspend(moveHandle);
            }

            // Debugging
            sprintf(msg, "Mode changed to: %d\r\n", mode);
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
        }

        osDelay(10); // Avoid starving other tasks
    }
  /* USER CODE END StartSwitchModeTask */
}

/* USER CODE BEGIN Header_StartDeathTask */
/**
* @brief Function implementing the deathTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDeathTask */
void StartDeathTask(void const * argument)
{
  /* USER CODE BEGIN StartDeathTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDeathTask */
}

/* USER CODE BEGIN Header_StartHealTask */
/**
* @brief Function implementing the healTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHealTask */
void StartHealTask(void const * argument)
{
  /* USER CODE BEGIN StartHealTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHealTask */
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
