/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/udp.h"
#include <sys/socket.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Port on remote PC */
#define PORT 					(5050)

/* Constants for current calculation */
#define VDD_5	 				(4.76)
#define VDD_3					(3.3)
#define ADC_RES_12				(4096)
#define ADC_RES_16				(65536)
#define VOL_OFFSET				(2.365)
#define SENSITIVITY				(0.400)
#define MAX_NUMBER_OF_DIGITS	(16)

/* Registers in VCT Monitor Click*/
#define	STATUS_REG				(0x00)
#define	CONTROL_REG				(0x01)
#define TRIGGER_REG				(0x02)
#define CURRENT_REG				(0x06)

/* Device addresses */
#define HAL_CLICK_9_ADDR		(0x9A)
#define HAL_CLICK_6_ADDR		(0x9B)
#define VCT_MONITOR_ADDR		(0x98)

/* Board combination macros*/
//#define HAL_CLICK_9_ADC
#define HAL_CLICK_9_I2C
#define USING_VCT

/* Connected click board */
#define LOW_CURRENT_BOARD		(VCT_MONITOR_ADDR)
#define HIGH_CURRENT_BOARD		(HAL_CLICK_9_ADDR)


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c4;

TIM_HandleTypeDef htim16;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* I2C VCT trigger value */
uint8_t triggerValue = 0x01;

/* I2C VCT configuration value */
uint8_t configVct = 0x19;

/* VCT current - test time */
float realLowCurrent = 0;

/* Increments every 1ms */
volatile uint32_t timeInUs = 0;

/* Synchronization between SenseTask and UDP task (StartDefaultTask) */
QueueHandle_t currentQueue;

/* Queue storing data structure */
struct currentAndTime {
	uint32_t currentHigh;
	uint32_t currentLow;
	uint32_t timeStamp;
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C4_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Checks if right measuring board is connected to the Nucleo */
HAL_StatusTypeDef PingSensors(void);

/* VCT Monitor Click initialization*/
void Vct_Init(void);

/* Getting high current value */
uint32_t GetHighCurrent(void);

/* Getting low current value */
uint32_t GetLowCurrent(void);

/* Getting data from VCT Monitor Click */
uint32_t GetDataFromVct(void);

/* Getting data from HAL_CLICK */
uint32_t GetDataFromHal(void);

/* Getting data from ADC */
uint32_t GetDataFromAdc(void);

/* Timer to determine I2C polling time, which is 10ms  */
void MyTimerCallback(TimerHandle_t xTimer);
TimerHandle_t timer = NULL;

/* SenseTask to communicate with HALL click board*/
void SenseTask(void const * argument);
TaskHandle_t senseTaskHandle;

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

  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
	int32_t timeout;
/* USER CODE END Boot_Mode_Sequence_0 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
	/* Wait until CPU2 boots and enters in stop mode or timeout*/
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		Error_Handler();
	}
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
/* USER CODE BEGIN Boot_Mode_Sequence_2 */
	/* When system initialization is finished, Cortex-M7 will release Cortex-M4 by means of
	HSEM notification */
	/*HW semaphore Clock enable*/
	__HAL_RCC_HSEM_CLK_ENABLE();
	/*Take HSEM */
	HAL_HSEM_FastTake(HSEM_ID_0);
	/*Release HSEM in order to notify the CPU2(CM4)*/
	HAL_HSEM_Release(HSEM_ID_0,0);
	/* wait until CPU2 wakes up from stop mode */
	timeout = 0xFFFF;
	while((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) == RESET) && (timeout-- > 0));
	if ( timeout < 0 )
	{
		Error_Handler();
	}
/* USER CODE END Boot_Mode_Sequence_2 */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C4_Init();
  MX_TIM16_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */


	if(PingSensors() != HAL_OK)
	{
		while(1);
	}

#ifdef USING_VCT
	Vct_Init();
#endif


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
	/* Creating queue to synchronize reception of current samples from I2C and its sending to UDP server */
	currentQueue = xQueueCreate(10, sizeof(struct currentAndTime));
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/*Timer creation and starting */
	timer = xTimerCreate("Continuous", pdMS_TO_TICKS(1), pdTRUE, (void *)0, MyTimerCallback);
	xTimerStart(timer,portMAX_DELAY);

	/* SenseTask creating */
	osThreadDef(senseTask, SenseTask, osPriorityNormal, 0, 256);
	senseTaskHandle = osThreadCreate(osThread(senseTask), NULL);
	/* add threads, ... */

	/* Starting timer16 */
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_ADC_Start_IT(&hadc2);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 24;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_387CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c4.Init.Timing = 0x10C0ECFF;
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
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 200-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : CEC_CK_MCO1_Pin */
  GPIO_InitStruct.Pin = CEC_CK_MCO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(CEC_CK_MCO1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* Get data from HAL CLICK*/
uint32_t GetDataFromHal(void)
{
	static float currentHall = 0;
	static uint8_t receiveBufferHall[2];
	static uint16_t voltageDigitalHall = 0;

	HAL_I2C_Master_Receive(&hi2c4, 0x9B, receiveBufferHall, 2, 10);
	voltageDigitalHall = (((((uint16_t)receiveBufferHall[0]) << 8) & 0xFF00) | (((uint16_t)receiveBufferHall[1]) & 0x00FF));

	/* Formula which transforms voltage into current [mA] */
	currentHall = abs((floor((VDD_5*voltageDigitalHall/ADC_RES_12 - VOL_OFFSET)*1000/SENSITIVITY)));
	return ((uint32_t)(currentHall));



}
/* Get data from VCT Monitor */
uint32_t GetDataFromVct(void)
{
	static uint16_t currentVtc = 0;
	static uint32_t currVtc = 0;
	static uint8_t currentVtcBytes[2];

	/* Getting data */
	if(HAL_I2C_Mem_Read(&hi2c4, (uint16_t)VCT_MONITOR_ADDR | 0x01, CURRENT_REG, 1, currentVtcBytes, 2, 10) != HAL_OK)
	{
		return 0;
	}
	/* Getting 16-bit current value */
	currentVtc = (((uint16_t)currentVtcBytes[0]) << 8 & 0x7F00) | ((uint16_t)currentVtcBytes[1] & 0x00FF);

	/* Checking bit 14 -> sign bit, if sign = 1, current is negative and needs to be complemented */

	if(currentVtc & 0x4000)
	{
		/* 2's complement value needs to be calculated */
		currVtc = (~(currentVtc) & 0x00007FFF) + 0x01;
		realLowCurrent = ((currVtc*19.42)/(0.47));
		return ((uint32_t)(floor(realLowCurrent)));
	}
	else
	{
		realLowCurrent = ((currentVtc*19.42)/(0.47));
		return ((uint32_t)(floor(realLowCurrent)));

	}

}
/* Get data from ADC HAL CLICK 9*/
uint32_t GetDataFromAdc(void)
{
	static float adcValue = 0;
	static float adcCurrent = 0;

	adcValue =(HAL_ADC_GetValue(&hadc2)*VDD_3/ADC_RES_16) - 2.13;
	adcCurrent = abs(floor(adcValue*1000/SENSITIVITY));

	return ((uint32_t)(adcCurrent));
}
/* Get high current value */
uint32_t GetHighCurrent(void)
{

	switch (HIGH_CURRENT_BOARD)
	{

	case HAL_CLICK_9_ADDR :
#ifdef HAL_CLICK_9_ADC
		return GetDataFromAdc();
#endif
#ifdef HAL_CLICK_9_I2C
		return GetDataFromHal();
#endif
		break;
	case HAL_CLICK_6_ADDR :
		return GetDataFromHal();
		break;
	default:
		return 0;
		break;
	}
	return 0;
}
/* Get low current value */
uint32_t GetLowCurrent()
{

	switch (LOW_CURRENT_BOARD)
	{
	case VCT_MONITOR_ADDR :
		return GetDataFromVct();
		break;
	case HAL_CLICK_9_ADDR :
		return GetDataFromHal();
		break;
	default :
		return 0;
		break;
	}

}
/* Initialization of VCT Monitor Click */
void Vct_Init(void)
{
	/* Configuration of VCT Monitor Click -> Current measurement */

	HAL_I2C_Mem_Write(&hi2c4,
			(uint16_t)VCT_MONITOR_ADDR,
			(uint16_t)CONTROL_REG,
			1,
			&configVct,
			sizeof(configVct),
			10);

	/* Once triggered, VCT Monitor Click continuously measure current */
	HAL_I2C_Mem_Write(&hi2c4,
			VCT_MONITOR_ADDR,
			(uint16_t)TRIGGER_REG,
			1,
			&triggerValue,
			sizeof(triggerValue),
			10);

}

/* Pinging particular sensors */
HAL_StatusTypeDef PingSensors(void)
{
	/* Checks if low current click is connected */
#ifndef HAL_CLICK_9_ADC
	if (HAL_I2C_IsDeviceReady(&hi2c4, (uint16_t)LOW_CURRENT_BOARD, 2, 10) != HAL_OK)
	{
		return HAL_ERROR;
	}
#endif

	/* Checks if high current device is connected */
//	if (HAL_I2C_IsDeviceReady(&hi2c4, (uint16_t)HIGH_CURRENT_BOARD, 2, 10) != HAL_OK)
//	{
//		return HAL_ERROR;
//	}

	return HAL_OK;
}
/* Timer16 callback, increment value every 100us*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim16)
	{
		timeInUs += 1;
	}
}

/* TimerCallback notifies SenseTask that 1ms has expired */
void MyTimerCallback (TimerHandle_t xTimer)
{

	xTaskNotifyGive(senseTaskHandle);

}
/* SenseTask periodically polls I2C and puts current's sample values in currentQueue */

void SenseTask(void const * argument)
{
	uint8_t bufferCounter = 0;
	uint8_t bufferFull = 0;
	uint32_t timeStamp = 0;
	uint32_t bufferCurrent [20];
	uint32_t currentAverageHigh = 0;
	uint32_t currentAverageLow = 0;
	uint32_t highCurrentValue = 0;
	uint32_t lowCurrentValue = 0;
	struct currentAndTime sendingData;


	for(;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		lowCurrentValue = GetLowCurrent(); //GetLowCurrent();
		highCurrentValue = GetHighCurrent(); //GetHighCurrent();

		/* Disable interrupts to get exact time */
		portDISABLE_INTERRUPTS();
		/* Scaling to ms */
		timeStamp = timeInUs/10;
		/* Enable interrupts */
		portENABLE_INTERRUPTS();

		/* Buffer to store last 10 current values for its average calculation */
		bufferCurrent[bufferCounter] = highCurrentValue;
		bufferCurrent[bufferCounter + 10] = lowCurrentValue;

		bufferCounter += 1;

		if(bufferCounter % 10 == 0){
			bufferCounter = 0;
			bufferFull = 1;
		}
		if(bufferFull == 1){

			for(uint8_t i = 0; i < 10; i++)
			{
				currentAverageHigh += bufferCurrent[i];
				currentAverageLow += bufferCurrent[i + 10];
			}
			currentAverageHigh /= 10;
			currentAverageLow /= 10;

			sendingData.currentHigh = currentAverageHigh;
			sendingData.currentLow = currentAverageLow;
			sendingData.timeStamp = timeStamp;
			/* Sending timeStamp and currentAverage values in currTime variable*/
			//			currTime = (((((uint64_t)currentAverage) << 32) & 0xFFFFFFFF00000000) | ((uint64_t)(timeStamp) & 0x00000000FFFFFFFF));
			xQueueSend(currentQueue, &sendingData, 100);


		}
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */

/* UDP Task, which sends current samples through udp socket via Ethernet  */

/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */


	osDelay(1000);

	ip_addr_t PC_IPADDR;
	struct currentAndTime currentTime;
	uint32_t packetID = 0;
	uint8_t text[MAX_NUMBER_OF_DIGITS];

	/* Remote IP address */
	IP_ADDR4(&PC_IPADDR, 192, 168, 1, 101);

	struct udp_pcb* my_udp = udp_new();
	udp_connect(my_udp, &PC_IPADDR, PORT);
	struct pbuf* udp_buffer = NULL;




	/* Infinite loop */
	for (;;) {
		if(xQueueReceive(currentQueue, &currentTime, portMAX_DELAY))

		{
			/* Incrementing packet ID */
			packetID += 1;

			/* Disable interrupt to get time : */

			udp_buffer = pbuf_alloc(PBUF_TRANSPORT, MAX_NUMBER_OF_DIGITS, PBUF_RAM);

			/* Whole data to be sent : packet ID, current values and captured time of current reading  */
			text[0] = 0x000000FF & packetID;
			text[1] = (0x0000FF00 & packetID) >> 8;
			text[2] = (0x00FF0000 & packetID) >> 16;
			text[3] = (0xFF000000 & packetID) >> 24;

			/* High current value */
			text[4] = 0x000000FF & currentTime.currentHigh;
			text[5] = (0x0000FF00 & currentTime.currentHigh) >> 8;
			text[6] = (0x00FF0000 & currentTime.currentHigh) >> 16;
			text[7] = (0xFF000000 & currentTime.currentHigh) >> 24;

			/* Low current value */
			text[8] = 0x000000FF & currentTime.currentLow;
			text[9] = (0x0000FF00 & currentTime.currentLow) >> 8;
			text[10] = (0x00FF0000 & currentTime.currentLow) >> 16;
			text[11] = (0xFF000000 & currentTime.currentLow) >> 24;

			/* Time stamp */
			text[12] = (0x000000FF & currentTime.timeStamp);
			text[13] = (0x0000FF00 & currentTime.timeStamp) >> 8;
			text[14] = (0x00FF0000 & currentTime.timeStamp) >> 16;
			text[15] = (0xFF000000 & currentTime.timeStamp) >> 24;



			if (udp_buffer != NULL) {
				memcpy(udp_buffer->payload, text, MAX_NUMBER_OF_DIGITS);
				udp_send(my_udp, udp_buffer);
				pbuf_free(udp_buffer);
			}
		}
	}

  /* USER CODE END 5 */
}

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.Size = MPU_REGION_SIZE_256B;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
