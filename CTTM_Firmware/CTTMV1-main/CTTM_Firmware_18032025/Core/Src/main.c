/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t buffer[50];
char rxData[50];

uint16_t var1 = 0;
uint16_t var2 = 0;
int rxFlag = 0;
uint8_t limit_switch_flag;
uint8_t count;
int abcd;
volatile int Total_Steps;
volatile int Distance;
volatile int Process=0;
int xtemp,temp, xforce, force, set_force,force, set_length ,dist, set_speed , speed;
char task,ev;
volatile uint16_t Interrupt_Tim2;
const char *mfeedback = "*MAN:RES#";
const char *sfeedback = "*PRS:STR#";
const char *pfeedback = "*PRS:PUS#";
const char *rfeedback = "*PRS:HOM#";
const char *nfeedback = "*PRS:RED#";
const char *hfeedback = "*PRS:HET#";
uint8_t insertion=1;
uint8_t retraction=0;
int dir;
volatile uint16_t sum_intterupt=0;
volatile uint16_t Total_interrupt=0;
volatile uint16_t distance=0;
volatile uint8_t total_distance=0;
volatile uint16_t interrupt_per_mm=969;
volatile uint16_t a=0;
uint16_t local_interrupt_tim2;
uint8_t abc=0;
volatile uint8_t update_dist;
 char disttotal[25];
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STEPS_PER_REV 6400        // Steps per revolution (including microstepping)
#define LEAD_SCREW_PITCH 2        // Lead screw pitch in mm
#define MICROSTEPS 43             // Microstepping factor
#define TIMER_CLOCK_FREQ 72000000 // Timer clock frequency (72 MHz)
const int STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH;
int steps_per_sec;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc2;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void ConfigRXData(void);
void ConfigTXData(char *TxData);
void Retraction(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// CONTROL FUNCTIONS
void SET_STEPPER_START(int dir, int mmps);
void SET_STEPPER_STOP(void);
void SET_HEATER_CONTROL(int ctrl);

void SET_PNEUMATIC_VALVE(int ctrl);
void SET_MOTOR_SPEED(int speed_mm_sec);
void DECELERATE_MOTOR(void); // LOGIC UNDER DEVELOPMENT
void Homing(void);
// READ FUNCTIONS
uint16_t GET_TEMPERATURE_VALUE(void);
uint16_t GET_FORCE_VALUE(void);
int Hom;
// GENERAL HANDLES
void MAC_HEATER(void);
void MAC_HOMING(void);

// AUTO MODE HANDLES
void HANDLE_START_PRESSED(void);
void HANDLE_STOP_PRESSED(void);
void HANDLE_RESET_PRESSED(void);

// MANUAL MODE HANDLES
void HANDLE_CLAMP(void);
void HANDLE_LEFTMOV(void);
void HANDLE_RIGHTMOV(void);
void Dist(void);

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
  MX_USB_DEVICE_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_SPI3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
Homing();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		SET_STEPPER_START(1,1);
//		HAL_Delay(1000);
//		SET_STEPPER_START(0,1);
//		HAL_Delay(1000);
		
		if (rxFlag == 1)
		{
			ConfigRXData();
		}
		
		
		
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Heater_Switch_GPIO_Port, Heater_Switch_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|Valve1_Pin
                          |Valve2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, Heater_Pin|Pump_Out_Pin|Pump_In_Pin|Air_Valve_Pin
                          |LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, En_Pin|Dir_Pin|NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Water_Out_Pin */
  GPIO_InitStruct.Pin = Water_Out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Water_Out_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Water_In_Pin */
  GPIO_InitStruct.Pin = Water_In_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Water_In_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Min_Lim_Pin Max_Lim_Pin */
  GPIO_InitStruct.Pin = Min_Lim_Pin|Max_Lim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : Heater_Switch_Pin */
  GPIO_InitStruct.Pin = Heater_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Heater_Switch_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE8 PE9 PE10 Valve1_Pin
                           Valve2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|Valve1_Pin
                          |Valve2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Heater_Pin Pump_Out_Pin Pump_In_Pin Air_Valve_Pin
                           LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = Heater_Pin|Pump_Out_Pin|Pump_In_Pin|Air_Valve_Pin
                          |LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : En_Pin Dir_Pin NSS_Pin */
  GPIO_InitStruct.Pin = En_Pin|Dir_Pin|NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SET_STEPPER_START(int dir, int mmps)
{	
	// 0 for BWD, 1 for FWD
	HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, dir);
	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
	
	__HAL_TIM_SetCounter(&htim1, 0);
	
	SET_MOTOR_SPEED(mmps);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}		
void Dist(){
	
	
	
	 if(Process==1)
	{
		if(insertion==1){
			a++;
		if(Interrupt_Tim2<=19369&&dir==0){
			SET_PNEUMATIC_VALVE(1);		
			dir=0;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			sum_intterupt=sum_intterupt+Interrupt_Tim2;
			
		}
		else if(Interrupt_Tim2>19369&&dir==0){
			dir=1;
			distance=distance+1;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			Interrupt_Tim2=0;
		a=0;
		}
		else if(Interrupt_Tim2<=19369&&dir==1){
			SET_PNEUMATIC_VALVE(0);			
			dir=1;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			
		}
		else if(Interrupt_Tim2>19369&&dir==1){
			dir=0;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			Interrupt_Tim2=0;
		a=0;
		}
		if(a==969&&dir==0){
		distance++;
			a=0;
		}
	}
	
	}
	
  Total_interrupt=interrupt_per_mm*60;
	total_distance=set_length;
	distance=sum_intterupt/interrupt_per_mm;
}
void SET_STEPPER_STOP()
{
	//DECELERATE_MOTOR();
	HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 0);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

void SET_HEATER_CONTROL(int ctrl)
{
  //1 for OFF, 0 for ON
	HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, ctrl);
}

void SET_PNEUMATIC_VALVE(int ctrl)
{
	//1 for OFF, 0 for ON
	HAL_GPIO_WritePin(Air_Valve_GPIO_Port,Air_Valve_Pin, ctrl);
}

uint16_t GET_TEMPERATURE_VALUE(void)
{
	uint8_t rxData[2];
  uint16_t tempData;

  // Assert CS low to select MAX6675
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 0);

  // Send dummy byte to read temperature data
  HAL_SPI_Receive(&hspi3, rxData, 2, HAL_MAX_DELAY);

  // Deassert CS high to deselect MAX6675
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, 1);

  // Combine received bytes into 16-bit temperature data
  tempData = (rxData[0] << 8) | rxData[1];

  // Extract temperature value (12-bit data)
  tempData = (tempData >> 3); // Convert to Celsius
  return tempData * 0.25;
}

uint16_t GET_FORCE_VALUE(void)
{
	uint16_t readings[100];
  float sum = 0.0;
  float measured_force = 0;
	uint16_t forceData = 0;

  for (uint8_t i = 0; i < 100; i++)
  {
    HAL_ADC_Start(&hadc2);
    HAL_ADC_PollForConversion(&hadc2, 5);
    readings[i] = HAL_ADC_GetValue(&hadc2);
    sum += readings[i];
  }

  float average = sum / 100;
  measured_force = average * (5 / 4095.0) * 718.84628047;
  measured_force = (2120 - 807.05) - measured_force;
	forceData = (uint16_t)(measured_force * 1000);
	return forceData;
	
	/* CODE CHUNK TO BE VERIFIED YET
	if (measured_force <= -75 || measured_force >= 20)  
	{
    g = measured_force > 0 ? measured_force - 20 : measured_force + 95;
	}
	else
	{
    g = 0;
	}

  *measuredforce = measured_force * 1000; // Convert to appropriate units (force in mN)
  snprintf(force, 16, "*FRC:%d#", g); // Format the output string
	*/
	
}
void Retraction(){
			HAL_Delay(2000);
			SET_STEPPER_START(1,1);
			HAL_TIM_Base_Start_IT(&htim1);
			
}
void SET_MOTOR_SPEED(int speed_mm_sec)
{
   //STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH;
   steps_per_sec = STEPS_PER_MM * speed_mm_sec; // For 1mm/sec, this will be STEPS_PER_MM
  uint32_t timer_period = (TIMER_CLOCK_FREQ / steps_per_sec) - 1;
	Total_Steps = STEPS_PER_MM*set_length;//calculate total steps
  if (timer_period > 0xFFFF)
  {
    timer_period = 0xFFFF; // Limit the period to 16-bit max value
  }

  __HAL_TIM_SET_AUTORELOAD(&htim1, timer_period);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, timer_period / 2);
}

void Homing(){
	Process = 0;
	Hom=1;
	insertion=1;
	retraction =0;
SET_STEPPER_START(1,1);
		HAL_TIM_Base_Stop_IT(&htim1);
}
void DECELERATE_MOTOR()
{
    const int DECELERATION_TIME_MS = 200; // 200ms to stop the motor
    const int INITIAL_SPEED_MM_SEC = __HAL_TIM_GET_AUTORELOAD(&htim1); // Get the current speed (mm/sec)
    
    // Calculate the initial steps per second based on the current speed
    const int STEPS_PER_MM = (STEPS_PER_REV * MICROSTEPS) / LEAD_SCREW_PITCH;
    int initial_steps_per_sec = STEPS_PER_MM * INITIAL_SPEED_MM_SEC; // Convert speed to steps per second
    
    // Initial timer period calculation based on speed
    uint32_t initial_timer_period = (TIMER_CLOCK_FREQ / initial_steps_per_sec) - 1;

    // Number of deceleration steps over 200ms (Assume a small number of steps for each loop)
    int steps_to_decelerate = 200;  // We will use 200 steps for 200ms (one step per ms)
    int step_decrease = initial_timer_period / steps_to_decelerate;  // Calculate the decrease in timer period per step

    // Gradually decrease the speed over 200ms
    for (int i = 0; i < steps_to_decelerate; i++)
    {
        int new_timer_period = initial_timer_period - (i * step_decrease);
        if (new_timer_period < 0) new_timer_period = 0; // Prevent negative timer values
        
        // Update the timer with the new speed (new timer period)
        __HAL_TIM_SET_AUTORELOAD(&htim1, new_timer_period);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, new_timer_period / 2);

        HAL_Delay(1); // Wait 1ms before the next update (using 1ms delay)
    }

    // Ensure the motor is completely stopped after 200ms
    __HAL_TIM_SET_AUTORELOAD(&htim1, 0); 
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	count++;
			HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_13);
	if(GPIO_Pin==Min_Lim_Pin)
	{
			HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin,0);
		if(Process==1){
		limit_switch_flag=1;
		HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin,1);
		//HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 0);//disable
		//HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
		}
		else if(Hom==1){
			Interrupt_Tim2=0;
		
			HAL_TIM_Base_Start_IT(&htim1);//for count pulse of motor captures pulses of motor
			
		}
	}
	else if(GPIO_Pin==Max_Lim_Pin)
	{
		limit_switch_flag=2;
				HAL_GPIO_TogglePin(Dir_GPIO_Port, Dir_Pin);
	}
}
void ConfigTXData(char *TxData)
{
	CDC_Transmit_HS((uint8_t *)TxData, strlen(TxData));
}

void ConfigRXData(void)
{
	rxFlag = 0;
	memset(rxData, 0, sizeof(rxData));
	
	for (int i = 0; i < 50; i++)
	{
		rxData[i] = (char)buffer[i];
		if (buffer[i] == '#') break;
  }
	
	 if (strcmp(rxData, "*2:5:2#") == 0) SET_PNEUMATIC_VALVE(0);
	else if (strcmp(rxData, "*2:5:1#") == 0) SET_PNEUMATIC_VALVE(1);
	else if (strcmp(rxData, "*2:4:1:2#") == 0){ SET_STEPPER_START(1,1);
	Process=0;}//move stepper backward
	else if (strcmp(rxData, "*2:4:2:2#") == 0) {SET_STEPPER_STOP();
	Process=0;}//stepper stopped
	else if (strcmp(rxData, "*2:4:1:1#") == 0) {SET_STEPPER_START(0,1);
	Process=0;}//move stepper forward
	else if (strcmp(rxData, "*2:4:2:1#") == 0) {SET_STEPPER_STOP();
	Process=0;}//stepper stopped
	else if (strcmp(rxData, "*2:7:1#") == 0) {SET_HEATER_CONTROL(1);
	Process=0;}//start the heater
	else if (strcmp(rxData, "*2:7:2#") == 0) SET_HEATER_CONTROL(0);//stop the heater
	//start,pause,reset button
	/****************************PROCESS DATA*************/
	if (rxData[1] == '1' &&rxData[2]=='1')
  {
	
//    sscanf((char *)rxData, "*%c:%c:%d:%d:%d:%d#", &task, &ev, &speed, &dist, &temp, &force);
//    xtemp = temp;
//    xforce = force;
//		set_force = force;
//    set_length = dist;
//    set_speed = speed;
	}
	else if(rxData[1] == '1' && rxData[3] == '1'){
	// int temp = GET_TEMPERATURE_VALUE(); // Read the current temperature
		sscanf((char *)rxData, "*%c:%c:%d:%d:%d:%d#", &task, &ev, &speed, &dist, &temp, &force);
		//dist = (rxData[10]<<4)+(rxData[11]);
		xtemp = temp;
    xforce = force;
		set_force = force;
    set_length = dist;
    set_speed = speed;
		Hom=0;	
		
		CDC_Transmit_HS((uint8_t *)sfeedback, strlen(sfeedback));
		Process = 1;//flag to indicate process started  
    
		SET_PNEUMATIC_VALVE(1);
		SET_STEPPER_START(0,1);
			HAL_TIM_Base_Start_IT(&htim1);//for count pulse of motor captures pulses of motor
  	update_dist++;
//    if (temp < xtemp)
//    {
//      // Send heating notification
//     
//      CDC_Transmit_HS((uint8_t *)hfeedback, strlen(hfeedback));
//      
//		

//      while (temp < xtemp)
//      {
//        HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_SET);
//        
//        temp = GET_TEMPERATURE_VALUE();
//      }
//      HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);
//    }

	
//		if(distance<=60&&insertion==1){
//		retraction=0;
//			insertion=1;
//			abc++;
//		}
//		else{
//		retraction=1;
//			insertion=0;
//			SET_STEPPER_STOP();
//			HAL_TIM_Base_Stop_IT(&htim1);
//		}
//		 if(Process==1)
//	{
//		if(insertion==1){
//			a++;
//		if(Interrupt_Tim2<=19369&&dir==0){
//			SET_PNEUMATIC_VALVE(1);		
//			dir=0;
//		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
//			sum_intterupt=sum_intterupt+Interrupt_Tim2;
//			
//		}
//		else if(Interrupt_Tim2>19369&&dir==0){
//			dir=1;
//		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
//			Interrupt_Tim2=0;
//		a=0;
//		}
//		else if(Interrupt_Tim2<=19369&&dir==1){
//			SET_PNEUMATIC_VALVE(0);			
//			dir=1;
//		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
//			
//		}
//		else if(Interrupt_Tim2>19369&&dir==1){
//			dir=0;
//		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
//			Interrupt_Tim2=0;
//		a=0;
//		}
//		if(a==969&&dir==0){
//		distance++;
//			a=0;
//		}
//	}
	
	}
	
	

	/******PAUSE BUTTON*****/
	else if(rxData[1] == '1' && rxData[3] == '2')
  {
		//pause every function
		CDC_Transmit_HS((uint8_t*) pfeedback,strlen(pfeedback));//give pause feedback
		 SET_STEPPER_STOP();//stop the motor
		  
	}	
	/*******RESET Button****/
	else if(rxData[1]== '1' && rxData[3]== '3'){
	CDC_Transmit_HS((uint8_t*) rfeedback,strlen(rfeedback));//give pause feedback
		Hom=1;
		insertion=1;
		a=0;
		distance=0;
		retraction=0;
		Interrupt_Tim2=0;
		snprintf(disttotal, sizeof(disttotal), "*DIS:%d#", distance);
			CDC_Transmit_HS((uint8_t *)disttotal, strlen(disttotal));
		Homing();
		
		
	}
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
if(htim == &htim1)
{
	Interrupt_Tim2++;//increment each time interrupt is called
	
	if(Hom==1){
		if(Interrupt_Tim2==14976)
		{
			HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);//stop the motor
			HAL_TIM_Base_Stop_IT(&htim1);
			Hom =0;
			dir=0;
			Interrupt_Tim2=0;
			
CDC_Transmit_HS((uint8_t*)nfeedback ,strlen(nfeedback));//after homing is completed then send this command to enable start button
		}
	}
//	if(update_dist==1)
//	{
//		Dist();
//	}
////	
	else if(Process==1)
	{

		if(insertion==1){
			a++;
		if(Interrupt_Tim2<=19369&&dir==0){
			SET_PNEUMATIC_VALVE(1);		
			dir=0;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			sum_intterupt=sum_intterupt+Interrupt_Tim2;
			
		}
		else if(Interrupt_Tim2>19369&&dir==0){
			dir=1;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			distance=distance+1;
			Interrupt_Tim2=0;
		a=0;
		}
		else if(Interrupt_Tim2<=19369&&dir==1){
			SET_PNEUMATIC_VALVE(0);			
			dir=1;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			
		}
		else if(Interrupt_Tim2>19369&&dir==1){
			dir=0;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			Interrupt_Tim2=0;
		a=0;
		}
		if(a==969&&dir==0){
		distance++;
			snprintf(disttotal, sizeof(disttotal), "*DIS:%d#", distance+1);
			CDC_Transmit_HS((uint8_t *)disttotal, strlen(disttotal));
			a=0;
		}
		if(distance<=dist&&insertion==1){
		retraction=0;
			insertion=1;
				
			abc++;
		}
		else{
		retraction=1;
			insertion=0;
			dir=1;
//			HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 0);
//			HAL_Delay(1000);
//			HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
			Interrupt_Tim2=0;
			a=0;
			
		}
	}
		else if(retraction==1){
			abc--;
			a++;
		if(Interrupt_Tim2<=19369&&dir==1){
			SET_PNEUMATIC_VALVE(1);		
			dir=1;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			sum_intterupt=sum_intterupt+Interrupt_Tim2;
			
		}
		else if(Interrupt_Tim2>19369&&dir==1){
			dir=0;
			distance=distance-1;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			Interrupt_Tim2=0;
		a=0;
		}
		else if(Interrupt_Tim2<=19369&&dir==0){
			SET_PNEUMATIC_VALVE(0);			
			dir=0;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			
		}
		else if(Interrupt_Tim2>19369&&dir==0){
			dir=1;
		HAL_GPIO_WritePin(Dir_GPIO_Port,Dir_Pin,dir);
			Interrupt_Tim2=0;
		a=0;
		}
		if(a==969&&dir==1){
		distance--;
			snprintf(disttotal, sizeof(disttotal), "*DIS:%d#", distance);
			CDC_Transmit_HS((uint8_t *)disttotal, strlen(disttotal));
			a=0;
		}
		if(distance==0&&retraction==1){
		retraction=0;
			insertion=0;
			Homing();
		
		
		}
	}	
	}
}
}	
  /* USER CODE END Callback 1 */


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
