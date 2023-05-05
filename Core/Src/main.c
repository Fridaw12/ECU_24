/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
// RTD variables
volatile uint16_t RTDS_time = 0;		// active time for rtds
volatile uint16_t RTDS_maxtime;		// deactivation time for rtds, calculate value
volatile uint8_t RTD_state = 0;		// ready to drive state


// Dash control variables
volatile uint8_t log_sw;		// log switch state



// Temperature

volatile uint8_t temp;		// temperature reported from temperature module
volatile uint8_t tempmax = 50;		//maximum temperature threshold before warning

// APPS

volatile uint16_t APPS1_raw;		// read APPS1 adc value
volatile uint16_t APPS2_raw;		// read APPS2 adc value
volatile uint16_t APPS1;		// APPS1 travel percentage
volatile uint16_t APPS2;		// APPS2 travel percentage
volatile uint16_t APPS1_idle = 1560;	// idle value for apps1
volatile uint16_t APPS2_idle = 2660;	// idle value for apps2
volatile float APPS1_x = 7.078 ;	// apps1 conversion multiplier
volatile float APPS2_x = 7.067 ;	// apps2 conversion  multiplier
volatile uint16_t APPS_avg;		// average value from APPS sensors
volatile int8_t APPS_error;		// percent error from plausability check
volatile uint16_t APPS_maxerror = 900;		// Maximum error from plausability check
volatile uint16_t APPS_dz = 300;		// APPS deadzone


// BSE

volatile uint16_t BSE1_raw;		// read BSE1 adc value
volatile uint16_t BSE2_raw;		// read BSE2 adc value
volatile uint8_t BSE1;		// BSE1 travel percentage
volatile uint8_t BSE2;		// BSE2 travel percentage
volatile uint8_t BSE_error;		// percent error from plausability check
volatile uint8_t BSE_maxerror = 9;		// Maximum error from plausability check, find better value
volatile uint8_t BSE_avg;		// average value from BSE sensors
volatile uint8_t BSPD_min;		// Minimum travel for BSPD to activate
volatile uint8_t BSEStart = 80;		// minimum BSE travel to start


// Torq

volatile uint16_t TQ = 0;		// Commanding motortorq(%)
volatile uint16_t TQ_max = 32767;		// Max torq
volatile uint8_t TQ_lim = 1;		// Torq limit
volatile uint8_t TQ_mode = 0;		// Torq mode


// CAN

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
uint8_t canRX[8] = {0,0,0,0,0,0,0,0};  //CAN Bus Receive Buffer
CAN_FilterTypeDef canfil; //CAN Bus Filter




// Debug variables

volatile uint16_t APPS1_dbflt = 0;	// apps1 negative error
volatile uint16_t APPS2_dbflt = 0;	// apps2 negative error
volatile uint16_t test = 0;		//blank testvariable
volatile uint8_t CAN_dbflt = 0;		// CAN sendingfaults

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

static void TempErrrorCheck(void);
static void Plaus(void);
static void BSPD(void);
static void APPS_read(void);
static void Torqcalc(void);
static void BrakeLED(void);
static void runRTDS(void);
static void motorStart(void);
static void logSend(void);
static void CAN_TQ(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void TempErrrorCheck(void){		// Checks for overtemperature, sends to LED

	 if (temp > tempmax){
	 	 //HAL_GPIO_WritePin(GPIOB, LED_temp, GPIO_PIN_SET);}	// enables dash temperature LED if overtemperature
	 }
	 //add CAN tempwarning

	 else
	 	 {//HAL_GPIO_WritePin(GPIOB, LED_temp, GPIO_PIN_RESET);}	// resets LED
		}
*/
/*
void Plaus(void){		// contains plausability checks for apps and bse

	APPS_error = abs(APPS1 - APPS2);

	if (APPS_error > APPS_maxerror ){
		//HAL_GPIO_WritePin(GPIOB, LED_APPS, GPIO_PIN_SET);		// enables APPS dash fault LED
		//add CAN appswarning
		//deactivate RFE/RUN
		APPS_avg = 0;		// commands no torq
		RTD_state = 0;

		BSE_error = abs(BSE1 - BSE2);
	 	if (BSE_error > BSE_maxerror)
	 		{//add CAN bsewarning
	 		}}

	 else{
	 	APPS_avg = (APPS1+APPS2)/2;	// Takes average from apps pedals
	 	BSE_error = abs(BSE1 - BSE2);

	 	if (BSE_error > BSE_maxerror){
	 		//{HAL_GPIO_WritePin(GPIOB, LED_APPS, GPIO_PIN_SET);		// enables APPS dash fault LED
	 		//add CAN bsewarning
	 		//deactivate RFE/RUN
	 		//RTD_State = 0;
	 		}

	 	 else
	 	 	 {//HAL_GPIO_WritePin(GPIOB, LED_APPS, GPIO_PIN_RESET);
	 	 	 BSE_avg = (BSE1+BSE2)/2;}	// Takes average from bse pedals
	 	 	 }

	}
	*/
/*
void BSPD(void){		// BSPD check (mainly done on bspd card)

	if (BSE_avg > BSPD_min) {
		if(APPS_avg > 0)
		//{HAL_GPIO_WritePin(GPIOB, LED_BSPD, GPIO_PIN_SET);
			// CAN send BSPD error
			}
		}
*/


void APPS_read(void){		// reads APPS
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_14;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

	HAL_ADC_Start(&hadc1); // Start ADC1
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversions to be done
	APPS1_raw  = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);


	//ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

		HAL_ADC_Start(&hadc1); // Start ADC1
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversions to be done
		APPS2_raw  = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);



}

void Torqcalc(void){		// Calculate torq

	if (RTD_state == 1){
	//calculate adc to percentage value here
		APPS_read();
		//apps raw to 0-10k
		if (APPS1_raw <=APPS1_idle){APPS1_dbflt++;}		// maybe useless
		if (APPS2_raw <=APPS2_idle){APPS2_dbflt++;}
		APPS1 = (APPS1_raw-APPS1_idle)*APPS1_x;
		APPS2 = (APPS2_raw-APPS2_idle)*APPS2_x;

		APPS_avg = (APPS1+APPS2)/2;		// replace with plaus

		if (APPS_avg < APPS_dz){TQ=0;}
		else{
			TQ = APPS_avg*TQ_lim*3.276*1.0309;
			if (TQ>TQ_max){TQ = TQ_max;}	// Stopps TQ from going to high
			}



/*
		switch(TQ_mode){

		case 0:		// Normal torq mode
			TQ = APPS_avg*TQ_lim*-3.276*1.0309;		// calculate torq
		case 1:		// exponential torq mode
			;		// exponential mode code
		}
		*/


		}

	//else{TQ = 0;}		// no torq if not in ready to drive state

}
/*
void BrakeLED(void){ 	// Brakelight activation

	//if (BSE_avg > BSPD_min){HAL_GPIO_WritePin(GPIOB, Bremselys_Pin, GPIO_PIN_SET);} 	// lys på
	//else {HAL_GPIO_WritePin(GPIOB, LED_brake, GPIO_PIN_RESET);} 	// lys av
}

void runRTDS(void){		// Controll RTDS, could be made nonblocking, rtds_time need to be incremented with clock

	//RTDS_time = 0;		// resetts timer
	//HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS, GPIO_PIN_SET);	// activate RTDS
	while(1)	//timeloop
	{
		if (RTDS_time > RTDS_maxtime){		// check inn acttivationtime is past
			HAL_GPIO_WritePin(GPIOB, RTDS, GPIO_PIN_RESET);		// deactivates RTDS
			break;		// breaks loop
			}
	}
}

void motorStart(void){		// starts the motor

	//check brake
	if (BSE_avg > BSEStart){
	runRTDS();
	//activate run/rfe

	}

	else{
		//failed start
	}
}

void logSend(void){		// sends data to CAN
	if (log_sw = 1){
		//log cancommand to sensormodule
		//apps data
		//bse data
	}
	else{
		// no log command to sensormodule
	}

}

*/

void CAN_TQ(void){


	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x446;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;

	TxData[0] = 50;
	TxData[1] = 0xAA;
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}

void CAN2_RX(void){


	void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	{
	  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  if ((RxHeader.StdId == 0x446))
	  {test+=1000;}
	  if ((RxHeader.StdId != 0x446))
	  	  {test+=10000;}
	}


}



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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

   canfil.FilterBank = 0;
   canfil.FilterMode = CAN_FILTERMODE_IDMASK;
   canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
   canfil.FilterIdHigh = 0;
   canfil.FilterIdLow = 0;
   canfil.FilterMaskIdHigh = 0;
   canfil.FilterMaskIdLow = 0;
   canfil.FilterScale = CAN_FILTERSCALE_32BIT;
   canfil.FilterActivation = ENABLE;
   canfil.SlaveStartFilterBank = 14;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);		// starts filter clock
  HAL_CAN_ConfigFilter(&hcan2,&canfil);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {Error_Handler();}


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


		 //HAL_ADC_Start(&hadc1); // Start ADC1
		 //HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversions to be done
		 //APPS1_raw = HAL_ADC_GetValue(&hadc1);

	  if (RTD_state==1)
	  {CAN_TQ();
	  RTD_state = 0;
	  test++;}



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 20;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 240;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 120;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_debug_Pin|LED_flt1_Pin|LED_flt2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_12V_GPIO_Port, LED_12V_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_flt3_Pin|RTDS_Pin|LED_RTD_Pin|LED_temp_Pin
                          |LED_brake_Pin|RUN_Pin|RFE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_Pin STOP_Pin LOG_Pin */
  GPIO_InitStruct.Pin = START_Pin|STOP_Pin|LOG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_debug_Pin LED_flt1_Pin LED_flt2_Pin */
  GPIO_InitStruct.Pin = LED_debug_Pin|LED_flt1_Pin|LED_flt2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RS_TX_Pin RS_RX_Pin */
  GPIO_InitStruct.Pin = RS_TX_Pin|RS_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_12V_Pin */
  GPIO_InitStruct.Pin = LED_12V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_12V_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_flt3_Pin RTDS_Pin LED_RTD_Pin LED_temp_Pin
                           LED_brake_Pin RUN_Pin RFE_Pin */
  GPIO_InitStruct.Pin = LED_flt3_Pin|RTDS_Pin|LED_RTD_Pin|LED_temp_Pin
                          |LED_brake_Pin|RUN_Pin|RFE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
