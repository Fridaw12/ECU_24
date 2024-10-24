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
#include "power_limiter.h"

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
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */

// Regen
volatile uint8_t Regen_on = 0;

volatile uint8_t TS_state;			//TS state from accumulator
volatile int32_t MCU_Data;

// RTD variables
volatile uint16_t RTDS_time = 0;		// active time for rtds
volatile uint16_t RTDS_maxtime = 30;		// deactivation time for rtds, calculate value
volatile uint8_t RTDS_mode = 0;
volatile uint8_t RTD_state = 0;		// ready to drive state

// Startup LED
volatile uint16_t Startup_time = 0;		// active time for rtds
volatile uint16_t Startup_maxtime = 30;		// deactivation time for rtds, calculate value
volatile uint8_t Startup_mode = 0;

// Dash control variables
volatile uint8_t log_sw;		// log switch state
volatile uint8_t Disp_mode = 0;
volatile uint16_t RSW_predisp1 = 0;
volatile uint16_t RSW_predisp2 = 0;
volatile uint16_t RSW_predisp3 = 0;
volatile uint16_t RSW_predisp4 = 0;
volatile uint16_t RSW_predisp5 = 0;
volatile uint16_t RSW_predisp6 = 0;
volatile uint16_t RSW_predisp7 = 0;
volatile uint16_t RSW_disp;
volatile uint16_t RSW_pretemp1 = 0;
volatile uint16_t RSW_pretemp2 = 0;
volatile uint16_t RSW_pretemp3 = 0;
volatile uint16_t RSW_pretemp4 = 0;
volatile uint16_t RSW_pretemp5 = 0;
volatile uint16_t RSW_pretemp6 = 0;
volatile uint16_t RSW_pretemp7 = 0;
volatile uint16_t RSW_temp;
volatile uint16_t RSW_preTQlim1 = 0;
volatile uint16_t RSW_preTQlim2 = 0;
volatile uint16_t RSW_preTQlim3 = 0;
volatile uint16_t RSW_preTQlim4 = 0;
volatile uint16_t RSW_preTQlim5 = 0;
volatile uint16_t RSW_preTQlim6 = 0;
volatile uint16_t RSW_preTQlim7 = 0;
volatile uint16_t RSW_TQlim;
volatile uint16_t RSW_preTQchar1 = 0;
volatile uint16_t RSW_preTQchar2 = 0;
volatile uint16_t RSW_preTQchar3 = 0;
volatile uint16_t RSW_preTQchar4 = 0;
volatile uint16_t RSW_preTQchar5 = 0;
volatile uint16_t RSW_preTQchar6 = 0;
volatile uint16_t RSW_preTQchar7 = 0;
volatile uint16_t RSW_TQchar;
volatile uint8_t Disp_pos;
volatile uint8_t Temp_pos;
volatile uint8_t TQchar_pos;
volatile uint8_t TQlim_pos;
volatile uint8_t Pre_Disp_pos;
volatile uint8_t Pre_Temp_pos;
volatile uint8_t Pre_TQchar_pos;
volatile uint8_t Pre_TQlim_pos;

// IMU
volatile uint8_t acc;



// Temperature

volatile uint8_t temp;		// temperature reported from temperature module
volatile uint8_t tempmax = 50;		//maximum temperature threshold before warning
volatile uint8_t Temp_mode = 0;		// temp control mode


// APPS CAL
#define APPS_DZ 			300
#define APPS1_RAW_IDLE_L	1567
#define APPS1_RAW_IDLE_R	1561
#define APPS1_RAW_MAX_L		2900
#define APPS1_RAW_MAX_R		2900
#define APPS2_RAW_IDLE_L	2680
#define APPS2_RAW_IDLE_R	2710
#define APPS2_RAW_MAX_L		4050
#define APPS2_RAW_MAX_R		4083

#define APPS1_RAW_IDLE_AVG	((APPS1_RAW_IDLE_L+APPS1_RAW_IDLE_R)/2)
#define APPS1_RAW_MAX_AVG	((APPS1_RAW_MAX_L+APPS1_RAW_MAX_R)/2)
#define APPS2_RAW_IDLE_AVG	((APPS2_RAW_IDLE_L+APPS2_RAW_IDLE_R)/2)
#define APPS2_RAW_MAX_AVG	((APPS2_RAW_MAX_L+APPS2_RAW_MAX_R)/2)

int16_t MCU_id, MCU_iq, MCU_vd, MCU_vq, MCU_RPM, MCU_Vdc;
int16_t AMS_idc, AMS_ilimp, AMS_ilimn;
uint16_t AMS_vdc;


volatile uint16_t APPS1_raw;        // read APPS1 adc value

volatile uint16_t APPS2_raw;        // read APPS2 adc value
volatile uint16_t APPS1;        // APPS1 travel percentage
volatile uint16_t APPS2;        // APPS2 travel percentage
volatile uint16_t APPS_offset = 3266;
volatile uint16_t APPS1_idle = APPS1_RAW_IDLE_AVG;    // idle value for apps1
volatile uint16_t APPS2_idle = APPS2_RAW_IDLE_AVG;    // idle value for apps2
volatile float APPS1_x = (10000 + APPS_DZ)/(APPS1_RAW_MAX_AVG-APPS1_RAW_IDLE_AVG);    // apps1 conversion multiplier
volatile float APPS2_x = (10000 + APPS_DZ)/(APPS2_RAW_MAX_AVG-APPS2_RAW_IDLE_AVG);    // apps2 conversion  multiplier
volatile uint16_t APPS_avg;        // average value from APPS sensors
volatile uint16_t APPS_error;        // percent error from plausability check
volatile uint16_t APPS_maxerror = 910;        // Maximum error from plausability check
volatile uint16_t APPS_dz = APPS_DZ;        // APPS deadzone
volatile uint16_t APPS_min = 800; 		// Minimum travel for BSPD to activate (før 300)


// BSE

volatile uint16_t BSE1_raw;        // read BSE1 adc value
volatile uint16_t BSE2_raw;        // read BSE2 adc value
volatile uint16_t BSE1;        // BSE1 travel percentage
volatile uint16_t BSE2;        // BSE2 travel percentage
volatile uint16_t BSE1_idle = 390;    // idle value for BSE1
volatile uint16_t BSE2_idle = 385;    // idle value for BSE2
volatile uint16_t BSE_brake;        // minimum BSE value for brakelight
volatile uint16_t BSE1_error;        // percent error from plausability check
volatile uint16_t BSE2_error;
//volatile uint16_t BSE_maxerror = 2000;        // Maximum error from plausability check, find better value (var 350)
volatile uint16_t BSE_avg;        // average value from BSE sensors
volatile uint16_t BSE_min = 200;        // Minimum travel for BSPD to activate (tidligere 14)
volatile uint16_t brakeL_min = 120;
volatile uint16_t BSE_start = 1000;        // minimum BSE travel to start
volatile uint16_t BSE1_minerror = 100;		//Må måles
volatile uint16_t BSE2_minerror = 100;		//Må måles
volatile uint16_t BSE1_maxerror = 8000;		//Må måles
volatile uint16_t BSE2_maxerror = 8000;		//Må måles

// Torq

volatile int32_t TQ = 0;        // Commanding motortorq(%)
volatile int32_t TQ_pre = 0;
volatile uint16_t TQ_MaxPos = 32767;        // Max torq
volatile int16_t TQ_MaxNeg = -32767;
volatile uint8_t TQ_lim = 1;        // Torq limit
volatile uint8_t TQ_mode = 0;        // Torq mode
volatile uint16_t TQ_hex;        // hexadecimal torq
volatile int8_t TQ_CAN[2];        // Torq for CAN
volatile uint8_t TQ_online = 0;  // Sending torq state
volatile uint16_t TQ_prev = 0;
volatile uint16_t TQ_diff;
volatile uint16_t diff_lim = 6000;


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
volatile uint16_t test2 = 0;		//second blank testvariable
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
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

static void TempErrrorCheck(void);
static void Plaus(void);
static void BSPD(void);
static void APPS_read(void);
static void BSE_read(void);
static void RSW_read(void);
static void RSW_calc(void);
static void Bremselys(void);
static void Torq_calc(void);
static void BSE_calc(void);
static void BrakeLED(void);
static void RTDS_run(void);
static void Startup_run(void);
static void CAN_TQ(void);
static void CAN_APPS(int);
static void CAN_BSE1(int);
static void CAN_BSE2(int);
static void CAN_log(int);
static void CAN_tempmode(void);
static void CAN_dispmode(void);
static void CAN2_test(void);
static void CAN_MCU_read(void);

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


void Plaus(void){        // contains plausability checks for apps and bse

    APPS_error = abs(APPS1 - APPS2);

    if (APPS_error > APPS_maxerror ){		// Checking if the APPS sensors differ from each other
        //HAL_GPIO_WritePin(GPIOB, LED_APPS, GPIO_PIN_SET);        // enables APPS dash fault LED
        //add CAN appswarning
        //deactivate RFE/RUN
        RTD_state = 0;        // turn MCU off
        TQ_online = 0;
        CAN_APPS(1);
        HAL_GPIO_WritePin(GPIOA, LED_flt1_Pin, GPIO_PIN_SET);		//Turn on fault LED 1 on circuit board
        HAL_GPIO_WritePin(GPIOB, LED_dash2_Pin, GPIO_PIN_SET);		//Turn on fault LED 1 on dashboard
        HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_RESET);    //Turn off RTD led on dashboard

        BSE1_error = abs(BSE1_raw);
        BSE2_error = abs(BSE2_raw);
         if (BSE1_error > BSE1_maxerror || BSE1_error < BSE1_minerror)		// Checking if the brake pressure is valid
             {//add CAN bsewarning
             RTD_state = 0;        // turn MCU off
             TQ_online = 0;
        	 CAN_BSE1(1);
        	 HAL_GPIO_WritePin(GPIOA, LED_debug_Pin, GPIO_PIN_SET);		//Turn on fault LED 2 on circuit board
        	 HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin, GPIO_PIN_SET);		//Turn on fault LED 3 on dashboard
        	 HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_RESET);   //Turn RTD led on dashboard off
             }

         if (BSE2_error > BSE2_maxerror || BSE2_error < BSE2_minerror)		// Checking if the brake pressure is valid
             {//add CAN bsewarning
             RTD_state = 0;        // turn MCU off
             TQ_online = 0;
             CAN_BSE2(1);
             HAL_GPIO_WritePin(GPIOA, LED_debug_Pin, GPIO_PIN_SET);		//Turn on fault LED 2 on circuit board
             HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin, GPIO_PIN_SET);		//Turn on fault LED 3 on dashboard
             HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_RESET);   //Turn RTD led on dashboard off
             }

    }

     else{
         APPS_avg = (APPS1+APPS2)/2;    // Takes average from apps pedals
         BSE1_error = abs(BSE1_raw);
         BSE2_error = abs(BSE2_raw);

         if (BSE1_error > BSE1_maxerror || BSE1_error < BSE1_minerror)		// Checking if the brake pressure is valid
             {//add CAN bsewarning
             RTD_state = 0;        // turn MCU off
             TQ_online = 0;
           	 CAN_BSE1(1);
          	 HAL_GPIO_WritePin(GPIOA, LED_debug_Pin, GPIO_PIN_SET);		//Turn on fault LED 2 on circuit board
           	 HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin, GPIO_PIN_SET);		//Turn on fault LED 3 on dashboard
           	 HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_RESET);   //Turn RTD led on dashboard off
             }

          if (BSE2_error > BSE2_maxerror || BSE2_error < BSE2_minerror)		// Checking if the brake pressure is valid
              {//add CAN bsewarning
              RTD_state = 0;        // turn MCU off
              TQ_online = 0;
              CAN_BSE2(1);
              HAL_GPIO_WritePin(GPIOA, LED_debug_Pin, GPIO_PIN_SET);		//Turn on fault LED 2 on circuit board
              HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin, GPIO_PIN_SET);		//Turn on fault LED 3 on dashboard
              HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_RESET);   //Turn RTD led on dashboard off
              }

          else
               {//HAL_GPIO_WritePin(GPIOB, LED_APPS, GPIO_PIN_RESET);
               BSE_avg = (BSE1+BSE2)/2;}    // Takes average from bse pedals
               }

    }


void BSPD(void){        // BSPD check (mainly done on bspd card)
    if (BSE_avg > BSE_min) {
        if(APPS_avg > APPS_min){
            TQ_online = 0;
            TQ = 0;
            HAL_GPIO_WritePin(GPIOA, LED_flt2_Pin, GPIO_PIN_SET);		//Turn on fault LED 1 on dashboard
            HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_SET);		//Turn on temp LED on dashboard

            //test++;
        }
            // CAN send BSPD error
    }
}
void APPS_read(void){		// reads APPS
	ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_14;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){Error_Handler();}

	HAL_ADC_Start(&hadc1); // Start ADC1
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversions to be done
	APPS1_raw  = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);


	//ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){Error_Handler();}

		HAL_ADC_Start(&hadc1); // Start ADC1
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversions to be done
		APPS2_raw  = HAL_ADC_GetValue(&hadc1);
		HAL_ADC_Stop(&hadc1);}

void BSE_read(void){		//Reads BSE

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_15;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {Error_Handler();}

    HAL_ADC_Start(&hadc1); // Start ADC1
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversions to be done
    BSE1_raw  = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){Error_Handler();}

    HAL_ADC_Start(&hadc1); // Start ADC1
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversions to be done
    BSE2_raw  = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
}

void RSW_read(void){


	//	display switch

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_4;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {Error_Handler();}

    RSW_predisp7 = RSW_predisp6;
	RSW_predisp6 = RSW_predisp5;
    RSW_predisp5 = RSW_predisp4;
    RSW_predisp4 = RSW_predisp3;
    RSW_predisp3 = RSW_predisp2;
    RSW_predisp2 = RSW_predisp1;
    HAL_ADC_Start(&hadc2); // Start ADC1
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY); // Wait for conversions to be done
    RSW_predisp1  = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    RSW_disp = (RSW_predisp1+RSW_predisp2+RSW_predisp3+RSW_predisp4+RSW_predisp5+RSW_predisp6+RSW_predisp7)/7;


    //	cooling

    sConfig.Channel = ADC_CHANNEL_5;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK){Error_Handler();}

    RSW_pretemp7 = RSW_pretemp6;
	RSW_pretemp6 = RSW_pretemp5;
    RSW_pretemp5 = RSW_pretemp4;
    RSW_pretemp4 = RSW_pretemp3;
    RSW_pretemp3 = RSW_pretemp2;
    RSW_pretemp2 = RSW_pretemp1;
    HAL_ADC_Start(&hadc2); // Start ADC1
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY); // Wait for conversions to be done
    RSW_pretemp1  = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    RSW_temp = (RSW_pretemp1+RSW_pretemp2+RSW_pretemp3+RSW_pretemp4+RSW_pretemp5+RSW_pretemp6+RSW_pretemp7)/7;

	//	TQ char

    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {Error_Handler();}

    RSW_preTQchar7 = RSW_preTQchar6;
	RSW_preTQchar6 = RSW_preTQchar5;
    RSW_preTQchar5 = RSW_preTQchar4;
    RSW_preTQchar4 = RSW_preTQchar3;
    RSW_preTQchar3 = RSW_preTQchar2;
    RSW_preTQchar2 = RSW_preTQchar1;
    HAL_ADC_Start(&hadc2); // Start ADC1
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY); // Wait for conversions to be done
    RSW_preTQchar1  = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    RSW_TQchar = (RSW_preTQchar1+RSW_preTQchar2+RSW_preTQchar3+RSW_preTQchar4+RSW_preTQchar5+RSW_preTQchar6+RSW_preTQchar7)/7;

    //	TQ limiter

    sConfig.Channel = ADC_CHANNEL_7;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK){Error_Handler();}

    RSW_preTQlim7 = RSW_preTQlim6;
    RSW_preTQlim6 = RSW_preTQlim5;
    RSW_preTQlim5 = RSW_preTQlim4;
    RSW_preTQlim4 = RSW_preTQlim3;
    RSW_preTQlim3 = RSW_preTQlim2;
    RSW_preTQlim2 = RSW_preTQlim1;
    HAL_ADC_Start(&hadc2); // Start ADC1
    HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY); // Wait for conversions to be done
    RSW_preTQlim1  = HAL_ADC_GetValue(&hadc2);
    HAL_ADC_Stop(&hadc2);
    RSW_TQlim = (RSW_preTQlim1+RSW_preTQlim2+RSW_preTQlim3+RSW_preTQlim4+RSW_preTQlim5+RSW_preTQlim6+RSW_preTQlim7)/7;
}


void RSW_calc(void){		// Calculating rotary switch position

	//	display switch
//	Pre_Disp_pos = Disp_pos;
	if (RSW_disp >= 3612 && RSW_disp < 3650 ){Disp_pos = 1;}
	else if (RSW_disp >= 3582 && RSW_disp < 3614 ){Disp_pos = 2;}
	else if (RSW_disp >= 3554 && RSW_disp < 3582 ){Disp_pos = 3;}
	else if (RSW_disp >= 3500 && RSW_disp < 3554 ){Disp_pos = 4;}
	else {Disp_pos = 5;}
//	if (Disp_pos != Pre_Disp_pos){Disp_pos = Pre_Disp_pos;}

	//	Cooling switch
//	Pre_Temp_pos = Temp_pos;
	if (RSW_temp >= 3500 && RSW_temp < 3554 ){Temp_pos = 1;}
	else if (RSW_temp >= 3554 && RSW_temp < 3582 ){Temp_pos = 2;}
	else if (RSW_temp >= 3582 && RSW_temp < 3612 ){Temp_pos = 3;}
	else if (RSW_temp >= 3612 && RSW_temp < 3668 ){Temp_pos = 4;}
	else {Temp_pos = 5;}
//	if (Temp_pos != Pre_Temp_pos){Temp_pos = Pre_Temp_pos;}


	//	TQ char switch
//	Pre_TQchar_pos = TQchar_pos;
	if (RSW_TQchar >= 3500 && RSW_TQchar < 3559){TQchar_pos = 1;}
	else if (RSW_TQchar >= 3559 && RSW_TQchar < 3587){TQchar_pos = 2;}
	else if (RSW_TQchar >= 3587 && RSW_TQchar < 3618){TQchar_pos = 3;}
	else if (RSW_TQchar >= 3618 && RSW_TQchar < 3669){TQchar_pos = 4;}
	else {TQchar_pos = 5;}
//	if (TQchar_pos != Pre_TQchar_pos){TQchar_pos = Pre_TQchar_pos;}


	//	TQ limiter switch
//	Pre_TQlim_pos = TQlim_pos;
	if (RSW_TQlim >= 3500 && RSW_TQlim < 3550 && TQ <= 0){TQlim_pos = 1;}
	else if (RSW_TQlim >= 3550 && RSW_TQlim < 3577 && TQ <= 0){TQlim_pos = 2;}
	else if (RSW_TQlim >= 3577 && RSW_TQlim < 3609 && TQ <= 0){TQlim_pos = 3;}
	else if (RSW_TQlim >= 3609 && RSW_TQlim < 3656 && TQ <= 0){TQlim_pos = 4;}
	else {TQlim_pos = 5;}
//	if (TQlim_pos != Pre_TQlim_pos){TQlim_pos = Pre_TQlim_pos;}
}

void Torq_calc(void){        // Calculate torq


    //calculate adc to percentage value here
        APPS_read();
        //apps raw to 0-10k
        if (APPS1_raw <=APPS1_idle){APPS1_dbflt++;}        // maybe useless
        if (APPS2_raw <=APPS2_idle){APPS2_dbflt++;}


        APPS1 = (APPS1_raw-APPS1_idle)*APPS1_x;
        APPS2 = (APPS2_raw-APPS2_idle)*APPS2_x;

        if (APPS1 > 15000){APPS1 = 0;}		// protects from overflow
        if (APPS2 > 15000){APPS2 = 0;}



//        Code for regen (apps controlled)
//        if (Regen_on == 1){
//        	if (TQlim_pos == 1){
//        		TQ = (((APPS_avg)-1225)*1.1428)*TQ_lim*3.276*1.0309;}
//        	else if (TQlim_pos == 2){
//        		TQ = (((APPS_avg)-2450)*1.3333)*TQ_lim*3.276*1.0309;}
//        	else if (TQlim_pos == 3){
//        		TQ = (((APPS_avg)-3675)*1.6)*TQ_lim*3.276*1.0309;}
//        	else if (TQlim_pos == 4){
//        		TQ = (((APPS_avg)-4900)*2)*TQ_lim*3.276*1.0309;}
//        	else{
//                if (APPS_avg < APPS_dz){TQ=0;}
//                else{TQ = (APPS_avg) * TQ_lim*3.276*1.0309;}
//        	}
//        }



//                Code for regen (BSE controlled)
//		if (Regen_on == 1 && BSE_avg > BSE_min){
//			TQ = -(BSE_avg*10);}
//
//		else {
//			if (APPS_avg < APPS_dz){TQ=0;}   // Use this when regen
//			else{TQ = (APPS_avg) * TQ_lim*3.276*1.0309;}
//		}



		//    code for without regen
		TQ = (APPS_avg) * TQ_lim*3.276*1.0309;

        if (TQ>TQ_MaxPos){TQ = TQ_MaxPos;}    // Stops TQ from going to high
        if (TQ<TQ_MaxNeg){TQ = TQ_MaxNeg;}    // Stops TQ from going to high




}

void Bremselys(void){     //Activate the brakelight

//	if(BSE_avg > brakeL_min || acc < -1){HAL_GPIO_WritePin(GPIOB, LED_brake_Pin, GPIO_PIN_SET);}

	if(BSE_avg > brakeL_min){HAL_GPIO_WritePin(GPIOB, LED_brake_Pin, GPIO_PIN_SET);}
	else{HAL_GPIO_WritePin(GPIOB, LED_brake_Pin, GPIO_PIN_RESET);}}


void RTDS_run(void){

	//HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS, GPIO_PIN_SET);	// activate RTDS
	//for(int i = 0; i < 1; i++) {
		//HAL_Delay(100);
		RTDS_mode = 1;
		HAL_GPIO_WritePin(GPIOB,RTDS_Pin, GPIO_PIN_SET);
		test++;

  // Code block to be executed 5 times
//}
}

void Startup_run(void){

	//HAL_GPIO_WritePin(RTDS_GPIO_Port, RTDS, GPIO_PIN_SET);	// activate RTDS
	//for(int i = 0; i < 1; i++) {
		//HAL_Delay(100);
		Startup_mode = 1;
		HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_dash2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin, GPIO_PIN_SET);


  // Code block to be executed 5 times
//}
}



void BSE_calc(void){		//Calculate the BSE
        BSE_read();
        BSE1 = (BSE1_raw - BSE1_idle);
        BSE2 = (BSE2_raw - BSE2_idle);
        if (BSE1 > 5000){BSE1 = 0;}
        if (BSE2 > 5000){BSE2 = 0;}
        BSE_avg = (BSE1 + BSE2)/2;

}

void CAN_TQ(void){		// send torq to mcu


	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x210;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 3;

//    char hex_string[5];
//    sprintf(hex_string, "%04X", TQ);
//
//    TQ_CAN[0] = strtol(hex_string, NULL, 16) & 0xFF; // lower byte
//    TQ_CAN[1] = (strtol(hex_string, NULL, 16) >> 8) & 0xFF; // upper byte

    //test++;
    TxData[0] = 0x90;
	TxData[1] = ((int16_t)TQ); //TQ_CAN[0];
	TxData[2] = ((int16_t)TQ) >> 8; //TQ_CAN[1];
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}

void CAN_APPS(int plausflt){	// send apps

	TxHeader.IDE = CAN_ID_STD;
	if (plausflt == 1){TxHeader.StdId = 0x102;}		// Error
	else{TxHeader.StdId = 0x248;}		// No error

	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 6;

	TxData[0] = APPS_error & 0xFF; // Lower byte of APPS_error
	TxData[1] = (APPS_error >> 8) & 0xFF; // Upper byte of APPS_error
	TxData[2] = APPS1 & 0xFF; // Lower byte of APPS1
	TxData[3] = (APPS1 >> 8) & 0xFF; // Upper byte of APPS1
	TxData[4] = APPS2 & 0xFF; // Lower byte of APPS2
	TxData[5] = (APPS2 >> 8) & 0xFF; // Upper byte of APPS2

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}

void CAN_BSE1(int plausflt){		// send bse

	TxHeader.IDE = CAN_ID_STD;
	if (plausflt == 1){TxHeader.StdId = 0x103;}		// Error
	else{TxHeader.StdId = 0x249;}		// No error

	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 6;

	TxData[0] = BSE1_error & 0xFF; // Lower byte of APPS_error
	TxData[1] = (BSE1_error >> 8) & 0xFF; // Upper byte of APPS_error
	TxData[2] = BSE1 & 0xFF; // Lower byte of APPS1
	TxData[3] = (BSE1 >> 8) & 0xFF; // Upper byte of APPS1
	TxData[4] = BSE2 & 0xFF; // Lower byte of APPS2
	TxData[5] = (BSE2 >> 8) & 0xFF; // Upper byte of APPS2

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}
void CAN_BSE2(int plausflt){		// send bse

	TxHeader.IDE = CAN_ID_STD;
	if (plausflt == 1){TxHeader.StdId = 0x103;}		// Error
	else{TxHeader.StdId = 0x249;}		// No error

	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 6;

	TxData[0] = BSE2_error & 0xFF; // Lower byte of APPS_error
	TxData[1] = (BSE2_error >> 8) & 0xFF; // Upper byte of APPS_error
	TxData[2] = BSE1 & 0xFF; // Lower byte of APPS1
	TxData[3] = (BSE1 >> 8) & 0xFF; // Upper byte of APPS1
	TxData[4] = BSE2 & 0xFF; // Lower byte of APPS2
	TxData[5] = (BSE2 >> 8) & 0xFF; // Upper byte of APPS2

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}
void CAN_log(int logval){		// send log status

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x212;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 1;
	if(logval == 0){TxData[0] = 0;}
	else{TxData[0] = 255;}
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}

void CAN_tempmode(void){		// send tempmode to tempcard

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x213;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 1;
	Temp_mode = Temp_pos;
	TxData[0] = Temp_mode;
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}

void CAN_dispmode(void){		// send tempmode to tempcard

	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x215;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 1;
	TxData[0] = Disp_mode;
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}



void CAN2_test(void){


	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x447;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 2;

	TxData[0] = 50;
	TxData[1] = 0xAA;
	if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		CAN_dbflt++;
	   Error_Handler ();
	}}




void CAN_MCU_read_init(void){

	CAN_TxHeaderTypeDef RxInitHeader;
	RxInitHeader.IDE = CAN_ID_STD;
	RxInitHeader.StdId = 0x210;
	RxInitHeader.RTR = CAN_RTR_DATA;
	RxInitHeader.DLC = 3;

	uint8_t tmpTxData[8];

	    //test++;
	tmpTxData[0] = 0x3D;
	tmpTxData[1] = 0x27;
	tmpTxData[2] = 10;

	uint32_t tmpMailbox;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
	{
		CAN_dbflt++;
//		Error_Handler ()
	}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(10);
	}


	tmpTxData[1] = 0x28;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
		{
			CAN_dbflt++;
	//		Error_Handler ()
		}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(10);
	}

	tmpTxData[1] = 0x29;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
		{
			CAN_dbflt++;
	//		Error_Handler ()
			}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(10);
	}

	tmpTxData[1] = 0x2A;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
		{
			CAN_dbflt++;
	//		Error_Handler ()
		}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(10);
	}

	tmpTxData[1] = 0x30;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
		{
			CAN_dbflt++;
	//		Error_Handler ()
		}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(10);
	}

	tmpTxData[1] = 0xEB;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
		{
			CAN_dbflt++;
	//		Error_Handler ()
		}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(1);
	}

	tmpTxData[1] = 0x8F;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
		{
			CAN_dbflt++;
	//		Error_Handler ()
		}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(1);
	}

	tmpTxData[1] = 0xF6;

	if (HAL_CAN_AddTxMessage(&hcan1, &RxInitHeader, tmpTxData, &tmpMailbox) != HAL_OK)
		{
			CAN_dbflt++;
	//		Error_Handler ()
		}

	for(uint16_t i; i < 1000; i++){
		if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 1) break;
		HAL_Delay(1);
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

  Startup_run();
  HAL_Delay(1000);

  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

   canfil.FilterBank = 0;
   canfil.FilterMode = CAN_FILTERMODE_IDMASK;
   canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
   canfil.FilterIdHigh = 0x220;
   canfil.FilterIdLow = 0x179;
   canfil.FilterMaskIdHigh = 0;
   canfil.FilterMaskIdLow = 0;
   canfil.FilterScale = CAN_FILTERSCALE_32BIT;
   canfil.FilterActivation = ENABLE;
   canfil.SlaveStartFilterBank = 14;

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);		// starts filter clock
  HAL_TIM_Base_Start_IT(&htim6);		// Starts clock
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_CAN_ConfigFilter(&hcan1,&canfil);
  HAL_CAN_ConfigFilter(&hcan2,&canfil);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK){Error_Handler();}
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {Error_Handler();}



  HAL_Delay(100);


  CAN_MCU_read_init();

  power_limiter_initialize();

  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 240;
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
  htim3.Init.Prescaler = 10-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 240-1;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 600-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 600-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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

  /*Configure GPIO pins : CAN1_RX, CAN1TX */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /*Configure GPIO pins : CAN2_RX, CAN2TX */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin|RTDS_Pin|LED_dash4_Pin|LED_dash2_Pin
                          |LED_brake_Pin|RUN_Pin|RFE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_Pin STOP_Pin */
  GPIO_InitStruct.Pin = START_Pin|STOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TS_state_Pin Regen_Pin */
  GPIO_InitStruct.Pin = TS_state_Pin|Regen_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_dash1_Pin */
  GPIO_InitStruct.Pin = LED_dash1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_dash1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_dash3_Pin RTDS_Pin LED_dash4_Pin LED_dash2_Pin
                           LED_brake_Pin RUN_Pin RFE_Pin */
  GPIO_InitStruct.Pin = LED_dash3_Pin|RTDS_Pin|LED_dash4_Pin|LED_dash2_Pin
                          |LED_brake_Pin|RUN_Pin|RFE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	test++;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }

  if ((RxHeader.StdId == 0x215)){
	  temp = RxData[0];

//	  if(temp>tempmax){HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_SET);}
//	  else{HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_RESET);}
  }

//  if ((RxHeader.StdId == 0xxxx)){
//	  acc = RxData[0];
//  }



  if (RxHeader.StdId == 0x191) // Check if the received message has standard ID
  {
	  switch(RxData[0]){
	  case 0x27:
		  MCU_iq = (int16_t)RxData[2] << 8 | RxData[1];
		  break;
	  case 0x28:
		  MCU_id = (int16_t)RxData[2] << 8 | RxData[1];
		  break;
	  case 0x29:
		  MCU_vq = (int16_t)RxData[2] << 8 | RxData[1];
		 break;
	  case 0x2A:
		  MCU_vd = (int16_t)RxData[2] << 8 | RxData[1];
		  break;
	case 0x30:
		  MCU_RPM = (int16_t)RxData[2] << 8 | RxData[1];
		 break;
	  case 0xEB:
		  MCU_Vdc = (int16_t)RxData[2] << 8 | RxData[1];
		  break;
	  default:
		  break;

	  }
  }

	  if (RxHeader.StdId == 0x201){
		  AMS_vdc = (uint16_t)RxData[0] << 8 | RxData[1];
		  AMS_idc = (int16_t)RxData[2] << 8 | RxData[3];
		  AMS_ilimp = (int16_t)RxData[4] << 8 | RxData[5];
		  AMS_ilimn = (int16_t)RxData[6] << 8 | RxData[7];
	  }


}


float tqtest = 0, iq_test, id_test, vq_test, vd_test, vdc_test, p_ams_test, p_diff_test, p_mcu_test;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)


{
	if (htim == &htim7){			//running 100 times per sec
		//TQ = TQ_prev;
		Torq_calc();
		BSPD();
		if (TQ_online == 0){TQ = 0;}
		TQ_pre = TQ;

//		vdc_test = 0.9 * vdc_test + 0.1 * (float)MCU_Vdc / 54.64;
//		id_test = 0.9 * id_test + 0.1 * (float)MCU_id * 28 / 70;
//		iq_test = 0.9 * iq_test + 0.1 * (float)MCU_iq * 28 / 70;
//		vd_test = 0.9 * vd_test + 0.1 * (float)MCU_vd / 4096 * vdc_test / sqrt(2);
//		vq_test = 0.9 * vq_test + 0.1 * (float)MCU_vq / 4096 * vdc_test / sqrt(2);


		float mcu_vdc = (float)MCU_Vdc / 54.64;
		float mcu_id = (2 / sqrt(3)) * 28 / 70;
		float mcu_iq = (2 / sqrt(3)) * 28 / 70;
		float mcu_vd = (sqrt(3) / 2) / 4096;
		float mcu_vq = (sqrt(3) / 2) / 4096;
//
////		p_mcu_test = mcu_vd * mcu_id + mcu_vq * mcu_iq;
////		p_ams_test = 0.9 * p_ams_test + 0.1 * (float)AMS_vdc/100 * (float)AMS_idc/100;
////		p_diff_test = p_ams_test - p_mcu_test;
//
//
//		power_limiter_U.i_lim_fuse = 200;
//		power_limiter_U.i_q_max = 360;
//		power_limiter_U.p_limp = 15000;
//		power_limiter_U.p_limn = 40000;
//		power_limiter_U.p_max = 100000;
//		power_limiter_U.u_dc_nom = 400;
//
//		power_limiter_U.i_dc_ams = (float)AMS_idc / 100;
//		power_limiter_U.i_limp_ams = (float)AMS_ilimp / 100;
//		power_limiter_U.i_limn_ams = -(float)AMS_ilimn / 100;
////		power_limiter_U.i_limp_ams = 10000;
////		power_limiter_U.i_limn_ams = 1000;
//		power_limiter_U.m_ref = (float)TQ / INT16_MAX;
//		power_limiter_U.p_mcu = mcu_vd * mcu_id + mcu_vq * mcu_iq;
//		power_limiter_U.u_dc_ams = (float)AMS_vdc / 100;
//		power_limiter_U.u_dc_mcu = mcu_vdc;
//		power_limiter_U.u_q_mcu = mcu_vq;
//
//
//
//		power_limiter_step();
//
//		TQ = (int16_t)(power_limiter_Y.m_set * INT16_MAX);

//		TQ = tqtest * (INT16_MAX / 200);



//		float i_lim;
//		if(mcu_vq > 10) i_lim = 40000 / mcu_vq;
//		else i_lim = 40000 / 10;
//
//		if(TQ > i_lim) TQ = i_lim;






		//TQ_diff = TQ_prev - TQ;
		//if(TQ_diff < 0) {TQ_diff = 0;}
		//if(TQ_diff > diff_lim){
			//TQ = TQ_prev * 0.008;
		//}
		{if (RTD_state==1)
		{

		  CAN_TQ();
		      //TQ=12345;
		      //RTD_state = 0;
		}
		  else {
			  TQ = 0;
		      CAN_TQ();
		  }
	 }}



  if (htim == &htim6 ){				//Running 20 times per sec
	  BSE_calc();
	  Bremselys();
	  BSPD();
	  RSW_read();
	  RSW_calc();
	  Plaus();
	  CAN_tempmode();
	  TS_state = HAL_GPIO_ReadPin(GPIOC,TS_state_Pin);			// checks if TS is on
	  Regen_on = HAL_GPIO_ReadPin(GPIOC,Regen_Pin);				// Checks Regen toggle switch

	  if (TS_state == 0) {
		  RTD_state = 0;
	  }

	  if (Startup_mode == 1){Startup_time++;
		  if(Startup_time > Startup_maxtime){
			HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_dash2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin, GPIO_PIN_RESET);
			Startup_mode = 0;
			Startup_time = 0;
			  }
		  }


	  if (APPS_avg < APPS_min && BSE_avg < BSE_min){
		  TQ_online = 1;												//Deactivate BSPD
		  if (Startup_mode == 0){
			  HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_RESET);  	//Turn on fault LED 2 on dashboard
		  	  }
		  }

	  {if (RTD_state==1)
  	  {

  		  //test++;
  	      HAL_GPIO_WritePin(GPIOB,RFE_Pin, GPIO_PIN_RESET); 		//Start sending RFE signal to the MCU
  	      HAL_GPIO_WritePin(GPIOB,RUN_Pin, GPIO_PIN_RESET);			//Stop send RUN signal to the MCU

  		}
  		  else {
  			  HAL_GPIO_WritePin(GPIOB, RFE_Pin, GPIO_PIN_SET);		//Stop sending RFE signal to the MCU
  		      HAL_GPIO_WritePin(GPIOB, RUN_Pin, GPIO_PIN_SET);		//Stop sending RUN signal to the MCU
			  HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_RESET);   //Turn RTD led on dashboard Off

  		      //TQ = 0;
  		      //CAN_TQ();
  		  }
  	}

  if (RTDS_mode == 1){RTDS_time++;
  	  if(RTDS_time > RTDS_maxtime){
  		HAL_GPIO_WritePin(GPIOB,RTDS_Pin, GPIO_PIN_RESET);
		RTDS_mode = 0;
		RTDS_time = 0;
  	  	  }
  	  }



  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

    if (GPIO_Pin == START_Pin){
        BSE_read();

        if (BSE1_raw > BSE_start){		//Pressing the start button while holding the brake
        	if (TS_state == 1){			//Check if TS is on
        		RTD_state = 1;			//Set the car in RTD state
        		HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_SET);   //Turn RTD led on dashboard ON
        		RTDS_run();//Activate the RTDS
        	}
            HAL_GPIO_WritePin(GPIOB, LED_dash2_Pin, GPIO_PIN_RESET);	//Resets the APPS plaus fault led on dash
            HAL_GPIO_WritePin(GPIOB, LED_dash3_Pin, GPIO_PIN_RESET);	//Resets the BSE plaus fault led on dash
            HAL_GPIO_WritePin(LED_dash1_GPIO_Port, LED_dash1_Pin, GPIO_PIN_RESET);		//Reset the temp LED on dash

            }
        }

    if (GPIO_Pin == STOP_Pin){
        RTD_state = 0;		//Takes the care out of RTD state
        HAL_GPIO_WritePin(GPIOB, LED_dash4_Pin, GPIO_PIN_RESET);   //Turn RTD led on dashboard off

        }
}
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
