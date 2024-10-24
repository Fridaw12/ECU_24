/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f2xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define START_Pin GPIO_PIN_0
#define START_GPIO_Port GPIOC
#define START_EXTI_IRQn EXTI0_IRQn
#define STOP_Pin GPIO_PIN_1
#define STOP_GPIO_Port GPIOC
#define STOP_EXTI_IRQn EXTI1_IRQn
#define TS_state_Pin GPIO_PIN_2
#define TS_state_GPIO_Port GPIOC
#define Regen_Pin GPIO_PIN_3
#define Regen_GPIO_Port GPIOC
#define LED_debug_Pin GPIO_PIN_1
#define LED_debug_GPIO_Port GPIOA
#define LED_flt1_Pin GPIO_PIN_2
#define LED_flt1_GPIO_Port GPIOA
#define LED_flt2_Pin GPIO_PIN_3
#define LED_flt2_GPIO_Port GPIOA
#define RSWITCH_disp_Pin GPIO_PIN_4
#define RSWITCH_disp_GPIO_Port GPIOA
#define RSWITCH_cool_Pin GPIO_PIN_5
#define RSWITCH_cool_GPIO_Port GPIOA
#define RSWITCH_tqlim_Pin GPIO_PIN_6
#define RSWITCH_tqlim_GPIO_Port GPIOA
#define RSWITCH_tqchar_Pin GPIO_PIN_7
#define RSWITCH_tqchar_GPIO_Port GPIOA
#define APPS1_Pin GPIO_PIN_4
#define APPS1_GPIO_Port GPIOC
#define BSE1_Pin GPIO_PIN_5
#define BSE1_GPIO_Port GPIOC
#define APPS2_Pin GPIO_PIN_0
#define APPS2_GPIO_Port GPIOB
#define BSE2_Pin GPIO_PIN_1
#define BSE2_GPIO_Port GPIOB
#define CLK_Pin GPIO_PIN_9
#define CLK_GPIO_Port GPIOC
#define RS_TX_Pin GPIO_PIN_9
#define RS_TX_GPIO_Port GPIOA
#define RS_RX_Pin GPIO_PIN_10
#define RS_RX_GPIO_Port GPIOA
#define LED_dash1_Pin GPIO_PIN_2
#define LED_dash1_GPIO_Port GPIOD
#define LED_dash3_Pin GPIO_PIN_3
#define LED_dash3_GPIO_Port GPIOB
#define RTDS_Pin GPIO_PIN_4
#define RTDS_GPIO_Port GPIOB
#define LED_dash4_Pin GPIO_PIN_5
#define LED_dash4_GPIO_Port GPIOB
#define LED_dash2_Pin GPIO_PIN_6
#define LED_dash2_GPIO_Port GPIOB
#define LED_brake_Pin GPIO_PIN_7
#define LED_brake_GPIO_Port GPIOB
#define RUN_Pin GPIO_PIN_8
#define RUN_GPIO_Port GPIOB
#define RFE_Pin GPIO_PIN_9
#define RFE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
