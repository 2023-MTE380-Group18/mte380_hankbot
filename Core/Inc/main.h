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
#include "stm32f4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define TIM5_CH1_ENC_L_A_Pin GPIO_PIN_0
#define TIM5_CH1_ENC_L_A_GPIO_Port GPIOA
#define TIM5_CH2_ENC_L_B_Pin GPIO_PIN_1
#define TIM5_CH2_ENC_L_B_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define TIM2_CH1_ENC_R_A_Pin GPIO_PIN_5
#define TIM2_CH1_ENC_R_A_GPIO_Port GPIOA
#define TIM3_CH3_SVO_WRIST_Pin GPIO_PIN_0
#define TIM3_CH3_SVO_WRIST_GPIO_Port GPIOB
#define TIM3_CH4_SVO_GRAB_Pin GPIO_PIN_1
#define TIM3_CH4_SVO_GRAB_GPIO_Port GPIOB
#define TIM1_CH3_MTR_R_Pin GPIO_PIN_10
#define TIM1_CH3_MTR_R_GPIO_Port GPIOA
#define TIM1_CH4_MTR_L_Pin GPIO_PIN_11
#define TIM1_CH4_MTR_L_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define TIM2_CH2_ENC_R_B_Pin GPIO_PIN_3
#define TIM2_CH2_ENC_R_B_GPIO_Port GPIOB
#define MTR_L_FWD_Pin GPIO_PIN_4
#define MTR_L_FWD_GPIO_Port GPIOB
#define MTR_L_BACK_Pin GPIO_PIN_5
#define MTR_L_BACK_GPIO_Port GPIOB
#define MTR_R_FWD_Pin GPIO_PIN_6
#define MTR_R_FWD_GPIO_Port GPIOB
#define MTR_R_BACK_Pin GPIO_PIN_7
#define MTR_R_BACK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
