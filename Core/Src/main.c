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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sparkfun_isl29125.h"
#include "servo.h"
#include "l298n_motor_control.h"
#include "encoder.h"

#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  uint16_t color_data_L[3];
  uint16_t color_data_R[3];

  uint32_t encoder_tick_r;
  uint32_t encoder_tick_l;
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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  L298N_Init(&htim1);
  Servo_Init(&htim3);
  Encoder_Init(&htim2); // Right Motor Encoder
  Encoder_Init(&htim5); // Left Motor Encoder

  ISL29125_Init(&hi2c1);
  ISL29125_Init(&hi2c3);
  ISL29125_Config(&hi2c1, CFG1_MODE_RGB | CFG1_10KLUX | CFG1_12BIT, CFG2_IR_ADJUST_HIGH, CFG3_NO_INT);
  ISL29125_Config(&hi2c3, CFG1_MODE_RGB | CFG1_10KLUX | CFG1_12BIT, CFG2_IR_ADJUST_HIGH, CFG3_NO_INT);

  HAL_Delay(4000);


  // Arm demo
  Servo_Claw_Open(&htim3);
  HAL_Delay(500);
  Servo_Wrist_Down(&htim3);
  HAL_Delay(500);
  Servo_Claw_Close(&htim3);
  HAL_Delay(500);

  Servo_Wrist_Up(&htim3);
  HAL_Delay(500);

  L298N_Motor_R_Control(&htim1, 0, 150);
  L298N_Motor_L_Control(&htim1, 1, 150);

  HAL_Delay(900);

  L298N_Motor_R_Control(&htim1, 0, 0);
  L298N_Motor_L_Control(&htim1, 1, 0);

  Servo_Wrist_Down(&htim3);
  HAL_Delay(500);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // *** Encoder Example ***
    // uint8_t MSG[50] = {'\0'};
    encoder_tick_r = Encoder_Get_Ticks(&htim2);
    encoder_tick_l = Encoder_Get_Ticks(&htim5);
    if (encoder_tick_r % 1000 > 500) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Blue LED
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Blue LED
    }

    if (encoder_tick_l % 1000 > 500) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Green LED
    } else {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
    }
//    sprintf(MSG, "TIM2 Ticks = %d, TIM5 Ticks = %d  |  ", Encoder_Get_Ticks(&htim2), Encoder_Get_Ticks(&htim5));
//    HAL_UART_Transmit(&huart2, MSG, strlen(MSG), HAL_MAX_DELAY); // Print to UART Terminal

    // *** Color Sensor Example ***
    color_data_L[0] = ISL29125_ReadRed(&hi2c1);
    color_data_L[1] = ISL29125_ReadBlue(&hi2c1);
    color_data_L[2] = ISL29125_ReadGreen(&hi2c1);
    color_data_R[0] = ISL29125_ReadRed(&hi2c3);
    color_data_R[1] = ISL29125_ReadBlue(&hi2c3);
    color_data_R[2] = ISL29125_ReadGreen(&hi2c3);

    if (18 < color_data_L[0] && color_data_L[0] < 35
     && 12 < color_data_L[1] && color_data_L[1] < 30
     && 14 < color_data_L[2] && color_data_L[2] < 45)
    {
      // LEFT SENSOR Red Tape Detection Data, 12Bit Data (Experiemental):
      // Red: 19-43, Blue: 13-70, Green: 15-76
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Yellow LED
      L298N_Motor_L_Control(&htim1, 1, 180);
      L298N_Motor_R_Control(&htim1, 0, 100);

    } else if (32 < color_data_R[0] && color_data_R[0] < 58
            && 16 < color_data_R[1] && color_data_R[1] < 50
            && 22 < color_data_R[2] && color_data_R[2] < 70)
    {
      // RIGHT SENSOR Red Tape Detection Data, 12Bit Data (Experiemental):
      // Red: 33- ~55, Blue: 17-59, Green: 23-67
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Red LED
      L298N_Motor_L_Control(&htim1, 0, 100);
      L298N_Motor_R_Control(&htim1, 1, 180);

    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
      L298N_Motor_L_Control(&htim1, 0, 180);
      L298N_Motor_R_Control(&htim1, 0, 180);
    }

    char buf[64];
    sprintf(buf, "Left | Red: %d, Blue: %d, Green: %d | Right | Red: %d, Blue: %d, Green: %d |\r\n",
        color_data_L[0], color_data_L[1], color_data_L[2], color_data_R[0], color_data_R[1], color_data_R[2]);
    HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);

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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
