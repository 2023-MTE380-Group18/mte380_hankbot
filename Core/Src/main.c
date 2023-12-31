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
#define CURVATURE_DIST 1.6 // 1.6m
#define BACKING_DIST 0.05 // 0.05m or 5cm
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

  // Sensor Data
  uint16_t color_data_L[3];
  uint16_t color_data_R[3];
  double encoder_dist_r;
  double encoder_dist_l;

  // Robot Data
  uint16_t speed = 70;
  uint8_t blue_detected_L = 0;
  uint8_t blue_detected_R = 0;


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
  Encoder_Init(&htim5); // Left Motor Encoder
  Encoder_Init(&htim2); // Right Motor Encoder
  ISL29125_Init(&hi2c1); // Left RGB Sensor
  ISL29125_Init(&hi2c3); // Right RGB Sensor
  ISL29125_Config(&hi2c1, CFG1_MODE_RGB | CFG1_10KLUX | CFG1_12BIT, CFG2_IR_ADJUST_HIGH, CFG3_NO_INT);
  ISL29125_Config(&hi2c3, CFG1_MODE_RGB | CFG1_10KLUX | CFG1_12BIT, CFG2_IR_ADJUST_HIGH, CFG3_NO_INT);

  // LED Test & Delay Code Execution
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Blue LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Green LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Blue LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Green LED
  HAL_Delay(3000);


  // *** State 1: Line Following Sequence ***
  while(!(blue_detected_L && blue_detected_R))
  {
    // Update sensor data
    color_data_L[0] = ISL29125_ReadRed(&hi2c1);
    color_data_L[1] = ISL29125_ReadBlue(&hi2c1);
    color_data_L[2] = ISL29125_ReadGreen(&hi2c1);
    color_data_R[0] = ISL29125_ReadRed(&hi2c3);
    color_data_R[1] = ISL29125_ReadBlue(&hi2c3);
    color_data_R[2] = ISL29125_ReadGreen(&hi2c3);
    encoder_dist_r = Encoder_Get_Distance(&htim2);
    encoder_dist_l = Encoder_Get_Distance(&htim5);

    // Check how far across curvature traveled.
    // Slow down at the end, indicated by Yellow LED
    if ((encoder_dist_r + encoder_dist_l)/2 > CURVATURE_DIST) {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Yellow LED
      speed = 55;
    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
      speed = 100;
    }

    // LEFT SENSOR Blue Bullseye Detection, indicated by Blue LED
    if (145 < color_data_L[1]) // Tigher threshold for left sensor due to bullseye location
    {
      blue_detected_L = 1;
      L298N_Motor_L_Control(&htim1, 0, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Blue LED
    }

    // RIGHT SENSOR Blue Bullseye Detection, indicated by Blue LED
    if (14 < color_data_R[0] && color_data_R[0] < 29
     && 60 < color_data_R[1] && color_data_R[1] < 90
     && 50 < color_data_R[2] && color_data_R[2] < 81)
    {
      blue_detected_R = 1;
      L298N_Motor_R_Control(&htim1, 0, 0);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Blue LED
    }

    // Line Following, indicated by Red LED
    // LEFT SENSOR Red Tape Detection Experiemental Data, 12Bit Data:
    if (18 < color_data_L[0] && color_data_L[0] < 35  // Red: 18-35
     && 12 < color_data_L[1] && color_data_L[1] < 30  // Blue: 12-30
     && 14 < color_data_L[2] && color_data_L[2] < 45) // Green: 14-45
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Red LED
      if(!blue_detected_L) L298N_Motor_L_Control(&htim1, 1, speed-40);
      if(!blue_detected_R) L298N_Motor_R_Control(&htim1, 0, speed);

    // RIGHT SENSOR Red Tape Experiemental Detection Data, 12Bit Data:
    } else if (32 < color_data_R[0] && color_data_R[0] < 58  // Red: 32-58
            && 16 < color_data_R[1] && color_data_R[1] < 50  // Blue: 16-50
            && 22 < color_data_R[2] && color_data_R[2] < 70) // Green: 22-70
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Red LED
      if(!blue_detected_L) L298N_Motor_L_Control(&htim1, 0, speed);
      if(!blue_detected_R) L298N_Motor_R_Control(&htim1, 1, speed-40);

    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
      if(!blue_detected_L) L298N_Motor_L_Control(&htim1, 0, speed);
      if(!blue_detected_R) L298N_Motor_R_Control(&htim1, 0, speed);
    }

    // Debug Printout
    // char buf[64];
    // sprintf(buf, "Left | Red: %d, Blue: %d, Green: %d | Right | Red: %d, Blue: %d, Green: %d |\r\n",
    //     color_data_L[0], color_data_L[1], color_data_L[2], color_data_R[0], color_data_R[1], color_data_R[2]);
    // HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
  }
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED


  // *** State 2-1: Pick up Sequence - Move out of bullseye ***
  speed = 55;
  L298N_Motor_L_Control(&htim1, 1, speed);
  L298N_Motor_R_Control(&htim1, 1, speed);
  while(blue_detected_L || blue_detected_R) {
    // Update Sensors
    color_data_L[0] = ISL29125_ReadRed(&hi2c1);
    color_data_L[1] = ISL29125_ReadBlue(&hi2c1);
    color_data_L[2] = ISL29125_ReadGreen(&hi2c1);
    color_data_R[0] = ISL29125_ReadRed(&hi2c3);
    color_data_R[1] = ISL29125_ReadBlue(&hi2c3);
    color_data_R[2] = ISL29125_ReadGreen(&hi2c3);

    // LEFT SENSOR Wood Detection
    if ( 5 < color_data_L[0] && color_data_L[0] < 25
     && 54 < color_data_L[1] && color_data_L[1] < 100
     && 35 < color_data_L[2] && color_data_L[2] < 58)
    {
      blue_detected_L = 1;
    } else {
      blue_detected_L = 0;
    }

    // RIGHT SENSOR Wood Detection
    if (14 < color_data_R[0] && color_data_R[0] < 29
     && 60 < color_data_R[1] && color_data_R[1] < 90
     && 50 < color_data_R[2] && color_data_R[2] < 81)
    {
      blue_detected_R = 1;
    } else {
      blue_detected_R = 0;
    }
    // Stop each motor seperately when corresponding side detects blue:
    if(!blue_detected_L) L298N_Motor_L_Control(&htim1, 1, 0);
    if(!blue_detected_R) L298N_Motor_R_Control(&htim1, 1, 0);
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Blue LED


  // *** State 2-2: Pick up Sequence - Move backwards even further ***
  // Start indicated by Green LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
  // Store current position
  double curr_pos = (Encoder_Get_Distance(&htim2) + Encoder_Get_Distance(&htim5))/2;
  double delta = 0.0;
  // Drive backwards for (BACKING_DIST) [m]
  while(delta < BACKING_DIST) {
    encoder_dist_r = Encoder_Get_Distance(&htim2);
    encoder_dist_l = Encoder_Get_Distance(&htim5);
    delta = curr_pos - (encoder_dist_r + encoder_dist_l)/2;
    L298N_Motor_L_Control(&htim1, 1, speed);
    L298N_Motor_R_Control(&htim1, 1, speed);
  }
  // Stop Motors
  L298N_Motors_Stop(&htim1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED


  // *** State 2-3: Pick up Sequence - Pick up Lego Man ***
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
  Servo_Claw_Open(&htim3);
  Servo_Wrist_Up(&htim1);
  HAL_Delay(500);
  L298N_Motor_R_Control(&htim1, 0, 150);
  L298N_Motor_L_Control(&htim1, 1, 150);
  HAL_Delay(900);
  L298N_Motors_Stop(&htim1);


  // *** State 3: Delivery Sequence ***


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
