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
#include "math.h"
#include "sparkfun_isl29125.h"
#include "servo.h"
#include "l298n_motor_control.h"
#include "encoder.h"
#include "pid.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CURVATURE_DIST 1.55 // 1.6m
#define BACKING_DIST 0.03 // 0.03m or 3cm
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char OutBuf[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

// deg - degrees to point-turn
// dir - 0 is CW, 1 is CCW
// speed - speed of rotation in RPM
void Rotate_degrees(uint16_t deg, uint8_t dir, uint16_t speed)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
  // Store current position
  double initial_pose_L = Encoder_Get_Distance(&htim5);
  double initial_pose_R = Encoder_Get_Distance(&htim2);
  double delta_L = 0.0;
  double delta_R = 0.0;
  uint8_t rotate_L_done = 0;
  uint8_t rotate_R_done = 0;
  double rotate_tick_L = (double)WHEEL_CENTER_DIST * (double)M_PI * ((double)deg/360.0);
  double rotate_tick_R = (double)WHEEL_CENTER_DIST * (double)M_PI * ((double)deg/360.0);
  if (dir) { // CCW
    L298N_Motor_L_Control(&htim1, 1, speed);
    L298N_Motor_R_Control(&htim1, 0, speed);
  } else { // CW
    L298N_Motor_L_Control(&htim1, 0, speed);
    L298N_Motor_R_Control(&htim1, 1, speed);
  }
  do {
    delta_L = Encoder_Get_Distance(&htim5) - initial_pose_L;
    delta_R = Encoder_Get_Distance(&htim2) - initial_pose_R;
    rotate_L_done = fabs(delta_L) >= rotate_tick_L;
    rotate_R_done = fabs(delta_R) >= rotate_tick_R;
    if (rotate_L_done) L298N_Motor_L_Control(&htim1, 1, 0);
    if (rotate_R_done) L298N_Motor_R_Control(&htim1, 0, 0);
  } while(!(rotate_L_done && rotate_R_done));
  L298N_Motors_Stop(&htim1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
}

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
  uint16_t speed = 0;
  uint16_t turning_speed = 0;
  uint8_t blue_detected_L = 0;
  uint8_t blue_detected_R = 0;
  uint8_t green_detected_L = 0;
  uint8_t green_detected_R = 0;
  uint8_t red_both_detected = 0;


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

  // LED Test & Code Execution Countdown
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Red LED
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Yellow LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Blue LED
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Green LED
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Blue LED

  // TEST: Color Sensor Readings
//  while(1)
//  {
//	  // Update sensor data
//	  color_data_L[0] = ISL29125_ReadRed(&hi2c1);
//	  color_data_L[1] = ISL29125_ReadBlue(&hi2c1);
//	  color_data_L[2] = ISL29125_ReadGreen(&hi2c1);
//	  color_data_R[0] = ISL29125_ReadRed(&hi2c3);
//	  color_data_R[1] = ISL29125_ReadBlue(&hi2c3);
//	  color_data_R[2] = ISL29125_ReadGreen(&hi2c3);
//	// Debug Printout
//	char buf[64];
//	sprintf(buf, "Left | Red: %d, Blue: %d, Green: %d | Right | Red: %d, Blue: %d, Green: %d |\r\n",
//	   color_data_L[0], color_data_L[1], color_data_L[2], color_data_R[0], color_data_R[1], color_data_R[2]);
//	HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
//  }
//

  // TEST: Turning
//    while(1)
//    {
//      Rotate_degrees(180, 0, 120);
//      HAL_Delay(1000);
//    }


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
      speed = 65;
      turning_speed = 65;
    } else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
      speed = 85;
      turning_speed = 70;
    }

    // Debug Printout for Encoders
//    uint8_t MSG[50] = {'\0'};
//
//    sprintf(MSG, "TIM2 (Right) Distance = %f, TIM5 (Left) Distance = %f  | \r\n ", encoder_dist_r, encoder_dist_l);
//    HAL_UART_Transmit(&huart2, MSG, strlen(MSG), HAL_MAX_DELAY); // Print to UART Terminal

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
      if(!blue_detected_L) L298N_Motor_L_Control(&htim1, 1, turning_speed-5);
      if(!blue_detected_R) L298N_Motor_R_Control(&htim1, 0, turning_speed);
    }
    // RIGHT SENSOR Red Tape Experiemental Detection Data, 12Bit Data:
    else if (32 < color_data_R[0] && color_data_R[0] < 54  // Red: 32-58
            && 16 < color_data_R[1] && color_data_R[1] < 45  // Blue: 16-50
            && 22 < color_data_R[2] && color_data_R[2] < 58) // Green: 22-70
    {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Red LED
      if(!blue_detected_L) L298N_Motor_L_Control(&htim1, 0, turning_speed);
      if(!blue_detected_R) L298N_Motor_R_Control(&htim1, 1, turning_speed-5);
    }
    else {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
      if(!blue_detected_L) L298N_Motor_L_Control(&htim1, 0, speed);
      if(!blue_detected_R) L298N_Motor_R_Control(&htim1, 0, speed);
    }

    // Debug Printout for Color Sensors
//     char buf[64];
//     sprintf(buf, "Left | Red: %d, Blue: %d, Green: %d | Right | Red: %d, Blue: %d, Green: %d |\r\n",
//         color_data_L[0], color_data_L[1], color_data_L[2], color_data_R[0], color_data_R[1], color_data_R[2]);
//     HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);

  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Blue LED
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED


  // *** State 2-1: Pick up Sequence - Move out of bullseye ***
  speed = 70;
  L298N_Motor_L_Control(&htim1, 1, speed);
  L298N_Motor_R_Control(&htim1, 1, speed);
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Green LED


  // *** State 2-2: Pick up Sequence - Pick up Lego Man ***
  Servo_Claw_Open(&htim3);
  Servo_Wrist_Down(&htim3);
  HAL_Delay(100);
  Servo_Claw_Close(&htim3);
  HAL_Delay(400);
  Servo_Wrist_Up(&htim3);
  HAL_Delay(100);


  // *** State 3-1: Delivery Sequence - Rotate Robot in pre-determined path ***
  encoder_dist_r = Encoder_Get_Distance(&htim2);
  double backupTravel = 0;
  double backupDistance = 0.14;
  while(backupTravel < backupDistance)
  {
	  L298N_Motor_R_Control(&htim1, 1, 100);
	  L298N_Motor_L_Control(&htim1, 1, 100);
	  backupTravel = encoder_dist_r - Encoder_Get_Distance(&htim2);
  }
  L298N_Motors_Stop(&htim1);

  HAL_Delay(100);
  Rotate_degrees(40, 1, 100);

  // *** State 3-2: Delivery Sequence - Drive Straight until Green ***
  speed = 70;
  L298N_Motor_R_Control(&htim1, 0, speed);
  L298N_Motor_L_Control(&htim1, 0, speed);
  while(!(green_detected_L && green_detected_R))
  {
    // Update Color Sensors
    color_data_L[0] = ISL29125_ReadRed(&hi2c1);
    color_data_L[1] = ISL29125_ReadBlue(&hi2c1);
    color_data_L[2] = ISL29125_ReadGreen(&hi2c1);
    color_data_R[0] = ISL29125_ReadRed(&hi2c3);
    color_data_R[1] = ISL29125_ReadBlue(&hi2c3);
    color_data_R[2] = ISL29125_ReadGreen(&hi2c3);
    //Left Green Detection
    if(4 < color_data_L[0] && color_data_L[0] < 21 && // left red range is 5-20
      16 < color_data_L[1] && color_data_L[1] < 37 && // left blue range is 17-36
      17 < color_data_L[2] && color_data_L[2] < 41) // left green range is 18-40
    {
      L298N_Motor_L_Control(&htim1, 1, 0);
      green_detected_L = 1;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
    }
    //Right Green Detection
    if(7 < color_data_R[0] && color_data_R[0] < 27 && // right red range is 8-26
      20 < color_data_R[1] && color_data_R[1] < 32 && //  right blue range is 21-31
      24 < color_data_R[2] && color_data_R[2] < 45) // right green range is 25-43
    {
      L298N_Motor_R_Control(&htim1, 1, 0);
      green_detected_R = 1;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
    }
  }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Green LED

  // *** State 3-3: Delivery Sequence - Move backwards by Pre-determined Distance ***
  L298N_Motor_R_Control(&htim1, 1, 100);
  L298N_Motor_L_Control(&htim1, 1, 100);
  HAL_Delay(500);
  L298N_Motor_R_Control(&htim1, 1, 0);
  L298N_Motor_L_Control(&htim1, 1, 0);

  // *** State 3-4: Delivery Sequence - Unload Lego Man ***
  Servo_Wrist_Down(&htim3);
  Servo_Claw_Open(&htim3);
  HAL_Delay(500);
  Servo_Wrist_Up(&htim3);

  // *** State 4-1: Return Sequence - Move backwards & Rotate 180 ***
  L298N_Motor_R_Control(&htim1, 1, 70);
  L298N_Motor_L_Control(&htim1, 1, 70);
  HAL_Delay(500);
  Rotate_degrees(170, 0, 120);


  // *** State 4-2: Return Sequence - Line Following ***
  speed = 70;
  turning_speed = 80;
  red_both_detected = 0;
  double initialTravel = Encoder_Get_Distance(&htim2);
  double currentTravel = 0;
  double minDistanceTraveled = 1.0;
  uint8_t first_left_red_detect = 0;

  while(!red_both_detected)
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

    currentTravel = Encoder_Get_Distance(&htim2) - initialTravel;

    switch (first_left_red_detect) {
      case 0:
        speed = 60;
        break;
      case 1:
        speed = 50;
        break;
      case 2:
        speed = 50;
        break;
      case 3:
        speed = 70;
      default:
        break;
    }

    if (18 < color_data_L[0] && color_data_L[0] < 35
     && 12 < color_data_L[1] && color_data_L[1] < 30
     && 14 < color_data_L[2] && color_data_L[2] < 45
     && 32 < color_data_R[0] && color_data_R[0] < 58
     && 16 < color_data_R[1] && color_data_R[1] < 50
     && 22 < color_data_R[2] && color_data_R[2] < 70
     && currentTravel > minDistanceTraveled)
    {
    	red_both_detected = 1;
    	break;
    }

    // Line Following, indicated by Red LED
    // LEFT SENSOR Red Tape Detection Experiemental Data, 12Bit Data:
    if (18 < color_data_L[0] && color_data_L[0] < 35  // Red: 18-35
     && 12 < color_data_L[1] && color_data_L[1] < 30  // Blue: 12-30
     && 14 < color_data_L[2] && color_data_L[2] < 45) // Green: 14-45
    {
      if (first_left_red_detect == 0)
        first_left_red_detect = 1;
      else if (first_left_red_detect > 1) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Red LED
        L298N_Motor_L_Control(&htim1, 1, 70);
        L298N_Motor_R_Control(&htim1, 0, turning_speed);
      }
    }
    // RIGHT SENSOR Red Tape Experiemental Detection Data, 12Bit Data:
    else if (32 < color_data_R[0] && color_data_R[0] < 48  // Red: 32-58
          && 16 < color_data_R[1] && color_data_R[1] < 40  // Blue: 16-50
          && 22 < color_data_R[2] && color_data_R[2] < 48) // Green: 22-70
    {
      if (first_left_red_detect == 2)
        first_left_red_detect = 3;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Yellow LED
      L298N_Motor_L_Control(&htim1, 0, turning_speed);
      L298N_Motor_R_Control(&htim1, 1, 70);
    }
    else
    {
      if (first_left_red_detect == 1)
        first_left_red_detect = 2;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
      L298N_Motor_L_Control(&htim1, 0, speed);
      L298N_Motor_R_Control(&htim1, 0, speed);
    }
  }

  L298N_Motors_Stop(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // Red LED
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET); // Yellow LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // Blue LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // Green LED
      HAL_Delay(400);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // Red LED
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // Yellow LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // Blue LED
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // Green LED
      HAL_Delay(400);
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
