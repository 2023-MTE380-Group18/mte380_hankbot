/*
 * l298n_motor_control.c
 * This file contains function implementations for l298n DC motor control
 * Created on: Nov 6, 2023
 * Author: eckim
 */

#include "l298n_motor_control.h"

static uint8_t direction_L = 0;
static uint8_t direction_R = 0;

void L298N_Init(TIM_HandleTypeDef *htim)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_TIM_Base_Start(htim);
  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
  L298N_Motors_Stop(htim);
}

// htim = &htim3  [CH3 for Left Motor]
// direction = 0 => Forwards Direction
// direction = 1 => Backwards Direction
// speed => in RPM. Maximum allowed value is MAX_RPM=300
void L298N_Motor_R_Control(TIM_HandleTypeDef *htim, uint8_t direction, uint16_t speed)
{
  // If the direction is being changed
  if (direction != direction_R) {
    direction_R = direction;
    // Avoid closing all H-Bridge Switches simutaniously
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3);
    HAL_Delay(20);
    if (direction == 0) {
      // Right Motor Forward Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    } else {
      // Right Motor Backward Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    }
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
  }
//  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint16_t)(speed/MAX_RPM)*20000);
  htim->Instance->CCR3 = (uint16_t)(((double)speed/MAX_RPM)*20000);
}

// htim = &htim3  [CH4 for Left Motor]
// direction = 0 => Forwards Direction
// direction = 1 => Backwards Direction
// speed => in RPM. Maximum allowed value is MAX_RPM=300
void L298N_Motor_L_Control(TIM_HandleTypeDef *htim, uint8_t direction, uint16_t speed)
{
  // If the direction is being changed
  if (direction != direction_L) {
    direction_L = direction;
    // Avoid closing all H-Bridge Switches simutaniously
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_4);
    HAL_Delay(20);
    if (direction == 0) {
      // Left Motor Forward Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    } else {
      // Left Motor Backwards Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
  }
//  __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, (uint16_t)(speed/MAX_RPM)*20000);
  htim->Instance->CCR4 = (uint16_t)(((double)speed/MAX_RPM)*20000);
}

void L298N_Motors_Stop(TIM_HandleTypeDef *htim)
{
  L298N_Motor_L_Control(htim, direction_L, 0);
  L298N_Motor_R_Control(htim, direction_R, 0);
}



