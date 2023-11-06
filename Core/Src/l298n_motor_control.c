/*
 * l298n_motor_control.c
 * This file contains function implementations for l298n DC motor control
 * Created on: Nov 6, 2023
 * Author: eckim
 */

#include "l298n_motor_control.h"

// htim = &htim3  [CH3 for Left Motor]
// direction = 1 => Forward Direction
// direction = 0 => Backwards Direction
void L298N_Motor_L_Control(TIM_HandleTypeDef *htim, uint8_t direction, uint16_t speed)
{
  // If the direction is being changed
  if (direction != direction_L) {
    direction_L = direction;
    if (direction == 0) {
      // Left Motor Backwards Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
      HAL_Delay(10);     // Avoid closing all H-Bridge Switches simutaniously.
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    } else {
      // Left Motor Forward Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
      HAL_Delay(10);    // Avoid closing all H-Bridge Switches simutaniously.
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    }
  }
  htim->Instance->CCR3 = (uint16_t)((speed/MAX_RPM)*20000);
}

// htim = &htim3  [CH4 for Left Motor]
// direction = 1 => Forward Direction
// direction = 0 => Backwards Direction
void L298N_Motor_R_Control(TIM_HandleTypeDef *htim, uint8_t direction, uint16_t speed)
{
  // If the direction is being changed
  if (direction != direction_R) {
    direction_R = direction;
    if (direction == 0) {
      // Right Motor Backwards Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_Delay(10);     // Avoid closing all H-Bridge Switches simutaniously.
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
    } else {
      // Right Motor Forward Direction
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
      HAL_Delay(10);    // Avoid closing all H-Bridge Switches simutaniously.
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    }
  }
  htim->Instance->CCR4 = (uint16_t)((speed/MAX_RPM)*20000);
}

void L298N_Motors_Stop(TIM_HandleTypeDef *htim)
{
  L298N_Motor_L_Control(htim, direction_L, 0);
  L298N_Motor_R_Control(htim, direction_R, 0);
}



