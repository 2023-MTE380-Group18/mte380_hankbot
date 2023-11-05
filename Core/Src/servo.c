/*
 * servo.c
 * This file contains function implementations for servo motor control
 * Created on: Nov 4, 2023
 * Author: eckim
 */

#include "servo.h"

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t Channel)
{
  HAL_TIM_PWM_Start(htim, Channel);
}

void Servo_Claw_Open(TIM_HandleTypeDef *htim)
{
  htim->Instance->CCR4 = 40;
}

void Servo_Claw_Close(TIM_HandleTypeDef *htim)
{
  htim->Instance->CCR4 = 120;
}

void Servo_Wrist_Up(TIM_HandleTypeDef *htim)
{
  htim->Instance->CCR3 = 145+70;
}

void Servo_Wrist_Down(TIM_HandleTypeDef *htim)
{
  htim->Instance->CCR3 = 145;
}
