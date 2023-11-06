/*
 * servo.c
 * This file contains function implementations for servo motor control
 * Created on: Nov 4, 2023
 * Author: eckim
 */

#include "servo.h"

void Servo_Init(TIM_HandleTypeDef *htim)
{
  HAL_TIM_Base_Start(htim);
  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3); //Wrist Servo [TIM3 CH3]
  HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4); //Gripper Servo [TIM3 CH4]
  Servo_Wrist_Up(htim);
  Servo_Claw_Open(htim);
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
