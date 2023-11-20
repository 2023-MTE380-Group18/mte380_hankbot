/*
 * encoder.c
 * This file contains function implementations for encoders
 * Created on: Nov 6, 2023
 * Author: eckim
 */

#include "encoder.h"

void Encoder_Init(TIM_HandleTypeDef *htim)
{
  HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

uint32_t Encoder_Get_Ticks(TIM_HandleTypeDef *htim)
{
  return htim->Instance->CNT;
}

// returns distance traveled in m, specifically for each wheel
double Encoder_Get_Distance(TIM_HandleTypeDef *htim)
{
  return htim->Instance->CNT / (double)TICKS_PER_M;
}
