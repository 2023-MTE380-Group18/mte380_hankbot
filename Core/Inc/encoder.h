/*
 * encoder.h
 * This file contains function headers for encoders
 * Created on: Nov 6, 2023
 * Author: eckim
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

static const uint16_t TICKS_PER_ROT = 0;
static const uint16_t CM_PER_ROT = 0;

void Encoder_Init(TIM_HandleTypeDef *htim);
uint32_t Encoder_Get_Ticks(TIM_HandleTypeDef *htim);
uint32_t Encoder_Get_Distance(TIM_HandleTypeDef *htim);


#endif /* INC_ENCODER_H_ */
