/*
 * encoder.h
 * This file contains function headers for encoders
 * Created on: Nov 6, 2023
 * Author: eckim
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include "main.h"

static const double TICKS_PER_ROT = 1432.0; // 1432 ticks/rot, (358*4)
static const double WHEEL_DIAMETER = 0.06; // 0.06 m or 6 cm
static const double TICKS_PER_M = 7597.0; // 1432/(pi*0.06) = 7597 ticks/m
static const double TICKS_PER_CM = 75.97; // 75.97 ticks/cm

static const double MAX_TICKS = 65535;
static const double MAX_DIST = MAX_TICKS/TICKS_PER_M; // 65535 / 7597 = 8.6264 m

void Encoder_Init(TIM_HandleTypeDef *htim);
uint32_t Encoder_Get_Ticks(TIM_HandleTypeDef *htim);
double Encoder_Get_Distance(TIM_HandleTypeDef *htim);

#endif /* INC_ENCODER_H_ */
