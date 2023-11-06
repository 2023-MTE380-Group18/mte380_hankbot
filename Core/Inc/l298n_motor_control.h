/*
 * l298n_motor_control.h
 * This file contains function headers for l298n DC motor control
 * Created on: Nov 6, 2023
 * Author: eckim
 */

#ifndef INC_L298N_MOTOR_CONTROL_H_
#define INC_L298N_MOTOR_CONTROL_H_

#include "main.h"

static const double MAX_RPM = 300.0f;

void L298N_Init(TIM_HandleTypeDef *htim);

void L298N_Motor_R_Control(TIM_HandleTypeDef *htim, uint8_t direction, uint16_t speed);
void L298N_Motor_L_Control(TIM_HandleTypeDef *htim, uint8_t direction, uint16_t speed);

void L298N_Motors_Stop(TIM_HandleTypeDef *htim);

#endif /* INC_L298N_MOTOR_CONTROL_H_ */
