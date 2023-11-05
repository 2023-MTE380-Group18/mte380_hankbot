/*
 * servo.h
 * This file contains constants, parameters function headers for servo motor control
 * Created on: Nov 4, 2023
 * Author: eckim
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "main.h"

void Servo_Init(TIM_HandleTypeDef *htim, uint32_t Channel);
void Servo_Claw_Open(TIM_HandleTypeDef *htim);
void Servo_Claw_Close(TIM_HandleTypeDef *htim);
void Servo_Wrist_Up(TIM_HandleTypeDef *htim);
void Servo_Wrist_Down(TIM_HandleTypeDef *htim);

#endif /* INC_SERVO_H_ */
