/******************************************************************************
sparkfun_isl29125.cpp
Core implementation file for the ISL29125 RGB sensor library.
Jordan McConnell @ SparkFun Electronics
25 Mar 2014
https://github.com/sparkfun/ISL29125_Breakout

This file implements the functions of the SFE_ISL29125 sensor class as well as
providing documentation on what each function does.

Developed/Tested with:
Arduino Uno
Arduino IDE 1.0.5

This code is beerware; if you see me (or any other SparkFun employee) at the local, and you've found our code helpful, please buy us a round!
Distributed as-is; no warranty is given.

Modified by Ethan Kim on 4 Nov 2023 for use on STM32-F410RE
******************************************************************************/

#include "sparkfun_isl29125.h"
#include <string.h>

// Data Buffer
uint8_t buf[12];
// 01000100 <- 7 bit I2C address (0x44)
// 1000100X <- 8 bit I2C address (X as R/~W)
static const uint8_t ISL_I2C_8BIT_ADDR = ISL_I2C_ADDR << 1;

// Initialize - returns 0 if successful
// Verifies sensor is there by checking its device ID
// Resets all registers/configurations to factory default
// Sets configuration registers for the common use case
uint8_t ISL29125_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t ret = 0;
    uint8_t data = 0x00;

    //assume MX_I2C#_Init() is already called

    // Check device ID
    data = I2C_Read8(hi2c, DEVICE_ID);
    if (data != 0x7D)
      ret |= 1;

    // Reset registers
    ret |= ISL29125_Reset(hi2c);

    // Set to RGB mode, 10k lux, and high IR compensation
    ret |= ISL29125_Config(hi2c, CFG1_MODE_R | CFG1_375LUX, CFG2_IR_ADJUST_HIGH, CFG_DEFAULT);

    return ret;
}

// Reset all registers - returns 0 if successful
uint8_t ISL29125_Reset(I2C_HandleTypeDef *hi2c)
{
  uint8_t data = 0x00;
  // Reset registers
  I2C_Write8(hi2c, DEVICE_ID, 0x46);
  // Check reset
  data = I2C_Read8(hi2c, CONFIG_1);
  data |= I2C_Read8(hi2c, CONFIG_2);
  data |= I2C_Read8(hi2c, CONFIG_3);
  data |= I2C_Read8(hi2c, STATUS);
  if (data != 0x00)
  {
    return 1;
  }
  return 0;
}

// Setup Configuration registers (three registers) - returns 0 if successful
// Use CONFIG1 variables from SFE_ISL29125.h for first parameter config1, CONFIG2 for config2, 3 for 3
// Use CFG_DEFAULT for default configuration for that register
uint8_t ISL29125_Config(I2C_HandleTypeDef *hi2c, uint8_t config1, uint8_t config2, uint8_t config3)
{
  uint8_t ret = 0;
  uint8_t data = 0x00;

  // Set 1st configuration register
  I2C_Write8(hi2c, CONFIG_1, config1);
  // Set 2nd configuration register
  I2C_Write8(hi2c, CONFIG_2, config2);
  // Set 3rd configuration register
  I2C_Write8(hi2c, CONFIG_3, config3);

  // Check if configurations were set correctly
  data = I2C_Read8(hi2c, CONFIG_1);
  if (data != config1)
  {
    ret |= 1;
  }
  data = I2C_Read8(hi2c, CONFIG_2);
  if (data != config2)
  {
    ret |= 1;
  }
  data = I2C_Read8(hi2c, CONFIG_3);
  if (data != config3)
  {
    ret |= 1;
  }
  return ret;
}

// Sets upper threshold value for triggering interrupts
void ISL29125_SetUpperThreshold(I2C_HandleTypeDef *hi2c, uint16_t data)
{
  I2C_Write16(hi2c, THRESHOLD_HL, data);
}

// Sets lower threshold value for triggering interrupts
void ISL29125_SetLowerThreshold(I2C_HandleTypeDef *hi2c, uint16_t data)
{
  I2C_Write16(hi2c, THRESHOLD_LL, data);
}

// Check what the upper threshold is, 0xFFFF by default
uint16_t ISL29125_ReadUpperThreshold(I2C_HandleTypeDef *hi2c)
{
  return I2C_Read16(hi2c, THRESHOLD_HL);
}

// Check what the upper threshold is, 0x0000 by default
uint16_t ISL29125_ReadLowerThreshold(I2C_HandleTypeDef *hi2c)
{
  return I2C_Read16(hi2c, THRESHOLD_LL);
}

// Read the latest Sensor ADC reading for the color Red
uint16_t ISL29125_ReadRed(I2C_HandleTypeDef *hi2c)
{
  uint8_t dataLo, dataHi;
  dataLo = I2C_Read8(hi2c, RED_L);
  dataHi = I2C_Read8(hi2c, RED_H);
  // 16-bit unsigned result for Red Data of ISL29125
  return (uint16_t)((dataHi << 8) | dataLo);

}

// Read the latest Sensor ADC reading for the color Green
uint16_t ISL29125_ReadGreen(I2C_HandleTypeDef *hi2c)
{
  uint8_t dataLo, dataHi;
  dataLo = I2C_Read8(hi2c, GREEN_L);
  dataHi = I2C_Read8(hi2c, GREEN_H);
  // 16-bit unsigned result for Green Data of ISL29125
  return (uint16_t)((dataHi << 8) | dataLo);
}

// Read the latest Sensor ADC reading for the color Blue
uint16_t ISL29125_ReadBlue(I2C_HandleTypeDef *hi2c)
{
    uint8_t dataLo, dataHi;
    dataLo = I2C_Read8(hi2c, BLUE_L);
    dataHi = I2C_Read8(hi2c, BLUE_H);
    // 16-bit unsigned result for Green Data of ISL29125
    return (uint16_t)((dataHi << 8) | dataLo);
}

// Check status flag register that allows for checking for interrupts, brownouts, and ADC conversion completions
uint8_t ISL29125_ReadStatus(I2C_HandleTypeDef *hi2c)
{
  return I2C_Read8(hi2c, STATUS);
}

// Generic I2C read register (single byte)
uint8_t I2C_Read8(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
  HAL_StatusTypeDef ret;
  uint8_t val;

  // Tell I2C sensor that we want to read from register
  val = reg;
  ret = HAL_I2C_Master_Transmit(hi2c, ISL_I2C_8BIT_ADDR, (uint8_t *)&val, 1, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {
    val = 0x01;
  } else {
    // Read 1 byte from register
    ret = HAL_I2C_Master_Receive(hi2c, ISL_I2C_8BIT_ADDR, (uint8_t *)&val, 1, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {
      val = 0x01;
    }
  }
  return val;
}

// Generic I2C write data to register (single byte)
uint8_t I2C_Write8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[12];

  // Byte Write Sequence: [start] Device Address Byte (ack) Address Byte (ack) Data Byte (ack) [stop]
  buf[0] = reg;
  buf[1] = data;
  ret = HAL_I2C_Master_Transmit(hi2c, ISL_I2C_8BIT_ADDR, buf, 2, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {
    strcpy((char*)buf, "Error Tx\r\n");
    return 1;
  }
  return 0;
}

uint16_t I2C_Read16(I2C_HandleTypeDef *hi2c, uint8_t reg)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[12];
  uint16_t val = 0;

  // Tell I2C sensor that we want to read from register
  buf[0] = reg;
  ret = HAL_I2C_Master_Transmit(hi2c, ISL_I2C_8BIT_ADDR, buf, 1, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {
    strcpy((char*)buf, "Error Tx\r\n");
  } else {
    // Read 2 byte from register
    ret = HAL_I2C_Master_Receive(hi2c, ISL_I2C_8BIT_ADDR, buf, 2, HAL_MAX_DELAY);
    if ( ret != HAL_OK ) {
      strcpy((char*)buf, "Error Rx\r\n");
    } else {
      //Combine the bytes
      val = ((int16_t)buf[0] << 4) | (buf[1] >> 4);
    }
  }
  return val;
}

uint16_t I2C_Write16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t data)
{
  HAL_StatusTypeDef ret;
  uint8_t buf[12];

  // Byte Write Sequence: [start] Device Address Byte (ack) Address Byte (ack) Data Byte (ack) [stop]
  buf[0] = reg;
  buf[1] = data & 0xff;
  buf[2] = (data >> 8) & 0xff;
  ret = HAL_I2C_Master_Transmit(hi2c, ISL_I2C_8BIT_ADDR, buf, 3, HAL_MAX_DELAY);
  if ( ret != HAL_OK ) {
    strcpy((char*)buf, "Error Tx\r\n");
    return 1;
  }
  return 0;
}
