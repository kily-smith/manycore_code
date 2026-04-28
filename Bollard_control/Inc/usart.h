/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>

/* USER CODE END Includes */

extern UART_HandleTypeDef huart7;

extern UART_HandleTypeDef huart8;

extern UART_HandleTypeDef huart6;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_UART7_Init(void);
void MX_UART8_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void USART6_StartReceiveIT(void);
void USART6_RxCpltCallback(void);
void USART6_ErrorCallback(void);
void UART8_StartReceiveIT(void);
void UART8_RxCpltCallback(void);
void UART8_ErrorCallback(void);

void EmmMotor_Init(void);
void EmmMotor_Task(uint32_t now_ms);
float EmmMotor_GetCurrentPosDeg(void);
float EmmMotor_GetTargetPosDeg(void);
float EmmMotor_GetSpeedRpm(void);
void EmmMotor_SetHeightSettingMm(int32_t height_mm);
int32_t EmmMotor_GetHeightSettingMm(void);
float EmmMotor_GetCurrentHeightMm(void);
float EmmMotor_GetTargetHeightMm(void);
void EmmMotor_StartMoveToSetting(void);
void EmmMotor_StopMove(void);

void USB_UART8_BridgeInit(void);
void USB_UART8_BridgeTask(uint32_t now_ms);
void USB_UART8_OnUsbReceive(const uint8_t *data, uint16_t len);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

