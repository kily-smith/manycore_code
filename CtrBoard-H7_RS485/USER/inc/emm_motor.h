#ifndef __EMM_MOTOR_H
#define __EMM_MOTOR_H

#include <stdint.h>
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

void EmmMotor_Init(UART_HandleTypeDef *huart, uint8_t addr);
void EmmMotor_Task(uint32_t now_ms);

/* Call from HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback. */
void EmmMotor_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void EmmMotor_UART_ErrorCallback(UART_HandleTypeDef *huart);

/* UI-facing API (used by `USER/ui.c`). */
float EmmMotor_GetCurrentPosDeg(void);
float EmmMotor_GetTargetPosDeg(void);
float EmmMotor_GetSpeedRpm(void);

void EmmMotor_SetHeightSettingMm(int32_t height_mm, uint8_t dir_up);
int32_t EmmMotor_GetHeightSettingMm(void);
uint8_t EmmMotor_GetDirUp(void);

void EmmMotor_StartMoveToSetting(void);
void EmmMotor_StopMove(void);

/* Optional limit switches: define these in `main.h` to enable. */
#ifdef EMM_LIMIT_UP_GPIO_Port
uint8_t EmmMotor_IsUpperLimitActive(void);
#endif
#ifdef EMM_LIMIT_DOWN_GPIO_Port
uint8_t EmmMotor_IsLowerLimitActive(void);
#endif

#ifdef __cplusplus
}
#endif

#endif

