#ifndef __GM6020_H__
#define __GM6020_H__

#include "main.h"
#include "ui.h"

void GM6020_Init(void);
void GM6020_Task(uint32_t now_ms, GPIO_PinState key_sample);

void GM6020_UIStart(UI_MotorId_t motor, int16_t target_speed);
void GM6020_UISetSpeed(UI_MotorId_t motor, int16_t target_speed);
void GM6020_UIStartPos(UI_MotorId_t motor, int16_t target_pos_deg);
void GM6020_UISetPos(UI_MotorId_t motor, int16_t target_pos_deg);
void GM6020_UIStop(UI_MotorId_t motor);

uint8_t GM6020_HasFeedback(void);
int16_t GM6020_GetTargetSpeed(void);
uint8_t GM6020_IsRunning(void);

#endif
