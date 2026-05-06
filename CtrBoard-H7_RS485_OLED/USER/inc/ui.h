#ifndef __USER_UI_H__
#define __USER_UI_H__

#include "stdint.h"
#include "Button_AD.h"

typedef enum
{
    UI_MOTOR_GM6020 = 0,
    UI_MOTOR_RM3508,
    UI_MOTOR_DM4310,
    UI_MOTOR_ZDT_STEPMOTOR,
    UI_MOTOR_HT,
    UI_MOTOR_COUNT
} UI_MotorId_t;

typedef struct
{
    void (*on_motor_start)(UI_MotorId_t motor, int16_t target_speed);
    void (*on_motor_set_speed)(UI_MotorId_t motor, int16_t target_speed);
    void (*on_motor_start_pos)(UI_MotorId_t motor, int16_t target_pos_deg);
    void (*on_motor_set_pos)(UI_MotorId_t motor, int16_t target_pos_deg);
    void (*on_motor_stop)(UI_MotorId_t motor);
} UI_Callbacks_t;

void UI_Init(const UI_Callbacks_t *callbacks);
void UI_Task(uint32_t now_ms, ButtonAD_Key_t key_now);
void UI_RequestRedraw(void);
void UI_SetTargetPositionDeg(int32_t target_pos_deg);
int32_t UI_GetTargetPositionDeg(void);

#endif
