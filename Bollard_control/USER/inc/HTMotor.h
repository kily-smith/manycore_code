#ifndef __HT_MOTOR_H__
#define __HT_MOTOR_H__

#include <stdint.h>
#include "ui.h"

/* Default CAN standard ID for HT MIT motor (change to match your motor). */
#ifndef HT_MOTOR_CAN_ID
#define HT_MOTOR_CAN_ID 0x01U
#endif

/* HT output shaft ratio: motor:output = 6:1 */
#ifndef HT_MOTOR_GEAR_RATIO
#define HT_MOTOR_GEAR_RATIO 6.0f
#endif

typedef struct
{
    int16_t pos_raw;    /* 0..65535 mapped from MIT feedback */
    int16_t vel_raw;    /* 0..4095 mapped from MIT feedback */
    int16_t tor_raw;    /* 0..4095 mapped from MIT feedback */
    uint32_t last_rx_ms;
} HTMotor_Feedback_t;

void HTMotor_Init(void);
void HTMotor_Task(uint32_t now_ms);

void HTMotor_UIStart(UI_MotorId_t motor, int16_t target_speed);
void HTMotor_UISetSpeed(UI_MotorId_t motor, int16_t target_speed);
void HTMotor_UIStartPos(UI_MotorId_t motor, int16_t target_pos_deg);
void HTMotor_UISetPos(UI_MotorId_t motor, int16_t target_pos_deg);
void HTMotor_UIStop(UI_MotorId_t motor);

/* Called from CAN RX callback */
void HTMotor_OnCanRx(uint32_t now_ms, uint16_t std_id, uint8_t dlc, const uint8_t data[8]);
HTMotor_Feedback_t HTMotor_GetFeedback(void);

#endif
