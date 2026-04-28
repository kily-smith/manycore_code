#ifndef __UI_TEST_H__
#define __UI_TEST_H__

#include "ui.h"

typedef enum
{
    UI_TEST_VIEW_MENU = 0,
    UI_TEST_VIEW_SPEED,
    UI_TEST_VIEW_POS,
    UI_TEST_VIEW_OLED_TEST,
    UI_TEST_VIEW_PLACEHOLDER
} UI_TestView_t;

void UI_TestInit(const UI_Callbacks_t *callbacks);
void UI_TestTask(uint32_t now_ms, ButtonAD_Key_t key_now);
void UI_TestRequestRedraw(void);
uint8_t UI_TestConsumeRedraw(void);

UI_TestView_t UI_TestGetView(void);
void UI_TestGetMenuLine(uint8_t row, char *buf, uint32_t buf_len);
const char *UI_TestGetPlaceholderTitle(void);
const char *UI_TestGetSpeedMotorName(void);
UI_MotorId_t UI_TestGetSpeedMotorId(void);
int16_t UI_TestGetSpeedTarget(void);
uint8_t UI_TestGetSpeedRunning(void);
const char *UI_TestGetPosMotorName(void);
UI_MotorId_t UI_TestGetPosMotorId(void);
int16_t UI_TestGetPosTargetDeg(void);
uint8_t UI_TestGetPosRunning(void);

#endif
