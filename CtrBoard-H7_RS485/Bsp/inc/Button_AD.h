#ifndef __BUTTON_AD_H__
#define __BUTTON_AD_H__

#include "main.h"

typedef enum
{
    BUTTON_AD_NONE = 0,
    BUTTON_AD_CENTER,
    BUTTON_AD_LEFT,
    BUTTON_AD_RIGHT,
    BUTTON_AD_UP,
    BUTTON_AD_DOWN,
} ButtonAD_Key_t;

void ButtonAD_Init(void);
uint16_t ButtonAD_ReadRaw(void);
ButtonAD_Key_t ButtonAD_ReadKey(void);
ButtonAD_Key_t ButtonAD_GetEvent(void);

#endif
