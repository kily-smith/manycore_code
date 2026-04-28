#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Task(uint32_t now_ms, uint8_t enable_play);

#endif
