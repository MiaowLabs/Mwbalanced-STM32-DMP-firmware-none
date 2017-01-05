#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "stm32f10x.h"


extern uint16_t SoftTimer[5];

void SoftTimerCountDown(void);
void SysTick_Init(void);
void Delay_us(__IO u32 nTime);

#endif /* __SYSTICK_H */
