#ifndef _DELAY__H_
#define _DELAY__H_

#include "stm32f10x.h"

void delay_us(u32 n);
void delay_ms(u32 n);
void get_ms(unsigned long *time);
#endif
