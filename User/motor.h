
#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void TIM2_PWM_Init(void);

void MOTOR_GPIO_Config(void);
void TIM3_Encoder_Init(void);
void TIM4_Encoder_Init(void);
void TIM2_PWM_CHANGE(u16 CCR3,u16 CCR4);
void TIM4_External_Clock_CountingMode(void);
void TIM3_External_Clock_CountingMode(void);
#endif
