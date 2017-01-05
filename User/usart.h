#ifndef __USART_H
#define	__USART_H

#include "stm32f10x.h"
#include <stdio.h>
//´®¿Ú½ÓÊÕDMA»º´æ
#define UART_RX_LEN		128
extern uint8_t Uart_Rx[UART_RX_LEN];

#define Uart3BufferSize 128

extern char Uart3Buffer[];
extern u8 Uart3Index;

void USART1_Config(void);
void USART3_Config(char baudrate);
void NVIC_Configuration(void);


void Uart1SendByte(char byte) ;
void Uart1SendBuf(char *buf, u16 len);
void Uart1SendStr(char * str);
	
void Uart3SendByte(char byte)  ;
void Uart3SendBuf(char *buf, u16 len);
void Uart3SendStr(char *str);

#define DebugOut(str)	Uart1SendStr(str)
#define BluetoothOut(str) Uart3SendStr(str)


#endif /* __USART1_H */

