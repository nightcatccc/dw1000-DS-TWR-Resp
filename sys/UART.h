
#ifndef _UART_H_
#define _UART_H_

#include "stm32f10x.h"                  // Device header

void UART_Init(void);
void UART_SendData(USART_TypeDef* USARTx,uint8_t ch);
void UART_SendStr(USART_TypeDef* USARTx,uint8_t *str);
void Serial_Printf1(char *format, ...);
#endif
