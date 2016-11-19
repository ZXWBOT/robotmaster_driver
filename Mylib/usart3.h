#ifndef __USART1_H__
#define __USART1_H__

#include <main.h>

void USART3_Configuration(void);
void USART3_SendChar(unsigned char b);
void shanwai_send_data1(uint8_t *value,uint32_t size );

void Usart_Send_Char(unsigned int Right_value,unsigned int Left_value,unsigned int Magnet_value);

#endif
