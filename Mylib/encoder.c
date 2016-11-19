#include "main.h"

//TIM3_CH1 ----- PC6
//TIM3_CH2 ----- PC7

//TIM5_CH1 ----- PA0
//TIM5_CH2 ----- PA1

void RGBLed_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
}


void RGB_Led_Control(unsigned char LEFT_LED ,unsigned char Right_LED)
{
	switch(LEFT_LED)
	{
		case LEFT_LED_RAD_COLOR:     GPIO_SetBits(GPIOA, GPIO_Pin_0);GPIO_ResetBits(GPIOA, GPIO_Pin_1);break;
		case LEFT_LED_BULE_COLOR:    GPIO_ResetBits(GPIOA, GPIO_Pin_0);GPIO_SetBits(GPIOA, GPIO_Pin_1);break;
		case LEFT_LED_PURPLE_COLOR:  GPIO_ResetBits(GPIOA, GPIO_Pin_0);GPIO_ResetBits(GPIOA, GPIO_Pin_1);break;
	}	
	switch(Right_LED)
	{
		case RIGHT_LED_RAD_COLOR:    GPIO_SetBits(GPIOC, GPIO_Pin_6);GPIO_ResetBits(GPIOC, GPIO_Pin_7);break;
		case RIGHT_LED_BULE_COLOR:   GPIO_ResetBits(GPIOC, GPIO_Pin_6);GPIO_SetBits(GPIOC, GPIO_Pin_7);break;
		case RIGHT_LED_PURPLE_COLOR: GPIO_ResetBits(GPIOC, GPIO_Pin_6);GPIO_ResetBits(GPIOC, GPIO_Pin_7);break;
	}
	
}



