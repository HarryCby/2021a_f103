#include "stm32f10x.h"                  // Device header

void LED_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	GPIO_InitTypeDef LED_jgt;
	LED_jgt.GPIO_Pin=GPIO_Pin_5;
	LED_jgt.GPIO_Mode=GPIO_Mode_Out_PP;
	LED_jgt.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&LED_jgt);
}
