#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"                  // Device header
#define LED_Port GPIOB
#define LED_Pin GPIO_Pin_5
void LED_Init(void);

#endif
