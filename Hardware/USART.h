#ifndef __USART_H
#define __USART_H
#include "stm32f10x.h"                  // Device header
#include "bool.h"
#include <arm_math.h>
#include "general_usart.h"
#define AMPLITUDE_X (457)
#define AMPLITUDE_Y (10)
#define AMPLITUDE_W (790-AMPLITUDE_X)
#define AMPLITUDE_H (206-AMPLITUDE_Y)
#define AMPLITUDE_ZERO_LINE ((AMPLITUDE_Y)+(AMPLITUDE_H))
#define AMPLITUDE_W_TO_DRAW 30
//(790,210) 是幅度显示的右下角坐标
//添加显示框显示相对幅值
#define PHASE_X (457)
#define PHASE_Y (AMPLITUDE_Y+AMPLITUDE_H+41)
#define PHASE_W (790-PHASE_X)
#define PHASE_H (451-PHASE_Y)
#define PHASE_W_TO_DRAW 30
#define PHASE_ZERO_LINE (PHASE_Y+(PHASE_H/2))
//上述宏的修改要与上位机文件一起修改！！！！！！！！
#define WAVE_W 417
#define SHOW_NUM 5
extern uint32_t Mag_max;
extern float32_t U[5];
extern float32_t Phase[5];
extern float32_t freq_basic;
extern float32_t THD;
void usart2_init(uint32_t bound);
void HMISendStr(char* buf);
void HMISendEnd(void);
void USART_SendByte(uint8_t ch);
int fputc(int ch, FILE * file);
void WaveForm_Draw(void);
void HMIDraw(void);
#endif
