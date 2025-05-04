#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x.h"                  // Device header
#include "bool.h"
#include "arm_math.h"
#include "BlueTooth_Usart.h"
#define FFT_Len 256
#define ADC_HIGHEST_SAMPLE_FREQ 400000
void My_ADC_DMA_Init(void);
extern flag adc_finish_fg;
extern int16_t ADC_SourceData[FFT_Len];//用adc采样只有12位  右对齐
extern float32_t Fs;//采样频率
extern float32_t Fs_eq;//等效采样频率
void Adc_Init(void);
void ADC_Stop(void);
void ADC_Start(void);
void Change_freq(float32_t freq,uint16_t wantid);
void ADC_Change_freq(float32_t freq);
void ADC_Set_Freq(u32 period,u32 psc);
#endif
