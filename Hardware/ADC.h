#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x.h"                  // Device header
#include "bool.h"
#define FFT_Len 256
void My_ADC_DMA_Init(void);
extern flag adc_finish_fg;
extern int16_t ADC_SourceData[FFT_Len];//用adc采样只有12位  右对齐
extern u32 Fs;//采样频率
extern u32 Fs_eq;
void Adc_Init(void);
void ADC_Stop(void);
void ADC_Start(void);
void ADC_Change_freq(u32 freq);
#endif
