#ifndef __FFT_H
#define __FFT_H
#include "stm32f10x.h"                  // Device header
#include "arm_math.h"
#include "arm_const_structs.h"
#include "ADC.h"    //#define FFT_Len在ADC.h中
#define Pi 3.1415926f
extern flag improve;
extern float32_t FFT_In[FFT_Len*2];
extern float32_t FFT_IN_Non[FFT_Len*2];
extern float32_t FFT_Mag[FFT_Len];
void FFT_TEST(void);
#endif
