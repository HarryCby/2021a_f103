#include "arm_math.h"
#include "math.h"//?mdk? ARMCC\include??
#include "get_rad.h"
void get_rad(float32_t* phase,float32_t* fftmag,float32_t* fft,uint16_t* id)
{
	uint8_t i;
	float32_t y0,y1,y2;
	//PHASE_LNGTH 5
	for(i=0;i<PHASE_LNGTH;i++){
		phase[i]=atan2(fft[2*id[i]+1],fft[2*id[i]]);
		y0=fftmag[id[i]-1];
		y1=fftmag[id[i]];
		y2=fftmag[id[i]+1];
		phase[i]-=0.5*(y2-y0)/(2*y1-y0-y2)*pi;
	}

}

