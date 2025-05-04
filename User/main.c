#include "stm32f10x.h"                  // Device header
#include "arm_math.h"
#include "math.h"//在mdk的 ARMCC\include里面
//在文件管理下删除Start下的core_cm3.h这样就是用的DSP下的core_cm3.h了，注意不是在keil里remove

#include "Delay.h"
//#include "OLED.h"
#include "ADC.h"
#include "FFT.h"
#include "USART.h"
#include "BlueTooth_Usart.h"
#include "general_usart.h"
#include "get_rad.h"

uint32_t Mag_max;
uint32_t Find_MagMax(float32_t * arr,uint16_t sizeofFFT_Mag){
	uint32_t max=0;
	u32 i;//size可是uint16_t  i的类型必须比size max大
	
	for(i=0;i<sizeofFFT_Mag/2;i++){
		if(max<arr[i]){
			max=arr[i];
		}
	}
	return max;
}
u8 test_num;
flag ADC_Started = False;

usart_user selected_USART=usart2_u;
volatile flag start_sign_0=False;
volatile flag start_sign_1=False;

float32_t freq_basic;

float32_t Fudu[5];
float32_t Phase[5];
float32_t U[5];
uint16_t id_freq[5];
float32_t THD;



//求平均用
float32_t Phase_temp[5];
float32_t U_temp[5];
float32_t THD_temp; 

//u16 id_freq_1;
//u16 id_freq_2;
//u16 id_freq_3;
//u16 id_freq_4;
//u16 id_freq_5;


//序号和幅度值均改变
void Seek_Max(float32_t arr[],uint16_t size,float32_t* temp,u16 *id_max)
{
	u8 i;
	*id_max=1;
	float32_t temp_max=arr[1];//调过直流分量
	for(i=1;i<size;i++)
	{
		if(arr[i]>temp_max)
		{
			temp_max=arr[i];
			*id_max=i;
		}
	}
	*temp=temp_max;
}

//仅仅改变幅度值
void Seek_Right(float32_t arr[],uint16_t Len,uint16_t medium,float32_t* temp,u16 *id_max)
{
	u8 i;
	*id_max=medium;
	*temp=arr[medium];
	if(medium>10)//可修改
	{
		 for(i=medium-3;i<=medium+3&&i<Len;i++)
		 {
				 if(arr[i]>*temp)
				 {
					*temp = arr[i];
					*id_max=i;
				 }
		 }
	 }
}


BluetoothPacket btpacket;//数据包
void Init_Packet(void)
{
	btpacket.head=HEAD;
	btpacket.tail=TAIL;
	uint8_t i;
	//所有数据扩大10000倍之后在安卓软件中恢复
	btpacket.Freq=freq_basic*10000;
	btpacket.THD_P=THD*10000;
	for(i=0;i<5;i++){
		btpacket.data[i]=U[i]*10000;
		btpacket.data[i+5]=Phase[i]/Pi*180*10000;
	}
}

//可以考虑FFT_Len 变为1024
//FFT_Len*Freq_basic/64;
//matlab上效果小
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//使用了中断，则这一句必须要有
	Adc_Init();//里面TIM3没启动  
	//！！！！小心cmd顺序问题
	usart2_init(9600);
	BTUSART_Init(9600);
//	Delay_ms(25);//等待串口初始化
	while(1){
		//注意start_sign_0||start_sign_1是由两个东西操控，凡是start_sign_0||start_sign_1为真，则设置无法置True
		if(start_sign_0||start_sign_1){
//			u8 i;
			//清空缓存区
//			for(i=0;i<5;i++){
//				Phase_temp[i]=0;
//				U_temp[i]=0;
//				THD_temp=0;
//			}
			if(start_sign_1&&!start_sign_0){
				ADC_Start();//本质是启动TIM3还有复位dma
	      ADC_Started=True;
				//下面while跑完这个if就结束了要用ADC_Start();启动
				while(start_sign_1){
				if(ADC_Started==False){
						ADC_Start();//本质是启动TIM3
						ADC_Started=True;
				}
				//加入求平均值？？？   或者由串口屏实现开启功能
				//都来
				//test_num==0 第一次不要修正
				if(adc_finish_fg==True){
					FFT_TEST();
					Seek_Max(FFT_Mag,FFT_Len/2,&Fudu[0],&id_freq[0]);
					if(test_num==0){
						freq_basic=(id_freq[0])*Fs_eq/(FFT_Len);
						Change_freq(freq_basic,16);
					}
					//todo
					if(Fs_eq*FFT_Len/16<1500){
							if(test_num<=2&&test_num>0){      //临界值之后的加窗后的频率误差不可接受)
								float32_t delta;
								delta=0.5*(FFT_Mag[id_freq[0]+1]-FFT_Mag[id_freq[0]-1])/(2*FFT_Mag[id_freq[0]]-FFT_Mag[id_freq[0]+1]-FFT_Mag[id_freq[0]-1]);
								freq_basic=(id_freq[0]+delta)*Fs_eq/(FFT_Len);//加窗后频率精度上有一点损失(可能在10khz起步后 误差不可接受) 只接受低频的
							/*************************************/
							if(test_num!=2){
								Change_freq(freq_basic,16);
							}else{
								//test_num==2时正式采用freq_basic 但是采样率要相比提高！！！！
								Change_freq(freq_basic,8);//更高的采样频率，越发要求更高的频率准确性
							}
						}
					}
					else{
						Change_freq(freq_basic,8);
						if(test_num<2){
							test_num=2;
						}
					}
					//test_num==2时正式采用freq_basic
					/*****************************************************/
						//todo 最高为？？？KHz的采样频率  结合顺序采样
//					if(test_num<=1){
//							if(freq_basic!=0){
//									//调整adc的采样频率freq_basic
//								if(FFT_Len*freq_basic/32<=500000){
//									ADC_Change_freq(FFT_Len*freq_basic/32);//基頻只能到25k  理论上 之后基频 在第32个左右  但是freq_basic不完全等于实际基频就不会刚好出现在第32位！！！！！！
//									Fs_eq=FFT_Len*freq_basic/32;
//								}else{
//									ADC_Change_freq(8* freq_basic / (2*8 + 1));
//									Fs_eq=8*freq_basic;
//								}
//							}
//						}
						/*********************开始计算***********************test_num==3使用hann测相位*****/
						
					if(test_num==3){
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1]);
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2]);
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3]);          
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4]);
						//主要是找id
					uint8_t i;
					float32_t y0,y1,y2;
					for(i=0;i<5;i++){
						Phase[i]=atan2(FFT_In[2*id_freq[i]+1],FFT_In[2*id_freq[i]]);
						y0=FFT_Mag[id_freq[i]-1];
						y1=FFT_Mag[id_freq[i]];
						y2=FFT_Mag[id_freq[i]+1];
						Phase[i]-=0.5*(y2-y0)/(2*y1-y0-y2)*pi;
					}	
//					U[0]=1;
//					U[1]=Fudu[1]/Fudu[0];
//					U[2]=Fudu[2]/Fudu[0];
//					U[3]=Fudu[3]/Fudu[0];
//					U[4]=Fudu[4]/Fudu[0];
//					THD=sqrt(U[1]*U[1]+U[2]*U[2]+U[3]*U[3]+U[4]*U[4]);
					//第一次用hann窗测相位
//						for(i=0;i<5;i++){
//						Phase_temp[i]+=Phase[i];
//						U_temp[i]+=U[i];
//						THD_temp+=THD;
//						}
					}
				/*test_num==3使用bk测相位*/
					if(test_num==4){
						u8 i;
//						for(i=0;i<5;i++){
//						Phase[i]=Phase_temp[i]/2;
//						U[i]=U_temp[i]/2;
//						THD=THD_temp/2;
//						}
						float32_t tmp;
						tmp=Phase[0];
						for(i=0;i<5;i++){
							Phase[i]-=tmp;
							Phase[i] = fmod(Phase[i], 2 * pi);
							if (Phase[i] > pi) {
              Phase[i] -= 2 * pi;
              } else if (Phase[i] <= -pi) {
              Phase[i] += 2 * pi;
              }
						}
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1]);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2]);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3]);          
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4]);
						U[0]=1;
						U[1]=Fudu[1]/Fudu[0];
						U[2]=Fudu[2]/Fudu[0];
						U[3]=Fudu[3]/Fudu[0];
						U[4]=Fudu[4]/Fudu[0];
						/**************有用的*****/
//						u32 index;
//						index=(int)(freq_basic/1000)-1;
//						for(i=1;i<5;i++){
//							U[i]/=xiuzheng[index][i];
//						}
						THD=sqrt(U[1]*U[1]+U[2]*U[2]+U[3]*U[3]+U[4]*U[4]);
						HMIDraw();
						WaveForm_Draw();
						test_num=0;
						Init_Packet();
						uint8_t * Buffer=(uint8_t*)&btpacket;
						SendPacket(Buffer);
						adc_finish_fg=False;
						ADC_Started=False;
						ADC_Change_freq(256000);
						Fs_eq=256000;
						Fs=256000;
						start_sign_1=False;
						continue; // 跳过本次循环的test_num++，直接进入下一轮采样
					}
					adc_finish_fg=False;
					ADC_Started=False;
					test_num++;
				}
	    }
			
			}
			else if(!start_sign_1&&start_sign_0){
					ADC_Set_Freq(4500-1,1-1);//针对256
					ADC_Start();//本质是启动TIM3还有复位dma
					ADC_Started=True;
				//下面while跑完这个if就结束了要用ADC_Start();启动
				while(start_sign_0){
					if(ADC_Started==False){
							ADC_Start();//本质是启动TIM3
							ADC_Started=True;
					}
					if(adc_finish_fg==True){
						FFT_TEST();
						Seek_Max(FFT_Mag,FFT_Len/2,&Fudu[0],&id_freq[0]);
						if(test_num==0){
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1]);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2]);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3]);          
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4]);
						uint8_t i;
						float32_t y0,y1,y2;
						for(i=0;i<5;i++){
							Phase[i]=atan2(FFT_In[2*id_freq[i]+1],FFT_In[2*id_freq[i]]);
							y0=FFT_Mag[id_freq[i]-1];
							y1=FFT_Mag[id_freq[i]];
							y2=FFT_Mag[id_freq[i]+1];
							Phase[i]-=0.5*(y2-y0)/(2*y1-y0-y2)*pi;
						}	
						float32_t tmp;
						tmp=Phase[0];
						for(i=0;i<5;i++){
							Phase[i]-=tmp;
							Phase[i] = fmod(Phase[i], 2 * pi);
							if (Phase[i] > pi) {
							Phase[i] -= 2 * pi;
							} else if (Phase[i] <= -pi) {
							Phase[i] += 2 * pi;
							}
						}
	//					U[0]=1;
	//					U[1]=Fudu[1]/Fudu[0];
	//					U[2]=Fudu[2]/Fudu[0];
	//					U[3]=Fudu[3]/Fudu[0];
	//					U[4]=Fudu[4]/Fudu[0];
	//					THD=sqrt(U[1]*U[1]+U[2]*U[2]+U[3]*U[3]+U[4]*U[4]);
	//						for(i=0;i<5;i++){
	//						Phase_temp[i]+=Phase[i];
	//						U_temp[i]+=U[i];
	//						THD_temp+=THD;
	//						}
						}		
						if(test_num==1){
//							u8 i;
	//						for(i=0;i<5;i++){
	//						Phase[i]=Phase_temp[i]/2;
	//						U[i]=U_temp[i]/2;
	//						THD=THD_temp/2;
	//						}
							HMIDraw();
							WaveForm_Draw();
							test_num=0;
							Init_Packet();
							btpacket.Freq=1000*10000;
							uint8_t * Buffer=(uint8_t*)&btpacket;
							SendPacket(Buffer);
							adc_finish_fg=False;
							ADC_Started=False;
							ADC_Change_freq(256000);
							Fs_eq=256000;
							Fs=256000;
							start_sign_0=False;
							continue; // 跳过本次循环的test_num++，直接进入下一轮采样
						}
						adc_finish_fg=False;
						ADC_Started=False;
						test_num++;
					}
				}
			}
		}

	}//while(1)

}

