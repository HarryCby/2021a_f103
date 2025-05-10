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
float xiuzheng[100][4] = {
    {1.435744, 1.7015921, 1.869248, 1.9602082},
    {1.2132189, 1.2877998, 1.339385, 1.3696841},
    {1.1071125, 1.1455721, 1.16474, 1.1843374},
    {1.0604541, 1.0927114, 1.0915028, 1.0999267},
    {1.037265, 1.0638301, 1.0569658, 1.0698228},
    {1.0226414, 1.0505966, 1.0388833, 1.0453885},
    {1.012349, 1.0387, 1.0271057, 1.0341638},
    {1.0082952, 1.0403738, 1.0134062, 1.0268834},
    {1.006438, 1.0544316, 1.006044, 1.0137298},
    {1.0032328, 1.0239286, 1.0023689, 1.0060346},
    {0.9970248, 1.0201346, 0.99447936, 0.99926484},
    {1.0033405, 1.0167308, 0.98924166, 0.9954012},
    {0.98746705, 1.0226147, 0.99409676, 0.997548},
    {1.0056455, 1.0173084, 0.9859058, 1.0082071},
    {0.99112093, 1.0115464, 0.97594213, 0.9955},
    {0.9924783, 1.0108868, 0.9816227, 0.9877578},
    {0.9836417, 1.0172337, 0.97551244, 1.013995},
    {0.98841697, 1.0060725, 0.9897183, 0.98020774},
    {0.98428845, 0.9992014, 0.9629144, 0.9518441},
    {0.98136574, 0.997896, 0.9613143, 0.9533166},
    {0.97721946, 0.99864584, 0.9738823, 0.98314637},
    {0.98672014, 1.009581, 0.9802617, 0.9912548},
    {0.9886464, 1.0027196, 0.96585304, 0.9925532},
    {0.987685, 1.0013425, 0.96907717, 0.985979},
    {0.9961376, 1.0006348, 0.9501898, 0.97691154},
    {1.0300267, 0.9936527, 1.0068517, 0.98301816},
    {0.9864406, 1.010953, 0.9794809, 0.9726925},
    {0.9857289, 0.98640305, 0.9678695, 0.9724858},
    {0.98622656, 0.98920804, 0.9722374, 0.9791057},
    {0.9839647, 0.99192816, 0.97030884, 0.9655661},
    {0.9808794, 0.9886787, 0.9655582, 0.95422333},
    {0.97444147, 0.9787735, 0.94981515, 0.93023646},
    {0.97805685, 0.9833995, 0.96014833, 0.9581408},
    {0.97152925, 0.98778, 0.92442703, 0.89342666},
    {0.9708544, 0.9939716, 0.94217646, 0.9203726},
    {0.971369, 0.9919674, 0.93093157, 0.90474343},
    {0.97004354, 0.98853546, 0.9466346, 0.92545485},
    {0.9756969, 0.9962428, 0.9507486, 0.90809304},
    {0.974674, 0.9987809, 0.94928706, 0.92713785},
    {0.97049326, 0.9855677, 0.9240657, 0.90074277},
    {0.96846586, 0.9872794, 0.9369052, 0.9192947},
    {0.9740415, 0.9846256, 0.92449015, 0.90872186},
    {0.9714808, 0.9828766, 0.92653775, 0.8996225},
    {0.9674536, 0.98072934, 0.92919457, 0.9000163},
    {0.971821, 0.98175067, 0.9236639, 0.9023201},
    {0.9595244, 0.954571, 0.87840927, 0.8361265},
    {0.96335053, 0.9641794, 0.8942802, 0.8663865},
    {0.9585151, 0.97106767, 0.9227817, 0.8873596},
    {0.9521143, 0.98624533, 0.8919437, 0.8483633},
    {0.9689541, 0.97151685, 0.90070283, 0.8784313},
    {0.9908112, 0.9661904, 0.8767741, 0.8355606},
    {0.96953636, 0.91256386, 0.82501566, 0.7658951},
    {0.9679266, 0.91169906, 0.8173643, 0.7542859},
    {0.96835846, 0.91755396, 0.82961136, 0.77327764},
    {0.98367935, 0.9527913, 0.8776163, 0.8369457},
    {0.94866556, 0.9170639, 0.8329925, 0.7766189},
    {0.9856531, 0.9371174, 0.8507959, 0.8073804},
    {0.9463451, 0.90545434, 0.79102385, 0.7455715},
    {0.95677495, 0.96800524, 0.84229255, 0.79402333},
    {0.9358872, 0.88084006, 0.8026378, 0.6932645},
    {0.93629366, 0.8770886, 0.8444448, 0.7056334},
    {0.9510299, 0.9041017, 0.82295287, 0.7694087},
    {0.92551684, 0.85907394, 0.7373551, 0.8111722},
    {0.9459423, 0.88896567, 0.7944113, 0.74303937},
    {0.9405428, 0.8967394, 0.79830414, 0.75080466},
    {0.941051, 0.8847908, 0.78857404, 0.74500024},
    {0.936725, 0.87117577, 0.7765156, 0.7207},
    {0.9331825, 0.8672554, 0.7494583, 0.68234205},
    {0.9134877, 0.8263529, 0.69755715, 0.6145748},
    {0.91977364, 0.82695097, 0.71019125, 0.6325716},
    {0.92382735, 0.84648305, 0.74178773, 0.68119544},
    {0.90998054, 0.8244577, 0.6966517, 0.62275195},
    {0.89466345, 0.7843372, 0.6335571, 0.606215},
    {0.8946694, 0.78964674, 0.6491512, 0.58602834},
    {0.90954953, 0.82523286, 0.71250284, 0.6508246},
    {0.8874151, 0.7748967, 0.63069797, 0.5600964},
    {0.88324416, 0.76595384, 0.61325395, 0.547538},
    {0.9013208, 0.79873514, 0.67456895, 0.6120228},
    {0.8929863, 0.7681506, 0.6212434, 0.5451685},
    {0.88390785, 0.77872306, 0.64617693, 0.5754794},
    {0.8751502, 0.7259105, 0.5648115, 0.5320697},
    {0.86089486, 0.70691824, 0.5567122, 0.53110546},
    {0.8827059, 0.75473046, 0.61835825, 0.56039816},
    {0.87715805, 0.7472946, 0.600951, 0.547944},
    {0.8461191, 0.69889784, 0.549523, 0.4802628},
    {0.84207433, 0.686794, 0.5281665, 0.48024523},
    {0.86096954, 0.72491187, 0.58119535, 0.5228609},
    {0.8597393, 0.72199416, 0.5731919, 0.45224723},
    {0.8436988, 0.680264, 0.5275839, 0.45224723},
    {0.8509699, 0.7072852, 0.5509197, 0.49694884},
    {0.8409314, 0.6852096, 0.53685844, 0.47974706},
    {0.82564265, 0.6522482, 0.4878092, 0.440971},
    {0.8071128, 0.618253, 0.47454816, 0.44389153},
    {0.8287503, 0.662365, 0.51643497, 0.46115777},
    {0.78208697, 0.5857672, 0.52489674, 0.47103912},
    {0.78128165, 0.58945143, 0.46596187, 0.44740483},
    {0.80358565, 0.6178371, 0.45694315, 0.4020521},
    {0.77849555, 0.59506875, 0.42141888, 0.41451302},
    {0.783162, 0.5892614, 0.42271632, 0.37917033},
    {0.77683127, 0.54748935, 0.4340623, 0.41822347}
};
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

#define TEST_LEN 50

//求平均用
//float32_t Phase_temp[5];
float32_t U_temp[5];
//float32_t THD_temp; 

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
void Seek_Right(float32_t arr[],uint16_t Len,uint16_t medium,float32_t* temp,u16 *id_max,u8 which)
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
//		if(*temp<FFT_Mag[id_freq[0]]*0.01f/xiuzheng[(int)(freq_basic/1000)-1][which]){
//			*temp=0;
//		}
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
			u8 i;
			//清空缓存区
			for(i=0;i<5;i++){		
				U_temp[i]=0;
			}
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
						/*********************开始计算***********************test_num==3使用hann测相位*****/
						
					if(test_num>=3&&test_num<TEST_LEN-1+3){
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1],0);
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2],1);
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3],2);          
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4],3);
						//主要是找id
					U[0]=1;
					U[1]=Fudu[1]/Fudu[0];
					U[2]=Fudu[2]/Fudu[0];
					U[3]=Fudu[3]/Fudu[0];
					U[4]=Fudu[4]/Fudu[0];
//					THD=sqrt(U[1]*U[1]+U[2]*U[2]+U[3]*U[3]+U[4]*U[4]);
					u8 i;
					for(i=1;i<5;i++){
						U_temp[i]+=U[i];
						}
					}
				/*test_num==3使用bk测相位*/
					if(test_num==TEST_LEN-1+3){
						u8 i;
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1],0);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2],1);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3],2);          
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4],3);
						U[0]=1;
						U[1]=Fudu[1]/Fudu[0];
						U[2]=Fudu[2]/Fudu[0];
						U[3]=Fudu[3]/Fudu[0];
						U[4]=Fudu[4]/Fudu[0];
						for(i=1;i<5;i++){
							U_temp[i]+=U[i];
						}
					
						for(i=1;i<5;i++){
							U[i]=U_temp[i]/TEST_LEN;//7-3+1
						}
						/**************有用的*****/
						u32 index;
						index=(int)(freq_basic/1000)-1;
						for(i=1;i<5;i++){
							U[i]/=xiuzheng[index][i-1];
							U[i]*=1.02f;
							
						}
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
//					Change_freq(1000,16);
//					ADC_Start();//本质是启动TIM3还有复位dma
//					ADC_Started=True;
//				//下面while跑完这个if就结束了要用ADC_Start();启动
//				while(start_sign_0){
//					if(ADC_Started==False){
//							ADC_Start();//本质是启动TIM3
//							ADC_Started=True;
//					}
//					if(adc_finish_fg==True){
//						FFT_TEST();
//						Seek_Max(FFT_Mag,FFT_Len/2,&Fudu[0],&id_freq[0]);
//						if(test_num<TEST_LEN-1){
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1],0);
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2],1);
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3],2);          
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4],3);
//						uint8_t i;
//						U[0]=1;
//						U[1]=Fudu[1]/Fudu[0];
//						U[2]=Fudu[2]/Fudu[0];
//						U[3]=Fudu[3]/Fudu[0];
//						U[4]=Fudu[4]/Fudu[0];
//						for(i=0;i<5;i++){
//							U_temp[i]+=U[i];
//						}
//					}		
//						if(test_num==TEST_LEN-1){
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1],0);
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2],1);
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3],2);          
//						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4],3);
//							uint8_t i;
//							U[0]=1;
//							U[1]=Fudu[1]/Fudu[0];
//							U[2]=Fudu[2]/Fudu[0];
//							U[3]=Fudu[3]/Fudu[0];
//							U[4]=Fudu[4]/Fudu[0];
//							for(i=0;i<5;i++){
//								U_temp[i]+=U[i];
//							}
//							for(i=1;i<5;i++){
//								U[i]=U_temp[i]/TEST_LEN;//4-0+1
//							}
//					}	
//							/**************有用的*****/
//						u32 index;
//						index=(int)(freq_basic/1000)-1;
//						for(i=1;i<5;i++){
//							U[i]/=xiuzheng[index][i-1];
//							U[i]*=1.02f;
//						}
//							THD=sqrt(U[1]*U[1]+U[2]*U[2]+U[3]*U[3]+U[4]*U[4]);
//							HMIDraw();
//							WaveForm_Draw();
//							test_num=0;
//							Init_Packet();
//							btpacket.Freq=1000*10000;
//							uint8_t * Buffer=(uint8_t*)&btpacket;
//							SendPacket(Buffer);
//							adc_finish_fg=False;
//							ADC_Started=False;
//							ADC_Change_freq(256000);
//							Fs_eq=256000;
//							Fs=256000;
//							start_sign_0=False;
//							continue; // 跳过本次循环的test_num++，直接进入下一轮采样
//						}
//						adc_finish_fg=False;
//						ADC_Started=False;
//						test_num++;
//					}








				ADC_Start();//本质是启动TIM3还有复位dma
	      ADC_Started=True;
				//下面while跑完这个if就结束了要用ADC_Start();启动
				while(start_sign_0){
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
						/*********************开始计算***********************test_num==3使用hann测相位*****/
						
					if(test_num>=3&&test_num<TEST_LEN-1+3){
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1],0);
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2],1);
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3],2);          
					Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4],3);
						//主要是找id
					U[0]=1;
					U[1]=Fudu[1]/Fudu[0];
					U[2]=Fudu[2]/Fudu[0];
					U[3]=Fudu[3]/Fudu[0];
					U[4]=Fudu[4]/Fudu[0];
//					THD=sqrt(U[1]*U[1]+U[2]*U[2]+U[3]*U[3]+U[4]*U[4]);
					u8 i;
					for(i=1;i<5;i++){
						U_temp[i]+=U[i];
						}
					}
				/*test_num==3使用bk测相位*/
					if(test_num==TEST_LEN-1+3){
						u8 i;
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*2,&Fudu[1],&id_freq[1],0);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*3,&Fudu[2],&id_freq[2],1);
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*4,&Fudu[3],&id_freq[3],2);          
						Seek_Right(FFT_Mag,FFT_Len/2,id_freq[0]*5,&Fudu[4],&id_freq[4],3);
						U[0]=1;
						U[1]=Fudu[1]/Fudu[0];
						U[2]=Fudu[2]/Fudu[0];
						U[3]=Fudu[3]/Fudu[0];
						U[4]=Fudu[4]/Fudu[0];
						for(i=1;i<5;i++){
							U_temp[i]+=U[i];
						}
					
						for(i=1;i<5;i++){
							U[i]=U_temp[i]/TEST_LEN;//7-3+1
						}
						/**************有用的*****/
						u32 index;
						index=(int)(freq_basic/1000)-1;
						for(i=1;i<5;i++){
							U[i]/=xiuzheng[index][i-1];
							U[i]*=1.02f;
							
						}
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

