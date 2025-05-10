#include "stm32f10x.h"                  // Device header
#include "FFT.h"//数据使用
#include "USART.h"
#include "Delay.h"
#include <arm_math.h>


//usart2 TX:PA2  RX:PA3  用来与上位机通信
u8 TXT_ID[12]={0,1,2,3,4,5,6,7,8,9,10,11};
USART_RX_Packet rxPacket_2;
uint8_t dataIndex_2 = 0;
uint8_t packetState_2 = 0; // 0:等待头 1:接收数据 2:等待尾
void usart2_init(u32 bound)
{
   GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);	//使能USART2时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);	//使能GPIOA时钟
	USART_DeInit(USART2);  //复位串口2
	 //USART2_TX   PA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2
 
	 //USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3
		
    // USART2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);                           // 根据指定的参数初始化VIC寄存器

	// USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound; // 一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); // 开启中断  处理接受的数据
	USART_Cmd(USART2, ENABLE);                     // 使能串口
}
void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        // 读取接收到的数据
				uint8_t receivedByte;
        receivedByte = USART_ReceiveData(USART2);
        
        switch(packetState_2) {
            case 0: // 等待头
                if(receivedByte == HEAD) {
                    packetState_2 = 1;
                    dataIndex_2 = 0;
                }
                break;
                
            case 1: // 接收数据
                rxPacket_2.data[dataIndex_2++] = receivedByte;
                if(dataIndex_2 >= RX_DATA_LEN) {
                    packetState_2 = 2;
                }
                break;
                
            case 2: // 等待尾
                if(receivedByte == TAIL) {
                    rxPacket_2.received = 1;
                    // 数据包接收完成，可以处理
                    processPacket(&rxPacket_2);
                }
                packetState_2 = 0; // 无论是否匹配，都重置状态
                break;
        }
        
        // 清除中断标志
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}
void HMISendStr(char* buf)
{
	uint8_t i=0;
	while(buf[i]!=0)
	{
		USART_SendData(USART2,buf[i]);
		//发送ascll码
		//只能发送低九位的数据，实际为8位，有一位是奇偶校验位
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
		i++;
	}
}
void HMISendEnd(void)
{
	u8 i;
	for(i=0;i<3;i++)
	{
		USART_SendData(USART2,0xff);
		while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
	}
}
void USART_SendByte(uint8_t ch)
{
	USART_SendData(USART2,(uint8_t)ch);
	while(USART_GetFlagStatus(USART2,USART_FLAG_TXE)==RESET);
}


//todo 
//使用mag
	/*
	// 生成波形数据
        ArrayList<Entry> entries = new ArrayList<>();
        int pointCount = 200; // 采样点数

        for (int i = 0; i < pointCount; i++) {
            float x = (float)i / pointCount * 2 * (float)Math.PI;
            float y = 0;

            // 叠加各次谐波
            for (int h = 0; h < amplitudes.length; h++) {
                // (h+1)   1=基波, 2=二次谐波...
                y += (float) (amplitudes[h] * Math.sin((h + 1) * x + phases[h]* (Math.PI / 180.0)));//x前不加频率系数  因为只展示一个周期的波形
            }

            entries.add(new Entry(i, y));
        }
	*/
void WaveForm_Draw(void){
	selected_USART=usart2_u;
	uint16_t i;
	uint8_t j;
	float x,h;
//	printf("addt s0.id,0,%d\xff\xff\xff",WAVE_W);
	 //清除曲线控件s0的0通道数据,cle指令不支持跨页面
	 printf("cle s0.id,0\xff\xff\xff");
	 Delay_ms(10);
	 printf("addt s0.id,0,%d\xff\xff\xff",WAVE_W);
	 //等待适量时间
   Delay_ms(100);
	 for(i=0;i<WAVE_W;i++){
		h=0;
		x = (float)i* 2 * 3.1415926f / WAVE_W ;
		for(j=0;j<5;j++){
			h+=U[j]*arm_sin_f32((j+1)*x+Phase[j]);
		}	
	h=h*128/2.0f+128;
		if(h>256)h=255;
		else if(h<0)h=0;

		printf("%c",(int)h);//tjc会对h%256
	}
 //确保透传结束，以免影响下一条指令
	printf("\x01\xff\xff\xff");
}


void HMIDraw(void)
{
	//U[i]   Phase[i]  波形信息
		selected_USART=usart2_u;
		u8 i;
		for(i=0;i<SHOW_NUM;i++)
		{
			printf("t%d.txt=\"%.2f\"\xff\xff\xff",TXT_ID[i],U[i]);
			printf("t%d.txt=\"%.2f\"\xff\xff\xff",TXT_ID[i+5],Phase[i]);
		}
		printf("t%d.txt=\"%.2f\"\xff\xff\xff",10,freq_basic);
		printf("t%d.txt=\"%.2f%%\"\xff\xff\xff", 11, THD * 100);
		
}

//void HMICLS(void)
//{
//	selected_USART=usart2_u;
////	uint16_t posX, posY, width, height, color;
////	posX=AMPLITUDE_X;posY=AMPLITUDE_Y;width=AMPLITUDE_W;height=AMPLITUDE_H;color=0;
////  printf("fill %d,%d,%d,%d,%d\xff\xff\xff", posX, posY, width, height, color);
////	posX=PHASE_X, posY=PHASE_Y, width=PHASE_W, height=PHASE_H, color=0;
////  printf("fill %d,%d,%d,%d,%d\xff\xff\xff", posX, posY, width, height, color);

////	uint8_t i;
////	for(i=0;i<10;i++){
////	printf("t%d.txt=\"...\"",TXT_ID[i]);
////	}
//	printf("rest\xff\xff\xff");
//}
