#include "stm32f10x.h"                  // Device header
#include "BlueTooth_Usart.h"
#include "bool.h"
USART_RX_Packet rxPacket;
uint8_t dataIndex = 0;
uint8_t packetState = 0; // 0:等待头 1:接收数据 2:等待尾
flag start_sign_0=False;
flag start_sign_1=False;
//USART3 用以蓝牙通信 PB10：TX  PB11：RX  蓝牙设置为9600 波特率
void BTUSART_Init(u32 bound)
{
	    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    //USART3_TX   PB10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //USART3_RX          PB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		// USART2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级2
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;        // 子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);                           // 根据指定的参数初始化VIC寄存器
	
		USART_DeInit(USART1);//大概意思是解除此串口的其他配置
		USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = bound; // 一般设置为9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
		
		USART_ClearFlag(USART3, USART_FLAG_TC | USART_FLAG_RXNE);  // 清除 USART3 的标志//初始配置时清除接受置位
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE); // 开启中断  处理接受的数据
		USART_ITConfig(USART3, USART_IT_TC, DISABLE);
    USART_Cmd(USART3, ENABLE);                     // 使能串口
}

void USART3_SendByte(uint8_t ch)
{
	
	USART_SendData(USART3,ch);
	/*！！！！！！！！！！！！！！！！！！TC*/
  while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//TC 标志位是在 USART 的发送数据寄存器和移位寄存器都为空时置位的，这表示一个完整的数据帧（包括停止位）已经完全发送出去
	//while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET);

}

//todo 写传输函数 printf 使用前：selected_USART=usart3_u;
/*中断处理函数触发条件：
USART3_IRQHandler 是 USART3 的中断处理函数，它会在任何启用的中断标志位被设置时触发。
如果 USART_IT_RXNE 中断被启用，当接收到数据时，RXNE 标志位会被设置，从而触发中断处理函数。
即使 USART_IT_TC 被禁用，只要 RXNE 仍然有效，中断处理函数仍然会被调用。*/
void processPacket(USART_RX_Packet *packet) {
		u8 cnt_0=0,i;
		for(i=0;i<RX_DATA_LEN;i++){
			if(packet->data[i]==0x30){
				cnt_0++;
			}
		}
		if(cnt_0>=3){
			start_sign_0=True;
		}
    packet->received = 0;
}

void USART3_IRQHandler(void)
{
		
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) {
        // 读取接收到的数据
				uint8_t receivedByte;
        receivedByte = USART_ReceiveData(USART3);
        
        switch(packetState) {
            case 0: // 等待头
                if(receivedByte == HEAD) {
                    packetState = 1;
                    dataIndex = 0;
                }
                break;
                
            case 1: // 接收数据
                rxPacket.data[dataIndex++] = receivedByte;
                if(dataIndex >= RX_DATA_LEN) {
                    packetState = 2;
                }
                break;
                
            case 2: // 等待尾
                if(receivedByte == TAIL) {
                    rxPacket.received = 1;
                    // 数据包接收完成，可以处理
                    processPacket(&rxPacket);
                }
                packetState = 0; // 无论是否匹配，都重置状态
                break;
        }
        
        // 清除中断标志
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}


void Serial_SendArr(uint8_t * Arr, uint8_t Length)
{
	selected_USART=usart3_u;
	for(uint8_t i=0; i<Length; ++i)
	{
		USART3_SendByte(Arr[i]);
	}
}

void SendPacket(uint8_t *buffer)
{
	selected_USART=usart3_u;
				for (int i = 0; i < sizeof(BluetoothPacket); i++) {
				USART3_SendByte(buffer[i]);  // 逐字节发送
			}
}

void Send_Debug(void)
{
	selected_USART=usart3_u;
	USART3_SendByte(0xff);
	USART3_SendByte('h');	
	USART3_SendByte('i');	
	USART3_SendByte(0xFE);
	USART3_SendByte('\r');
	USART3_SendByte('\n');
}












