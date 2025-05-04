#include "usart.h"
//#include "BlueTooth_Usart.h"
#include "general_usart.h"
int fputc(int ch, FILE * file)
{
		if(selected_USART==usart2_u){
		USART_SendByte((uint8_t)ch);
		}/*else if(selected_USART==usart3_u){
			//todo
			USART3_SendByte(ch);
			return ch;
		}*/
		return ch;
}
void processPacket(USART_RX_Packet *packet) {
		u8 cnt_0=0,cnt_1=0,i;
		for(i=0;i<RX_DATA_LEN;i++){
			if(packet->data[i]==S_0){
				cnt_0++;
			}else if(packet->data[i]==S_1){
				cnt_1++;
			}
		}
		//??start_sign_0?start_sign_1??0???? 
		if(cnt_0>=3&&!(start_sign_0||start_sign_1)){
			start_sign_0=True;
		}else if(cnt_1>=3&&!(start_sign_0||start_sign_1)){
			start_sign_1=True;
		}
    packet->received = 0;
}
