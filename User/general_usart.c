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
