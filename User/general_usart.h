#ifndef GENERAL_USART__H
#define GENERAL_USART__H
#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include "bool.h"
#define HEAD '$'       // '$' 
#define TAIL '+'       // '+'
#define S_0 0x30
#define S_1 0x3F
//与Android的中的协议对应
#define RX_DATA_LEN 5
typedef enum {
    usart2_u = 0,
    usart3_u = 1,
    // 其他标志位定义
}usart_user;
typedef struct {
    uint8_t data[RX_DATA_LEN];
    uint8_t received;
} USART_RX_Packet;
extern usart_user selected_USART;
extern volatile flag start_sign_0;
extern volatile flag start_sign_1;
void processPacket(USART_RX_Packet *packet);
#endif
