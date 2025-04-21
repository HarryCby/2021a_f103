#ifndef BLUETOOTH_USART__H
#define BLUETOOTH_USART__H
#include "stdio.h"
#include "bool.h"
#include <arm_math.h>
#include "general_usart.h"
#define HEAD '$'       // '$'
#define TAIL '#'       // '#'
#define TX_DATA_LEN 10
#define RX_DATA_LEN 5
#pragma pack(push, 1)
typedef struct {
    uint8_t head;      // 0xAA)
	//all kinds of data ll be transmit in the way of uint32_t
	int32_t Freq;
	int32_t THD_P;
	int32_t data[TX_DATA_LEN];  //  1. HZ  2. THD 3-7:U (float into int) 8-12 Phase (float into int) all the data ll be multiple 1000 times
    uint8_t tail;      // 0x55)
} BluetoothPacket;
#pragma pack(pop) 
typedef struct {
    uint8_t data[RX_DATA_LEN];
    uint8_t received;
} USART_RX_Packet;

extern flag start_sign_0;
extern flag start_sign_1;
void BTUSART_Init(uint32_t bound);
void USART3_SendByte(uint8_t ch);
void SendPacket(uint8_t *buffer);
void Send_Debug(void);
#endif
