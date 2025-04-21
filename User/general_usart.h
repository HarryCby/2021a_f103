#ifndef GENERAL_USART__H
#define GENERAL_USART__H
#include <stdio.h>
typedef enum {
    usart2_u = 0,
    usart3_u = 1,
    // 其他标志位定义
}usart_user;
extern usart_user selected_USART;
#endif
