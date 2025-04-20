#ifndef __USART2_H
#define __USART2_H

#include "sys.h" 


#define USART2_REC_LEN  			256  	//定义最大接收字节数 200


extern uint8_t  USART2_RX_BUF[USART2_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern uint8_t  USART2_RX_LEN;
extern uint8_t  USART2_TimerOut;

//如果想串口中断接收，请不要注释以下宏定义
void USART2_Init(void);
void USART2_putString(unsigned char *string, int len);
void USART2_putChar(unsigned char ch);

#endif


