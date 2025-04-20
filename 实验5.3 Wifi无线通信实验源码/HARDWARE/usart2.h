#ifndef __USART2_H
#define __USART2_H

#include "sys.h" 


#define USART2_REC_LEN  			256  	//�����������ֽ��� 200


extern uint8_t  USART2_RX_BUF[USART2_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern uint8_t  USART2_RX_LEN;
extern uint8_t  USART2_TimerOut;

//����봮���жϽ��գ��벻Ҫע�����º궨��
void USART2_Init(void);
void USART2_putString(unsigned char *string, int len);
void USART2_putChar(unsigned char ch);

#endif


