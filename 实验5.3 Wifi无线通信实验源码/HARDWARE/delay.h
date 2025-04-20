#ifndef __DELAY_H
#define __DELAY_H 

#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//使用SysTick的普通计数模式对延迟进行管理（适合STM32F10x系列）
void Delay_Init(void);
void delay_ms(uint16_t nms);
void delay_us(uint32_t nus);
//void TIM2_Init(void);
//void Delay_ms(__IO uint32_t nTime);

#endif





























