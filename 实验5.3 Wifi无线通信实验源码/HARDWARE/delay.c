#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 
//使用SysTick的普通计数模式对延迟进行管理（适合STM32F10x系列）
static uint8_t  fac_us=0;							//us延时倍乘数			   
static uint16_t fac_ms=0;							//ms延时倍乘数
		
/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/				   
//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为HCLK时钟的1/8
//SYSCLK:系统时钟
void Delay_Init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//选择外部时钟  HCLK/8
	fac_us=SystemCoreClock/8000000;				//为系统时钟的1/8  
	fac_ms=(uint16_t)fac_us*1000;					//非OS下,代表每个ms需要的systick时钟数   
}				    

//延时nus为要延时的us数.		    								   
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//时间加载	  		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 //清空计数器	 
}

/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/	
//延时nms
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864 
void delay_ms(uint16_t nms)
{
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;				//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;							//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       					//清空计数器	  	    
} 
/* SystemFrequency / 1000    1ms????
 * SystemFrequency / 100000     10us????
 * SystemFrequency / 1000000 1us????
 */

//#define SYSTICKPERIOD                    0.000001
//#define SYSTICKFREQUENCY            (1/SYSTICKPERIOD)

///**
//  * @brief  ???2????,,????1ms
//  * @param  ?
//  * @retval ?
//  */
//void TIM2_Init(void)
//{
//    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//    /*AHB = 72MHz,RCC_CFGR?PPRE1 = 2,??APB1 = 36MHz,TIM2CLK = APB1*2 = 72MHz */
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//    
//    /* Time base configuration */         
//    TIM_TimeBaseStructure.TIM_Period = 999;
//    TIM_TimeBaseStructure.TIM_Prescaler = SystemCoreClock/SYSTICKFREQUENCY -1;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
//    
//    TIM_ARRPreloadConfig(TIM2, ENABLE);
//    
//    /* ?????????????????????? */
//    TIM_UpdateRequestConfig(TIM2,TIM_UpdateSource_Global); 
//    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//}

///**
//  * @brief   ms????,1ms?????
//  * @param  
//  *        @arg nTime: Delay_ms( 10 ) ??????? 10 * 1ms = 10ms
//  * @retval  ?
//  */
//void Delay_ms(__IO uint32_t nTime)
//{     
//    /* ????????????? */  
//    TIM2->CNT   = 0;  
//    TIM_Cmd(TIM2, ENABLE);     

//    for( ; nTime > 0 ; nTime--)
//    {
//     /* ??????????? */
//     while(TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) != SET);
//     TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//    }

//    TIM_Cmd(TIM2, DISABLE);
//}

