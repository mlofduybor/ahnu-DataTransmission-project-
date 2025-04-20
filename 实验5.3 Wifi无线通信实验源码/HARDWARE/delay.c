#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 
//ʹ��SysTick����ͨ����ģʽ���ӳٽ��й����ʺ�STM32F10xϵ�У�
static uint8_t  fac_us=0;							//us��ʱ������			   
static uint16_t fac_ms=0;							//ms��ʱ������
		
/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/				   
//��ʼ���ӳٺ���
//��ʹ��OS��ʱ��,�˺������ʼ��OS��ʱ�ӽ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void Delay_Init()
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);	//ѡ���ⲿʱ��  HCLK/8
	fac_us=SystemCoreClock/8000000;				//Ϊϵͳʱ�ӵ�1/8  
	fac_ms=(uint16_t)fac_us*1000;					//��OS��,����ÿ��ms��Ҫ��systickʱ����   
}				    

//��ʱnusΪҪ��ʱ��us��.		    								   
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 					//ʱ�����	  		 
	SysTick->VAL=0x00;        					//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;      					 //��ռ�����	 
}

/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/	
//��ʱnms
//ע��nms�ķ�Χ
//SysTick->LOADΪ24λ�Ĵ���,����,�����ʱΪ:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK��λΪHz,nms��λΪms
//��72M������,nms<=1864 
void delay_ms(uint16_t nms)
{
	uint32_t temp;		   
	SysTick->LOAD=(uint32_t)nms*fac_ms;				//ʱ�����(SysTick->LOADΪ24bit)
	SysTick->VAL =0x00;							//��ռ�����
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//��ʼ����  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));		//�ȴ�ʱ�䵽��   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//�رռ�����
	SysTick->VAL =0X00;       					//��ռ�����	  	    
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

