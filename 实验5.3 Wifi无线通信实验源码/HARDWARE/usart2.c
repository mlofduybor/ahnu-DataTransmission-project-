#include "usart2.h"
#include "stdio.h"

//////////////////////////////串口初始化//////////////////
//串口中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
uint8_t USART2_RX_BUF[USART2_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
uint8_t USART2_RX_LEN = 0;
uint8_t USART2_TimerOut = 0;

//重定义fputc应用printf函数
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0) ;
    USART2->DR = (uint8_t) ch;      
	return ch;
}

/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/	  
void USART2_Init(void)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  
	//初始化USART2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

  //Usart2 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//开启串口空闲中断
  USART_Cmd(USART2, ENABLE);                    //使能串口2 
}

/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/	
void USART2_putChar(unsigned char ch)
{
	USART_SendData(USART2, ch); 
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/	
void USART2_putString(unsigned char *string, int len)
{
	int i;
	
	for(i=0; i< len; i++)
		USART2_putChar(string[i]);
}
void USART_Putchar(u8 ch)
{   
   	USART2->DR = (u16)ch;
	while (!(USART2->SR & USART_FLAG_TXE));
}
void Send_USART_String(u8 *pString)
{
	while(*pString != '\0')
	{
		USART_Putchar(*pString++);
	}
}
/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/	
void USART2_IRQHandler(void)                	//串口2中断服务程序
{
	unsigned char ch;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		ch = USART_ReceiveData(USART2);
		if(USART2_RX_LEN < USART2_REC_LEN )
			USART2_RX_BUF[USART2_RX_LEN++] = ch;
		//清除标记
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
	
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)		
	{
		(void)USART2->SR;
    (void)USART2->DR;
		USART2_TimerOut = 1;
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
	}

}


