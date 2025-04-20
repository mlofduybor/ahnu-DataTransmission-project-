#include "usart2.h"
#include "stdio.h"

//////////////////////////////���ڳ�ʼ��//////////////////
//�����жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
uint8_t USART2_RX_BUF[USART2_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
uint8_t USART2_RX_LEN = 0;
uint8_t USART2_TimerOut = 0;

//�ض���fputcӦ��printf����
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
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
  
	//��ʼ��USART2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

  //Usart2 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = 115200;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART2, &USART_InitStructure); //��ʼ������2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
  USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2 
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
void USART2_IRQHandler(void)                	//����2�жϷ������
{
	unsigned char ch;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		ch = USART_ReceiveData(USART2);
		if(USART2_RX_LEN < USART2_REC_LEN )
			USART2_RX_BUF[USART2_RX_LEN++] = ch;
		//������
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


