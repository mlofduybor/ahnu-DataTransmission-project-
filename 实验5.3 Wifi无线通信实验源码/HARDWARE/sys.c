#include "sys.h"


/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/	
void NVIC_Configuration(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2
}
