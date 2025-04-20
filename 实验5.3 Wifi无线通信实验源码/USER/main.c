/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "oled.h"
#include "i2c.h"
#include "delay.h"
#include "usart2.h"
#include "esp8266.h"
#include "stdio.h"
#include "string.h"


#define type_sensor 4
/*
	sht3x      1
	Buzzer     2
	fan        3
	
	seg        4
	
*/


/*******************************************************************************
  * @brief  延时函数
  * @param  None
  * @retval None
*******************************************************************************/
void Delay(u16 x)
{
	u32 a,b;
	
	for(a = x;a > 0;a--)
	{
		for(b = 0x0001FFFF;b > 0;b--);
	}
}


/*sht3x*********************************************************************************************************************************************/

#if type_sensor == 1

#define CMD_FETCH_DATA			0xE000
#define CMD_MEAS_PERI_2_H		0x2236
#define POLYNOMIAL					0x131


#define  I2CADDR				0x44

#define I2CAddWrite			0x88//((I2CADDR << 1) )
#define I2CAddRead			0x89//((I2CADDR << 1) | 0x01)
/*******************************************************************************
* @fn      SHT3X_WriteCMD
* @brief   设置命令
* @return  none
******************************************************************************/
void SHT3X_WriteCMD(unsigned int cmd)
{
	IIC_Start();
  IIC_Send_Byte(I2CAddWrite);
  IIC_Wait_Ack();
  IIC_Send_Byte(cmd>>8);
  IIC_Wait_Ack();
  IIC_Send_Byte(cmd);
  IIC_Wait_Ack();
  IIC_Stop();
}

//设置工作模式
void SHT3X_SetPeriodicMeasurement(void)
{
  SHT3X_WriteCMD(CMD_MEAS_PERI_2_H);
}

//读取SHT30寄存器状态
void SHT3X_ReadState(unsigned char *temp)
{
    IIC_Start();
    IIC_Send_Byte(I2CAddWrite);
    IIC_Wait_Ack();
    IIC_Send_Byte(0xf3);
    IIC_Wait_Ack();
    IIC_Send_Byte(0x2d);
    IIC_Wait_Ack();
    
    IIC_Start();
    IIC_Send_Byte(I2CAddRead);
    IIC_Wait_Ack();

    temp[0] = IIC_Read_Byte(1);//高
    temp[1] = IIC_Read_Byte(1);//低
    temp[2] = IIC_Read_Byte(0);//校验
    
    IIC_Stop(); 
}

//读取SHT30结果
void SHX3X_ReadResults(unsigned int cmd,  unsigned char *p)
{
  IIC_Start();
  IIC_Send_Byte(I2CAddWrite);
  IIC_Wait_Ack();
  IIC_Send_Byte(cmd>>8);
  IIC_Wait_Ack();
  IIC_Send_Byte(cmd);
  IIC_Wait_Ack();
  
  IIC_Start();
  IIC_Send_Byte(I2CAddRead);
  IIC_Wait_Ack();   
  
  p[0] = IIC_Read_Byte(1);//温度高
  p[1] = IIC_Read_Byte(1);//温度低
  p[2] = IIC_Read_Byte(1);//校验
  p[3] = IIC_Read_Byte(1);//湿度高
  p[4] = IIC_Read_Byte(1);//湿度低
  p[5] = IIC_Read_Byte(0);//校验

  IIC_Stop();
}

//校验
unsigned char SHT3X_CalcCrc(unsigned char *data, unsigned char nbrOfBytes)
{
    unsigned char bit;        // bit mask
    unsigned char crc = 0xFF; // calculated checksum
    unsigned char byteCtr;    // byte counter

    // calculates 8-Bit checksum with given polynomial
    for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++) {
        crc ^= (data[byteCtr]);
        for(bit = 8; bit > 0; --bit) {
            if(crc & 0x80) {
                crc = (crc << 1) ^ POLYNOMIAL;
            }  else {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

//校验检测
unsigned char SHT3X_CheckCrc(unsigned char *pdata, unsigned char nbrOfBytes, unsigned char checksum)
{
    unsigned char crc;
    
		crc = SHT3X_CalcCrc(pdata, nbrOfBytes);// calculates 8-Bit checksum
    if(crc != checksum) 
    {   
        return 1;           
    }
    return 0;
}

//计算温度
unsigned int SHT3X_CalcTemperature(unsigned int rawValue)
{
    // calculate temperature 
    unsigned int temp;
    temp = (175 * (float)rawValue / 65535 - 45) ; // T = -45 + 175 * rawValue / (2^16-1)
    return temp;
}

//计算湿度
unsigned char SHT3X_CalcRH(unsigned int rawValue)
{
    // calculate relative humidity [%RH]
    unsigned char temp1 = (100 * (float)rawValue / 65535) ;  // RH = rawValue / (2^16-1) * 100

    return temp1;
}

/*******************************************************************************
* @fn      sht30_init
* @brief   初始化
* @return  none
******************************************************************************/
void Sensor_Init(void)
{
	IIC_Init();
	Delay(100);
	SHT3X_SetPeriodicMeasurement();//设置测量周期和模式
	Delay(100);
}
#endif


/***************************************************************************************************************************************/



/*******************************************************************************
  * @brief  LED初始化函数
  * @param  None
  * @retval None
*******************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能GPIOE时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化GPIOE.3~5
	
}

/*******************************************************************************
  * @brief  LED初始化函数
  * @param  None
  * @retval None
*******************************************************************************/
void LED_Toggle(void)
{
	uint8_t bit;
	
	bit = GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_1);
	if(bit)
		GPIO_ResetBits(GPIOB, GPIO_Pin_1 );
	else
		GPIO_SetBits(GPIOB, GPIO_Pin_1 );
}

/*buzzer************************************************************************************/
#if type_sensor == 2
/*******************************************************************************
  * @brief  Buzzer初始化函数
  * @param  None
  * @retval None
*******************************************************************************/
void Buzzer_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOE时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	//初始化GPIOE.3~5
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_6 );
	//Delay(500);
	//GPIO_SetBits(GPIOA, GPIO_Pin_6 );
	//Delay(500);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_6 );
}
#endif

/*********************************************************************************************/


/*fan************************************************************************************/
#if type_sensor == 3
/*******************************************************************************
  * @brief  fan初始化函数
  * @param  None
  * @retval None
*******************************************************************************/
void fan_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能GPIOE时钟
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//IO口速度为50MHz
	GPIO_Init(GPIOA, &GPIO_InitStructure); 	//初始化GPIOE.3~5
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_6 );
	//Delay(500);
	//GPIO_SetBits(GPIOA, GPIO_Pin_6 );
	//Delay(500);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_6 );
}
#endif

/*********************************************************************************************/


/*seg************************************************************************************/
//共阳端码表
#if type_sensor == 4
#define CLEAR 0xFFFF

#define HC595_RCC 	RCC_APB2Periph_GPIOA
#define HC595_PORT	GPIOA
#define SLCK_PIN 		GPIO_Pin_5
#define SCLK_PIN 		GPIO_Pin_6
#define DATA_PIN  	GPIO_Pin_7



static unsigned char segTable[] ={
/*#define DB0*/ 0xC0,
/*#define DB1*/ 0xF9,
/*#define DB2*/ 0xA4,
/*#define DB3*/ 0xB0,
/*#define DB4*/ 0x99,
/*#define DB5*/ 0X92,
/*#define DB6*/ 0x82,
/*#define DB7*/ 0xF8,
/*#define DB8*/ 0x80,
/*#define DB9*/ 0x90,
/*#define DBA*/ 0X88,
/*#define DBB*/ 0x83,
/*#define DBC*/ 0xC6,
/*#define DBD*/ 0xA1,
/*#define DBE*/ 0x86,	
/*#define DBF*/ 0X8E
};




/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/
void send2byte(unsigned short a)
{
	unsigned char i,j,temp,byte;
	
	for(j=0; j<2; j++)
	{
		byte = (a >>(j*8)) & 0xff;
		
		for(i=0;i<8;i++)
		{
			GPIO_ResetBits(HC595_PORT,SCLK_PIN);
			
			temp = byte & 0x80;
			if (temp == 0)
			{
				GPIO_ResetBits(HC595_PORT,DATA_PIN);
			}
			else
			{
				GPIO_SetBits(HC595_PORT,DATA_PIN);
			}
			byte = byte << 1;
			
			GPIO_SetBits(HC595_PORT,SCLK_PIN);
		}
	}
}



/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/
void out595(void)
{
	GPIO_ResetBits(HC595_PORT,SLCK_PIN);
	delay_ms(10);
	GPIO_SetBits(HC595_PORT,SLCK_PIN);
}

/*******************************************************************************
  * @brief  
  * @param  None
  * @retval None
*******************************************************************************/
void clear(void)
{
		send2byte(CLEAR);
		out595();
}

void Sensor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(HC595_PORT,SCLK_PIN);
	GPIO_SetBits(HC595_PORT,DATA_PIN);
	
	clear();
}



#endif

/*********************************************************************************************/


/*******************************************************************************
  * @brief  Main program.
  * @param  None
  * @retval None
*******************************************************************************/
int main(void)
{

	#if type_sensor == 1
	char packet[64]={0};
	uint8_t len=0,ret=0;
	uint8_t data[128]={0};
	unsigned char str[2],str1[2];
	unsigned int dat,i,j;
	unsigned char buffer[6] = {0};
	unsigned char p[3];
	unsigned char tem_status=0,hum_status=0;
	float TemValue = 0;
	float RhValue = 0;
	
	uint8_t ucTcpClientFlag = 1;		// 网络连接标记,0:SERVER 或1:CLIENT;
	uint8_t ucWifiType = 1;					// 修改参数0:SERVER 或1:CLIENT;然后编译
	
  unsigned char buf[16]={0xAA,0x0F,0x02,0x01,0x05,0x02,0x00,0xA0,0x01,0x04,0x00,0x00,0x00,0x00,0xC8};
	#endif
	
	
	
	#if type_sensor == 2
	uint8_t buzzer_status=0;
	char packet[64]={0};
	uint8_t len=0,ret=0;
	uint8_t data[128]={0};
  unsigned char buf[16]={0xAA,0x0C,0x02,0x01,0x05,0x02,0x00,0xA2,0x01,0x01,0x00,0xC8};
	#endif
	
	#if type_sensor == 3
	uint8_t buzzer_status=0;
	char packet[64]={0};
	uint8_t len=0,ret=0;
	uint8_t data[128]={0};
  unsigned char buf[16]={0xAA,0x0C,0x02,0x01,0x05,0x02,0x00,0xA1,0x01,0x01,0x00,0xC8};
	#endif
	
	#if type_sensor == 4
	uint8_t buzzer_status=0;
	char packet[64]={0};
	uint8_t len=0,ret=0;
	uint8_t data[128]={0};
  unsigned char buf[16]={0xAA,0x0C,0x02,0x01,0x05,0x02,0x00,0xA3,0x01,0x01,0x00,0xC8};
	#endif
	
	Delay_Init();
	LED_Init();						//LED设置
	

	
	#if type_sensor == 2
	Buzzer_Init();					//蜂鸣器
	#endif
	
	#if type_sensor == 3
	fan_Init();
	#endif
	
	USART2_Init();        //串口设置
	ESP8266_Init();				//wifi设置
	OLED_Init();					//初始化OLED  
	OLED_Clear(); 				//清屏
	#if type_sensor == 1
	Sensor_Init();
	#endif
	#if type_sensor == 4
	Sensor_Init();
	#endif
	

	OLED_ShowString(0,0,"Wifi Client!",16);
	
	#if type_sensor == 1
	OLED_ShowString(0,4,"Sht3x",16);
	ESP8266_StaTcpClient_UnvarnishTest();
	Delay(100);
	#endif
	
	
	#if type_sensor == 2
	OLED_ShowString(0,4,"Buzzer",16);
	
	ESP8266_StaTcpClient_UnvarnishTest();			//修改esp8266.h下SSI,"ESP8266Arm00"最后两位为网关编号xx;
	Delay(100);
	#endif
	
	#if type_sensor == 3
	OLED_ShowString(0,4,"Fan",16);
	
	ESP8266_StaTcpClient_UnvarnishTest();			//修改esp8266.h下SSI,"ESP8266Arm00"最后两位为网关编号xx;
	Delay(100);
	#endif
	
	#if type_sensor == 4
	OLED_ShowString(0, 4, "seg", 16);
	
	ESP8266_StaTcpClient_UnvarnishTest();
	Delay(100);
	#endif
	
	
	while(1)
	{
		#if type_sensor == 1
		SHX3X_ReadResults(CMD_FETCH_DATA, buffer);//获取温度湿度数据
    p[0] = buffer[0];
    p[1] = buffer[1];
		p[2] = buffer[2];
		tem_status = SHT3X_CheckCrc(p, 2, p[2]);
		if(!tem_status)
		{
			dat = (p[0] <<8) + p[1];
			TemValue = SHT3X_CalcTemperature(dat);
		}
    p[0] = buffer[3];
    p[1] = buffer[4];
		p[2] = buffer[5];
		hum_status = SHT3X_CheckCrc(p, 2, p[2]);
		if(!hum_status)
		{
			dat = (p[0] <<8) + p[1];
			RhValue = SHT3X_CalcRH(dat);
		}
	
		sprintf((char *)str1, "%.0f", TemValue);
		Send_USART_String(str1);
		//OLED_ShowString(12,4,str,16); 
	
		sprintf((char *)str, "%.0f", RhValue);
		Send_USART_String(str);	
		//OLED_ShowString(12,6,str,16);
		for(i=1;i<10;i++)
		  Delay(100);
		// 判断是否有数据过来
		
		buf[10] = (unsigned char  )str1[0];
		buf[11] = (unsigned char  )str1[1];
		buf[12] = (unsigned char  )str[0];
		buf[13] = (unsigned char  )str[1];
    buf[14] = (uint8_t)(buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]+buf[8]+buf[9]+buf[10]+buf[11]+buf[12]+buf[13]);
		
		//if (ret ++ >= 25)
		//{
			//ret = 0;
			len = Esp8266_Hex2Str(packet, buf, buf[1]);
			ESP8266_SendString(DISABLE, packet, len, Single_ID_0);
		//}
		#endif
		
		#if (type_sensor == 2 || type_sensor == 3 || type_sensor == 4)
		#if type_sensor == 2
		//协议数据更新,加入传感器及校验
    buf[10] = (unsigned char  )buzzer_status;
    buf[11] = (uint8_t)(buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]+buf[8]+buf[9]+buf[10]);

		// 定时上传节点信息
		//if(ret++ >= 25)
		//{
			ret = 0;
			len = Esp8266_Hex2Str(packet, buf, buf[1]);
			ESP8266_SendString(DISABLE, packet, len, Single_ID_0);
		//}
		#endif
		
		
		#if type_sensor == 3
		//协议数据更新,加入传感器及校验
    buf[10] = (unsigned char  )buzzer_status;
    buf[11] = (uint8_t)(buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]+buf[8]+buf[9]+buf[10]);

		// 定时上传节点信息
		//if(ret++ >= 25)
		//{
			ret = 0;
			len = Esp8266_Hex2Str(packet, buf, buf[1]);
			ESP8266_SendString(DISABLE, packet, len, Single_ID_0);
		//}
		#endif
		
		#if type_sensor == 4
		//协议数据更新,加入传感器及校验
    buf[10] = (unsigned char  )buzzer_status;
    buf[11] = (uint8_t)(buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6]+buf[7]+buf[8]+buf[9]+buf[10]);

		// 定时上传节点信息
		//if(ret++ >= 25)
		//{
			ret = 0;
			len = Esp8266_Hex2Str(packet, buf, buf[1]);
			ESP8266_SendString(DISABLE, packet, len, Single_ID_0);
		//}
		#endif
		
		// 判断是否有数据过来
		if( strEsp8266_Fram_Record.InfBit.FramFinishFlag )
		{
			//OLED_ShowString(0,4,"AAAAAAAAA",16);
			strEsp8266_Fram_Record.Data_RX_BUF [ strEsp8266_Fram_Record.InfBit.FramLength ] = '\0';
			// 判断是否连接或断开提示数据
			if(strstr ( strEsp8266_Fram_Record.Data_RX_BUF, "CLOSED" ) )
			{
				ucTcpClientFlag--;
			}
			else if(strstr ( strEsp8266_Fram_Record.Data_RX_BUF, "CONNECT" ) )
			{
				ucTcpClientFlag++;
			}
			

			else if(strstr ( strEsp8266_Fram_Record.Data_RX_BUF, "+IPD" ) )
			{
				//OLED_ShowString(0,4,"BBBBBBBBB",16);

				// 串口显示接收的消息
				USART2_putString((uint8_t*)strEsp8266_Fram_Record.Data_RX_BUF, 
													strEsp8266_Fram_Record.InfBit.FramLength);
				
				
				#if type_sensor == 2
				Esp8266_Str2Hex(data);
				//???????????
				if((data[2]==0x02)&&(data[4]==0x0f)&&(data[7]== 0xa2)) //a1 --> buzzer
				{ 
					buzzer_status = data[10];
					if(data[10]== 0x01)
						GPIO_SetBits(GPIOA, GPIO_Pin_6);
					else
						GPIO_ResetBits(GPIOA, GPIO_Pin_6 );
				}		
				#endif
				
				
				
				
				#if type_sensor == 3
				Esp8266_Str2Hex(data);
				//???????????
				if((data[2]==0x02)&&(data[4]==0x0f)&&(data[7]== 0xa1)) //a2 --> fan
				{ 
					buzzer_status = data[10];
					if(data[10]== 0x01)
						GPIO_SetBits(GPIOA, GPIO_Pin_6);
					else
						GPIO_ResetBits(GPIOA, GPIO_Pin_6 );
				}		
				#endif
				
				
				#if type_sensor == 4
				Esp8266_Str2Hex(data);
				//???????????
				if((data[2]==0x02)&&(data[4]==0x0f)&&(data[7]== 0xa3)) //a3 --> seg
				{ 
					buzzer_status = data[10];
					if(data[10]== 0x01){
						uint8_t i=60;
						send2byte((segTable[i&0xf] <<8) | segTable[(i>>4)&0x0f]);
						out595();
					}
					else{
						uint8_t i=0;
						send2byte((segTable[i&0xf] <<8) | segTable[(i>>4)&0x0f]);
						out595();
					}
				}		
				#endif
				
				
			}
			// 其他命令处理，不能直接清楚缓存长度和标记
			strEsp8266_Fram_Record.InfBit.FramLength = 0;
			strEsp8266_Fram_Record.InfBit.FramFinishFlag = 0;
		}
		#endif
		
		// LED闪烁
		LED_Toggle();
		Delay(50);
	}
}
#ifdef USE_FULL_ASSERT


void assert_failed(uint8_t* file, uint32_t line)
{
	while (1)
	{
	}
}
#endif


/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
