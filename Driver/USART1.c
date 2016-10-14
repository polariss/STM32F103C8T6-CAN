/**
  ******************************************************************************
  * @file    USART1.c
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   USART1配置文件。
  *
  * ============================================================================
  *  Note: This file is intended for This project only.
  * ============================================================================
  *  Description:
  *      配置STM32F405的USART1，使能USART1接收，使能USART1中断。编写发送、接收功能
  *  函数
  *  Target     :	STM32F103
  *  Tool chain :	Keil MDK
  ******************************************************************************
  * @attention
  *<--->
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "USART1.h"


uint8_t Data_ReveivedFlag = 0;

unsigned char RxBuffer[USART1_RX_BUFFER_SIZE]; //接收缓冲区
unsigned char RxIndex = 0; //接收缓冲区位置指针


/*******************************************************************************
*函 数 名：USART1_Init
*输    入：void
*功能描述：初始化USART1的外部管脚，时钟，配置、中断等参数
*返 回 值：void
*注    意：PA9  -  TX1 (USART1), Push-Pull,  Digital
*          PA10 -  RX1 (USART1), Open-Drain, Digital
*******************************************************************************/
void USART1_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	/* 使能 USART1 模块的时钟  使能 USART1对应的引脚端口的时钟 */
	/* config the extiline clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,ENABLE);

	/* 配置 USART1 的发送引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* 配置 USART1 的接收引脚 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//开漏输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/******************************************************** 
		USART1的配置:
		1.波特率为指定          BAUDRATE_USART1;
		2.8位数据               USART_WordLength_8b;
		3.一个停止位            USART_StopBits_1;
		4.无奇偶效验            USART_Parity_No ;
		5.不使用硬件流控制      USART_HardwareFlowControl_None;
		6.使能发送和接收功能    USART_Mode_Rx | USART_Mode_Tx;
	*********************************************************/
	USART_InitStructure.USART_BaudRate = BAUDRATE_USART1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//应用配置到USART1
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);        
	USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//使能接收中断
	//启动USART1
	USART_Cmd(USART1, ENABLE);

	//配置中断优先级
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
*函 数 名：USART1_PutChar
*输    入：(unsigned char)DataToSend
*功能描述：通过USART1发送一个byte
*返 回 值：void
*注    意：
*******************************************************************************/
void USART1_PutChar(unsigned char DataToSend)
{
	USART_SendData(USART1, DataToSend);
	//等待发送完成.
  	while (!(USART1->SR & USART_FLAG_TXE));
}
/*******************************************************************************
*函 数 名：USART1_PutStringAuto
*输    入：(unsigned char *)Str
*功能描述：通过USART1发送一字符串
*返 回 值：void
*注    意：经过该函数发送的字符串必需以‘0’或‘\0’结尾
*******************************************************************************/
void USART1_PutStringAuto(unsigned char *Str)
{
	while (*Str != 0) //判断是否达到字符串结束符
	{
		USART1_PutChar(*Str++);
	}
}
/*******************************************************************************
*函 数 名：USART1_PutStringManual
*输    入：(unsigned char *)Str
*          (unsigned char  )Length		手动制定待发送字符串长度
*功能描述：通过USART1发送一字符串
*返 回 值：void
*注    意：
*******************************************************************************/
void USART1_PutStringManual(unsigned char *Str, unsigned char Length)
{
	unsigned char i;
	for( i = 0 ; i < Length ; i++ )
	{
		USART1_PutChar(*(Str+i));
	}
}

/*******************************************************************************
*函 数 名：USART1_IRQHandler
*输    入：void
*功能描述：USART1的中断处理函数
*返 回 值：void
*注    意：
*******************************************************************************/
void USART1_IRQHandler(void)
{
	unsigned char temp = 0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		temp = USART_ReceiveData(USART1);
		
		//以下是接收中断处理函数
		/*********************************************************************/
		if(Data_ReveivedFlag == 0)
		{
			switch(RxIndex)
			{
				case 0:
					if(temp == 'T')
						RxBuffer[RxIndex++] = temp;
					break;
				case 1:
					if(temp == 'x')
						RxBuffer[RxIndex++] = temp;
					else
						RxIndex = 0;
					break;
				case 2:
					if(temp == ':')
						RxBuffer[RxIndex++] = temp;
					else
						RxIndex = 0;
					break;
				case 3:
					RxBuffer[RxIndex++] = temp;
					break;
				case 4:
					if((temp == '\r') ||((temp == '\n')))
						Data_ReveivedFlag = 1;//完整的数据接收完毕
					else
						RxIndex = 0;
					break;
				default:break;
			}
		}
		/*********************************************************************/
		/* Clear the USART1 RX interrupt */
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}


int fputc(int ch, FILE *f)
{
	while (!(USART1->SR & USART_FLAG_TXE));
	
	USART_SendData(USART1, (uint8_t) ch); 

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

	return ch;
}




