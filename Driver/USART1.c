/**
  ******************************************************************************
  * @file    USART1.c
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   USART1�����ļ���
  *
  * ============================================================================
  *  Note: This file is intended for This project only.
  * ============================================================================
  *  Description:
  *      ����STM32F405��USART1��ʹ��USART1���գ�ʹ��USART1�жϡ���д���͡����չ���
  *  ����
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

unsigned char RxBuffer[USART1_RX_BUFFER_SIZE]; //���ջ�����
unsigned char RxIndex = 0; //���ջ�����λ��ָ��


/*******************************************************************************
*�� �� ����USART1_Init
*��    �룺void
*������������ʼ��USART1���ⲿ�ܽţ�ʱ�ӣ����á��жϵȲ���
*�� �� ֵ��void
*ע    �⣺PA9  -  TX1 (USART1), Push-Pull,  Digital
*          PA10 -  RX1 (USART1), Open-Drain, Digital
*******************************************************************************/
void USART1_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	USART_InitTypeDef	USART_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStructure;

	/* ʹ�� USART1 ģ���ʱ��  ʹ�� USART1��Ӧ�����Ŷ˿ڵ�ʱ�� */
	/* config the extiline clock and AFIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA,ENABLE);

	/* ���� USART1 �ķ������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	/* ���� USART1 �Ľ������� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	//��©����
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/******************************************************** 
		USART1������:
		1.������Ϊָ��          BAUDRATE_USART1;
		2.8λ����               USART_WordLength_8b;
		3.һ��ֹͣλ            USART_StopBits_1;
		4.����żЧ��            USART_Parity_No ;
		5.��ʹ��Ӳ��������      USART_HardwareFlowControl_None;
		6.ʹ�ܷ��ͺͽ��չ���    USART_Mode_Rx | USART_Mode_Tx;
	*********************************************************/
	USART_InitStructure.USART_BaudRate = BAUDRATE_USART1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//Ӧ�����õ�USART1
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);        
	USART_ClearFlag(USART1,USART_FLAG_TC);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	//ʹ�ܽ����ж�
	//����USART1
	USART_Cmd(USART1, ENABLE);

	//�����ж����ȼ�
	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
*�� �� ����USART1_PutChar
*��    �룺(unsigned char)DataToSend
*����������ͨ��USART1����һ��byte
*�� �� ֵ��void
*ע    �⣺
*******************************************************************************/
void USART1_PutChar(unsigned char DataToSend)
{
	USART_SendData(USART1, DataToSend);
	//�ȴ��������.
  	while (!(USART1->SR & USART_FLAG_TXE));
}
/*******************************************************************************
*�� �� ����USART1_PutStringAuto
*��    �룺(unsigned char *)Str
*����������ͨ��USART1����һ�ַ���
*�� �� ֵ��void
*ע    �⣺�����ú������͵��ַ��������ԡ�0����\0����β
*******************************************************************************/
void USART1_PutStringAuto(unsigned char *Str)
{
	while (*Str != 0) //�ж��Ƿ�ﵽ�ַ���������
	{
		USART1_PutChar(*Str++);
	}
}
/*******************************************************************************
*�� �� ����USART1_PutStringManual
*��    �룺(unsigned char *)Str
*          (unsigned char  )Length		�ֶ��ƶ��������ַ�������
*����������ͨ��USART1����һ�ַ���
*�� �� ֵ��void
*ע    �⣺
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
*�� �� ����USART1_IRQHandler
*��    �룺void
*����������USART1���жϴ�����
*�� �� ֵ��void
*ע    �⣺
*******************************************************************************/
void USART1_IRQHandler(void)
{
	unsigned char temp = 0;
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		temp = USART_ReceiveData(USART1);
		
		//�����ǽ����жϴ�����
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
						Data_ReveivedFlag = 1;//���������ݽ������
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




