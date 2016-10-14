/**
  ******************************************************************************
  * @file    USART1.h
  * @author  
  * @version V1.0.0
  * @date    
  * @brief   USART1配置文件。
  *
  * ============================================================================
  *  Note: 
  * ============================================================================
  *  Description:
  *      配置STM32F103的USART1，使能USART1接收，使能USART1中断。编写发送、接收功
  *  能函数，其中已包含中断处理函数。
  ******************************************************************************
  *  USART1的配置:
  *    1.波特率为指定          BAUDRATE_USART1;
  *    2.8位数据               USART_WordLength_8b;
  *    3.一个停止位            USART_StopBits_1;
  *    4.无奇偶效验            USART_Parity_No ;
  *    5.不使用硬件流控制      USART_HardwareFlowControl_None;
  *    6.使能发送和接收功能    USART_Mode_Rx | USART_Mode_Tx;
  ******************************************************************************
  *  Target     :	STM32F103
  *  Tool chain :	Keil MDK
  ******************************************************************************
  * @attention
  *<--->
  ******************************************************************************
  */ 
#ifndef __USART1_H
#define __USART1_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>

/****** Define ****************************************************************/

#define	BAUDRATE_USART1				115200		//USART1串口通信波特率

#define USART1_RX_BUFFER_SIZE		15			//接收缓冲区大小

/****** Global Variable *******************************************************/

extern unsigned char RxBuffer[USART1_RX_BUFFER_SIZE]; //接收缓冲区
extern unsigned char RxIndex; //接收缓冲区位置指针
extern uint8_t Data_ReveivedFlag;

/****** Function declaration***************************************************/
/*******************************************************************************
*函 数 名：USART1_Init
*输    入：void
*功能描述：初始化USART1的外部管脚，时钟，配置、中断等参数
*返 回 值：void
*注    意：PA9  -  TX1 (USART1), Push-Pull,  Digital
*          PA10 -  RX1 (USART1), Open-Drain, Digital
*******************************************************************************/
extern void USART1_Init(void);
/*******************************************************************************
*函 数 名：USART1_PutChar
*输    入：(unsigned char)DataToSend
*功能描述：通过USART1发送一个byte
*返 回 值：void
*注    意：
*******************************************************************************/
extern void USART1_PutChar(unsigned char DataToSend);
/*******************************************************************************
*函 数 名：USART1_PutStringAuto
*输    入：(unsigned char *)Str
*功能描述：通过USART1发送一字符串
*返 回 值：void
*注    意：经过该函数发送的字符串必需以‘0’或‘\0’结尾
*******************************************************************************/
extern void USART1_PutStringAuto(unsigned char *Str);
/*******************************************************************************
*函 数 名：USART1_PutStringManual
*输    入：(unsigned char *)Str
*          (unsigned char  )Length		手动制定待发送字符串长度
*功能描述：通过USART1发送一字符串
*返 回 值：void
*注    意：
*******************************************************************************/
extern void USART1_PutStringManual(unsigned char *Str, unsigned char Length);





#endif /*__USART1_H*/

