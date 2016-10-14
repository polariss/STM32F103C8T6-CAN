/**
  ******************************************************************************
  * @file    CAN.c
  * @author  马龙飞
  * @version V1.0.0
  * @date    2016-08-06
  * @brief   STM32F103 CAN1接口驱动。
  *
  * ============================================================================
  *  Note: This file is intended for This project only.
  * ============================================================================
  *  Description:
  *      配置STM32F103的CAN1，使能CAN1接收，使能CAN1中断。编写发送、接收功能
  *  函数。
  *      可通过取消注释宏定义LOOPBACK_TEST、等来选择测试程序。
  *      CAN1 RX pin——PB8	/	CAN1 TX pin——PB9
  *  Target     :	STM32F103
  *  Tool chain :	Keil MDK
  ******************************************************************************
  * @attention
  *<--->
  ******************************************************************************
  */ 

#ifndef __CAN_H
#define __CAN_H

/* Includes ------------------------------------------------------------------*/
#include "Head.h"

/****************下面两个宏定义只能定义一个*********************/
//指定设备在CAN总线上的标准ID(0-0x7ff,11bit)
#define	DEVICE_STAND_ID		((uint32_t)0x00000002)
//指定设备在CAN总线上的拓展ID(0-0x1fffffff,29bit)
//#define	DEVICE_EXTEND_ID	((uint32_t)0x00000001)	
/****************上面两个宏定义只能定义一个*********************/

//选择使能测试程序，只能选其中一个
//或者全部注释
//#define LOOPBACK_TEST


#define DEBUG_Info	printf


#define CANx                       CAN1
#define CANx_BUS_CLOCK             RCC_APB1Periph_CAN1
//#define GPIO_Remapping_CAN         GPIO_Remap1_CAN1
#define GPIO_CAN                   GPIOA
#define GPIO_CANx_CLOCK            RCC_APB2Periph_GPIOA
#define GPIO_Pin_CAN_RX            GPIO_Pin_11
#define GPIO_Pin_CAN_TX            GPIO_Pin_12




typedef enum{
	FAILED = 0,
	PASSED = !FAILED
} TestStatus;


void CAN_Config(void);
void CAN_SendBytes(uint8_t *data, uint8_t len);

#ifdef	LOOPBACK_TEST
  void CAN_LoopbackTest(void);
#endif /*LOOPBACK_TEST*/

#endif	/*__CAN_H*/
