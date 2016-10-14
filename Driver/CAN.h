/**
  ******************************************************************************
  * @file    CAN.c
  * @author  ������
  * @version V1.0.0
  * @date    2016-08-06
  * @brief   STM32F103 CAN1�ӿ�������
  *
  * ============================================================================
  *  Note: This file is intended for This project only.
  * ============================================================================
  *  Description:
  *      ����STM32F103��CAN1��ʹ��CAN1���գ�ʹ��CAN1�жϡ���д���͡����չ���
  *  ������
  *      ��ͨ��ȡ��ע�ͺ궨��LOOPBACK_TEST������ѡ����Գ���
  *      CAN1 RX pin����PB8	/	CAN1 TX pin����PB9
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

/****************���������궨��ֻ�ܶ���һ��*********************/
//ָ���豸��CAN�����ϵı�׼ID(0-0x7ff,11bit)
#define	DEVICE_STAND_ID		((uint32_t)0x00000002)
//ָ���豸��CAN�����ϵ���չID(0-0x1fffffff,29bit)
//#define	DEVICE_EXTEND_ID	((uint32_t)0x00000001)	
/****************���������궨��ֻ�ܶ���һ��*********************/

//ѡ��ʹ�ܲ��Գ���ֻ��ѡ����һ��
//����ȫ��ע��
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
