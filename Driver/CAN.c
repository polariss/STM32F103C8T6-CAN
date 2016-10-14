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

/* Includes ------------------------------------------------------------------*/
#include "Head.h"
#include "CAN.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/

/*select one from the preconfigured CAN baud rates 
  by uncommenting the desired define below: */
#define CAN_BAUDRATE  1000 /* 1MBps   */
//#define CAN_BAUDRATE  500  /* 500kBps */
//#define CAN_BAUDRATE  250  /* 250kBps */
//#define CAN_BAUDRATE  125  /* 125kBps */
//#define CAN_BAUDRATE  100  /* 100kBps */ 
//#define CAN_BAUDRATE  50   /* 50kBps  */ 
//#define CAN_BAUDRATE  20   /* 20kBps  */ 
//#define CAN_BAUDRATE  10   /* 10kBps  */ 

/* Private variables ---------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/**
  * @brief  Configures CAN1's NVIC.
  * @param  None
  * @retval None
  */
static void CAN_NVICConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/* Configure CAN1 **************************************************/  
	/* CAN1 Periph clocks enable */
	RCC_APB1PeriphClockCmd(CANx_BUS_CLOCK, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

#ifndef	LOOPBACK_TEST
/**
  * @brief  Configures CAN1.
  * @param  None
  * @retval None
  */
void CAN_Config(void)
{
	CAN_InitTypeDef			CAN_InitStructure;
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	GPIO_InitTypeDef		GPIO_InitStructure;
	
	CAN_NVICConfig();
	
	/* Configure CANx I/Os **********************************************/
	/* GPIOB and AFIO clocks enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | GPIO_CANx_CLOCK, ENABLE);

	/* Configure CANx RX pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIO_CAN, &GPIO_InitStructure);

	/* Configure CANx TX pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_TX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIO_CAN, &GPIO_InitStructure);

	/* Remap CANx GPIOs */
//	GPIO_PinRemapConfig(GPIO_Remapping_CAN, ENABLE);

	/* Configure CANx **************************************************/  
	/* CANx Periph clocks enable */
	//RCC_APB1PeriphClockCmd(CANx_BUS_CLOCK, ENABLE);  //CAN_NVICConfig已调

	/* CANx register init */
	CAN_DeInit(CANx);
	/* Struct init*/
	CAN_StructInit(&CAN_InitStructure);

	/* CANx cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = DISABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;//ENABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	
	/* CAN Baudrate = 1MBps*/
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;  
	CAN_InitStructure.CAN_BS1 = CAN_BS1_3tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_5tq;
 
#if CAN_BAUDRATE == 1000  /* 1MBps */
	CAN_InitStructure.CAN_Prescaler =4;
#elif CAN_BAUDRATE == 500 /* 500KBps */
	CAN_InitStructure.CAN_Prescaler =8;
#elif CAN_BAUDRATE == 250 /* 250KBps */
	CAN_InitStructure.CAN_Prescaler =16;
#elif CAN_BAUDRATE == 125 /* 125KBps */
	CAN_InitStructure.CAN_Prescaler =32;
#elif  CAN_BAUDRATE == 100 /* 100KBps */
	CAN_InitStructure.CAN_Prescaler =40;
#elif  CAN_BAUDRATE == 50 /* 50KBps */
	CAN_InitStructure.CAN_Prescaler =80;
#elif  CAN_BAUDRATE == 20 /* 20KBps */
	CAN_InitStructure.CAN_Prescaler =200;
#elif  CAN_BAUDRATE == 10 /* 10KBps */
	CAN_InitStructure.CAN_Prescaler =400;
#else
	#error "Please select first the CAN Baudrate in Private defines in main.c "
#endif  /* CAN_BAUDRATE == 1000 */
	
	/* Initializes the CANx */
	CAN_Init(CANx, &CAN_InitStructure);
	
//————————>>>>以下两种模式只可取消注释其中一种
/******************************过滤器-标识符列表模式***********************************/
	/* CANx filter init */
	//指定过滤器组编号(0-13)
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;//配置过滤器为：屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;//设置过滤器组位宽为32
	//配置过滤器组标识符ID
	CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t)((DEVICE_STAND_ID<<5) & 0xFFFF);//0x002;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	//配置过滤器组屏蔽位ID，为1的位必须满足：过滤器组标识符ID 与 接收到的数据的标识符ID完全相同方可通过过滤器，存入FIFO
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFF80;//可通过的ID有0x0000/0x0001/0x0002/0x0003
	//CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0006;//bit2:IDE bit1:RTR 这两个必须与标识符ID一样,这样的话只能接收标准数据帧或者远程帧
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0004;//bit2:IDE bit1:RTR 第一个必须与标识符ID一样,这样的话同时能接收标准数据帧和远程帧
	//指定FIFO，通过过滤器的帧将被存入该FIFO中。
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	//使能或禁止过滤器组
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

/******************************过滤器-标识符列表模式***********************************/

/******************************过滤器-标识符列表模式***********************************/
//	/* CANx filter init */
//	//指定过滤器组编号(0-13)
//	CAN_FilterInitStructure.CAN_FilterNumber = 0;
//	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;//配置过滤器为：列表模式
//	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;//设置过滤器组位宽为32
//	//配置过滤器组列表ID
//	CAN_FilterInitStructure.CAN_FilterIdHigh = (uint16_t)((DEVICE_STAND_ID<<5) & 0xFFFF);//0x002;
//	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
//	//————>>>下面的三行配置只能接收ID为0x007的数据帧
//	////配置过滤器组列表ID：只要与两个ID任意一个一样，即可存入FIFO
//	//CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t)(((DEVICE_STAND_ID+5)<<5) & 0xFFFF);//0x007;
//	//CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
//	//————>>>下面的三行配置只能接收ID为0x002的远程帧，与CAN_FilterIdHigh搭配可接收一个ID的数据帧与远程帧
//	//CAN_FilterInitStructure.CAN_FilterMaskIdHigh = (uint16_t)((DEVICE_STAND_ID<<5) & 0xFFFF);//0x002;
//	//CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0002;//bit1:RTR = 1

//	//指定FIFO，通过过滤器的帧将被存入该FIFO中。
//	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
//	//使能或禁止过滤器组
//	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
//	CAN_FilterInit(&CAN_FilterInitStructure);
/******************************过滤器-标识符列表模式***********************************/

	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);	//使能FIFO0消息挂号中断
}

/**
  * @brief  Configures CAN1.
  * @param  None
  * @retval None
  */
void CAN_SendBytes(uint8_t *data, uint8_t len)
{
	CanTxMsg	TxMessage;
	uint8_t TransmitMailbox = 0;
	uint8_t i = 0;
	
	/* Transmit */
#ifdef DEVICE_STAND_ID
	TxMessage.StdId = DEVICE_STAND_ID;
	TxMessage.RTR = CAN_RTR_DATA;//数据帧
	TxMessage.IDE = CAN_ID_STD;//设定消息标识符的类型：标准帧
#elif defined(DEVICE_EXTEND_ID)
	TxMessage.StdId = DEVICE_EXTEND_ID;
	TxMessage.RTR = CAN_RTR_DATA;//数据帧
	TxMessage.IDE = CAN_ID_EXT;//设定消息标识符的类型：扩展帧
#endif

	while(len >> 3)
	{
		memcpy(TxMessage.Data, data+i, 8);
		TxMessage.DLC = 8;  
		TransmitMailbox = CAN_Transmit(CANx, &TxMessage);
		while(CAN_TransmitStatus(CANx, TransmitMailbox) != CANTXOK);
		
		len -= 8;
		i += 8;
	}
	if(len)
	{
		memcpy(TxMessage.Data, data+i, len);
		TxMessage.DLC = len;
		TransmitMailbox = CAN_Transmit(CANx, &TxMessage);
		while(CAN_TransmitStatus(CANx, TransmitMailbox) != CANTXOK);
	}
}

/**
  * @brief  Initializes a Rx Message.
  * @param  CanRxMsg *RxMessage
  * @retval None
  */
void Init_RxMes(CanRxMsg *rxMsg)
{
	rxMsg->StdId = 0x00;//用来设定标准标识符(0-0x7ff,11bit),如果是扩展帧，可以不用管它
	rxMsg->ExtId = 0x00;//用来设定扩展标识符(0-0x1fffffff,29bit),如果是标准帧，可以不用管它
	rxMsg->IDE = CAN_ID_STD;//设定消息标识符的类型：标准帧/扩展帧
	rxMsg->DLC = 0;//传输消息的帧长度(0-8)
	rxMsg->FMI = 0;//该变量用来记录接收到的数据所 通过的过滤器组的序号(0-0xff)
	memset( rxMsg->Data,0,8);
}

/**
  * @brief  This function handles CAN1 Handler.
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	Init_RxMes(&RxMessage);
	CAN_Receive(CANx, CAN_FIFO0, &RxMessage);
	
	printf("ID:%x, RTR:%d, DLC=%x, FMI=%x, Data[0]=%x \r\n",
			RxMessage.StdId,
			RxMessage.RTR,
			RxMessage.DLC,
			RxMessage.FMI,
			RxMessage.Data[0]);
}



#else	/*LOOPBACK_TEST*/

/**
  * @brief  Configures the CAN, transmit and receive by polling
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
static TestStatus CAN_Polling(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CanTxMsg TxMessage;
	CanRxMsg RxMessage;
	uint32_t i = 0;
	uint8_t TransmitMailbox = 0;

	/* CAN register init */
	CAN_DeInit(CANx);

	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;//CAN报文只被发送1次，不管发送的结果如何(成功、出错或仲裁丢失)
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;

	/* Baudrate = 125kbps*/
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
	CAN_InitStructure.CAN_Prescaler=48;
	CAN_Init(CANx, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;

	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;  
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=0;


	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* transmit */
	TxMessage.StdId=0x11;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.DLC=2;
	TxMessage.Data[0]=0xCA;
	TxMessage.Data[1]=0xFE;

	TransmitMailbox=CAN_Transmit(CANx, &TxMessage);
	i = 0;
	while((CAN_TransmitStatus(CANx, TransmitMailbox) != CANTXOK) && (i != 0xFFFF))
	{
		i++;
	}

	i = 0;
	while((CAN_MessagePending(CANx, CAN_FIFO0) < 1) && (i != 0xFFFF))
	{
		i++;
	}

	/* receive */
	RxMessage.StdId=0x00;
	RxMessage.IDE=CAN_ID_STD;
	RxMessage.DLC=0;
	RxMessage.Data[0]=0x00;
	RxMessage.Data[1]=0x00;
	CAN_Receive(CANx, CAN_FIFO0, &RxMessage);

	if (RxMessage.StdId!=0x11)
	{
		return FAILED;  
	}

	if (RxMessage.IDE!=CAN_ID_STD)
	{
		return FAILED;
	}

	if (RxMessage.DLC!=2)
	{
		return FAILED;  
	}

	if ((RxMessage.Data[0]<<8|RxMessage.Data[1])!=0xCAFE)
	{
		return FAILED;
	}

	return PASSED; /* Test Passed */
}


__IO uint32_t ret = 0; /* for return of the interrupt handling */
/**
  * @brief  Configures the CAN, transmit and receive using interrupt.
  * @param  None
  * @retval PASSED if the reception is well done, FAILED in other case
  */
static TestStatus CAN_Interrupt(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	CanTxMsg TxMessage;
	uint32_t i = 0;

	/* CAN register init */
	CAN_DeInit(CANx);


	CAN_StructInit(&CAN_InitStructure);

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_LoopBack;
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;

	/* Baudrate = 500 Kbps */
	CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;
	CAN_InitStructure.CAN_Prescaler=12;
	CAN_Init(CANx, &CAN_InitStructure);

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=1;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	/* CAN FIFO0 message pending interrupt enable */ 
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);

	/* transmit 1 message */
	TxMessage.StdId=0;
	TxMessage.ExtId=0x1234;
	TxMessage.IDE=CAN_ID_EXT;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=2;
	TxMessage.Data[0]=0xDE;
	TxMessage.Data[1]=0xCA;
	CAN_Transmit(CANx, &TxMessage);

	/* initialize the value that will be returned */
	ret = 0xFF;

	/* receive message with interrupt handling */
	i=0;
	while((ret == 0xFF) && (i < 0xFFF))
	{
		i++;
	}

	if (i == 0xFFF)
	{
		ret=0;  
	}

	/* disable interrupt handling */
	CAN_ITConfig(CANx, CAN_IT_FMP0, DISABLE);

	return (TestStatus)ret;
}

/**
  * @brief  This function handles CANx Handler.
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;

	RxMessage.StdId=0x00;
	RxMessage.ExtId=0x00;
	RxMessage.IDE=0;
	RxMessage.DLC=0;
	RxMessage.FMI=0;
	RxMessage.Data[0]=0x00;
	RxMessage.Data[1]=0x00;

	CAN_Receive(CANx, CAN_FIFO0, &RxMessage);

	if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
	 && (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))
	{
		ret = 1; 
	}
	else
	{
		ret = 0; 
	}
}

void CAN_LoopbackTest(void)
{
	volatile TestStatus TestRx;
	
	CAN_NVICConfig();
	
	/* CAN transmit at 125Kb/s and receive by polling in loopback mode */
	TestRx = CAN_Polling();
	if (TestRx == FAILED)
	{
		DEBUG_Info("Poll test failed!\n");
	}
	else
	{
		DEBUG_Info("Poll test success!\n");
	}
	
	/* CAN transmit at 500Kb/s and receive by interrupt in loopback mode */
	TestRx = CAN_Interrupt();
	if (TestRx == FAILED)
	{
		DEBUG_Info("Interrupt test failed!\n");
	}
	else
	{
		DEBUG_Info("Interrupt test success!\n");
	}
}
#endif	/*LOOPBACK_TEST*/

