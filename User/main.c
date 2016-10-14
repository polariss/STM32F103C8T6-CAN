/* Includes ------------------------------------------------------------------*/
#include "Head.h"

void LED_Display(uint8_t Ledstatus);

int main(void)
{
	uint8_t temp = 0x30;
 	SystemInit();//Դ��system_stm32f10x.c�ļ�,ֻ��Ҫ���ô˺���,������RCC������.�����뿴2_RCC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	USART1_Init();
	LED_Init();
	
#ifdef	LOOPBACK_TEST
	printf("CAN test start!\n");
	CAN_LoopbackTest();
	while(1);
#else
	CAN_Config();
#endif /*LOOPBACK_TEST*/
	
	
	printf("CAN demo start!\n");
	while(1)
	{
//		LED_RED_TOGGLE;
//		LED_GREEN_TOGGLE;
//		LED_BLUE_TOGGLE;
		CAN_SendBytes( &temp, 1);
		temp++;
		delay_ms(1000);
	}
}

/**
  * @brief  Turn ON/OFF the dedicate led
  * @param  Ledstatus: Led number from 0 to 3.
  * @retval None
  */
void LED_Display(uint8_t Ledstatus)
{
  switch(Ledstatus)
  {
    case '1': 
      LED_RED_TOGGLE;
      break;
	  
    case '2':  
      LED_GREEN_TOGGLE;
      break;
    case '3':  
	
      LED_BLUE_TOGGLE;
      break;
    default:
      break;
  }
}
