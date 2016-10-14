/**
  ******************************************************************************
  * @file		LED.c
  * @author		马龙飞
  * @version	V1.0.1
  * @date		2014-09-26
  * @brief		driver of the LED
  * @target		STM32F103C8T6
  * @toolchain	MDK-ARM V4.72.0.0
  * ============================================================================
  * @Note		This driver is intended for this project only.
  * ============================================================================
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT : 2014 深圳市乐之航科技有限公司</center></h2>
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "LED.h"


/** @defgroup LED_Private_macro
 *  @{
 */

/**
  * @} //group of Buzzer_Private_macro
  */





/** @defgroup LED_File_Functions
 *  @{
 */
 
/**
 *******************************************************************************
 *  @brief   Initialization the GPIO that control the buzzer of the board
 *  @param   None
 *  @retval  None
 ******************************************************************************/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* config the GPIO clock */
	RCC_APB2PeriphClockCmd(LED_RED_GPIO_CLK | LED_GREEN_GPIO_CLK | LED_BLUE_GPIO_CLK,ENABLE);
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//set red led GPIO
	GPIO_InitStructure.GPIO_Pin = LED_RED_PIN;
	GPIO_Init(LED_RED_GPIO_PORT, &GPIO_InitStructure);
	
	//set green led GPIO
	GPIO_InitStructure.GPIO_Pin = LED_GREEN_PIN;
	GPIO_Init(LED_GREEN_GPIO_PORT, &GPIO_InitStructure);
	
	//set blue led GPIO
	GPIO_InitStructure.GPIO_Pin = LED_BLUE_PIN;
	GPIO_Init(LED_BLUE_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(LED_RED_GPIO_PORT, LED_RED_PIN);
	GPIO_SetBits(LED_GREEN_GPIO_PORT, LED_GREEN_PIN);
	GPIO_SetBits(LED_BLUE_GPIO_PORT, LED_BLUE_PIN);
	
}
 
 
/**
  * @} //group of LED_File_Functions
  */

/************ (C) COPYRIGHT 2014 深圳市乐之航科技有限公司 *****END OF FILE*****/
