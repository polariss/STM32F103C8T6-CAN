/**
  ******************************************************************************
  * @file		LED.h
  * @author		马龙飞
  * @version	V1.0.1
  * @date		2014-09-26
  * @brief		This head file for LED.c
  * @target		STM32F103C8T6
  * @toolchain	MDK-ARM V4.72.0.0
  * ============================================================================
  * @Note		This file is intended for this project only.
  * ============================================================================
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT : 2014 深圳市乐之航科技有限公司</center></h2>
  *
  ******************************************************************************
  */ 

#ifndef __LED_H
#define __LED_H

/* Public includes -----------------------------------------------------------*/
#include "stm32f10x.h"
#include "Head.h"

/** @defgroup LED_Public_macro
 *  @{
 */

/**
 * @brief define the GPIO used to control Buzzer
 */
#define LED_RED_PIN						GPIO_Pin_10
#define LED_RED_GPIO_PORT				GPIOB
#define LED_RED_GPIO_CLK				RCC_APB2Periph_GPIOB

#define LED_GREEN_PIN					GPIO_Pin_11
#define LED_GREEN_GPIO_PORT				GPIOB
#define LED_GREEN_GPIO_CLK				RCC_APB2Periph_GPIOB

#define LED_BLUE_PIN					GPIO_Pin_12
#define LED_BLUE_GPIO_PORT				GPIOB
#define LED_BLUE_GPIO_CLK				RCC_APB2Periph_GPIOB
/**
 * @brief Control LED
 */
#define LED_RED_OFF				GPIO_ResetBits(LED_RED_GPIO_PORT, LED_RED_PIN)
#define LED_RED_ON				GPIO_SetBits(LED_RED_GPIO_PORT, LED_RED_PIN)
#define LED_RED_TOGGLE			LED_RED_GPIO_PORT->ODR ^= LED_RED_PIN

#define LED_GREEN_OFF			GPIO_ResetBits(LED_GREEN_GPIO_PORT, LED_GREEN_PIN)
#define LED_GREEN_ON			GPIO_SetBits(LED_GREEN_GPIO_PORT, LED_GREEN_PIN)
#define LED_GREEN_TOGGLE		LED_GREEN_GPIO_PORT->ODR ^= LED_GREEN_PIN

#define LED_BLUE_OFF			GPIO_ResetBits(LED_BLUE_GPIO_PORT, LED_BLUE_PIN)
#define LED_BLUE_ON				GPIO_SetBits(LED_BLUE_GPIO_PORT, LED_BLUE_PIN)
#define LED_BLUE_TOGGLE			LED_BLUE_GPIO_PORT->ODR ^= LED_BLUE_PIN
/**
  * @} //group of LED_Public_macro
  */





/** @defgroup LED_Exported_Variables
 *  @{
 */

/**
  * @} //group of LED_Exported_Variables
  */





/** @defgroup LED_Exported_Types
 *  @{
 */

/**
  * @} //group of LED_Exported_Types
  */





/** @defgroup LED_Exported_Functions
  * @{
  */
void LED_Init(void);

/**
  * @}//group of LED_Exported_Functions
  */



#endif /*__LED_H*/

/************ (C) COPYRIGHT 2014 深圳市乐之航科技有限公司 *****END OF FILE****/
