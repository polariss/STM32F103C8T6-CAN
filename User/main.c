/* Includes ------------------------------------------------------------------*/
#include "Head.h"

void LED_Display(uint8_t Ledstatus);
//◊¢ Õ≤‚ ‘


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
