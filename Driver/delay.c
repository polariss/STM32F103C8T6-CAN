#include "stm32f10x.h"
#include "delay.h"

void delay_us(u32 n)
{
	u32 i,j;
	for(i=0;i<n;i++)
	{
		for(j=0;j<10;j++);
	}
}


void delay_ms(u32 n)
{
	while(n--)
	{
		delay_us(1000);
	}
}
