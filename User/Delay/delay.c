#include "delay.h"

void delay_us(uint32_t us)
{
	volatile unsigned int num,t;
	for(num=0;num<us;num++)
	{
		t=11;
		while(t!=0) t--;
	}
}


void delay_ms(uint16_t ms)
{
	volatile unsigned int num;
	for(num=0;num<ms;num++)
	{
		delay_us(1000);
	}

}