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

void systick_delay_ms(uint32_t ms)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(72000);
	for(int i=0;i<ms;i++)
	{
	
		while(!((SysTick->CTRL)&(1<<16)));
	
	}
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
}

void systick_delay_us(uint32_t us)
{
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	SysTick_Config(72);
	for(int i=0;i<us;i++)
	{
	
		while(!((SysTick->CTRL)&(1<<16)));
	
	}
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;
}