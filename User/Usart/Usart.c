#include "usart.h"

void DebusUsart_Init(void)
{
	USART_InitTypeDef usartInitStructure;
	GPIO_InitTypeDef gpioInitStructure;
	//开启时钟
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	//配置GPIO
	gpioInitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	gpioInitStructure.GPIO_Pin=GPIO_Pin_10;
	gpioInitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpioInitStructure);
	
	//配置USART1
	usartInitStructure.USART_BaudRate=100000;
	usartInitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	usartInitStructure.USART_Mode=USART_Mode_Rx;
	usartInitStructure.USART_Parity=USART_Parity_Even;
	usartInitStructure.USART_StopBits=1;
	usartInitStructure.USART_WordLength=USART_WordLength_8b;

	USART_Init(USART1,&usartInitStructure);
	USART_Cmd(USART1,ENABLE);//使能usart1
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//请求DMA

}



