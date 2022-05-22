#include "RC.h"
#include <stdio.h>


extern uint8_t rc_data[RC_FRAME_LENGTH];


void RemoteToolDma_Init(void)
{
	DMA_InitTypeDef toolDmaInitStructure;
	
	//开启DMA时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	
	toolDmaInitStructure.DMA_BufferSize=RC_FRAME_LENGTH;
	toolDmaInitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
	toolDmaInitStructure.DMA_M2M=DMA_M2M_Disable;
	toolDmaInitStructure.DMA_MemoryBaseAddr=(uint32_t)&rc_data;
	toolDmaInitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	toolDmaInitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
	toolDmaInitStructure.DMA_Mode=DMA_Mode_Circular;
	toolDmaInitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(USART1->DR);
	toolDmaInitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	toolDmaInitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	toolDmaInitStructure.DMA_Priority=DMA_Priority_VeryHigh;
	DMA_Init(DMA1_Channel5,&toolDmaInitStructure);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	DMA_Cmd(DMA1_Channel5,ENABLE);
	
	
	
}
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

void NVIC_RC_Init(void)
{
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel=DMA1_Channel5_IRQn;
	nvic.NVIC_IRQChannelCmd=ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority=1;
	nvic.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&nvic);
}
void RemoteControl_Init(void)
{
	  DebusUsart_Init();
		RemoteToolDma_Init();
		NVIC_RC_Init();
}

void RemotePacketProcess(RC_PacketStructure rc_packet)
{
	if(&rc_packet==NULL) return;
	rc_packet.rc.channel0 = ((int16_t)rc_data[0] | ((int16_t)rc_data[1] << 8)) & 0x07FF; 
	rc_packet.rc.channel1 = (((int16_t)rc_data[1] >> 3) | ((int16_t)rc_data[2] << 5)) & 0x07FF;
	rc_packet.rc.channel2 = (((int16_t)rc_data[2] >> 6) | ((int16_t)rc_data[3] << 2) |((int16_t)rc_data[4] << 10)) & 0x07FF;
	rc_packet.rc.channel3 = (((int16_t)rc_data[4] >> 1) | ((int16_t)rc_data[5]<<7)) & 0x07FF;
 
	rc_packet.rc.s1 = ((rc_data[5] >> 4) & 0x000C) >> 2;
	rc_packet.rc.s2 = ((rc_data[5] >> 4) & 0x0003);
	rc_packet.mouse.x = ((int16_t)rc_data[6]) | ((int16_t)rc_data[7] << 8);
	rc_packet.mouse.y = ((int16_t)rc_data[8]) | ((int16_t)rc_data[9] << 8);
	rc_packet.mouse.z = ((int16_t)rc_data[10]) | ((int16_t)rc_data[11] << 8); 
	rc_packet.mouse.press_l = rc_data[12];
	rc_packet.mouse.press_r = rc_data[13];
	rc_packet.key.v = ((int16_t)rc_data[14]);// | ((int16_t)pData[15] << 8);
}
