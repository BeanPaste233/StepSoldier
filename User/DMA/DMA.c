#include "dma.h"

extern uint8_t rc_data[RC_FRAME_LENGTH];


void RemoteToolDma_Init(void)
{
	DMA_InitTypeDef toolDmaInitStructure;
	
	//¿ªÆôDMAÊ±ÖÓ
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
	
	toolDmaInitStructure.DMA_BufferSize=RC_FRAME_LENGTH;
	toolDmaInitStructure.DMA_DIR=DMA_DIR_PeripheralSRC;
	toolDmaInitStructure.DMA_M2M=DMA_M2M_Disable;
	toolDmaInitStructure.DMA_MemoryBaseAddr=(uint32_t)rc_data;
	toolDmaInitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;
	toolDmaInitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;
	toolDmaInitStructure.DMA_Mode=DMA_Mode_Circular;
	toolDmaInitStructure.DMA_PeripheralBaseAddr=(uint32_t)(USART1_BASE+0x04);
	toolDmaInitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;
	toolDmaInitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;
	toolDmaInitStructure.DMA_Priority=DMA_Priority_VeryHigh;
	DMA_Init(DMA1_Channel5,&toolDmaInitStructure);
	USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	DMA_Cmd(DMA1_Channel5,ENABLE);
}

void NVIC_DMA_Init(void)
{
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel=DMA1_Channel5_IRQn;
	nvic.NVIC_IRQChannelCmd=ENABLE;
	nvic.NVIC_IRQChannelPreemptionPriority=1;
	nvic.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&nvic);
}