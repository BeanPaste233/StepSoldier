#include "DMA_Extension.h"

uint32_t DMA_GetCurrentMemoryTarget(DMA_Channel_TypeDef *DMAx_Channely)
{
	return DMAx_Channely->CMAR;
}

