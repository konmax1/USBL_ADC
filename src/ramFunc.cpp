#include "Thread.h"
#include "MultiFifo.h"




volatile uint32_t ipos = 0;
volatile osStatus_t stat;	
volatile uint16_t cnt = 0;

adcBuffer *p_addrADCsmpl;
uint32_t addrADCsmpl;

__inline__ void sendQSPI(uint32_t mess){	
	while( (QUADSPI->SR & QUADSPI_SR_FLEVEL_Msk) > QUADSPI_SR_FLEVEL_4){		
	}
	QUADSPI->DR = mess;
}



void fmc_readADC(int16_t* adcval){
	SCB_InvalidateDCache_by_Addr((uint32_t*)0x80000000,32);
	for(volatile int i = 0; i < 8; i++){
	adcval[i] = *((int16_t *)NAND_DEVICE+i);
	}
}


extern "C" void EXTI9_5_IRQHandler(void)
{
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	SCB_InvalidateDCache_by_Addr((uint32_t*)0x80000000,32);
	DMA2_Stream2->M0AR = (uint32_t)&p_addrADCsmpl->mas[ipos][0];
	DMA2_Stream2->NDTR=8;	
	DMA2_Stream2->CR|=DMA_SxCR_EN;
}

extern "C" void DMA2_Stream2_IRQHandler(void)
{
	GPIOC->ODR |= GPIO_PIN_11;
	if(DMA2->LISR & DMA_LISR_TCIF2){
		DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
		ipos++;	
		if(ipos >= SMPL_CNT){
			ipos = 0;
			sendfifo.putBuf(addrADCsmpl);
			addrADCsmpl = adcfifo.getBuf(0);
			if(addrADCsmpl != 0)	{
				p_addrADCsmpl = (adcBuffer*)addrADCsmpl;			
				}
			else{
				//printf("Not Buf ADC");
			}	
		}
	}
	if(DMA2->LISR & DMA_LISR_TEIF2){
		DMA2->LIFCR |= DMA_LIFCR_CTEIF2;
	}	
	GPIOC->ODR &= ~GPIO_PIN_11;
}










