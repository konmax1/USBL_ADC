#include "Thread.h"




volatile uint32_t ipos = 0;
volatile osStatus_t stat;	
volatile uint16_t cnt = 0;

uint32_t addrADCsmpl;



uint32_t curbuf = 0;

uint32_t  GetBUF(){
	bufadc[curbuf][0] = tADCsmpl;
	bufadc[curbuf][1] = 0;
	uint32_t addr = (uint32_t)&bufadc[curbuf][0];
	curbuf =  (( curbuf + 1 ) % NUMBER_CIRCLE_BUF);
	return addr;
}

extern "C" void EXTI9_5_IRQHandler(void)
{
	SET_BIT(GPIOG->ODR,GPIO_PIN_6);
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
	if(addrADCsmpl == 0){		
		CLEAR_BIT(GPIOG->ODR,GPIO_PIN_6);
		return;
	}
	
	/*DMA1_Stream0->M0AR = (uint32_t)&p_addrADCsmpl->mas[ipos][0];
	DMA1_Stream0->NDTR=8;	
	DMA1_Stream0->CR|=DMA_SxCR_EN;*/
	
	///MDMA start
	if( ( MDMA_Channel0->CCR & MDMA_CCR_EN)  == 0){
		stat = osOK;
	}
	MDMA_Channel0->CCR|= MDMA_CCR_SWRQ;
	
	CLEAR_BIT(GPIOG->ODR,GPIO_PIN_6);
}

extern "C" void MDMA_IRQHandler(void){
	SET_BIT(GPIOG->ODR,GPIO_PIN_6);
	if(MDMA_Channel0->CISR & MDMA_CISR_CTCIF){
		MDMA_Channel0->CIFCR|= MDMA_CIFCR_CCTCIF ;
		
		DMA1->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3;
		SPI1->IFCR = 0xFFFFFFFF;
		DMA1_Stream3->NDTR = PacketSizeShort;
		DMA1_Stream3->M0AR = addrADCsmpl;
		DMA1_Stream3->CR|=DMA_SxCR_EN;
		SET_BIT(SPI1->CR1, SPI_CR1_CSTART);
		
		addrADCsmpl = GetBUF();		
		MDMA_Channel0->CBNDTR |= ( (SMPL_CNT-1) << MDMA_CBNDTR_BRC_Pos ) | 16;	
		MDMA_Channel0->CDAR = addrADCsmpl + HEADER_SIZE;
		MDMA_Channel0->CCR|=MDMA_CCR_EN ;
	}
	if(MDMA_Channel0->CISR & MDMA_CISR_TEIF){
		MDMA_Channel0->CIFCR|= MDMA_CIFCR_CTEIF ;
	}	
	CLEAR_BIT(GPIOG->ODR,GPIO_PIN_6);
}



/*extern "C" void DMA1_Stream0_IRQHandler(void)
{
	SET_BIT(GPIOG->ODR,GPIO_PIN_6);
	if(DMA1->LISR & DMA_LISR_TCIF0){
		DMA1->LIFCR |= DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
		ipos++;	
		if(ipos >= SMPL_CNT){			
			SET_BIT(GPIOG->ODR,GPIO_PIN_5);
			ipos = 0;	
			//SCB_InvalidateDCache_by_Addr((uint32_t*)addrADCsmpl,1440);			
			DMA1->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3;
			SPI1->IFCR = 0xFFFFFFFF;
			DMA1_Stream3->NDTR = 720;
			DMA1_Stream3->M0AR = addrADCsmpl;
			DMA1_Stream3->CR|=DMA_SxCR_EN;
			SET_BIT(SPI1->CR1, SPI_CR1_CSTART);
			
			//sendfifo.putBuf(addrADCsmpl);
			addrADCsmpl = GetBUF();
			//addrADCsmpl = adcfifo.getBuf(0);
			if(addrADCsmpl != 0){
				p_addrADCsmpl = (adcBuffer*)addrADCsmpl;
			}			
			CLEAR_BIT(GPIOG->ODR,GPIO_PIN_5);	
		}
	}
	if(DMA1->LISR & DMA_LISR_TEIF0){
		DMA1->LIFCR |= DMA_LIFCR_CTEIF0;
	}	
	CLEAR_BIT(GPIOG->ODR,GPIO_PIN_6);
}*/

extern "C" void DMA1_Stream3_IRQHandler(void)
{
	if(DMA1->LISR & DMA_LISR_TCIF3){
		DMA1->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
	}
	if(DMA1->LISR & DMA_LISR_TEIF3){
		DMA1->LIFCR |= DMA_LIFCR_CTEIF3;
	}	
  //HAL_DMA_IRQHandler(&hdma_spi1_tx);

}








