#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "Thread.h"
#include "string.h"
#include "main.h"
#include "arm_math.h"
#include "MultiFifo.h"




osThreadId_t tid_Thread;                                      // thread id
osThreadId_t adc_Task_id;
osThreadId_t uart_Task_id;
osMessageQueueId_t uartRec;

osSemaphoreId_t adcQspiSem_id;

void ThreadADC (void *argument);
void ThreadUart (void *argument);
	
MultiFifo adcfifo(1440,5, "ADCfifo");
MultiFifo sendfifo(1440,5, "SENDfifo");

void initDMA_adc()
{	
	DMA2_Stream2->PAR = NAND_DEVICE;	
	DMA2_Stream2->M0AR = 0;
	DMA2_Stream2->CR |=DMA_SxCR_TCIE | DMA_SxCR_TEIE;
}

void fmc_write16b(uint16_t &buf){
	*(__IO uint16_t *)((uint32_t)NAND_DEVICE) = buf;
	__DSB();
}

void fmc_writeCNFG(uint32_t reg){
	uint16_t *p1,*p2;
	p1 =  (uint16_t*)&reg;
	p2 = ((uint16_t*)&reg) + 1;
	fmc_write16b(*p2);
	fmc_write16b(*p1);
}

void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *hqspi){
	//if(hqspi->Instance
	adcfifo.freeBlock(MDMA_Channel0->CSAR - 1440);	
	adcfifo.fillFifo();
	osSemaphoreRelease(adcQspiSem_id);
}


void initQSPIcomm(){
	 QSPI_CommandTypeDef s_command;
	
  s_command.Instruction       = 4;
  s_command.Address           = 0;
	s_command.AlternateBytes		= 2;	
  s_command.AddressSize       = QSPI_ADDRESS_8_BITS;
	s_command.AddressSize				= QSPI_ALTERNATE_BYTES_8_BITS;	
  s_command.DummyCycles       = 0;
	s_command.InstructionMode   = QSPI_INSTRUCTION_NONE;
	s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.NbData            = 1440;
	s_command.DdrMode						= QSPI_DDR_MODE_DISABLE;
	s_command.DdrHoldHalfCycle	= QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  
  /* Configure the command */
  HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE);
}

__inline__ void sendQSPI(uint32_t mess){	
	while( (QUADSPI->SR & QUADSPI_SR_FLEVEL_Msk) > QUADSPI_SR_FLEVEL_4){		
	}
	QUADSPI->DR = mess;
}

extern "C" void Thread (void *argument) {		
	printf("Started");	
	osThreadAttr_t worker_attr;
	memset(&worker_attr, 0, sizeof(worker_attr));
  worker_attr.stack_size = 1000; 
	tid_Thread = osThreadNew (ThreadADC, NULL, &worker_attr);	
  worker_attr.stack_size = 3000; 
	uart_Task_id = osThreadNew (ThreadUart, NULL, &worker_attr);
	volatile float freqP = 2 * HAL_RCC_GetPCLK1Freq();
	volatile uint32_t debug = 0;
  while (1) {
		switch(debug){
			case 1:
				TIM2->CR1  |= TIM_CR1_CEN;
				TIM2->ARR = (float)(freqP / 500000.0) - 1;
				TIM2->CNT = 0;
				debug = 0;
				break;
			case 2:
				TIM2->CR1  &= ~TIM_CR1_CEN;
				debug = 0;
				break;
				
			case 3:
			//HAL_QSPI_Transmit_DMA(&hqspi,&masA[0]);
			
			MDMA_Channel0->CBNDTR |=0x10;
			__HAL_MDMA_CLEAR_FLAG(&hmdma_quadspi_fifo_th, MDMA_FLAG_TE | MDMA_FLAG_CTC | MDMA_CISR_BRTIF | MDMA_CISR_BTIF | MDMA_CISR_TCIF); 
			MDMA_Channel0->CCR |= MDMA_CCR_EN;
			/*sendQSPI(0x12345678);
			sendQSPI(0x87654321);
			sendQSPI(0x14725836);
			sendQSPI(0x36925814);*/
				break;
			case 4:
				break;
		};
    osDelay(1000);    
  }
}



extern adcBuffer *p_addrADCsmpl;
extern uint32_t addrADCsmpl;

void ThreadADC (void *argument) {
	uint32_t cnfgReg = 0;	
	initQSPIcomm();
	initDMA_adc();
	CLEAR_BIT(GPIOD->ODR,GPIO_PIN_3);	
	osDelay(1);
	SET_BIT(GPIOD->ODR,GPIO_PIN_3);
	osDelay(1);
	CLEAR_BIT(GPIOD->ODR,GPIO_PIN_3);		
	osDelay(1);	
	cnfgReg = FADC_REFEN | FADC_WRITE_EN | 0x1FF;
	fmc_writeCNFG(cnfgReg); // HAL_NVIC_SetPriority(FMC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	TIM2->CCER|=TIM_CCER_CC3E;
	//---init QSPI DMA
	/*MDMA_Channel0->CBNDTR = 1440;
	__HAL_MDMA_CLEAR_FLAG(&hmdma_quadspi_fifo_th, MDMA_FLAG_TE | MDMA_FLAG_CTC | MDMA_CISR_BRTIF | MDMA_CISR_BTIF | MDMA_CISR_TCIF);  
	MDMA_Channel0->CDAR = (uint32_t)&QUADSPI->DR; 
	MDMA_Channel0->CSAR = (uint32_t) 0x80000000;	
	HAL_QSPI_Transmit_DMA(&hqspi,(uint8_t*)0x80000000);*/
	//-----------------
	
	adcfifo.init();
	adcfifo.fillFifo();
	sendfifo.init(0);
	
	addrADCsmpl = adcfifo.getBuf();
	p_addrADCsmpl = (adcBuffer*)addrADCsmpl;
	uint32_t addrSrc;
	adcQspiSem_id = osSemaphoreNew(1, 1, NULL);
	netBuf *netb;
	while(1){
		addrSrc = sendfifo.getBuf();
		if(addrSrc){				
			osSemaphoreAcquire(adcQspiSem_id, osWaitForever);	
			netb = (netBuf*) addrSrc;
			HAL_QSPI_Transmit_DMA(&hqspi,(uint8_t*)addrSrc);
			/*MDMA_Channel0->CBNDTR = 1440;
			__HAL_MDMA_CLEAR_FLAG(&hmdma_quadspi_fifo_th, MDMA_FLAG_TE | MDMA_FLAG_CTC | MDMA_CISR_BRTIF | MDMA_CISR_BTIF | MDMA_CISR_TCIF);
			MDMA_Channel0->CSAR = addrSrc;				
			MDMA_Channel0->CCR |= MDMA_CCR_EN;*/
		}
			
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	MessageStat msg = mOK;
	osMessageQueuePut(uartRec,&msg,NULL,0);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	MessageStat msg = mERR;
	osMessageQueuePut(uartRec,&msg,NULL,0);
}

void parcePkt(uint8_t* header,uint8_t *data, uint32_t len){
	netHeader * h = (netHeader*) header;		
	uint32_t freq = 0;
	float freqP = 2 * HAL_RCC_GetPCLK1Freq();
	switch(h->type){
		case tInitConn:
		case tStartADC:
			TIM2->CR1  |= TIM_CR1_CEN;
			break;
		case tStopADC:
			TIM2->CR1  &= ~TIM_CR1_CEN;
			break;
		case tSetFreqADC:
			memcpy(&freq,data,4);
			TIM2->ARR = (float)(freqP / freq) - 1;
			TIM2->CNT = 0;
			break;
		case tLogADC:
			SET_BIT(GPIOC->ODR,GPIO_PIN_2);
			CLEAR_BIT(GPIOC->ODR,GPIO_PIN_3);
			break;
		case tLinADC:
			CLEAR_BIT(GPIOC->ODR,GPIO_PIN_2);
			CLEAR_BIT(GPIOC->ODR,GPIO_PIN_3);
			break;
		case tOffADC:
			SET_BIT(GPIOC->ODR,GPIO_PIN_3);
			break;
		case tSetOuter:
			break;
	
		default:
			break;
	}
}

MessageStat recData(uint8_t * buf,uint32_t len){	
	osStatus_t stat;
	MessageStat msg;
	uint8_t tick = 0;
	SCB_InvalidateDCache_by_Addr((uint32_t*)buf,len);
	HAL_UART_Receive_DMA(&huart4,buf,len);
	while(1){
		stat = osMessageQueueGet(uartRec,&msg,NULL,100);
		if(stat == osOK){
			if(msg == mOK){
				return msg;
			}
			if(msg == mERR){				
				return msg;
			}
		}else
		if(stat == osErrorTimeout){
			if(tick < 1){
				if(DMA1_Stream0->NDTR != (len ) ){
					tick++;
				}
			}
			else{
				//return mTimeout;
			}
		}
	}
}


MessageStat recDataPkt(uint8_t * buf){	
	osStatus_t stat;
	MessageStat msg;
	netHeader *h;
	uint32_t lendata = 0;
	//Receive Header
	msg = recData(buf,HEADER_SIZE);
	if(msg == mOK){
		h = (netHeader*) buf;
	}else{
		return mERR;
	}
	
	switch(h->type){
		case tInitConn:
		case tStartADC:
			break;
		case tStopADC:
			break;
		case tSetFreqADC:
			lendata = 4;
			break;
		case tLogADC:
			break;
		case tLinADC:
			break;
		case tOffADC:
			break;
		case tSetOuter:
			break;
	
		default:
			break;
	}
	if(lendata >0){
		msg = recData(buf+HEADER_SIZE,lendata);
		if(msg == mOK){
			parcePkt(buf,buf+HEADER_SIZE,lendata);
			return mOK;
		}
		else{
			return mERR;
		}
	}
	else{
		parcePkt(buf,buf,lendata);
		return mOK;
	}
	//Receive Data
	

}

void ThreadUart (void *argument) {
	uint8_t rxbuf[1452];
	MessageStat msg;
	uartRec = osMessageQueueNew(2,sizeof(MessageStat),NULL);
	while(1){
		msg = recDataPkt(&rxbuf[0]);
	}
}





