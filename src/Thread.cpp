#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "Thread.h"
#include "string.h"
#include "main.h"
#include "arm_math.h"




osThreadId_t tid_Thread;                                      // thread id
osThreadId_t uart_Task_id;
osMessageQueueId_t uartRec;

void ThreadADC (void *argument);
void ThreadUart (void *argument);
	
uint16_t bufadc[NUMBER_CIRCLE_BUF][PacketSizeShort];


void initDMA_adc()
{	
	DMA1_Stream0->PAR = NAND_DEVICE;	
	DMA1_Stream0->M0AR = 0;
	DMA1_Stream0->CR |=DMA_SxCR_TCIE | DMA_SxCR_TEIE;
	
	MDMA_Channel0->CSAR = NAND_DEVICE;	
	addrADCsmpl = GetBUF() + HEADER_SIZE;
	MDMA_Channel0->CDAR = addrADCsmpl;
	MDMA_Channel0->CCR |=MDMA_CCR_CTCIE | MDMA_CCR_TEIE;
	MDMA_Channel0->CBNDTR |= (SMPL_CNT-1) << MDMA_CBNDTR_BRC_Pos | 16;	
	MDMA_Channel0->CCR|=MDMA_CCR_EN ;
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

extern "C" void Thread (void *argument) {		
	
	//printf("Started");	
	osThreadAttr_t worker_attr;
	memset(&worker_attr, 0, sizeof(worker_attr));
	
	//Init SPI
	SPI1->CR2 |= PacketSizeShort;
	SPI1->CR1 |= SPI_CR1_SPE;
	DMA1_Stream3->PAR = (uint32_t)&SPI1->TXDR;		
	SET_BIT(SPI1->CFG1, SPI_CFG1_TXDMAEN);
	
	//init ADC
	uint32_t cnfgReg = 0;		
	initDMA_adc();
	CLEAR_BIT(GPIOD->ODR,GPIO_PIN_3);	
	osDelay(1);
	SET_BIT(GPIOD->ODR,GPIO_PIN_3);
	osDelay(1);
	CLEAR_BIT(GPIOD->ODR,GPIO_PIN_3);		
	osDelay(1);	
	cnfgReg = FADC_REFEN | FADC_WRITE_EN | 0x1FF;
	fmc_writeCNFG(cnfgReg);
	TIM2->CCER|=TIM_CCER_CC3E;
	
	
  worker_attr.stack_size = 1000; 
	uart_Task_id = osThreadNew (ThreadUart, NULL, &worker_attr);
	
  while (1) {
    osDelay(1000000);    
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
	OuterData * outdata		;
	netBuf *netb;
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
			outdata = (OuterData*) data;	
			SetFreqOuter(outdata->freqOuter,outdata->Nperiod,outdata->lenPSP);
			setPSP(&outdata->pspMas[0]);
			netb = (netBuf*)addrADCsmpl;
			netb->counter=0;
			netb->data0=(MDMA_Channel0->CDAR - addrADCsmpl)/16;
			netb->type = tADCsmplOuter;
			/*p_addrADCsmpl->type = tADCsmplOuter;
			p_addrADCsmpl->data0 = ipos;	*/	
			StartOuter();
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
				if(DMA1_Stream5->NDTR != (len ) ){
					tick++;
				}
			}
			else{
				HAL_UART_AbortReceive(&huart4);
				return mTimeout;
			}
		}
	}
}


MessageStat recDataPkt(uint8_t * buf){	
	uint8_t * pb = buf;
	osStatus_t stat;
	MessageStat msg;
	netHeader *h;
	uint32_t lendata = 0;
	//Receive Header
	msg = recData(pb,HEADER_SIZE);
	if(msg == mOK){
		h = (netHeader*) pb;
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
			lendata = 523;
			break;
	
		default:
			break;
	}
	if(lendata >0){
		msg = recData(pb+HEADER_SIZE,lendata);
		if(msg == mOK){
			parcePkt(pb,pb+HEADER_SIZE,lendata);
			return mOK;
		}
		else{
			return mERR;
		}
	}
	else{
		parcePkt(pb,pb,lendata);
		return mOK;
	}
}

void ThreadUart (void *argument) {
	static uint8_t rxbuf[1600];
	MessageStat msg;
	uartRec = osMessageQueueNew(2,sizeof(MessageStat),NULL);
	while(1){
		msg = recDataPkt(&rxbuf[0]);
	}
}





