#pragma once

#include "cmsis_os2.h"                                        // CMSIS RTOS header file
#include "stm32h7xx_hal.h"
#include "string.h"
#include "stm32h7xx.h"                  // Device header
#include "main.h"
#include "Multififo.h"


#define FADC_WRITE_EN (uint32_t)(1<<31)
#define FADC_READ_EN (uint32_t)(1<<30)
#define FADC_CLKSEL (uint32_t)(1<<29)
#define FADC_CLKOUT (uint32_t)(1<<28)
#define FADC_BUSY_INT (uint32_t)(1<<27)
#define FADC_BUSY_POL (uint32_t)(1<<26)
#define FADC_STBY (uint32_t)(1<<25)
#define FADC_RANGE_A (uint32_t)(1<<24)

#define FADC_RANGE_B (uint32_t)(1<<23)
#define FADC_PD_B (uint32_t)(1<<22)
#define FADC_RANGE_C (uint32_t)(1<<21)
#define FADC_PD_C (uint32_t)(1<<20)
#define FADC_RANGE_D (uint32_t)(1<<19)
#define FADC_PD_D (uint32_t)(1<<18)

#define FADC_REFEN (uint32_t)(1<<15)
#define FADC_REFBUF (uint32_t)(1<<14)
#define FADC_VREF (uint32_t)(1<<13)

#define FADC_D9 (uint32_t)(1<<9)
#define FADC_D8 (uint32_t)(1<<8)
#define FADC_D7 (uint32_t)(1<<7)
#define FADC_D6 (uint32_t)(1<<6)
#define FADC_D5 (uint32_t)(1<<5)
#define FADC_D4 (uint32_t)(1<<4)
#define FADC_D3 (uint32_t)(1<<3)
#define FADC_D2 (uint32_t)(1<<2)
#define FADC_D1 (uint32_t)(1<<1)
#define FADC_D0 (uint32_t)(1<<0)

#define SMPL_CNT (90)
#define HEADER_SIZE (12)

struct adcBuffer{
	int16_t mas[SMPL_CNT][8];
};

enum MessageStat : int32_t{
	mOK = 1,
	mERR = -1,
	mTimeout = -2,
};

enum typeCMD : uint16_t{
	tInitConn			=	0x0001,
	tStartADC			=	0x0002,
	tStopADC 			=	0x0003,
	tSetFreqADC		= 0x0004,
	tADCsmpl 			=	0x0005,	
	tSetOuter			=	0x0006,
	tADCsmplOuter	=	0x0007,
	tLogADC				=	0x0010,
	tLinADC				=	0x0011,
	tOffADC				=	0x0012,
	tErr					=	0x00FF,
	tUnknown			=	0xFFFF,
};



struct netHeader{
	typeCMD type;
	uint16_t counter;
	uint16_t data0;
	uint16_t data1;
	uint16_t data2;
	uint16_t data3;
};

struct OuterData{
	uint32_t freqOuter;
	int32_t Nperiod;
	int32_t lenPSP;
	uint8_t pspMas[511];
};

struct netBuf{
	typeCMD type;
	uint16_t counter;	
	uint16_t data0;
	uint16_t data1;
	uint16_t data2;
	uint16_t data3;
	int16_t mas[SMPL_CNT][8];
};

extern uint16_t bufadc[10][720];

uint32_t  GetBUF();

extern osThreadId_t tid_Thread;                                      // thread id

extern TIM_HandleTypeDef htim2;

extern DMA_HandleTypeDef hdma_memtomem_dma2_stream2;

extern UART_HandleTypeDef huart4;

extern SPI_HandleTypeDef hspi1;

extern uint32_t addrADCsmpl;


