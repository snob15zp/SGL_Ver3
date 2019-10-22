#ifndef __BoardInit_h
#define __BoardInit_h

#include <ADUCM360.h>

#define SEL_2								(uint8_t)0x01						// for pin P0.0
#define UART_RX							(uint8_t)0x02						// for pin P0.1
#define UART_TX							(uint8_t)0x04						// for pin P0.2
#define SEL_1								(uint8_t)0x08						// for pin P0.3
#define CPU_FET							(uint8_t)0x10						// for pin P0.4
#define VEXT_PRES						(uint8_t)0x20						// for pin P0.5

#define	CNTR_PER						(uint8_t)0x01						// for pin P2.0	

#define RX_TIMEOUT					(uint32_t)(16000000 / 10) - 1

#define SPI0_CS							(uint8_t)0x80						// for pin P1.7

/***************************************************************************************************************

****************************************************************************************************************/	
void initSysTick(void);
void initCorePin(void);
void initADC(void);
void initUART(void);
uint16_t enableUART(void);
uint16_t disableUART(void);
void initSpi_0(void);
void enableSpi_0(void);
void disableSpi_0(void);
void enableADC(void);
void disableADC(void);
void startWakeUpOnEverySecond(void);

#endif

