#include "BoardInit.h"

/***************************************************************************************************************


****************************************************************************************************************/
void initSysTick(void){
	SysTick->LOAD = RX_TIMEOUT;																								// load time to SysTick
	SysTick->VAL = RX_TIMEOUT;																								// clear timer and flags 
	SysTick->CTRL =	SysTick_CTRL_CLKSOURCE_Msk;
}

/***************************************************************************************************************


****************************************************************************************************************/
void initCorePin(void){

	pADI_GP0->GPSET = 0xFF;
	pADI_GP0->GPOCE = ~UART_RX;																								// set unused pins as Open Circuit	
	pADI_GP0->GPOEN = ~UART_RX;																								// set all pins as Output floating (open drain) except VEXT_PRES
	pADI_GP0->GPPUL = 0x00;																										// disable pull-up resistors for all pins

	pADI_GP1->GPSET = 0xFF;					
	pADI_GP0->GPOCE = 0xFF;
	pADI_GP1->GPOEN = 0xFF;																										// set all pins as Output floating (open drain)
	pADI_GP1->GPPUL = 0x00;																										// disable pull-up resistors for all pins		

	pADI_GP2->GPSET = 0xFF;	
	pADI_GP2->GPOCE = 0xFF;																										// set all pin as Open Circuit
	pADI_GP2->GPOEN = 0xFF;																										// set all pin as Output floating (open drain)
	pADI_GP2->GPPUL = 0x00;																										// disable pull-up resistors for all pins

}
	
/*************************************************************************************************************** 


****************************************************************************************************************/
void initADC(void){

	pADI_ADC0->MDE |= (ADC0MDE_ADCMD_IDLE | ADC0MDE_PGA_G64);									// switch ADC0 to idle mode,  PGA gain = 64
	pADI_ADC0->MSKI |= ADC0MSKI_RDY;

	pADI_ADC1->MDE |= (ADC1MDE_ADCMD_IDLE | ADC1MDE_PGA_G2);									// switch ADC1 to idle mode,  PGA gain = 2
	pADI_ADC1->CON |= ADC1CON_ADCCP_TEMP | ADC1CON_ADCCN_TEMP |								// connect temperature sensor to ADC1
										ADC1CON_BUFBYPN | ADC1CON_BUFBYPP |											// negative and positive buffers bypass
										ADC1CON_BUFPOWN | ADC1CON_BUFPOWP ;											// negative and positive buffers power down
										
	pADI_ADC1->MSKI |= ADC1MSKI_RDY;

	NVIC_EnableIRQ(ADC0_IRQn);					// Enable ADC0 interrupt sources
  NVIC_EnableIRQ(ADC1_IRQn);					// Enable ADC1 interrupt sources
}

/*************************************************************************************************************** 


****************************************************************************************************************/
void enableADC(void){
	
	pADI_CLKCTL->CLKDIS &= ~CLKDIS_DISADCCLK;																	// enable ADCclk
	pADI_ADC0->CON |= ADC0CON_ADCEN; 																					// enable the ADC0 
	pADI_ADC1->CON |= ADC1CON_ADCEN; 																					// enable the ADC1 
	
	pADI_GP2->GPCLR = CNTR_PER;																								// out LOW to CNTR_PER
	pADI_GP2->GPOCE &= ~CNTR_PER; 																						
	
	pADI_GP0->GPCLR = (SEL_1 | SEL_2);																				// out LOW to SEL_1 and SEL_2
	pADI_GP0->GPOCE &= ~(SEL_1 | SEL_2); 																			// set SEL_1 and SEL_2 as Outputs
}

/*************************************************************************************************************** 


****************************************************************************************************************/
void disableADC(void){
	
	pADI_ADC0->CON &= ~ADC0CON_ADCEN; 																				// disable the ADC
	pADI_ADC1->CON &= ~ADC1CON_ADCEN; 																				// disable the ADC
	pADI_CLKCTL->CLKDIS |= CLKDIS_DISADCCLK;																	// disable ADCclk

	pADI_GP0->GPOCE |= (SEL_1 | SEL_2); 																			// set SEL_1 and SEL_2 as Open Drain Float
	pADI_GP0->GPSET = (SEL_1 | SEL_2);	

	pADI_GP2->GPSET = CNTR_PER;																								// set CNTR_PER as Open Drain Float
	pADI_GP2->GPOCE |= CNTR_PER; 
}

/***************************************************************************************************************


****************************************************************************************************************/	
#define	DIV_115200		1
#define	DIVM_115200  (1<<11)
#define DIVN_115200  	174

void initUART(void){
	pADI_UART->COMLCR |= COMLCR_WLS_7BITS | COMLCR_STOP | COMLCR_PEN | COMLCR_EPS; 	// set mode 7 bit, 1 start, 1 stop, parity EVEN check
	pADI_UART->COMMCR	|= COMMCR_OUT1;																					// enable parity 
	pADI_UART->COMDIV = DIV_115200;																						// set Baud Rate = 115 200
	pADI_UART->COMFBR = COMFBR_ENABLE_EN | DIVM_115200 | DIVN_115200;
	pADI_UART->COMIEN = COMIEN_ERBFI | COMIEN_ELSI;														// enable Rx interrupts
	NVIC_SetPriority(UART_IRQn, 3);																						// set Uart interrupt priority
	NVIC_EnableIRQ(UART_IRQn);																								// enable UART interrupt	
}	


/***************************************************************************************************************


****************************************************************************************************************/	
uint16_t enableUART(void){
	pADI_GP0->GPCON |= GP0CON_CON1_UARTRXD | GP0CON_CON2_UARTTXD;							// configure pins P0.1 and P0.2 for UART
	pADI_CLKCTL->CLKCON1 = (pADI_CLKCTL->CLKCON1 & ~CLKCON1_UARTCD_MSK) | 
													CLKCON1_UARTCD_DIV2;															// set UARTclk to 4MHz
	pADI_CLKCTL->CLKDIS &= ~CLKDIS_DISUARTCLK;																// enable UARTclk
	pADI_UART->COMCON = 0x00;																									// enable UART
	
	return 1;
}

/***************************************************************************************************************


****************************************************************************************************************/	
uint16_t disableUART(void){	
	pADI_UART->COMCON = 0x01;																									// disable UART
	pADI_CLKCTL->CLKCON1 |= CLKCON1_UARTCD_MSK;																// set UARTclk to UCLK/128
	pADI_CLKCTL->CLKDIS |= CLKDIS_DISUARTCLK;																	// disable UARTclk
	pADI_GP0->GPCON &= ~(GP0CON_CON1_MSK | GP0CON_CON2_MSK);									// configure pins P0.1 and P0.2 as GPIO
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
#define	PRE_DIV				256
#define ONE_SECUND		(32768 / PRE_DIV)			
void startWakeUpOnEverySecond(void){

	pADI_CLKCTL->XOSCCON |= XOSCCON_ENABLE;																	// enable the LFXTAL oscillator circuitry
	pADI_WUT->T2CON = T2CON_MOD_PERIODIC |																	// set Operate in PERIODIC mode; that is, counts up to the value in T2WUFD 
										T2CON_WUEN_EN | 																			// set Wake-up enable bits
										T2CON_PRE_DIV256 | 																		// clock prescaler select
										T2CON_CLK_LFXTAL;  																		// clock select LFXTAL: 32 kHz external crystal
	pADI_WUT->T2WUFD0 = (uint16_t)(ONE_SECUND - 1);													// set periodic interrupt\wake-up to 1 secund
	pADI_WUT->T2WUFD1 = (uint16_t)(0);
	pADI_WUT->T2IEN  =  T2IEN_WUFD;  																				// generate an interrupt when T2VAL reaches T2WUFD 
	NVIC_EnableIRQ(WUT_IRQn);																								// enable the Wake-up Timer IRQ
	T2CON_ENABLE_BBA = 1; 																									// enable the timer
}


/***************************************************************************************************************


****************************************************************************************************************/
void initSpi_0(void){
	pADI_SPI0->SPIDIV = 0x09;																									// set SPI_CLK to 100 KHz
	pADI_SPI0->SPICON |= SPICON_MASEN | 																			// enable master mode
											 SPICON_CON |																					// enable continuous transfer	
											 SPICON_MOD_TX1RX1 |																	// Tx interrupt occurs when one byte has been transferred
											 SPICON_TIM;  																				// initiate transfer with a write to the SPI TX register
	NVIC_EnableIRQ(SPI0_IRQn);																								// enable the SPI0 IRQ
}

/***************************************************************************************************************


****************************************************************************************************************/
void enableSpi_0(void){
	pADI_GP1->GPCON |= GP1CON_CON7_SPI0CS | GP1CON_CON6_SPI0MOSI | GP1CON_CON5_SPI0SCLK | GP1CON_CON4_SPI0MISO; 	// configure pins P1.4..P1.7 for SPI0

	pADI_GP2->GPCLR = CNTR_PER;																								// out LOW to CNTR_PER
	pADI_GP2->GPOCE &= ~CNTR_PER; 																						

	pADI_CLKCTL->CLKCON1 = (pADI_CLKCTL->CLKCON1 & ~CLKCON1_SPI0CD_MSK) | 
													CLKCON1_SPI0CD_DIV4;															// set SPI0clk to 1MHz
	pADI_CLKCTL->CLKDIS &= ~CLKDIS_DISSPI0CLK;																// enable SPI0clk
	pADI_SPI0->SPICON |= SPI0CON_ENABLE;																			// enable the SPI
}

/***************************************************************************************************************


****************************************************************************************************************/
void disableSpi_0(void){
	pADI_SPI0->SPICON &= ~SPI0CON_ENABLE;																			// disable the SPI
	pADI_CLKCTL->CLKCON1 |= CLKCON1_SPI0CD_MSK;																// set SPI0clk to UCLK/128
	pADI_CLKCTL->CLKDIS |= CLKDIS_DISSPI0CLK;																	// disable SPI0clk

	pADI_GP2->GPSET = CNTR_PER;																								// set CNTR_PER as Open Drain Float
	pADI_GP2->GPOCE |= CNTR_PER; 

	pADI_GP1->GPCON &= ~(GP1CON_CON7_SPI0CS | GP1CON_CON6_SPI0MOSI | GP1CON_CON5_SPI0SCLK | GP1CON_CON4_SPI0MISO);
}

/***************************************************************************************************************


***************************************************************************************************************
uint16_t crcCalc(uint8_t* buff, uint8_t length){
	uint16_t crcReg = 0xFFFF;
	while(length--){
		crcReg ^= *buff++;
		for (uint8_t i = 0; i < 8; i++ ){
			crcReg = (crcReg & 1) ? ((crcReg >> 1) ^ 0xA001) : (crcReg >> 1);
		}
	}
 	return crcReg;
}
*/
