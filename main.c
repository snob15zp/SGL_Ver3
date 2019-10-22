// Header:
// File Name: main.c
// Author: Yakov Churinov
// Date:

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include <ADUCM360.h>
#include <BoardInit.h>
#include "LoggerDef.h"

typedef  uint16_t (*type_MyFunct)(void);
static  const type_MyFunct funcAddress[] = {addrError, cmdError, setLoggerTime, getLoggerTime, getLoggerData, clrLoggerFlash, setLoggerLimitsNeg, setLoggerLimitsPos,
																						getLoggerLimitsNeg, getLoggerLimitsPos, setLoggerPeriod, getLoggerPeriod, setTestMode, resetTestMode};

/***************************************************************************************************************
	
	UART RX timeout interrupt handler

****************************************************************************************************************/
void SysTick_Handler(){
	pUartTxBuffer = (char*)&uartTxBuffer;																			// set the pointer to the beginning of the RX buffer
	pUartRxBuffer = (char*)&uartRxBuffer;
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
}

/***************************************************************************************************************
	
	UART interrupt handler

****************************************************************************************************************/
void UART_Int_Handler(){
	
	volatile uint8_t comIntID = pADI_UART->COMIIR;
	volatile uint8_t comLineStatus = pADI_UART->COMLSR;

	switch(comIntID){

		case TX_INTERRUPT:

			pADI_UART->COMTX = *pUartTxBuffer;																		// load byte to transmiter
			if (*pUartTxBuffer == LAST_BYTE){																			// if the last byte is loaded
				Flags.UART_TX_END = 1;
				if(!Flags.SEND_ARRAY){Flags.UART_BUSY = 0;}													// disable flag UART_BUSY
				pADI_UART->COMIEN &= ~COMIEN_ETBEI;																	// disable Tx interrupt;	
			} else { pUartTxBuffer++;}																						// else, shift buffer pointer
			break;

		case RX_INTERRUPT:

			SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk); // disable SysTick

			*pUartRxBuffer = pADI_UART->COMRX;																			// read byte from UART
			if(*pUartRxBuffer == LAST_BYTE){																				// if the last byte is readed
				Flags.UART_RX_END = 1;																								// set flag
				pUartRxBuffer = (char*)&uartRxBuffer;																	// set the pointer to the beginning of the RX buffer
			} else { 
					pUartRxBuffer++;
					if(pUartRxBuffer > ((char*)&uartRxBuffer + sizeof(uartRxBuffer))){pUartRxBuffer = (char*)&uartRxBuffer;}
					SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk); // enable SysTick
				}
			break;
	
		default:																																// if UART error occured
			if(comLineStatus & COMLSR_BI){Flags.EXTERN_PWR = 0;}									// if the BREAK condition is recognized, then there is no external power supply 
			*pUartRxBuffer = pADI_UART->COMRX;																		// read byte from UART
			pUartRxBuffer = (char*)&uartRxBuffer;																	// set the pointer to the beginning of the RX buffer
			pADI_UART->COMIEN = COMIEN_ERBFI | COMIEN_ELSI; 											// enable RX interrupts)
			break;
	}
} 

/***************************************************************************************************************


****************************************************************************************************************/
void WakeUp_Int_Handler(){ 
	
	pADI_WUT->T2CLRI = T2CLRI_WUFD;																						// clear interrupt flag
	Flags.UPDATE_RTC = 1;
	__DSB();
}
 
/***************************************************************************************************************


****************************************************************************************************************/
void SPI0_Int_Handler(){

	volatile uint16_t spiStatus = pADI_SPI0->SPISTA;

	switch (spiMode){
    case SPI_TX:																									// if SPI work mode is TX
			spiTxCount--;																								// decrease TX byte counter
			if (spiTxCount){																						// if the counter is not zero
					pADI_SPI0->SPITX = *pSpiBuffer++;												// load next byte for transmition from buffer 				
			} else {																										// if all necessary bytes have been transferred
					if (spiRxCount){																				// if the number of bytes to be received is not equal to zero
						pADI_SPI0->SPITX = 0xAA;															// load to transmitter any byte, for example 0xAA
						spiMode = SPI_RX;																			// switch SPI work mode to RX
						pSpiBuffer = (uint8_t*)&spiBuffer;										// set pointer to buffer begin				
          } else {Flags.SPI_BUSY = 0;}														// if receive bytes not need, clear SPI_BUSY flag
				}
      break;
  
    case SPI_RX:																									// if SPI work mode is TX
			if(spiRxCount){																							// if the number of bytes to be received is not equal to zero
				pADI_SPI0->SPITX = 0xAA;																	// load to transmitter any byte, for example 0xAA
				if(pADI_SPI0->SPICON & SPICON_RFLUSH) {										// if RX FLUSH enabled
					pADI_SPI0->SPICON &= ~SPICON_RFLUSH;										// disable RX FLUSH
				} else {							
						while (pADI_SPI0->SPISTA & SPISTA_RXFSTA_MSK){ 				// while RX_FIFO not empty
							*pSpiBuffer++ = pADI_SPI0->SPIRX; 									// store byte from RX_FIFO to buffer
							spiRxCount--;																				// decrease RX byte counter
						}
					}
			} else { Flags.SPI_BUSY = 0; }															// if all necessary bytes have been received, clear SPI_BUSY flag			
      break;
  }
}

/***************************************************************************************************************


****************************************************************************************************************/
void ADC0_Int_Handler(){
   volatile uint16_t adcStatus __attribute__((unused)) = 0;  
   adcStatus = pADI_ADC0->STA;               																// read ADC status register
   resultAdc0 = pADI_ADC0->DAT;            																	// read ADC result register
   Flags.ADC0_READY = 1;                  																	// Set flag to indicate ready
}

/***************************************************************************************************************


****************************************************************************************************************/
void ADC1_Int_Handler (){
   volatile uint16_t adcStatus __attribute__((unused)) = 0;  
   adcStatus = pADI_ADC1->STA;               																// read ADC status register
   resultAdc1 = pADI_ADC1->DAT;            																	// read ADC result register
   Flags.ADC1_READY = 1;                  																	// Set flag to indicate ready
}

/***************************************************************************************************************


****************************************************************************************************************/
int main (void){

	initCorePin();

	enableADC();
	initADC();
	disableADC();

	Flags.EXTERN_PWR = enableUART();
	initUART();

	enableSpi_0();
	initSpi_0();
//	clrLoggerFlash();
	disableSpi_0();

	initSysTick();
		
	recordsCount = 0;
	pageCount = 0;

	if(!(pADI_GP0->GPIN & UART_RX)){
		Flags.EXTERN_PWR = disableUART();
		Flags.SLEEP_READY = 1;
	}
	
	timeRecord.dateTime = setUtcDateTime(2019, 1, 1, 0, 0, 0);
	timeRecord.dataType = RT_DATE_TIME;
	startWakeUpOnEverySecond();
	
	interval = periodsArray[1];
	
	Flags.TEST_MODE = 1;
	
	while (1){

		if(Flags.UPDATE_RTC){
			Flags.UPDATE_RTC = 0;
			timeRecord.dateTime++;

			if(Flags.TEST_MODE){
				getSensorData();
				sendTestData();
			} else if ((timeRecord.dateTime % interval) == 0){
					Flags.GET_SENSOR_DATA = 1;
				}
		}

		if(Flags.GET_SENSOR_DATA){
			Flags.GET_SENSOR_DATA = 0;
			getSensorData();
			pStoreRecord = (uint16_t*)&dataArray;																			// set a pointer to the data to be stored
			Flags.STORE_DATA = 1;
		}

		if(Flags.STORE_DATA){
			Flags.STORE_DATA = 0;	
			if(storeNewData()){storeToFlash();}
			Flags.SLEEP_READY = 1;
    }

		if(Flags.SLEEP_READY){
			goToSleep();
			if(pADI_GP0->GPIN & UART_RX){Flags.EXTERN_PWR = enableUART();}		
		}
		
		if (Flags.UART_TX_END == 1){
			Flags.UART_TX_END = 0;
			if(Flags.SEND_ARRAY){
				Flags.SEND_ARRAY = createDataRecord();
				uartSendPacket();
			}
		}

		if(Flags.UART_RX_END && !Flags.UART_BUSY){
			Flags.UART_RX_END = 0;
			function = funcAddress[executeCommand((char*)&uartRxBuffer)];  
			if(function() != 0){sendMessage((char*)errStr);}
		}		
	}  
}

/***************************************************************************************************************


****************************************************************************************************************/
void goToSleep(void){

	if(Flags.EXTERN_PWR){
		Flags.SLEEP_READY = 0;
		return;
	}

	if(Flags.SPI_BUSY || Flags.UART_BUSY){return;}

	Flags.SLEEP_READY = 0;

	disableSpi_0();
	Flags.EXTERN_PWR = disableUART();
	disableADC();

/*
	SCB->SCR = 0x04;       																										// for deep sleep mode - write to the Cortex-m3 System Control register bit2

	pADI_PWRCTL->PWRKEY = 0x4859;   																					// key1 
	pADI_PWRCTL->PWRKEY = 0xF27B;   																					// key2  
	pADI_PWRCTL->PWRMOD = PWRMOD_MOD_TOTALHALT; 															// deep sleep mode 

	uint32_t i = 0;
	for(i = 0; i < 2; i++){}

	__WFI(); 
		
	for(i = 0; i < 2; i++){}
*/

}

/***************************************************************************************************************


****************************************************************************************************************/
void storeToFlash(void){
//	pADI_GP0->GPOEN &= ~CPU_FET;																							// enable VBAT switch
	
	/* Store data to FLASH memory */

	enableSpi_0();
	if(pageCount < 256){
		flashWriteEnable();
		spiStartTransfer((uint8_t*)&sPage, PAGE_PROGRAM, (pageCount * 256), 256, 0);
		while (Flags.SPI_BUSY){}																								// wait while SPI busy
		flashWriteDisable();
		pageCount++;
		recordsCount = 0;
	}
	disableSpi_0();
/*
	if(Flags.EXTERN_PWR == 1) {																								// if VEXT is present 
		pADI_GP0->GPOEN |= CPU_FET;																							// config CPU_FET as output																
		pADI_GP0->GPSET	= CPU_FET;																							// disable VBAT switch 
	}
*/
}

/***************************************************************************************************************


****************************************************************************************************************/
uint32_t setUtcDateTime (uint16_t year, uint16_t mon, uint16_t day, uint16_t hh, uint16_t mm, uint16_t ss ){

	struct tm dt;	
	dt.tm_year = year - 1900;
	dt.tm_mon = mon - 1;
	dt.tm_mday = day;
	dt.tm_hour = hh;
	dt.tm_min = mm;
	dt.tm_sec = ss;	
	time_t utcTime = mktime(&dt);
	return (uint32_t)utcTime;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint32_t getDateTimeValue(char* buff){
	struct tm dt = *localtime((uint32_t*)&timeRecord.dateTime);
	return sprintf(buff, "%4d/%.2d/%.2d;%.2d:%.2d:%.2d", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
}

/***************************************************************************************************************


****************************************************************************************************************/
int16_t getAdcResult(TAdcResult ar){

	uint32_t tmpAdcReg = pADI_ADC0->CON & 0xFFFFFC00;
	uint16_t chanNum = ar;
	switch (chanNum){

		case AR_CHAN_1:
			tmpAdcReg |= ADC0CON_ADCCP_AIN0 | ADC0CON_ADCCN_AIN1;									// set for ADC0: AIN0 as POS input and as AIN1 as NEG input
			pADI_GP0->GPSET = SEL_1;																							// out HIGH to SEL_1
			break;
		
		case AR_CHAN_2:
			tmpAdcReg |= ADC0CON_ADCCP_AIN2 | ADC0CON_ADCCN_AIN3;									// set for ADC0: AIN2 as POS input and as AIN3 as NEG input
			pADI_GP0->GPSET = SEL_2;																							// out HIGH to SEL_2
			break;
		
		case AR_CHAN_3:
			tmpAdcReg |= ADC0CON_ADCCP_AIN8 | ADC0CON_ADCCN_AIN9;									// set for ADC0: AIN8 as POS input and as AIN9 as NEG input
			pADI_GP0->GPSET = SEL_1 | SEL_2;																			// out HIGH to SEL_1 and SEL_2
			break;	
	}

	uint32_t waitTime = 32000;
	while (waitTime){waitTime--;}
	
	pADI_ADC0->CON = tmpAdcReg;
	Flags.ADC0_READY = 0;
	uint16_t tmp = pADI_ADC0->MDE & ~ADC0MDE_ADCMD_MSK; 
	pADI_ADC0->MDE = (tmp | ADC0MDE_ADCMD_SINGLE);	
	while(Flags.ADC0_READY == 0){}

	int64_t result = (int64_t)resultAdc0;
//	result = result * 1200000 / 268435455;
	result = NormalizationADC0(result); 
	pADI_GP0->GPCLR = SEL_1 | SEL_2;																					// out LOW to SEL_1 and SEL_2

	return (int16_t)result;
}

/***************************************************************************************************************


****************************************************************************************************************/
int16_t getTemperature(void){

	Flags.ADC1_READY = 0;
	uint16_t tmp = pADI_ADC1->MDE & 0xFFF8; 
	pADI_ADC1->MDE = (tmp | ADC1MDE_ADCMD_SINGLE);
	while(Flags.ADC1_READY == 0){}

	int64_t result = (int64_t)resultAdc1;
	result = result * 1200000 / 268435455;
	result = (result - 82100) + 25000;
	return (int16_t)(result / 1000); 
}
 
/***************************************************************************************************************


****************************************************************************************************************/
void uartSendPacket(void){	
	Flags.UART_BUSY = 1;
	pUartTxBuffer = (char*)&uartTxBuffer;																			// set the pointer to the beginning of the buffer
	pADI_UART->COMIEN |= COMIEN_ETBEI;																				// enable Tx interrupt
}

/***************************************************************************************************************


****************************************************************************************************************/
void spiStartTransfer(uint8_t* buff, enSpiInstr_t instr, uint32_t data, uint16_t txCount, uint16_t rxCount){

	while (Flags.SPI_BUSY == 1){}																							// wait while SPI busy
	while (pADI_SPI0->SPISTA & SPISTA_TXFSTA_MSK){}														// wait while TX_FIFO emty
	pSpiBuffer = buff;																												// set the pointer to the beginning of the buffer
	*pSpiBuffer++ = (uint8_t)instr;																						// write instruction to buffer
	*pSpiBuffer++ = (uint8_t)(data >> 16);																		// write bytes to Flash address positions
	*pSpiBuffer++ = (uint8_t)(data >> 8);
	*pSpiBuffer 	= (uint8_t)data;
	pSpiBuffer = buff;																												// set pointer to buffer begin
	spiTxCount = txCount;																											// remember RX and TX bytes count
	spiRxCount = rxCount;
	Flags.SPI_BUSY = 1;																												// set SPI_BUSY flag
	spiMode = SPI_TX;																													// set Spi mode to SPI_TX
	pADI_SPI0->SPICON |= SPICON_RFLUSH;																				// set the ban on receiving bytes to the RX_FIFO buffer
	pADI_SPI0->SPITX = *pSpiBuffer++;																					// load first byte to spi transmitter		
}

/***************************************************************************************************************


****************************************************************************************************************/
void getSensorData(void){
		
	enableADC();
	dataArray[3] = 0x00FF & (getTemperature() << 1);
	dataArray[0] = getAdcResult(AR_CHAN_1);
	dataArray[1] = getAdcResult(AR_CHAN_2);
	dataArray[2] = getAdcResult(AR_CHAN_3);
	disableADC();
	
	if (Flags.ALARM_LIMITS){dataArray[3] |= 0x0001;}
		else {dataArray[3] &= 0xFFFE;}

	if (!Flags.TEST_MODE){
		
		uint16_t error = ((dataArray[0] > limitsArrayPos[0]) || (dataArray[0] < (limitsArrayNeg[0]))) ? 1 : 0;
		error += ((dataArray[1] > limitsArrayPos[1]) || (dataArray[1] < (limitsArrayNeg[1]))) ? 1 : 0;
		error += ((dataArray[2] > limitsArrayPos[2]) || (dataArray[2] < (limitsArrayNeg[2]))) ? 1 : 0;
		
		Flags.ALARM_LIMITS = (error) ? 1 : 0;
		interval = (Flags.ALARM_LIMITS) ? periodsArray[0] : periodsArray[1];  
  }		
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t storeNewData(void){

	lastRecordIndex = recordsCount;
	
	if (recordsCount < RECORDS_ON_PAGE) {
		
		sPage.buffer[recordsCount].data_0 = *pStoreRecord++;
		sPage.buffer[recordsCount].data_1 = *pStoreRecord++;
		sPage.buffer[recordsCount].data_2 = *pStoreRecord++;
		sPage.buffer[recordsCount].data_3.word = *pStoreRecord;
		recordsCount++; 
		
	}
	
	return (recordsCount == RECORDS_ON_PAGE) ? 1 : 0;	
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t executeCommand(char* buff){
	char* p = strstr(buff, devAddress);
	if(p == NULL){return 0;}
	if(p != buff){return 0;}
	p = strchr(buff, ';');
	if((p - buff) != sizeof(devAddress)){return 0;}
	if(strstr(p, setTime) != NULL){return 2;}
	if(strstr(p, getTime) != NULL){return 3;}
	if(strstr(p, getData) != NULL){return 4;}
	if(strstr(p, clrFlash) != NULL){return 5;}
	if(strstr(p, setLimitsNeg) != NULL){return 6;}
	if(strstr(p, setLimitsPos) != NULL){return 7;}	
	if(strstr(p, getLimitsNeg) != NULL){return 8;}	
	if(strstr(p, getLimitsPos) != NULL){return 9;}
	if(strstr(p, setPeriod) != NULL){return 10;}
	if(strstr(p, getPeriod) != NULL){return 11;}
	if(strstr(p, testSet) != NULL){return 12;}
	if(strstr(p, testReset) != NULL){return 13;}
	return 1;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	setLoggerTime(void){
	
	pUartRxBuffer = strchr((char*)&uartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}		
	uint16_t year = atoi((pUartRxBuffer + 1));
	if((year < 1900) || (year > 2105)) {return 1;}
	
	pUartRxBuffer = strchr(pUartRxBuffer, '/');
	if (pUartRxBuffer == NULL){return 1;}				
	uint16_t mon = atoi((pUartRxBuffer + 1));
	if((mon < 1) || (mon > 12)) {return 1;}

	pUartRxBuffer = strchr(pUartRxBuffer + 1, '/');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t day = atoi((pUartRxBuffer + 1));
	if((day < 1) || (day > 31)) {return 1;} 
	
	pUartRxBuffer = strchr(pUartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t hh = atoi((pUartRxBuffer + 1));
	if(hh > 23) {return 1;} 

	pUartRxBuffer = strchr(pUartRxBuffer, ':');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t mm = atoi((pUartRxBuffer + 1)); 
	if(mm > 59) {return 1;} 
	
	pUartRxBuffer = strchr(pUartRxBuffer, ':');
	if (pUartRxBuffer == NULL){return 1;}
	uint16_t ss = atoi((pUartRxBuffer + 25));
	if(ss > 59) {return 1;} 

	T2CON_ENABLE_BBA = 0; 																										// disable the RTC timer	
	Flags.GET_SENSOR_DATA = 0;
	timeRecord.dateTime = setUtcDateTime(year, mon, day, hh, mm, ss);
	pADI_WUT->T2CLRI = T2CLRI_WUFD;																						// clear interrupt flag	
	T2CON_ENABLE_BBA = 1;																											// enable the RTC timer	
	Flags.UPDATE_RTC = 0;
	sendMessage((char*)okStr);
	pUartRxBuffer = (char*)&uartRxBuffer;																			// set the pointer to the beginning of the RX buffer

	pStoreRecord = (uint16_t*)&timeRecord;																		// set a pointer to the data to be stored
	Flags.STORE_DATA = 1;

	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	getLoggerTime(void){
	pUartTxBuffer = (char*)&uartTxBuffer;
	pUartTxBuffer += getDateTimeValue(pUartTxBuffer);
	*pUartTxBuffer++ = '\r';
	*pUartTxBuffer = '\n';
	uartSendPacket();
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	getLoggerData(void){
	sendsRecordsCount = 0;
	Flags.SEND_ARRAY = createDataRecord();
	uartSendPacket();
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t createDataRecord(void){
		
	struct tm dt;
	uint32_t pagesTransmitted;
	uint32_t flashAddr;
	uint32_t flashRecords = pageCount * RECORDS_ON_PAGE;
	uint16_t loopVar;
	uint16_t offset;

	while (Flags.SPI_BUSY){}

	do {
	
		if (flashRecords > sendsRecordsCount){
		
			pagesTransmitted = sendsRecordsCount / RECORDS_ON_PAGE;	
			flashAddr = (pagesTransmitted * 256) + ((sendsRecordsCount - (pagesTransmitted * RECORDS_ON_PAGE)) * RECORDS_SIZE);		
			spiStartTransfer((uint8_t*)&spiBuffer, READ_FLASH, flashAddr, 4, RECORDS_SIZE);
			while (Flags.SPI_BUSY){}
		
			loopVar = decodeRecordData((TDataRecord*)&spiBuffer);

		} else {
		
				offset = sendsRecordsCount - flashRecords;
				if(offset < recordsCount){
					loopVar = decodeRecordData((TDataRecord*)&sPage.buffer[offset]); 	
				}
			}

	} while(loopVar);

	dt = *localtime((uint32_t*)&sendPaket.dateTime);
	sprintf((char*)&uartTxBuffer, "%4d/%.2d/%.2d;%.2d:%.2d:%.2d;", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
	pUartTxBuffer = (char*)&uartTxBuffer + 20;
	sprintf(pUartTxBuffer,"%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;%i;\r\n", sendPaket.temperature, sendPaket.sensor_1, sendPaket.sensor_2, sendPaket.sensor_3,
					sendPaket.limitNeg_1, sendPaket.limitNeg_2, sendPaket.limitNeg_3, sendPaket.limitPos_1, sendPaket.limitPos_2, sendPaket.limitPos_3,
					sendPaket.bigPeriod, sendPaket.smallPeriod);
	return (sendsRecordsCount < (flashRecords + recordsCount)) ? 1 : 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	decodeRecordData(TDataRecord* pDataRecord){

uint16_t result = 1;

	switch(pDataRecord->data_3.LoByte){
			
		case RT_TIME_INTERVAL:
			sendPaket.smallPeriod = pDataRecord->data_0;
			sendPaket.bigPeriod = pDataRecord->data_1; 
			break;
						
		case RT_LIMITS_POS:
			sendPaket.limitPos_1 = pDataRecord->data_0;
			sendPaket.limitPos_2 = pDataRecord->data_1; 
			sendPaket.limitPos_3 = pDataRecord->data_2;
			break;
			
		case RT_LIMITS_NEG:
			sendPaket.limitNeg_1 = pDataRecord->data_0;
			sendPaket.limitNeg_2 = pDataRecord->data_1; 
			sendPaket.limitNeg_3 = pDataRecord->data_2;				
			break;
			
		case RT_DATE_TIME:
			sendPaket.dateTime = *((uint32_t*)&pDataRecord->data_0);
			break;
			
		default:
			sendPaket.sensor_1 = pDataRecord->data_0;
			sendPaket.sensor_2 = pDataRecord->data_1;
			sendPaket.sensor_3 = pDataRecord->data_2;
			sendPaket.dateTime += (pDataRecord->data_3.LoByte & 0x0001) ? periodsArray[0] : periodsArray[1];
			sendPaket.temperature = pDataRecord->data_3.LoByte;
			result = 0;
			break;		
	}

	sendsRecordsCount++;
	return result;
}



/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	sendTestData(void){
	struct tm dt;
	unData_t* pUnData = (unData_t*)&dataArray[3];
	dt = *localtime((uint32_t*)&timeRecord.dateTime);
	sprintf((char*)&uartTxBuffer, "%4d/%.2d/%.2d;%.2d:%.2d:%.2d;", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
	pUartTxBuffer = (char*)&uartTxBuffer + 20;
	sprintf(pUartTxBuffer,"%i;%i;%i;%i\r\n", pUnData->LoByte / 2, dataArray[0], dataArray[1], dataArray[2]);
	uartSendPacket();
	return 0;
}


/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	clrLoggerFlash(void){

	flashWriteEnable();
	writeFlashRegister(ERASE_CHIP);	
	flashWriteDisable();
	recordsCount = 0;
	pageCount = 0;
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	setLoggerLimitsNeg(void){
	
	pUartRxBuffer = strchr((char*)&uartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}	

	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}		
	limitsArrayNeg[0] = (int16_t)atoi((pUartRxBuffer + 1));
	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}
	limitsArrayNeg[1] = (int16_t)atoi((pUartRxBuffer + 1));
	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}				
	limitsArrayNeg[2] = (int16_t)atoi((pUartRxBuffer + 1));

	pStoreRecord = (uint16_t*)&limitsArrayNeg;																// set a pointer to the data to be stored
	Flags.STORE_DATA = 1;
	
	sendMessage((char*)okStr);
	pUartRxBuffer = (char*)&uartRxBuffer;																			// set the pointer to the beginning of the RX buffer
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	setLoggerLimitsPos(void){
	
	pUartRxBuffer = strchr((char*)&uartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}	

	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}		
	limitsArrayPos[0] = (int16_t)atoi((pUartRxBuffer + 1));
	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}
	limitsArrayPos[1] = (int16_t)atoi((pUartRxBuffer + 1));
	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}				
	limitsArrayPos[2] = (int16_t)atoi((pUartRxBuffer + 1));
	
	pStoreRecord = (uint16_t*)&limitsArrayPos;																// set a pointer to the data to be stored
	Flags.STORE_DATA = 1;
	
	sendMessage((char*)okStr);
	pUartRxBuffer = (char*)&uartRxBuffer;																			// set the pointer to the beginning of the RX buffer
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t	setLoggerPeriod(void){
	uint16_t i1;
	uint16_t i2;
	
	pUartRxBuffer = strchr((char*)&uartRxBuffer, ';');
	if (pUartRxBuffer == NULL){return 1;}	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}		
	i1 = (int16_t)atoi((pUartRxBuffer + 1));
	if(i1 < 2){return 1;}
	
	pUartRxBuffer = strchr(pUartRxBuffer + 1, ';');
	if (pUartRxBuffer == NULL){return 1;}				
	i2 = (int16_t)atoi((pUartRxBuffer + 1));
	if(i2 < 1 ){return 1;}
	if(i2 >= i1 ){return 1;}
	
	periodsArray[0] = i1;
	periodsArray[1] = i2;
  sendMessage((char*)okStr);
	pUartRxBuffer = (char*)&uartRxBuffer;																	// set the pointer to the beginning of the RX buffer
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t getLoggerLimitsNeg(void){
	sprintf((char*)&uartTxBuffer, "%d;%d;%d\r\n", limitsArrayNeg[0], limitsArrayNeg[1],  limitsArrayNeg[2]);
	uartSendPacket();
	return 0;
} 

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t getLoggerLimitsPos(void){
	sprintf((char*)&uartTxBuffer, "%d;%d;%d\r\n", limitsArrayPos[0], limitsArrayPos[1], limitsArrayPos[2]);
	uartSendPacket();
	return 0;
} 

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t getLoggerPeriod(void){
	sprintf((char*)&uartTxBuffer, "%d;%d\r\n", periodsArray[0], periodsArray[1]);
	uartSendPacket();
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t setTestMode(void){
	Flags.TEST_MODE = 1;
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t resetTestMode(void){
	Flags.TEST_MODE = 0;
	sendMessage((char*)okStr);
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t 	addrError(void){
	pUartRxBuffer = (char*)&uartRxBuffer;																	// set the pointer to the beginning of the RX buffer
	pADI_UART->COMIEN = COMIEN_ERBFI | COMIEN_ELSI; 									// enable RX interrupts)
	return 0;
}

/***************************************************************************************************************


****************************************************************************************************************/
uint16_t 	cmdError(void){
	return 1;
}

/***************************************************************************************************************


****************************************************************************************************************/
void	sendMessage(char* str){
	sprintf((char*)&uartTxBuffer, "%s\r\n", str);
	uartSendPacket();
}

/***************************************************************************************************************


****************************************************************************************************************/
uint8_t readFlashRegister(enSpiInstr_t reg){
	spiStartTransfer((uint8_t*)&spiBuffer, reg, 0, 1, 1);
	while (Flags.SPI_BUSY == 1){}
	pSpiBuffer = (uint8_t*)&spiBuffer;		
	return *pSpiBuffer;
}

/***************************************************************************************************************


****************************************************************************************************************/
void writeFlashRegister(enSpiInstr_t reg){
	spiStartTransfer((uint8_t*)&spiBuffer, reg, 0, 1, 0);
	while (Flags.SPI_BUSY == 1){}
}

/***************************************************************************************************************


****************************************************************************************************************/
void flashWriteEnable(void){
	while (readFlashRegister(READ_STATUS) & FLASH_STATUS_WIP);
	writeFlashRegister(WRITE_ENABLE);
	while ((readFlashRegister(READ_STATUS) & FLASH_STATUS_WEL) != FLASH_STATUS_WEL);
}

/***************************************************************************************************************


****************************************************************************************************************/
void flashWriteDisable(void){
	while (readFlashRegister(READ_STATUS) & FLASH_STATUS_WIP);
	writeFlashRegister(WRITE_DISABLE);
}

