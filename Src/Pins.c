/*********************************************************************************************************



                                          P I N   S E T U P



*********************************************************************************************************/

#include "Pins.h"
#include <ADUCM360.h>


/*
port          P0. 0   1   2   3     4      5   6  7  P1. 0  1  2  3  4    5   6    7 P2. 0     1   2
destination     sel2 Rxd TxD Sel1 CPUFET Vpres                      Miso SCK MOSI CS  CntrlPer 
Load Cell         *           *                                                           *
Connection            *   *         *      *                                              
Flash                                                                *    *   *    *
Work In/Out       O   I   O   O    OD      I                         I    O   O   OD     OD
HALT In/Out       O  OCP OCP OD     O      I                        OCP  OD  OD   OD     OD
Work PU           1   x   x   1     0      0                         x    x   x    x      0
Work OEN          1   x   x   1     1      0                         x    x   x    x      1
Work OCE          1   x   x   1     1      0                         x    x   x    x      1
WORK OUT         0/1  x   x  0/1   0/1     x                         x    x   x    x     0/1
HALT PU           0   Y   Y   0     0      0       1                 1    0   0    0      0
HALT OEN          1   0   0   1     1      0       0                 0    1   1    1      1
HALT OCE          1   1   1   1     1      0       1                 1    1   1    1      1
HALT OUT          0   0   0   0     1      x       x                 x    0   0    1      1
*/

//  ***************************************** NC Pins **************************************************

void PinNC_Init(void)
{
	// p0.6 p0.7
	#define p067 (0b11000000)
	pADI_GP0->GPOEN &= ~p067;
	pADI_GP0->GPOCE |= p067;
	pADI_GP0->GPPUL |= p067;
	//pADI_GP0->GPPUL &= ~p067;
	
	// p1.0 p1.1 p1.2 p1.3
	#define p10123 (0b00001111)
	pADI_GP1->GPOEN &= ~p10123;
	pADI_GP1->GPOCE |= p10123;
	pADI_GP1->GPPUL |= p10123;
	//pADI_GP1->GPPUL &= ~p10123;
	
	// p2.1 p2.2
	#define p212 (0b00000110)
	//#define p212 (0b11111110)
	pADI_GP2->GPOEN &= ~p212;
	pADI_GP2->GPOCE |= p212;
	pADI_GP2->GPPUL |= p212;
	//pADI_GP1->GPPUL &= ~ p10123;
	
}

//******************************************* SEL ********************************************************

void PinSel_Init(void)
{
	pADI_GP0->GPCLR = SEL_2 | SEL_1;
	pADI_GP0->GPOCE&= ~(SEL_2 | SEL_1);
	pADI_GP0->GPOEN|= SEL_2 | SEL_1;
  pADI_GP0->GPPUL&= ~(SEL_2 | SEL_1);
	//pADI_GP0->GPPUL|= (SEL_2 | SEL_1);

}

void PinSel_Off(void)
{
	PinSel_Init();
}
void PinSel_On(void)
{
	pADI_GP0->GPCLR = SEL_2 | SEL_1;
}
	
//********************************************** TxD RxD *****************************************************

void PinRTxD_Init(void)
{
	pADI_GP0->GPOEN&= ~( TxD ); 
	pADI_GP0->GPOCE|= (TxD);
	pADI_GP0->GPPUL|= (TxD);

	pADI_GP0->GPOEN&= ~(RxD ); 
	pADI_GP0->GPOCE&= ~(RxD );
	pADI_GP0->GPPUL&= ~(RxD );

}

void PinRTxD_On(void)
{
	pADI_GP0->GPCON |= GP0CON_CON1_UARTRXD | GP0CON_CON2_UARTTXD;					// configure pins P0.1 and P0.2 for UART
}

void PinRTxD_Off(void)
{
	pADI_GP0->GPCON &= ~(GP0CON_CON1_UARTRXD | GP0CON_CON2_UARTTXD);			// configure pins P0.1 and P0.2 for IO
}

//************************************** CPUFET ****************************************************************
//Do not used
void PinCPUFET_Init(void) //Set up an open drain and sit in this mode 
{
	//pADI_GP0->GPOCE|= CPU_FET;
	//pADI_GP0->GPOEN|= (CPU_FET); 
	//pADI_GP0->GPSET = CPU_FET;
	//pADI_GP0->GPPUL&= ~CPU_FET;
	////pADI_GP0->GPPUL|=CPU_FET;
	
	pADI_GP0->GPOEN&= ~(MISO); 
	pADI_GP0->GPOCE|= MISO;
	pADI_GP0->GPPUL|= MISO;
}
void PinCPUFET_On(void) { 	/*PinCPUFET_Init();*/ }
void PinCPUFET_Off(void) {	PinCPUFET_Init();   }

/*************************************** Vpres ******************************************************************/

void PinVpres_Init(void) //Set up an open drain and sit in this mode 
{
	pADI_GP0->GPOCE&= ~VEXT_PRES;
	pADI_GP0->GPOEN&= ~VEXT_PRES; 
	  pADI_GP0->GPPUL&= ~VEXT_PRES;

}
void PinVpres_On(void) {	/*PinVpres_Init(); */}
void PinVpres_Off(void){ 	/*PinVpres_Init();*/ }
	
//*************************************** MISO *****************************************************************

void PinMISO_Init(void) //Set up an open drain and sit in this mode 
{
	pADI_GP1->GPOEN|=(MISO); 
  pADI_GP1->GPCLR=MISO;
	//pADI_GP1->GPOCE|= (MISO);
	//pADI_GP1->GPOCE|= MISO;
	//pADI_GP1->GPPUL|= MISO;
	pADI_GP1->GPPUL&= ~MISO;
}
void PinMISO_On(void) 
{
	pADI_GP1->GPCON |=GP1CON_CON4_SPI0MISO; 	// configure  for SPI0
}
void PinMISO_Off(void)
{PinMISO_Init();
 pADI_GP1->GPCON &=~(GP1CON_CON4_SPI0MISO);  // 00 - GPIO
}

//*************************************** MOSI SCK ***************************************************************


void PinMOSISCK_Init(void) // 
{
	pADI_GP1->GPCLR = SCK | MOSI;
	pADI_GP1->GPOCE|= (SCK | MOSI);
	pADI_GP1->GPOEN|= SCK | MOSI;
	pADI_GP1->GPPUL&= ~(SCK | MOSI);
	//pADI_GP1->GPPUL|=(SCK | MOSI);
}
void PinMOSISCK_On(void) 
{
	pADI_GP1->GPCON |= GP1CON_CON6_SPI0MOSI | GP1CON_CON5_SPI0SCLK; 	// configure  for SPI0
}
void PinMOSISCK_Off(void)
{
 PinMOSISCK_Init();
 pADI_GP1->GPCON &=~( GP1CON_CON6_SPI0MOSI | GP1CON_CON5_SPI0SCLK);  // 00 - GPIO
}

//***************************************** CS *****************************************************************

void PinCS_Init(void) // 
{
	pADI_GP1->GPSET = SPI0_CS;
	pADI_GP1->GPOCE|= (SPI0_CS);
	pADI_GP1->GPOEN|= SPI0_CS;
	//pADI_GP1->GPPUL&= ~(SPI0_CS);
	pADI_GP1->GPPUL|=(SPI0_CS);
}

void PinCS_On(void) 
{
	pADI_GP1->GPCON |=  GP1CON_CON7_SPI0CS; 	// configure  for SPI0
}

void PinCS_Off(void)
{
 PinCS_Init();
 pADI_GP1->GPCON &=~(  GP1CON_CON7_SPI0CS);  // 00 - GPIO
}

//********************************************** CntrlPer ******************************************************

void PinCntrlPer_Init(void) // 
{
	pADI_GP2->GPSET = CNTR_PER;
	pADI_GP2->GPOCE|= (CNTR_PER);
	pADI_GP2->GPOEN|= CNTR_PER;
	//pADI_GP2->GPPUL&= ~(CNTR_PER);
	pADI_GP2->GPPUL|=(CNTR_PER);
}
void PinCntrlPer_On(void)
{
	pADI_GP2->GPCLR = CNTR_PER;
};

void PinCntrlPer_Off(void)	
{
	pADI_GP2->GPSET = CNTR_PER;
};

//************************************************ Init All Pins **********************************************

void PinAll_Init(void)
{
	PinCntrlPer_Init();
  PinNC_Init();
	PinSel_Init();
	PinRTxD_Init();
	PinCPUFET_Init();
	PinVpres_Init();
	PinMISO_Init();
	PinMOSISCK_Init();
	PinCS_Init();
	
}

//************************************************ SPI_ON/OFF    **********************************************

void PinSPI_On(void)
{
	PinMISO_On();
	PinMOSISCK_On();
	PinCS_On();
}
void PinSPI_Off(void)
{
	PinMISO_Off();
	PinMOSISCK_Off();
	PinCS_Off();
}



