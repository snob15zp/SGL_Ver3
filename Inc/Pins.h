
#ifndef __Pins_h
#define __Pins_h

#define SEL_2								((uint8_t) 1<<0)						// for pin P0.0
#define RxD                 ((uint8_t) 1<<1)            // for pin P0.1
#define TxD                 ((uint8_t) 1<<2)            // for pin P0.2
#define SEL_1								((uint8_t) 1<<3)						// for pin P0.3
#define CPU_FET							((uint8_t) 1<<4)						// for P0.4
#define VEXT_PRES						((uint8_t) 1<<5	)					// for pin P0.5
#define MISO                ((uint8_t) 1<<4	)         // for pin P1.4
#define SCK                 ((uint8_t) 1<<5	)         // for pin P1.5
#define MOSI                ((uint8_t) 1<<6	)         // for pin P1.6
#define SPI0_CS							((uint8_t) 1<<7	)					// for pin P1.7
#define	CNTR_PER						((uint8_t) 1<<0	)					// for pin P2.0	

void PinAll_Init(void); //must be call before any setting; turn off all circuit outside microcontroller

void PinCntrlPer_On(void);  // for turn on any circuit outside microcontroller; must be call before any command to outside circuit 
void PinCntrlPer_Off(void);	// for turn off any circuit outside microcontroller; before this must be call _Off function for active outside circuit

extern void PinSPI_On(void); // for flash, Resistor 1K must be install on MISO 
extern void PinSPI_Off(void);

extern void PinRTxD_On(void); //for uart
extern void PinRTxD_Off(void);

extern void PinSel_On(void); // for load sell
extern void PinSel_Off(void); 


	
#endif
