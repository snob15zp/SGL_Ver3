;/**************************************************************************//**
; * @file     startup_ADuCM360.s
; * @brief    CMSIS Cortex-M3 Core Device Startup File for
; *           Device ADuCM360
; * @version  V3.10
; * @date     23. November 2012
; *
; * @note
; *
; ******************************************************************************/
;/* Copyright (c) 2012 ARM LIMITED
;
;   All rights reserved.
;   Redistribution and use in source and binary forms, with or without
;   modification, are permitted provided that the following conditions are met:
;   - Redistributions of source code must retain the above copyright
;     notice, this list of conditions and the following disclaimer.
;   - Redistributions in binary form must reproduce the above copyright
;     notice, this list of conditions and the following disclaimer in the
;     documentation and/or other materials provided with the distribution.
;   - Neither the name of ARM nor the names of its contributors may be used
;     to endorse or promote products derived from this software without
;     specific prior written permission.
;   *
;   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
;   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;   POSSIBILITY OF SUCH DAMAGE.
;
;   Portions Copyright (c) 2017-2018 Analog Devices, Inc.
;   ---------------------------------------------------------------------------*/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/

; Amount of memory (in bytes) allocated for Stack
; Tailor this value to your application needs
; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000200

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit

                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp               ; Top of Stack
                DCD     Reset_Handler              ; Reset Handler
                DCD     NMI_Handler                ; The NMI handler        
                DCD     HardFault_Handler          ; The hard fault handler 
                DCD     MemManage_Handler          ; The MPU fault handler  
                DCD     BusFault_Handler           ; The bus fault handler  
                DCD     UsageFault_Handler         ; The usage fault handler
                DCD     0                          ; Reserved               
                DCD     0                          ; Reserved               
                DCD     0                          ; Reserved               
                DCD     0                          ; Reserved               
                DCD     SVC_Handler                ; SVCall handler         
                DCD     DebugMon_Handler           ; Debug monitor handler  
                DCD     0                          ; Reserved               
                DCD     PendSV_Handler             ; The PendSV handler     
                DCD     SysTick_Handler            ; The SysTick handler    

                ; External Interrupts
	        DCD     WakeUp_Int_Handler        ; Wake Up Timer              [ 0]
	        DCD     Ext_Int0_Handler          ; External Interrupt 0       [ 1]
	        DCD     Ext_Int1_Handler          ; External Interrupt 1       [ 2]
	        DCD     Ext_Int2_Handler          ; External Interrupt 2       [ 3]
	        DCD     Ext_Int3_Handler          ; External Interrupt 3       [ 4]
	        DCD     Ext_Int4_Handler          ; External Interrupt 4       [ 5]
	        DCD     Ext_Int5_Handler          ; External Interrupt 5       [ 6]
	        DCD     Ext_Int6_Handler          ; External Interrupt 6       [ 7]
	        DCD     Ext_Int7_Handler          ; External Interrupt 7       [ 8]
	        DCD     WDog_Tmr_Int_Handler      ; Watchdog timer handler     [ 9]
	        DCD     0                         ;                            [10]
	        DCD     GP_Tmr0_Int_Handler       ; General purpose timer 0    [11]
	        DCD     GP_Tmr1_Int_Handler       ; General purpose timer 1    [12]
	        DCD     ADC0_Int_Handler          ; ADC0 Interrupt             [13]
	        DCD     ADC1_Int_Handler          ; ADC1 Interrupt             [14]
	        DCD     SINC2_Int_Handler         ; SINC2 Interrupt            [15]
	        DCD     Flsh_Int_Handler          ; Flash Interrupt            [16]
	        DCD     UART_Int_Handler          ; UART0                      [17]
	        DCD     SPI0_Int_Handler          ; SPI 0                      [18]
	        DCD     SPI1_Int_Handler          ; SPI 1                      [19]
	        DCD     I2C0_Slave_Int_Handler    ; I2C0 Slave                 [20]
	        DCD     I2C0_Master_Int_Handler   ; I2C0 Master                [21]
	        DCD     DMA_Err_Int_Handler       ; DMA Error interrupt        [22]
	        DCD     DMA_SPI1_TX_Int_Handler   ; DMA SPI1 TX                [23]
	        DCD     DMA_SPI1_RX_Int_Handler   ; DMA SPI1 RX                [24]
	        DCD     DMA_UART_TX_Int_Handler   ; DMA UART TX                [25]
	        DCD     DMA_UART_RX_Int_Handler   ; DMA UART RX                [26]
	        DCD     DMA_I2C0_STX_Int_Handler  ; DMA I2C0 Slave TX          [27]
	        DCD     DMA_I2C0_SRX_Int_Handler  ; DMA I2C0 Slave RX          [28]
	        DCD     DMA_I2C0_MTX_Int_Handler  ; DMA I2C0 Master TX         [29]
	        DCD     DMA_I2C0_MRX_Int_Handler  ; DMA I2C0 Master RX         [30]
	        DCD     DMA_DAC_Out_Int_Handler   ; DMA DAC out                [31]
	        DCD     DMA_ADC0_Int_Handler      ; DMA ADC0                   [32]
	        DCD     DMA_ADC1_Int_Handler      ; DMA ADC1                   [33]
	        DCD     DMA_SINC2_Int_Handler     ; SINC2                      [34]
	        DCD     PWMTRIP_Int_Handler       ; PWMTRIP                    [35]
	        DCD     PWM0_Int_Handler          ; PWM0                       [36]
	        DCD     PWM1_Int_Handler          ; PWM1                       [37]
	        DCD     PWM2_Int_Handler          ; PWM2                       [38]
	        DCD     0                         ;                            [39]
__Vectors_End

__Vectors_Size  EQU  __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY

; Reset handler
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
        IMPORT  SystemInitForLowPower
        IMPORT  __main
                 LDR     R0, =SystemInitForLowPower
                 BLX     R0
                 LDR     R0, =__main
                 BX      R0
                 ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler                [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler          [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler          [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler           [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler                [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler           [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler             [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler            [WEAK]
                B       .
                ENDP

Default_Handler PROC








		EXPORT WakeUp_Int_Handler          [WEAK]
		EXPORT Ext_Int0_Handler            [WEAK]
		EXPORT Ext_Int1_Handler            [WEAK]
		EXPORT Ext_Int2_Handler            [WEAK]
		EXPORT Ext_Int3_Handler            [WEAK]
		EXPORT Ext_Int4_Handler            [WEAK]
		EXPORT Ext_Int5_Handler            [WEAK]
		EXPORT Ext_Int6_Handler            [WEAK]
		EXPORT Ext_Int7_Handler            [WEAK]
		EXPORT WDog_Tmr_Int_Handler        [WEAK]
		EXPORT GP_Tmr0_Int_Handler         [WEAK]
		EXPORT GP_Tmr1_Int_Handler         [WEAK]
		EXPORT ADC0_Int_Handler            [WEAK]
		EXPORT ADC1_Int_Handler            [WEAK]
		EXPORT SINC2_Int_Handler           [WEAK]
		EXPORT Flsh_Int_Handler            [WEAK]
		EXPORT UART_Int_Handler            [WEAK]
		EXPORT SPI0_Int_Handler            [WEAK]
		EXPORT SPI1_Int_Handler            [WEAK]
		EXPORT I2C0_Slave_Int_Handler      [WEAK]
		EXPORT I2C0_Master_Int_Handler     [WEAK]
		EXPORT DMA_Err_Int_Handler         [WEAK]
		EXPORT DMA_SPI1_TX_Int_Handler     [WEAK]
		EXPORT DMA_SPI1_RX_Int_Handler     [WEAK]
		EXPORT DMA_UART_TX_Int_Handler     [WEAK]
		EXPORT DMA_UART_RX_Int_Handler     [WEAK]
		EXPORT DMA_I2C0_STX_Int_Handler    [WEAK]
		EXPORT DMA_I2C0_SRX_Int_Handler    [WEAK]
		EXPORT DMA_I2C0_MTX_Int_Handler    [WEAK]
		EXPORT DMA_I2C0_MRX_Int_Handler    [WEAK]
		EXPORT DMA_DAC_Out_Int_Handler     [WEAK]
		EXPORT DMA_ADC0_Int_Handler        [WEAK]
		EXPORT DMA_ADC1_Int_Handler        [WEAK]
		EXPORT DMA_SINC2_Int_Handler       [WEAK]
		EXPORT PWMTRIP_Int_Handler         [WEAK]
		EXPORT PWM0_Int_Handler            [WEAK]
		EXPORT PWM1_Int_Handler            [WEAK]
		EXPORT PWM2_Int_Handler            [WEAK]


WakeUp_Int_Handler      
Ext_Int0_Handler        
Ext_Int1_Handler        
Ext_Int2_Handler        
Ext_Int3_Handler        
Ext_Int4_Handler        
Ext_Int5_Handler        
Ext_Int6_Handler        
Ext_Int7_Handler        
WDog_Tmr_Int_Handler    
GP_Tmr0_Int_Handler     
GP_Tmr1_Int_Handler     
ADC0_Int_Handler        
ADC1_Int_Handler        
SINC2_Int_Handler       
Flsh_Int_Handler        
UART_Int_Handler        
SPI0_Int_Handler        
SPI1_Int_Handler        
I2C0_Slave_Int_Handler  
I2C0_Master_Int_Handler 
DMA_Err_Int_Handler     
DMA_SPI1_TX_Int_Handler 
DMA_SPI1_RX_Int_Handler 
DMA_UART_TX_Int_Handler 
DMA_UART_RX_Int_Handler 
DMA_I2C0_STX_Int_Handler
DMA_I2C0_SRX_Int_Handler
DMA_I2C0_MTX_Int_Handler
DMA_I2C0_MRX_Int_Handler
DMA_DAC_Out_Int_Handler 
DMA_ADC0_Int_Handler    
DMA_ADC1_Int_Handler    
DMA_SINC2_Int_Handler   
PWMTRIP_Int_Handler     
PWM0_Int_Handler        
PWM1_Int_Handler        
PWM2_Int_Handler        
                B       .

                ENDP

                ALIGN

;*******************************************************************************
; User Stack and Heap initialization
;*******************************************************************************
                 IF      :DEF:__MICROLIB
                
                 EXPORT  __initial_sp
                 EXPORT  __heap_base
                 EXPORT  __heap_limit
                
                 ELSE
                
                 IMPORT  __use_two_region_memory
                 EXPORT  __user_initial_stackheap
                 
__user_initial_stackheap

                 LDR     R0, =  Heap_Mem
                 LDR     R1, =(Stack_Mem + Stack_Size)
                 LDR     R2, = (Heap_Mem +  Heap_Size)
                 LDR     R3, = Stack_Mem
                 BX      LR

                 ALIGN

                 ENDIF

                 END
