;*****************************************************************************
; In C code
; Prototype as
;  extern void AsmWFI(void);
; Call as
;  AsmWFI()

      AREA    |asmcode|, CODE, READONLY

AsmWFI          PROC
                EXPORT  AsmWFI

; Params 1 thru 4 in R0..R3
; Returns R0
             nop
             nop
             nop
             WFI
             nop
             nop
             BX LR               
                   ENDP ; AsmWFI
				   END	   

;*****************************************************************************