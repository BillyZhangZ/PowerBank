;/*****************************************************************************
; * @file:    startup_MKL02Z4.s
; * @purpose: CMSIS Cortex-M0plus Core Device Startup File
; *           MKL02Z4
; * @version: 1.1
; * @date:    2013-4-5
; *----------------------------------------------------------------------------
; *
; * Copyright: 1997 - 2013 Freescale Semiconductor, Inc. All Rights Reserved.
; *
; ******************************************************************************/


;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        EXTERN  SystemInit
        EXTERN  g_bootloaderTree
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        PUBLIC  __Vectors
        PUBLIC  __Vectors_End
        PUBLIC  __Vectors_Size

        DATA

__vector_table
        DCD     sfe(CSTACK)
        DCD     Reset_Handler

        DCD     NMI_Handler
        DCD     HardFault_Handler
        DCD     0
        DCD     0
        DCD     0
__vector_table_0x1c
        DCD     g_bootloaderTree
        DCD     0
        DCD     0
        DCD     0
        DCD     SVC_Handler
        DCD     0
        DCD     0
        DCD     PendSV_Handler
        DCD     SysTick_Handler

        ; External Interrupts
        DCD     Reserved16_IRQHandler  ; Reserved interrupt 16
        DCD     Reserved17_IRQHandler  ; Reserved interrupt 17
        DCD     Reserved18_IRQHandler  ; Reserved interrupt 18
        DCD     Reserved19_IRQHandler  ; Reserved interrupt 19
        DCD     Reserved20_IRQHandler  ; Reserved interrupt 20
        DCD     FTFA_IRQHandler  ; FTFA command complete/read collision interrupt
        DCD     LVD_LVW_IRQHandler  ; Low Voltage Detect, Low Voltage Warning
        DCD     Reserved23_IRQHandler  ; Reserved interrupt 23
        DCD     I2C0_IRQHandler  ; I2C0 interrupt
        DCD     I2C1_IRQHandler  ; I2C1 interrupt
        DCD     SPI0_IRQHandler  ; SPI0 interrupt
        DCD     Reserved27_IRQHandler  ; Reserved interrupt 27
        DCD     UART0_IRQHandler  ; UART0 status/error interrupt
        DCD     Reserved29_IRQHandler  ; Reserved interrupt 29
        DCD     Reserved30_IRQHandler  ; Reserved interrupt 30
        DCD     ADC0_IRQHandler  ; ADC0 interrupt
        DCD     CMP0_IRQHandler  ; CMP0 interrupt
        DCD     TPM0_IRQHandler  ; TPM0 fault, overflow and channels interrupt
        DCD     TPM1_IRQHandler  ; TPM1 fault, overflow and channels interrupt
        DCD     Reserved35_IRQHandler  ; Reserved interrupt 35
        DCD     Reserved36_IRQHandler  ; Reserved interrupt 36
        DCD     Reserved37_IRQHandler  ; Reserved interrupt 37
        DCD     Reserved38_IRQHandler  ; Reserved interrupt 38
        DCD     Reserved39_IRQHandler  ; Reserved interrupt 39
        DCD     Reserved40_IRQHandler  ; Reserved interrupt 40
        DCD     Reserved41_IRQHandler  ; Reserved interrupt 41
        DCD     Reserved42_IRQHandler  ; Reserved interrupt 42
        DCD     MCG_IRQHandler  ; MCG interrupt
        DCD     LPTimer_IRQHandler  ; LPTimer interrupt
        DCD     Reserved45_IRQHandler  ; Reserved interrupt 45
        DCD     PORTA_IRQHandler  ; Port A interrupt
        DCD     PORTB_IRQHandler  ; Port B interrupt
__Vectors_End

__Vectors       EQU   __vector_table
__Vectors_Size  EQU   __Vectors_End - __Vectors

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Flash configuration field.
;;
#ifdef BL_TARGET_FLASH
__flash_config
        DCD     0xFFFFFFFF  ; 0x400
        DCD     0xFFFFFFFF  ; 0x404
        DCD     0xFFFFFFFF  ; 0x408
        DCD     0xFFFFFFFE  ; 0x40c, FSEC=0xFE
#endif

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;
        THUMB

        PUBWEAK Reset_Handler
        SECTION .text:CODE:REORDER(2)
Reset_Handler
        B       .
;        LDR     R0, =SystemInit
;        BLX     R0
;        LDR     R0, =__iar_program_start
;        BX      R0

        PUBWEAK NMI_Handler
        SECTION .text:CODE:REORDER(1)
NMI_Handler
        B .

        PUBWEAK HardFault_Handler
        SECTION .text:CODE:REORDER(1)
HardFault_Handler
        B .

        PUBWEAK SVC_Handler
        SECTION .text:CODE:REORDER(1)
SVC_Handler
        B .

        PUBWEAK PendSV_Handler
        SECTION .text:CODE:REORDER(1)
PendSV_Handler
        B .

        PUBWEAK SysTick_Handler
        SECTION .text:CODE:REORDER(1)
SysTick_Handler
        B .


        PUBWEAK     Reserved16_IRQHandler  ; Reserved interrupt 16
        PUBWEAK     Reserved17_IRQHandler  ; Reserved interrupt 17
        PUBWEAK     Reserved18_IRQHandler  ; Reserved interrupt 18
        PUBWEAK     Reserved19_IRQHandler  ; Reserved interrupt 19
        PUBWEAK     Reserved20_IRQHandler  ; Reserved interrupt 20
        PUBWEAK     FTFA_IRQHandler  ; FTFA command complete/read collision interrupt
        PUBWEAK     LVD_LVW_IRQHandler  ; Low Voltage Detect, Low Voltage Warning
        PUBWEAK     Reserved23_IRQHandler  ; Reserved interrupt 23
        PUBWEAK     I2C0_IRQHandler  ; I2C0 interrupt
        PUBWEAK     I2C1_IRQHandler  ; I2C1 interrupt
        PUBWEAK     SPI0_IRQHandler  ; SPI0 interrupt
        PUBWEAK     Reserved27_IRQHandler  ; Reserved interrupt 27
        PUBWEAK     UART0_IRQHandler  ; UART0 status/error interrupt
        PUBWEAK     Reserved29_IRQHandler  ; Reserved interrupt 29
        PUBWEAK     Reserved30_IRQHandler  ; Reserved interrupt 30
        PUBWEAK     ADC0_IRQHandler  ; ADC0 interrupt
        PUBWEAK     CMP0_IRQHandler  ; CMP0 interrupt
        PUBWEAK     TPM0_IRQHandler  ; TPM0 fault, overflow and channels interrupt
        PUBWEAK     TPM1_IRQHandler  ; TPM1 fault, overflow and channels interrupt
        PUBWEAK     Reserved35_IRQHandler  ; Reserved interrupt 35
        PUBWEAK     Reserved36_IRQHandler  ; Reserved interrupt 36
        PUBWEAK     Reserved37_IRQHandler  ; Reserved interrupt 37
        PUBWEAK     Reserved38_IRQHandler  ; Reserved interrupt 38
        PUBWEAK     Reserved39_IRQHandler  ; Reserved interrupt 39
        PUBWEAK     Reserved40_IRQHandler  ; Reserved interrupt 40
        PUBWEAK     Reserved41_IRQHandler  ; Reserved interrupt 41
        PUBWEAK     Reserved42_IRQHandler  ; Reserved interrupt 42
        PUBWEAK     MCG_IRQHandler  ; MCG interrupt
        PUBWEAK     LPTimer_IRQHandler  ; LPTimer interrupt
        PUBWEAK     Reserved45_IRQHandler  ; Reserved interrupt 45
        PUBWEAK     PORTA_IRQHandler  ; Port A interrupt
        PUBWEAK     PORTB_IRQHandler  ; Port B interrupt

        SECTION .text:CODE:REORDER(2)

Reserved16_IRQHandler
Reserved17_IRQHandler
Reserved18_IRQHandler
Reserved19_IRQHandler
Reserved20_IRQHandler
FTFA_IRQHandler
LVD_LVW_IRQHandler
Reserved23_IRQHandler
I2C0_IRQHandler
I2C1_IRQHandler
SPI0_IRQHandler
Reserved27_IRQHandler
UART0_IRQHandler
Reserved29_IRQHandler
Reserved30_IRQHandler
ADC0_IRQHandler
CMP0_IRQHandler
TPM0_IRQHandler
TPM1_IRQHandler
Reserved35_IRQHandler
Reserved36_IRQHandler
Reserved37_IRQHandler
Reserved38_IRQHandler
Reserved39_IRQHandler
Reserved40_IRQHandler
Reserved41_IRQHandler
Reserved42_IRQHandler
MCG_IRQHandler
LPTimer_IRQHandler
Reserved45_IRQHandler
PORTA_IRQHandler
PORTB_IRQHandler
        B       .


        END
