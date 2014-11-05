/*****************************************************************************/
/* startup_MKL02Z4.s: Startup file for MKL02Z4 device series                   */
/*****************************************************************************/
/* Version: CodeSourcery Sourcery G++ Lite (with CS3)                        */
/*****************************************************************************/


/*
//*** <<< Use Configuration Wizard in Context Menu >>> ***
*/


/*
// <h> Stack Configuration
//   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Stack_Size, 0x00000400
    .section ".stack", "w"
    .align  3
    .globl  __cs3_stack_mem
    .globl  __cs3_stack_size
__cs3_stack_mem:
    .if     Stack_Size
    .space  Stack_Size
    .endif
    .size   __cs3_stack_mem,  . - __cs3_stack_mem
    .set    __cs3_stack_size, . - __cs3_stack_mem


/*
// <h> Heap Configuration
//   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
// </h>
*/

    .equ    Heap_Size,  0x00000000

    .section ".heap", "w"
    .align  3
    .globl  __cs3_heap_start
    .globl  __cs3_heap_end
__cs3_heap_start:
    .if     Heap_Size
    .space  Heap_Size
    .endif
__cs3_heap_end:


/* Vector Table */

    .section ".cs3.interrupt_vector"
    .globl  __cs3_interrupt_vector_cortex_m
    .type   __cs3_interrupt_vector_cortex_m, %object

__cs3_interrupt_vector_cortex_m:
    .long   __cs3_stack                 /* Top of Stack                 */
    .long   __cs3_reset                 /* Reset Handler                */
    .long   NMI_Handler                 /* NMI Handler                  */
    .long   HardFault_Handler           /* Hard Fault Handler           */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   SVC_Handler                 /* SVCall Handler               */
    .long   0                           /* Reserved                     */
    .long   0                           /* Reserved                     */
    .long   PendSV_Handler              /* PendSV Handler               */
    .long   SysTick_Handler             /* SysTick Handler              */

    /* External Interrupts */
    .long   Reserved16_IRQHandler  /* Reserved interrupt 16 */
    .long   Reserved17_IRQHandler  /* Reserved interrupt 17 */
    .long   Reserved18_IRQHandler  /* Reserved interrupt 18 */
    .long   Reserved19_IRQHandler  /* Reserved interrupt 19 */
    .long   Reserved20_IRQHandler  /* Reserved interrupt 20 */
    .long   FTFA_IRQHandler  /* FTFA command complete/read collision interrupt */
    .long   LVD_LVW_IRQHandler  /* Low Voltage Detect, Low Voltage Warning */
    .long   Reserved23_IRQHandler  /* Reserved interrupt 23 */
    .long   I2C0_IRQHandler  /* I2C0 interrupt */
    .long   I2C1_IRQHandler  /* I2C1 interrupt */
    .long   SPI0_IRQHandler  /* SPI0 interrupt */
    .long   Reserved27_IRQHandler  /* Reserved interrupt 27 */
    .long   UART0_IRQHandler  /* UART0 status/error interrupt */
    .long   Reserved29_IRQHandler  /* Reserved interrupt 29 */
    .long   Reserved30_IRQHandler  /* Reserved interrupt 30 */
    .long   ADC0_IRQHandler  /* ADC0 interrupt */
    .long   CMP0_IRQHandler  /* CMP0 interrupt */
    .long   TPM0_IRQHandler  /* TPM0 fault, overflow and channels interrupt */
    .long   TPM1_IRQHandler  /* TPM1 fault, overflow and channels interrupt */
    .long   Reserved35_IRQHandler  /* Reserved interrupt 35 */
    .long   Reserved36_IRQHandler  /* Reserved interrupt 36 */
    .long   Reserved37_IRQHandler  /* Reserved interrupt 37 */
    .long   Reserved38_IRQHandler  /* Reserved interrupt 38 */
    .long   Reserved39_IRQHandler  /* Reserved interrupt 39 */
    .long   Reserved40_IRQHandler  /* Reserved interrupt 40 */
    .long   Reserved41_IRQHandler  /* Reserved interrupt 41 */
    .long   Reserved42_IRQHandler  /* Reserved interrupt 42 */
    .long   MCG_IRQHandler  /* MCG interrupt */
    .long   LPTimer_IRQHandler  /* LPTimer interrupt */
    .long   Reserved45_IRQHandler  /* Reserved interrupt 45 */
    .long   PORTA_IRQHandler  /* Port A interrupt */
    .long   PORTB_IRQHandler  /* Port B interrupt */


    .size   __cs3_interrupt_vector_cortex_m, . - __cs3_interrupt_vector_cortex_m

/* Flash Configuration */

  	.long	0xFFFFFFFF
  	.long	0xFFFFFFFF
  	.long	0xFFFFFFFF
  	.long	0xFFFFFFFE

    .thumb


/* Reset Handler */

    .section .cs3.reset,"x",%progbits
    .thumb_func
    .globl  __cs3_reset_cortex_m
    .type   __cs3_reset_cortex_m, %function
__cs3_reset_cortex_m:
    .fnstart
    LDR     R0, =SystemInit
    BLX     R0
    LDR     R0,=_start
    BX      R0
    .pool
    .cantunwind
    .fnend
    .size   __cs3_reset_cortex_m,.-__cs3_reset_cortex_m

    .section ".text"

/* Exception Handlers */

    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B       .
    .size   NMI_Handler, . - NMI_Handler

    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B       .
    .size   HardFault_Handler, . - HardFault_Handler

    .weak   MemManage_Handler
    .type   MemManage_Handler, %function
SVC_Handler:
    B       .
    .size   SVC_Handler, . - SVC_Handler

    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
PendSV_Handler:
    B       .
    .size   PendSV_Handler, . - PendSV_Handler

    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B       .
    .size   SysTick_Handler, . - SysTick_Handler


/* IRQ Handlers */

    .globl  Default_Handler
    .type   Default_Handler, %function
Default_Handler:
    B       .
    .size   Default_Handler, . - Default_Handler

    .macro  IRQ handler
    .weak   \handler
    .set    \handler, Default_Handler
    .endm

    IRQ     Reserved16_IRQHandler
    IRQ     Reserved17_IRQHandler
    IRQ     Reserved18_IRQHandler
    IRQ     Reserved19_IRQHandler
    IRQ     Reserved20_IRQHandler
    IRQ     FTFA_IRQHandler
    IRQ     LVD_LVW_IRQHandler
    IRQ     Reserved23_IRQHandler
    IRQ     I2C0_IRQHandler
    IRQ     I2C1_IRQHandler
    IRQ     SPI0_IRQHandler
    IRQ     Reserved27_IRQHandler
    IRQ     UART0_IRQHandler
    IRQ     Reserved29_IRQHandler
    IRQ     Reserved30_IRQHandler
    IRQ     ADC0_IRQHandler
    IRQ     CMP0_IRQHandler
    IRQ     TPM0_IRQHandler
    IRQ     TPM1_IRQHandler
    IRQ     Reserved35_IRQHandler
    IRQ     Reserved36_IRQHandler
    IRQ     Reserved37_IRQHandler
    IRQ     Reserved38_IRQHandler
    IRQ     Reserved39_IRQHandler
    IRQ     Reserved40_IRQHandler
    IRQ     Reserved41_IRQHandler
    IRQ     Reserved42_IRQHandler
    IRQ     MCG_IRQHandler
    IRQ     LPTimer_IRQHandler
    IRQ     Reserved45_IRQHandler
    IRQ     PORTA_IRQHandler
    IRQ     PORTB_IRQHandler
    IRQ     DefaultISR

    .end
