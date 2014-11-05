/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_platform_common.h"
#include "scuart.h"
#include <stdarg.h>

static scuart_data_sink_func_t s_scuart_data_sink_callback;

#if defined (K64F12_SERIES) || defined (K70F12_SERIES) || defined (K22F51212_SERIES)
static const IRQn_Type scuart_irq_ids[HW_UART_INSTANCE_COUNT] =
{ UART0_RX_TX_IRQn,
  UART1_RX_TX_IRQn,
  UART2_RX_TX_IRQn,
#if defined (K64F12_SERIES) || defined (K70F12_SERIES)
  UART3_RX_TX_IRQn,
  UART4_RX_TX_IRQn,
  UART5_RX_TX_IRQn
#endif
};
#elif defined(K66F18_SERIES)
static const IRQn_Type scuart_irq_ids[HW_UART_INSTANCE_COUNT] =
{ UART0_RX_TX_IRQn,
  UART1_RX_TX_IRQn,
  UART2_RX_TX_IRQn,
  UART3_RX_TX_IRQn,
  UART4_RX_TX_IRQn
};
#endif

/********************************************************************/
/*
 * Initialize the uart for 8N1 operation, interrupts disabled, and
 * no hardware flow-control
 *
 * NOTE: Since the uarts are pinned out in multiple locations on most
 *       Kinetis devices, this driver does not enable uart pin functions.
 *       The desired pins should be enabled before calling this init function.
 *
 * Parameters:
 *  uartch      uart channel to initialize
 *  uartclk     uart module Clock in Hz(used to calculate baud)
 *  baud        uart baud rate
 */
status_t scuart_init (UART_Type * uartch, int uartclk, int baud, scuart_data_sink_func_t function)
{
    uint8_t temp;
    register uint16_t sbr = (uint16_t)(uartclk/(baud * 16));
    int baud_check = uartclk / (sbr * 16);
    uint32_t baud_diff;

    s_scuart_data_sink_callback = function;

    if (baud_check > baud)
    {
        baud_diff = baud_check - baud;
    }
    else
    {
        baud_diff = baud - baud_check;
    }

    // If the baud rate cannot be within 3% of the passed in value
    // return a failure
    if (baud_diff > ((baud / 100) * 3))
    {
        return kStatus_Fail;
    }

    switch((unsigned int)uartch)
    {
        case (unsigned int)UART0:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART0);
            break;
        case (unsigned int)UART1:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART1);
            break;
        case (unsigned int)UART2:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART2);
            break;
#if (HW_UART_INSTANCE_COUNT > 3U)
        case (unsigned int)UART3:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART3);
            break;
        case (unsigned int)UART4:
            HW_SIM_SCGC1_SET(BM_SIM_SCGC1_UART4);
            break;
#if (HW_UART_INSTANCE_COUNT > 5U)           
        case (unsigned int)UART5:
            HW_SIM_SCGC1_SET(BM_SIM_SCGC1_UART5);
            break;
#endif            
#endif // (HW_UART_INSTANCE_COUNT > 3U)
    }

    //Make sure that the transmitter and receiver are disabled while we
    //change settings.
    uartch->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK );

    // Configure the uart for 8-bit mode, no parity
    uartch->C1 = 0;    // We need all default settings, so entire register is cleared

    // Save off the current value of the uartx_BDH except for the SBR field
    temp = uartch->BDH & ~(UART_BDH_SBR(0x1F));

    uartch->BDH = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
    uartch->BDL = (uint8_t)(sbr & UART_BDL_SBR_MASK);

    // Flush the RX and TX FIFO's
    uartch->CFIFO = UART_CFIFO_RXFLUSH_MASK | UART_CFIFO_TXFLUSH_MASK;

    // Enable receive interrupts
    uartch->C2 |= BM_UART_C2_RIE;

    NVIC_EnableIRQ(scuart_irq_ids[REGS_UART_INSTANCE((unsigned int)uartch)]);

    // Enable receiver and transmitter
    uartch->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK );

    return kStatus_Success;
}

/********************************************************************/
/*
 * Wait for space in the uart Tx FIFO and then send a character
 *
 * Parameters:
 *  channel      uart channel to send to
 *  ch             character to send
 */
void scuart_putchar (UART_Type * channel, char ch)
{
    // Wait until space is available in the FIFO
    while(!(channel->S1 & UART_S1_TDRE_MASK));

    // Send the character
    channel->D = (uint8_t)ch;
}

/********************************************************************/
/*
 * UART0 IRQ Handler
 *
 */

void UART0_RX_TX_IRQHandler(void)
{
    if (UART0->S1 & BM_UART_S1_RDRF)
    {
        s_scuart_data_sink_callback(UART0->D);
    }
}

/********************************************************************/
/*
 * UART1 IRQ Handler
 *
 */

void UART1_RX_TX_IRQHandler(void)
{
    if (UART1->S1 & BM_UART_S1_RDRF)
    {
        s_scuart_data_sink_callback(UART1->D);
    }
}

/********************************************************************/
/*
 * UART2 IRQ Handler
 *
 */

void UART2_RX_TX_IRQHandler(void)
{
    if (UART2->S1 & BM_UART_S1_RDRF)
    {
        s_scuart_data_sink_callback(UART2->D);
    }
}

#if defined (K64F12_SERIES) || defined (K70F12_SERIES)

/********************************************************************/
/*
 * UART3 IRQ Handler
 *
 */

void UART3_RX_TX_IRQHandler(void)
{
    if (UART3->S1 & BM_UART_S1_RDRF)
    {
        s_scuart_data_sink_callback(UART3->D);
    }
}

/********************************************************************/
/*
 * UART4 IRQ Handler
 *
 */

void UART4_RX_TX_IRQHandler(void)
{
    if (UART4->S1 & BM_UART_S1_RDRF)
    {
        s_scuart_data_sink_callback(UART4->D);
    }
}

/********************************************************************/
/*
 * UART5 IRQ Handler
 *
 */

void UART5_RX_TX_IRQHandler(void)
{
    if (UART5->S1 & BM_UART_S1_RDRF)
    {
        s_scuart_data_sink_callback(UART5->D);
    }
}

#endif // defined (K64F12_SERIES) || defined (K70F12_SERIES)

/********************************************************************/
/*
 * Shutdown UART
 *
 * Parameters:
 *  uartch      the UART to shutdown
 *
 */
void scuart_shutdown (UART_Type * uartch)
{
    NVIC_DisableIRQ(scuart_irq_ids[REGS_UART_INSTANCE((unsigned int)uartch)]);

    // In case uart peripheral isn't active which also means uart clock doesn't open,
    // So enable clocking to UARTx in any case, then we can write control register.
    switch((unsigned int)uartch)
    {
        case (unsigned int)UART0:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART0);
            break;
        case (unsigned int)UART1:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART1);
            break;
        case (unsigned int)UART2:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART2);
            break;
#if (HW_UART_INSTANCE_COUNT > 3U)
        case (unsigned int)UART3:
            HW_SIM_SCGC4_SET(BM_SIM_SCGC4_UART3);
            break;
        case (unsigned int)UART4:
            HW_SIM_SCGC1_SET(BM_SIM_SCGC1_UART4);
            break;
#if (HW_UART_INSTANCE_COUNT > 5U)            
        case (unsigned int)UART5:
            HW_SIM_SCGC1_SET(BM_SIM_SCGC1_UART5);
            break;
#endif            
#endif // (HW_UART_INSTANCE_COUNT > 3U)
    }

    // Reset all control registers.
    uartch->C2 = 0;
    //uartch->C1 = 0;
    //uartch->C3 = 0;
    //uartch->C4 = 0;
    //uartch->C5 = 0;

    // Gate the uart clock.
    switch((unsigned int)uartch)
    {
        case (unsigned int)UART0:
            HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_UART0);
            break;
        case (unsigned int)UART1:
            HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_UART1);
            break;
        case (unsigned int)UART2:
            HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_UART2);
            break;
#if (HW_UART_INSTANCE_COUNT > 3U)
        case (unsigned int)UART3:
            HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_UART3);
            break;
        case (unsigned int)UART4:
            HW_SIM_SCGC1_CLR(BM_SIM_SCGC1_UART4);
            break;
#if (HW_UART_INSTANCE_COUNT > 5U)              
        case (unsigned int)UART5:
            HW_SIM_SCGC1_CLR(BM_SIM_SCGC1_UART5);
            break;
#endif            
#endif // (HW_UART_INSTANCE_COUNT > 3U)
    }
}

