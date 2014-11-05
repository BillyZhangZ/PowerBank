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

#include "bootloader_common.h"
#include "bootloader/context.h"
#include "device/fsl_device_registers.h"
#include "drivers/uart/uart.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#ifdef FREEDOM
#define UART0_RX_GPIO_PIN_NUM 1  // PIN 1 in the PTA group
#define UART0_RX_ALT_MODE 2      // ALT mode for UART0 functionality for pin 1
#define UART0_RX_GPIO_ALT_MODE 1 // ALT mode for GPIO functionality for pin 1

#define UART0_TX_GPIO_PIN_NUM 2  // PIN 2 in the PTA group
#define UART0_TX_ALT_MODE 2      // ALT mode for UART0 TX functionality for pin 2
#else
#define UART0_RX_GPIO_PIN_NUM 15 // PIN 15 in the PTA group
#define UART0_RX_ALT_MODE 3      // ALT mode for UART0 functionality for pin 15
#define UART0_RX_GPIO_ALT_MODE 1 // ALT mdoe for GPIO functionality for pin 15

#define UART0_TX_GPIO_PIN_NUM 14 // PIN 14 in the PTA group
#define UART0_TX_ALT_MODE 3      // ALT mode for UART0 TX functionality for pin 14
#endif

#define PORT_IRQC_INTERRUPT_FALLING_EDGE 0xA
#define PORT_IRQC_INTERRUPT_DISABLE 0

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
//! this is to store the function pointer for calling back to the function that wants
//! the UART RX instance pin that triggered the interrupt. This only supports 1 pin
static pin_irq_callback_t s_pin_irq_func = 0;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

/* This function is called for configurating pinmux for uart module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void uart_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
        case 0:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTA, UART0_RX_GPIO_PIN_NUM, 0);
                    BW_PORT_PCRn_MUX(HW_PORTA, UART0_TX_GPIO_PIN_NUM, 0);
                    break;
                case kPinmuxType_GPIO:
                    BW_PORT_PCRn_MUX(HW_PORTA, UART0_RX_GPIO_PIN_NUM, UART0_RX_GPIO_ALT_MODE); // Set UART0_RX pin in GPIO mode
                    HW_GPIO_PDDR_CLR(HW_GPIOA, 1 << UART0_RX_GPIO_PIN_NUM);                    // Set UART0_RX pin as an input
                    break;
                case kPinmuxType_Peripheral:
                    BW_PORT_PCRn_MUX(HW_PORTA, UART0_RX_GPIO_PIN_NUM, UART0_RX_ALT_MODE);   // Set UART0_RX pin to UART0_RX functionality
                    BW_PORT_PCRn_MUX(HW_PORTA, UART0_TX_GPIO_PIN_NUM, UART0_TX_ALT_MODE);   // Set UART0_TX pin to UART0_TX functionality
                    break;
                default:
                    break;
            }
            break;
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

/* This function is called for configurating pinmux for i2c module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void i2c_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
        case 0:
#ifdef FREEDOM
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTC, 8, 0);
                    BW_PORT_PCRn_MUX(HW_PORTC, 9, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C0.
                    BW_PORT_PCRn_MUX(HW_PORTC, 8, 2);  // I2C0_SCL is ALT2 for pin PTC8
                    BW_PORT_PCRn_MUX(HW_PORTC, 9, 2);  // I2C0_SDA is ALT2 for pin PTC9
                    break;
                default:
                    break;
            }
#else
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTE, 24, 0);
                    BW_PORT_PCRn_MUX(HW_PORTE, 25, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for I2C0.
                    BW_PORT_PCRn_MUX(HW_PORTE, 24, 5);  // I2C0_SCL is ALT5 for pin PTE24
                    BW_PORT_PCRn_MUX(HW_PORTE, 25, 5);  // I2C0_SDA is ALT5 for pin PTE25
                    break;
                default:
                    break;
            }
#endif
            break;
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

/* This function is called for configurating pinmux for spi module
 * This function only support switching default or gpio or fixed-ALTx mode on fixed pins
 * (Although there are many ALTx-pinmux configuration choices on various pins for the same
 * peripheral module) */
void spi_pinmux_config(unsigned int instance, pinmux_type_t pinmux)
{
    switch(instance)
    {
        case 0:
            switch(pinmux)
            {
                case kPinmuxType_Default:
                    BW_PORT_PCRn_MUX(HW_PORTD, 0, 0);
                    BW_PORT_PCRn_MUX(HW_PORTD, 1, 0);
                    BW_PORT_PCRn_MUX(HW_PORTD, 2, 0);
                    BW_PORT_PCRn_MUX(HW_PORTD, 3, 0);
                    break;
                case kPinmuxType_Peripheral:
                    // Enable pins for SPI0 on PTD0~3 (not available on 32-pin QFN package)
                    BW_PORT_PCRn_MUX(HW_PORTD, 0, 2);  // SPI0_PCS0 is ALT2 for pin PTD0
                    BW_PORT_PCRn_MUX(HW_PORTD, 1, 2);  // SPI0_SCK is ALT2 for pin PTD1
                    BW_PORT_PCRn_MUX(HW_PORTD, 2, 2);  // SPI0_MOSI is ALT2 for pin PTD2
                    BW_PORT_PCRn_MUX(HW_PORTD, 3, 2);  // SPI0_MISO is ALT2 for pin PTD3
                    break;
                default:
                    break;
            }
            break;
        case 1:
            break;
        case 2:
            break;
        default:
            break;
    }
}

void init_hardware(void)
{
    SIM->SCGC5 |= ( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );

    SIM->SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK // set PLLFLLSEL to select the PLL for this clock source
                | SIM_SOPT2_UART0SRC(1);   // select the PLLFLLCLK as UART0 clock source

#if DEBUG
    // Enable the pins for the debug UART1
    BW_PORT_PCRn_MUX(HW_PORTC, 3, 3);   // UART1_RX is PTC3 in ALT3
    BW_PORT_PCRn_MUX(HW_PORTC, 4, 3);   // UART1_TX is PTC4 in ALT3
#endif
}

void deinit_hardware(void)
{
    SIM->SCGC5 &= (uint32_t)~( SIM_SCGC5_PORTA_MASK
                  | SIM_SCGC5_PORTB_MASK
                  | SIM_SCGC5_PORTC_MASK
                  | SIM_SCGC5_PORTD_MASK
                  | SIM_SCGC5_PORTE_MASK );
}

uint32_t get_bus_clock(void)
{
    return SystemCoreClock / (HW_SIM_CLKDIV1.B.OUTDIV4 + 1);
}


bool usb_clock_init(void)
{
#define  CLOCK_SETUP    0
 #if (CLOCK_SETUP == 0)
     // Configure USB to be clocked from FLL
    SIM_SOPT2 |= SIM_SOPT2_USBSRC_MASK;
    SIM_SOPT2 &= ~SIM_SOPT2_PLLFLLSEL_MASK;
#else
      // Configure USB to be clocked from PLL
    SIM_SOPT2 |= (SIM_SOPT2_USBSRC_MASK | SIM_SOPT2_PLLFLLSEL_MASK);
#endif
    // Enable USB-OTG IP clocking
    SIM_SCGC4 |= (SIM_SCGC4_USBOTG_MASK);

    // Configure enable USB regulator for device
    SIM_SOPT1 |= SIM_SOPT1_USBREGEN_MASK;

    return true;
}


uint32_t get_uart_clock( unsigned int instance )
{
    switch(instance)
    {
        case 0:
            // UART0 uses the PLL clock selected in hardware_init
            return GetSystemMCGPLLClock();
        case 1:
            {
                // UART1 always uses the system clock / OUTDIV4
                const uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT) + 1;
                return (SystemCoreClock / busClockDivider);
            }
        case 2:
            {
                // UART2 always uses the system clock / OUTDIV4
                uint32_t busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT) + 1;
                return (SystemCoreClock / busClockDivider);
            }
        default:
            return 0;
    }
}

unsigned int read_autobaud_pin( unsigned int instance )
{
    switch(instance)
    {
        case 0:
            return (HW_GPIO_PDIR_RD(HW_GPIOA) >> UART0_RX_GPIO_PIN_NUM) & 1;
        case 1:
            return 0;
        case 2:
            return 0;
        default:
            return 0;
    }
}

bool is_boot_pin_asserted(void)
{
    // no boot from ROM option for KL25
    return false;
}

//! @brief this is going to be used for autobaud IRQ handling
void PORTA_IRQHandler(void)
{
    // Check if the pin for UART0 is what triggered the PORT A interrupt
    if (HW_PORT_PCRn(HW_PORTA, UART0_RX_GPIO_PIN_NUM).B.ISF && s_pin_irq_func)
    {
        s_pin_irq_func(0);
        HW_PORT_ISFR_WR(HW_PORTA, ~0U);
    }
    // else if would be added here for other UART RX pins, only supports UART0 currently
}

void enable_autobaud_pin_irq(unsigned int instance, pin_irq_callback_t func)
{
    switch(instance)
    {
        case 0:
            NVIC_EnableIRQ(PORTA_IRQn);
            // Only look for a falling edge for our interrupts
            HW_PORT_PCRn(HW_PORTA, UART0_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_FALLING_EDGE;
            s_pin_irq_func = func;
            break;
    }
}

void disable_autobaud_pin_irq(unsigned int instance)
{
    switch(instance)
    {
        case 0:
            NVIC_DisableIRQ(PORTA_IRQn);
            HW_PORT_PCRn(HW_PORTA, UART0_RX_GPIO_PIN_NUM).B.IRQC = PORT_IRQC_INTERRUPT_DISABLE;
            s_pin_irq_func = 0;
            break;
    }
}

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void debug_init(void)
{
    uart_init(UART1, get_uart_clock(1), TERMINAL_BAUD, dummy_byte_callback);
}

void update_available_peripherals()
{
}


#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    while (size--)
    {
        uart_putchar(UART1, *buf++);
    }

    return size;
}

#endif // __ICCARM__

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

