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
#ifndef __BOOTLOADER_CONFIG_H__
#define __BOOTLOADER_CONFIG_H__

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//
// Bootloader configuration options
//

#if !defined(BL_TARGET_FLASH)
#define BL_TARGET_FLASH (1)
#endif

#if !defined(BL_MIN_PROFILE)
#define BL_MIN_PROFILE (0)
#endif

// Bootloader peripheral detection default timeout in milliseconds
// After coming out of reset the bootloader will spin in a peripheral detection
// loop for this amount of time. A zero value means no time out.
#if DEBUG
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 0
#else
#define BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT 5000
#endif // DEBUG

//! Set to 1 if flash has mass erase support.
#define BL_HAS_MASS_ERASE (0)

#define BL_FLASH_VERIFY_DISABLE (0)

#define BL_ENABLE_CRC_CHECK (0)

#define BL_UART_AUTOBAUD_IRQ (1)

// The bootloader will check this address for the application vector table upon startup.
#if !defined(BL_APP_VECTOR_TABLE_ADDRESS)
#define BL_APP_VECTOR_TABLE_ADDRESS 0x8000
#endif

/* Serial Port Info */

/**************************************************************************
 * Note:
 *
 * Because of the changes to the UART modules, we can no longer define
 * the TERM_PORT as a base pointer.  The uart functions have been modified
 * accommodate this change.  Now, TERM_PORT_NUM must be defined as the
 * number of the UART port desired to use
 *
 * TERM_PORT_NUM = 0  -- This allows you to use UART0; default pins are
 *                        PTA14 and PTA15
 *
 * TERM_PORT_NUM = 1  -- This allows you to use UART1; default pins are
 *                        PTC3 and PTC4
 *
 * TERM_PORT_NUM = 2  -- This allows you to use UART2; default pins are
 *                        PTD2 and PTD3
 *
 *************************************************************************/
#define TERM_PORT_NUM       0

#define TERMINAL_BAUD       19200
#undef  HW_FLOW_CONTROL



#endif // __BOOTLOADER_CONFIG_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
