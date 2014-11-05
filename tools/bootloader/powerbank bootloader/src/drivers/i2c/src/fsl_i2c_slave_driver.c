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

#include "fsl_platform_common.h" // needed for NULL
#include "device/fsl_device_registers.h"
#include "i2c/fsl_i2c_slave_driver.h"
#include "bootloader_common.h"
#include <assert.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#define I2C_EMPTY_CHAR    (0x00)        //!< Empty character.

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! Place to store application callbacks for each of the I2C modules.
static i2c_slave_info_t s_applicationInfo[2] = {{0}};

////////////////////////////////////////////////////////////////////////////////
// Private Prototypes
////////////////////////////////////////////////////////////////////////////////

void I2C0_IRQHandler(void);
void I2C1_IRQHandler(void);
static void i2c_slave_irqhandler(int instance);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if !defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(0)
//! @brief Implementation of I2C0 handler named in startup code.
//!
//! Passes instance to generic I2C IRQ handler.
void I2C0_IRQHandler(void)
{
    i2c_slave_irqhandler(HW_I2C0);
}
#endif // !defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(0)

#if (!defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(1)) && (HW_I2C_INSTANCE_COUNT > 1)
//! @brief Implementation of I2C1 handler named in startup code.
//!
//! Passes instance to generic I2C IRQ handler.
void I2C1_IRQHandler(void)
{
    i2c_slave_irqhandler(HW_I2C1);
}
#endif // (!defined(BL_I2C_SIZE_OPTIMIZE) || USE_ONLY_I2C(1)) && (HW_I2C_INSTANCE_COUNT > 1)

//! @brief I2C Slave Generic IRQ handler.
//!
//! Implements the flow chart at the end of the I2C chapter in the Kinetis
//! KL25 Sub-Family Reference Manual. It uses callbacks to get/put data
//! from/to the application as well as alert the application of an error condition.
//!
//! @param instance Instance number of the I2C module.
static void i2c_slave_irqhandler(int instance)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#endif  //USE_ONLY_I2C(0)

    assert(instance < HW_I2C_INSTANCE_COUNT);

    i2c_slave_info_t * appInfo = &s_applicationInfo[instance];
    uint8_t status = HW_I2C_S_RD(instance);
    bool doTransmit = false;

    // Clear interrupt flag(s)
    HW_I2C_S_SET(instance, (BM_I2C_S_ARBL | BM_I2C_S_IICIF) & status);

    if ((status & BM_I2C_S_ARBL) && (!(status & BM_I2C_S_IAAS))) // ArbitrationLost and not AddressedAsSlave
    {
        // I2C_S_ARBL is already cleared by ClearInterruptFlags()
//         error = kStatus_I2C_AribtrationLost;
    }
    else if (status & BM_I2C_S_IAAS) // AddressedAsSlave
    {
        if (status & BM_I2C_S_SRW) // Master read from Slave. Slave transmit.
        {
            // Switch to TX mode
            HW_I2C_C1_SET(instance, BM_I2C_C1_TX);

            doTransmit = true;
        }
        else // Master write to Slave. Slave receive.
        {
            // Switch to RX mode.
            HW_I2C_C1_CLR(instance, BM_I2C_C1_TX);

            // Dummy read character.
            char dummy = HW_I2C_D_RD(instance);
        }
    }
    else // not AddressedAsSlave
    {
        if (HW_I2C_C1(instance).B.TX) // Transmit.
        {
            if (status & BM_I2C_S_RXAK) // No ACK from receiver.
            {
                // Switch to RX mode.
                HW_I2C_C1_CLR(instance, BM_I2C_C1_TX);

                // Dummy read character.
                char dummy = HW_I2C_D_RD(instance);
            }
            else // ACK from receiver.
            {
                // DO TRANSMIT
                doTransmit = true;
            }
        }
        else // Receive.
        {
            // Get byte from data register.
            uint8_t sink_byte = HW_I2C_D_RD(instance);

            appInfo->data_sink(sink_byte);
        }
    }

    if (doTransmit)
    {
        uint8_t source_byte = I2C_EMPTY_CHAR;

        appInfo->data_source(&source_byte);

        // Store char to transmit register.
        HW_I2C_D_WR(instance, source_byte);
    }
}

// See i2c_slave.h for documentation of this function.
void i2c_slave_init(int instance, i2c_slave_info_t * appInfo)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#endif  //USE_ONLY_I2C(0)

    assert(appInfo);
    assert(instance < HW_I2C_INSTANCE_COUNT);

    // Save the application info.
    s_applicationInfo[instance] = *appInfo;

    // Enable clock for I2C.
#if USE_ONLY_I2C(0) || (HW_I2C_INSTANCE_COUNT == 1)
    HW_SIM_SCGC4_SET(BM_SIM_SCGC4_I2C0);
#elif USE_ONLY_I2C(1)
    HW_SIM_SCGC4_SET(BM_SIM_SCGC4_I2C1);
#else
    HW_SIM_SCGC4_SET( instance == HW_I2C0 ? BM_SIM_SCGC4_I2C0 : BM_SIM_SCGC4_I2C1);
#endif // USE_ONLY_I2C(0) || (HW_I2C_INSTANCE_COUNT == 1)

    // Clear control register.
    HW_I2C_C1_WR(instance, 0);

    // Clear bus status interrupt flags.
    HW_I2C_FLT_WR(instance, BM_I2C_FLT_STOPF);

    // Clear interrupt flag.
    HW_I2C_S_WR(instance, BM_I2C_S_IICIF);

#if USE_ONLY_I2C(0) || (HW_I2C_INSTANCE_COUNT == 1)
    NVIC_EnableIRQ(I2C0_IRQn);
#elif USE_ONLY_I2C(1)
    NVIC_EnableIRQ(I2C1_IRQn);
#else
    NVIC_EnableIRQ(instance == HW_I2C0 ? I2C0_IRQn : I2C1_IRQn);
#endif // USE_ONLY_I2C(0) || (HW_I2C_INSTANCE_COUNT == 1)

    // Set Slave Address.
    BW_I2C_A1_AD(instance, appInfo->slaveAddress);
    // No extended functions.
    HW_I2C_C2_WR(instance, 0);
    // No glitch filter.
    HW_I2C_FLT_WR(instance, 0);
#if defined(HW_I2C_SMB)
    // No SMBus support.
    HW_I2C_SMB_WR(instance, BM_I2C_SMB_SLTF);
#endif
    // Set baud rate.
    HW_I2C_F_WR(instance, 0);
    // Enable I2C device.
    HW_I2C_C1_SET(instance, BM_I2C_C1_IICEN);
    // Enable interrupt.
    HW_I2C_C1_SET(instance, BM_I2C_C1_IICIE);
}

// See i2c_slave.h for documentation of this function.
void i2c_slave_deinit(int instance)
{
    assert(instance < HW_I2C_INSTANCE_COUNT);

    // Make sure the instance being de-initialized is actually clocked so that
    // registers writes won't cause an abort
#if USE_ONLY_I2C(0) || (HW_I2C_INSTANCE_COUNT == 1)
    if (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_I2C0)
    {
        NVIC_DisableIRQ(I2C0_IRQn);
        HW_I2C_C1_WR(instance, 0);
        HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_I2C0);

    }
#elif USE_ONLY_I2C(1)
    if (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_I2C1)
    {
        NVIC_DisableIRQ(I2C1_IRQn);
        HW_I2C_C1_WR(instance, 0);
        HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_I2C1);
    }
#else
    if (((instance == 0) && (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_I2C0)) ||
        ((instance == 1) && (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_I2C1)))
    {
        // Disable I2C interrupt
        IRQn_Type irqNumber = instance == HW_I2C0 ? I2C0_IRQn : I2C1_IRQn;
        NVIC_DisableIRQ(irqNumber);

        // Clear control register.
        HW_I2C_C1_WR(instance, 0);

        // Disable clock for I2C.
        HW_SIM_SCGC4_CLR(instance == HW_I2C0 ? BM_SIM_SCGC4_I2C0 : BM_SIM_SCGC4_I2C1);
    }
#endif // USE_ONLY_I2C(0) || (HW_I2C_INSTANCE_COUNT == 1)
}

// See i2c_slave.h for documentation of this function.
void i2c_set_glitch_filter_width(int instance, uint32_t busClock_Hz, uint32_t glitchWidth_ns)
{
#if USE_ONLY_I2C(0)
    instance = 0;
#elif USE_ONLY_I2C(1)
    instance = 1;
#endif  //USE_ONLY_I2C(0)

    uint32_t busCycle_ns = 1000000 / (busClock_Hz / 1000);

    // Search for the cycle count just below the desired glitch width.
    uint32_t cycles = 0;
    while (((cycles + 1) * busCycle_ns) < glitchWidth_ns)
    {
        ++cycles;
    }

    // If we end up with zero cycles, then set the filter to a single cycle unless the
    // bus clock is greater than 10x the desired glitch width.
    if ((cycles == 0) && (busCycle_ns <= (glitchWidth_ns * 10)))
    {
        cycles = 1;
    }

    BW_I2C_FLT_FLT(instance, cycles);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

