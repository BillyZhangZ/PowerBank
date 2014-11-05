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
#include "spi/fsl_spi_slave_driver.h"
#include "spi/hal/fsl_spi_hal.h"
#include "bootloader_common.h"
#include <assert.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! SPI slave constants
enum _spi_slave_constants
{
    kEmptyChar = 0,                        //!< Empty character
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

void spi_slave_irq_handler(unsigned instance);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! Place to store application configuration and callbacks for each of the SPI modules.
static spi_slave_callbacks_t s_spiSlaveInstanceCallbacks[HW_SPI_INSTANCE_COUNT];

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if !defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(0)
//! @brief Implementation of SPI0 handler named in startup code.
//!
//! Passes instance to generic SPI IRQ handler.
void SPI0_IRQHandler()
{
    spi_slave_irq_handler(HW_SPI0);
}
#endif // !defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(0)

#if (!defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(1)) && (HW_SPI_INSTANCE_COUNT > 1)
//! @brief Implementation of SPI1 handler named in startup code.
//!
//! Passes instance to generic SPI IRQ handler.
void SPI1_IRQHandler()
{
    spi_slave_irq_handler(HW_SPI1);
}
#endif // (!defined(BL_SPI_SIZE_OPTIMIZE) || USE_ONLY_SPI(1)) && (HW_SPI_INSTANCE_COUNT > 1)

//! @brief SPI Slave Generic IRQ handler.
//!
//! @param instance Instance number of the SPI module.
void spi_slave_irq_handler(unsigned instance)
{
#if USE_ONLY_SPI(0)
    instance = 0;
#elif USE_ONLY_SPI(1)
    instance = 1;
#endif  //USE_ONLY_SPI(0)

    spi_slave_callbacks_t * callbacks = &s_spiSlaveInstanceCallbacks[instance];

    if (spi_hal_is_read_buffer_full(instance))
    {
        // SPI receive interrupt
        uint8_t rd = spi_hal_read_data(instance);
        callbacks->dataSink(rd, instance);
    }

    if (spi_hal_is_transmit_buffer_empty(instance))
    {
        // SPI transimit interrupt
        uint8_t source_byte = kEmptyChar;
        callbacks->dataSource(&source_byte, instance);

        // Write the data to data register
        spi_hal_write_data(instance, source_byte);
    }
}

// See spi_slave.h for documentation of this function.
void spi_slave_init(uint32_t instance, const spi_slave_config_t * config)
{
#if USE_ONLY_SPI(0)
    instance = 0;
#elif USE_ONLY_SPI(1)
    instance = 1;
#endif  //USE_ONLY_SPI(0)

    assert(config);
    assert(instance < HW_SPI_INSTANCE_COUNT);

    // Save the application info.
    s_spiSlaveInstanceCallbacks[instance] = config->callbacks;

    // Enable clock for SPI
    HW_SIM_SCGC4_SET(
#if USE_ONLY_SPI(0) || (HW_SPI_INSTANCE_COUNT == 1)
            BM_SIM_SCGC4_SPI0
#elif USE_ONLY_SPI(1)
            BM_SIM_SCGC4_SPI1
#elif (HW_SPI_INSTANCE_COUNT > 1)
                    // SPI0 and SPI1
                    instance == HW_SPI0 ? BM_SIM_SCGC4_SPI0 : BM_SIM_SCGC4_SPI1
#endif // USE_ONLY_SPI(0) || (HW_SPI_INSTANCE_COUNT == 1)
                    );

    // Reset control register 1, slave mode
    spi_hal_reset(instance);

    // Set master or slave moe.
    spi_hal_set_master_slave(instance, kSpiSlave);

    // Set data format.
    spi_hal_set_data_format(instance, config->polarity, config->phase, config->direction);

    spi_hal_enable_receive_and_fault_interrupt(instance);
    spi_hal_enable_transmit_interrupt(instance);

    // Enable SPI interrupt
#if USE_ONLY_SPI(0) || (HW_SPI_INSTANCE_COUNT == 1)
    NVIC_EnableIRQ(SPI0_IRQn);
#elif USE_ONLY_SPI(1)
    NVIC_EnableIRQ(SPI1_IRQn);
#else
    // SPI0 and SPI1
    NVIC_EnableIRQ(instance == HW_SPI0 ? SPI0_IRQn : SPI1_IRQn);
#endif // USE_ONLY_SPI

    // SPI system enable
    spi_hal_enable(instance);

    // Fill in the data buffer so a master will receive a known first byte (0).
    spi_hal_write_data(instance, 0);
}

// See spi_slave.h for documentation of this function.
void spi_slave_shutdown(uint32_t instance)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    // Make sure the instance being de-initialized is actually clocked so that
    // registers writes won't cause an abort
#if USE_ONLY_SPI(0) || (HW_SPI_INSTANCE_COUNT == 1)
    if (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_SPI0)
    {
        NVIC_DisableIRQ(SPI0_IRQn);
        spi_hal_reset(0);
        HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_SPI0);
    }
#elif USE_ONLY_SPI(1)
    if (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_SPI1)
    {
        NVIC_DisableIRQ(SPI1_IRQn);
        spi_hal_reset(1);
        HW_SIM_SCGC4_CLR(BM_SIM_SCGC4_SPI1);
    }
#else
    // SPI0 and SPI1
    if (((instance == 0) && (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_SPI0)) ||
        ((instance == 1) && (HW_SIM_SCGC4_RD() & BM_SIM_SCGC4_SPI1)))
    {
        // Disable SPI interrupt
        IRQn_Type irqNumber = instance == HW_SPI0 ? SPI0_IRQn : SPI1_IRQn;
        NVIC_DisableIRQ(irqNumber);

        // Reset control register 1.
        spi_hal_reset(instance);

        // Disable clock for SPI
        HW_SIM_SCGC4_CLR(instance == HW_SPI0 ? BM_SIM_SCGC4_SPI0 : BM_SIM_SCGC4_SPI1);
    }
#endif // USE_ONLY_SPI(0) || (HW_SPI_INSTANCE_COUNT == 1)
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

