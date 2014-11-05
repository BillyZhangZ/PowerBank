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
#include "fsl_spi_hal.h"
#include "device/fsl_device_registers.h"
#include <assert.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Bit offsets for bits encoded in enum values.
enum _spi_pin_bit_encodings
{
    kSpiSsoeBit = 0U,    //!< SSOE is bit 0 of #spi_ss_output_mode_t.
    kSpiModfenBit = 1U,  //!< MODFEN is bit 1 of #spi_ss_output_mode_t.
    kSpiSpc0Bit = 0U,    //!< SPC0 is bit 0 of #spi_pin_mode_t.
    kSpiBidiroeBit = 1U  //!< BIDIROE is bit 1 of #spi_pin_mode_t.
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_init(uint32_t instance, const spi_config_t * config)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    // Enable or disable the module.
    BW_SPI_C1_SPE(instance, config->isEnabled == true);

    // Configure baud rate if a value is provided.
    if (config->kbitsPerSec != 0U)
    {
        spi_hal_set_baud(instance, config->kbitsPerSec);
    }

    // Set master or slave moe.
    spi_hal_set_master_slave(instance, config->masterOrSlave);

    // Set data format.
    spi_hal_set_data_format(instance, config->polarity, config->phase, config->shiftDirection);

    // Set output and pin modes.
    spi_hal_set_slave_select_output_mode(instance, config->ssOutputMode);
    spi_hal_set_pin_mode(instance, config->pinMode);

    // Enable requested interrupts.
    BW_SPI_C1_SPIE(instance, config->enableReceiveAndFaultInterrupt == true);
    BW_SPI_C1_SPTIE(instance, config->enableTransmitInterrupt == true);
    BW_SPI_C2_SPMIE(instance, config->enableMatchInterrupt == true);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_reset(uint32_t instance)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    HW_SPI_C1_WR(instance, BM_SPI_C1_CPHA);
    HW_SPI_C2_WR(instance, 0);
    HW_SPI_BR_WR(instance, 0);

#if FSL_FEATURE_SPI_16BIT_TRANSFERS
    HW_SPI_ML_WR(instance, 0);
#else // FSL_FEATURE_SPI_16BIT_TRANSFERS
    HW_SPI_M_WR(instance, 0);
#endif // FSL_FEATURE_SPI_16BIT_TRANSFERS
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_baud(uint32_t instance, uint32_t kbitsPerSec)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    uint32_t hz = kbitsPerSec * 1000U;
    uint32_t leastError = 0xffffffffU;
    uint8_t refSpr = 0U;
    uint8_t refPrescaler = 0U;

    uint8_t prescaler = 1U;
    uint8_t divisor = 2U;

    for (prescaler = 1U; prescaler <= 8U; prescaler++)
    {
        divisor = 2U;

        uint8_t spr = 0U;
        for (spr = 0U; spr <= 8U; spr++)
        {
            uint32_t ref = SystemCoreClock / (prescaler * divisor);
            uint32_t error = (ref > hz) ? ref - hz : hz - ref;
            if (error < leastError)
            {
                refSpr = spr;
                refPrescaler = prescaler - 1U;
                leastError = error;
            }
            divisor *= 2U;
        }
    }

    spi_hal_set_baud_divisors(instance, refPrescaler, refSpr);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_slave_select_output_mode(uint32_t instance, spi_ss_output_mode_t mode)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    // The mode enum values encode the SSOE and MODFEN bit values.
    // Bit 0: SSOE
    // Bit 1: MODFEN
    BW_SPI_C1_SSOE(instance, ((uint32_t)mode & (1U << kSpiSsoeBit)) >> kSpiSsoeBit);
    BW_SPI_C2_MODFEN(instance, ((uint32_t)mode & (1U << kSpiModfenBit)) >> kSpiModfenBit);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_data_format(uint32_t instance,
    spi_clock_polarity_t polarity,
    spi_clock_phase_t phase,
    spi_shift_direction_t direction)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);
    BW_SPI_C1_CPOL(instance, (uint32_t)polarity);
    BW_SPI_C1_CPHA(instance, (uint32_t)phase);
    BW_SPI_C1_LSBFE(instance, (uint32_t)direction);
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_set_pin_mode(uint32_t instance, spi_pin_mode_t mode)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    // The mode enum values encode the SPC0 and BIDIROE bit values.
    // Bit 0: SPC0
    // Bit 1: BIDIROE
    BW_SPI_C2_SPC0(instance, ((uint32_t)mode & (1U << kSpiSpc0Bit)) >> kSpiSpc0Bit);
    BW_SPI_C2_BIDIROE(instance, ((uint32_t)mode & (1U << kSpiBidiroeBit)) >> kSpiBidiroeBit);
}

#if FSL_FEATURE_SPI_DMA
// See fsl_spi_hal.h for documentation of this function.
void spi_hal_configure_dma(uint32_t instance, bool enableTransmit, bool enableReceive)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    BW_SPI_C2_TXDMAE(instance, (enableTransmit == true));
    BW_SPI_C2_RXDMAE(instance, (enableReceive == true));
}
#endif // FSL_FEATURE_SPI_DMA

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_clear_mode_fault(uint32_t instance)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    // Must make sure we read from the status register first.
    if (spi_hal_is_mode_fault(instance))
    {
        // Then we have to write to C1.
        HW_SPI_C1_WR(instance, HW_SPI_C1_RD(instance));
    }
}

// See fsl_spi_hal.h for documentation of this function.
void spi_hal_clear_match(uint32_t instance)
{
    assert(instance < HW_SPI_INSTANCE_COUNT);

    // Check that the match flag is set before writing 1 to clear it. This read
    // is required in order to clear the flag.
    if (spi_hal_is_match(instance))
    {
        // We have to hack this to write to the register because it is incorrectly
        // defined as a read-only register, even though the SPI_S.SPMF bitfield documentation
        // states you must write a 1 to the bitfield to clear it.
        *(volatile uint8_t *)HW_SPI_S_ADDR(instance) = BM_SPI_S_SPMF;
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

