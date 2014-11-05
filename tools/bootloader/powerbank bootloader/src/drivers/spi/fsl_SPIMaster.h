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
#if !defined(__FSL_SPI_MASTER_H__)
#define __FSL_SPI_MASTER_H__

#include "spi/fsl_spi_master_driver.h"
#include "spi/hal/fsl_spi_hal.h"
#include <stdint.h>
#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

namespace fsl {

/*!
 * @brief SPI master peripheral.
 *
 * @ingroup spi_classes
 *
 * This class provides a simple interface to the SPI master-mode peripheral driver. It
 * represents one instance of a SPI peripheral.
 *
 * To use the class, first create an instance. You can create a static instance either
 * globally or on the stack. Or you can dynamically allocate an instance using the @b
 * @c new operator.
 *
 * Once you have a SPIMaster instance, you should configure the frequency and data format
 * used to transfer data. This is done with the setFrequency() and setFormat() methods.
 * These can be called in any order, and you can call them at any time to change the
 * settings.
 *
 * Here is some example code demonstrating how to create a SPIMaster object statically
 * and then configure it.
 * @code
 *      // Create an object for the SPI0 peripheral.
 *      SPIMaster bus(0);
 *
 *      // Configure the bus settings.
 *      bus.setFrequency(1000); // 1 Mbit in kilobits/sec
 *      bus.setFormat(kSpiClockPolarity_ActiveHigh,
 *                      kSpiClockPhase_FirstEdge,
 *                      kSpiMsbFirst);
 * @endcode
 *
 * Finally, you can transfer data. Since SPI is intrinsically full duplex, the transfer()
 * method takes two buffers, for data to send and receive simultaneously.
 *
 * Here is an example showing how to transfer data.
 * @code
 *      // This is the buffer we'll send.
 *      const char * output = "Hello World!"
 *
 *      // And this buffer provides storage for data that is received.
 *      uint8_t receiveBuffer[32];
 *
 *      // Now we can transfer some data! This call will not return until the
 *      // specified number of bytes has been transferred.
 *      bus.transfer((uint8_t *)output, receiveBuffer, sizeof(output));
 *
 *      // Here we only send 4 bytes of data, ignoring the bytes that are received.
 *      bus.transfer(receiveBuffer, NULL, 4);
 * @endcode
 */
class SPIMaster
{
public:
    /*!
     * @brief Constructor.
     *
     * @param instance The SPI peripheral instance number.
     */
    SPIMaster(uint32_t instance)
    :   m_instance(instance)
    {
        spi_master_init(instance);
    }
    
    //! @brief Set the bit rate at which data is sent.
    //!
    //! @param kbps Requested bus frequency in kilobits/sec. For instance, to use a frequency
    //!     of 4 Mbit, pass a value of 4000.
    void setFrequency(uint32_t kbps)
    {
        spi_hal_set_baud(m_instance, kbps);
    }
    
    /*!
     * @brief Set the data format used to send data.
     *
     * @param polarity Clock polarity, either active high or active low. Pass either:
     *     - #kSpiClockPolarity_ActiveHigh
     *     - #kSpiClockPolarity_ActiveLow
     * @param phase Clock phase. Selects either the first or second clock edge as the first
     *     sample point. Pass either:
     *     - #kSpiClockPhase_FirstEdge
     *     - #kSpiClockPhase_SecondEdge
     * @param direction Whether the most significant or least significant bit in each word is
     *     shifted first. Pass either:
     *     - #kSpiMsbFirst
     *     - #kSpiLsbFirst
     */
    void setFormat(spi_clock_polarity_t polarity,
                    spi_clock_phase_t phase,
                    spi_shift_direction_t direction)
    {
        spi_hal_set_data_format(m_instance, polarity, phase, direction);
    }
    
    /*!
     * @brief Transfer buffer of data bidirectionally.
     *
     * This method is a blocking call to transfer data in full-duplex mode.
     *
     * @param sendBuffer The buffer of data to send. If NULL is passed, then @a transferByteCount
     *     number of zero bytes are sent.
     * @param receiveBuffer A buffer to store data received from the slave. NULL may be passed,
     *     in which case the received bytes are ignored and lost.
     * @param transferByteCount The number of bytes to transfer in both directions simultaneously.
     *     If the size is 0, then nothing will happen.
     */
    void transfer(const uint8_t * sendBuffer,
                    uint8_t * receiveBuffer,
                    size_t transferByteCount)
    {
        spi_master_transfer(m_instance, NULL, sendBuffer, receiveBuffer, transferByteCount, kSpiWaitForever);
    }

protected:
    uint32_t m_instance;    //!< The SPI peripheral instance number.
};

} // namespace fsl

#endif // __FSL_SPI_MASTER_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

