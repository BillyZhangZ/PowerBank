/*
 * Copyright (c) 2013-14, Freescale Semiconductor, Inc.
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

#include "blfwk/bus_pal_peripheral.h"
#include "blfwk/format_string.h"
#include "blfwk/Logging.h"
#include <string>
#include <cstring>

using namespace blfwk;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See uart_peripheral.h for documentation of this method.
BusPalUartPeripheral::BusPalUartPeripheral(const char * port, long speed, const BusPal::BusPalConfigData& config)
{
    if ( !init(port, speed) )
        throw std::runtime_error(format_string("Error: BusPalUartPeripheral() cannot open PC UART port(%s), speed(%d Hz).", port, speed));

    configure(config);

}

// See bus_pal_peripheral.h for documentation of this method.
bool BusPalUartPeripheral::init(const char * busPalPort, long uartSpeed)
{
    return m_busPal.init(busPalPort, uartSpeed);
}

// See bus_pal_peripheral.h for documentation of this method.
BusPalUartPeripheral::~BusPalUartPeripheral()
{
}

void BusPalUartPeripheral::configure(const BusPal::BusPalConfigData& config)
{
    switch (config.transport)
    {
        case BusPal::kBusPalTransport_SPI:
            if (!m_busPal.enterSpiMode())
            {
                throw std::runtime_error(format_string("Error: BusPalUartPeripheral() cannot enter SPI Mode."));
            }

            if (!m_busPal.setSpiSpeed(config.spiSpeedKHz))
            {
                throw std::runtime_error(format_string("Error: BusPalUartPeripheral() cannot set SPI speed(%d kHz).", config.spiSpeedKHz));
            }

            if (!m_busPal.setSpiConfig(config.spiPolarity, config.spiPhase, config.spiDirection))
            {
                throw std::runtime_error(format_string("Error: BusPalUartPeripheral() cannot set SPI polarity(%s(%d)), phase(%s(%d)), and direction(%s(%d)).",
                            config.spiPolarity == BusPal::kSpiClockPolarity_ActiveHigh ? "ActiveHigh" : "ActiveLow", config.spiPolarity,
                            config.spiPhase == BusPal::kSpiClockPhase_FirstEdge ? "FirstEdge" : "SecondEdge", config.spiPhase,
                            config.spiDirection == BusPal::kSpiLsbFirst ? "LsbFirst" : "MsbFirst", config.spiDirection));
            }
            break;

        case BusPal::kBusPalTransport_I2C:
            if (!m_busPal.enterI2cMode())
            {
                throw std::runtime_error(format_string("Error: BusPalUartPeripheral() cannot enter I2C Mode."));
            }
            if (!m_busPal.setI2cAddress(config.i2cAddress))
            {
                throw std::runtime_error(format_string("Error: BusPalUartPeripheral() cannot set I2C address %02X.", config.i2cAddress));
            }

            if(!m_busPal.setI2cSpeed(config.i2cSpeedKHz))
            {
                throw std::runtime_error(format_string("Error: BusPalUartPeripheral() cannot set I2C speed(%d KHz).", config.i2cSpeedKHz));
            }
            break;

        default:
            throw std::runtime_error("Unsupported BusPal transport type");
    }
}

status_t BusPalUartPeripheral::read(uint8_t * buffer, uint32_t requestedBytes)
{
    assert(buffer);

    int count = m_busPal.read(buffer, requestedBytes);
    if (count == 0)
    {
        Log::error("Error: Bus Pal read returned 0\n");
        return kStatus_Timeout;
    }

    return kStatus_Success;
}

status_t BusPalUartPeripheral::write(const uint8_t * buffer, uint32_t byteCount)
{
    assert(buffer);

    const int maxBulk = BusPal::kBulkTransferMax;
    int numBulk = (byteCount / maxBulk);
    int remaining = byteCount - (numBulk * maxBulk);
    bool rc = true;

    // Send buffer in max bulk transfer size chunks.
    for (int i = 0; i < numBulk; ++i)
    {
        rc = m_busPal.bulkTransfer(&buffer[i * maxBulk], maxBulk);
        if (!rc)
        {
            return kStatus_Fail;
        }
    }

    // Send the last OR partial chunk.
    if (rc && remaining)
    {
        rc = m_busPal.bulkTransfer(&buffer[numBulk * maxBulk], remaining);
        if (!rc)
        {
            return kStatus_Fail;
        }
    }

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

