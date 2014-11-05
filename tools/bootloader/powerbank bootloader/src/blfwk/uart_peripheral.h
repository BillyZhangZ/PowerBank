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

#ifndef _uart_peripheral_h_
#define _uart_peripheral_h_

#include "blfwk/host_peripheral.h"
#include "blfwk/host_command.h"

namespace blfwk {

/*!
 * @brief Peripheral that talks to the target device over COM port hardware.
 */
class UartPeripheral : public Peripheral
{
public:
    //! @breif Constants.
    enum _uart_peripheral_constants
    {
        kUartPeripheral_DefaultReadTimeoutMs = 5000,
        kUartPeripheral_DefaultBaudRate      = 9600
    };

public:
    //! @brief Default Constructor.
    UartPeripheral()
    :   m_fileDescriptor(-1)
    {}

    //! @brief Parameterized constructor that opens the serial port.
    //!
    //! Opens and configures the port. Throws exception if port configuration fails.
    //!
    //! Note: following COM port configuration is assumed: 8 bits, 1 stop bit, no parity.
    //!
    //! @param port OS file path for COM port. For example "COM1" on Windows.
    //! @param speed Port speed, e.g. 9600.
    //! @param serialTimeoutMs Serial read timeout in milliseconds, e.g. 5000.
    UartPeripheral(const char * port, long speed = kUartPeripheral_DefaultBaudRate,
        uint32_t serialTimeoutMs = kUartPeripheral_DefaultReadTimeoutMs);

    //! @brief Destructor.
    virtual ~UartPeripheral();

    //! @brief Initialize.
    //!
    //! Opens and configures the port.
    //!
    //! Note: following COM port configuration is assumed: 8 bits, 1 stop bit, no parity.
    //!
    //! @param port OS file path for COM port. For example "COM1" on Windows.
    //! @param speed Port speed, e.g. 9600.
    //! @param serialTimeoutMs Serial read timeout in milliseconds, e.g. 5000.
    bool init(const char * port, long speed, uint32_t serialTimeoutMs);

    //! @brief Initialize.
    //!
    //! Opens and configures with default COM port and speed.
    bool init() { return init("COM1", kUartPeripheral_DefaultBaudRate, kUartPeripheral_DefaultReadTimeoutMs); }

    //! @brief Set read timeout.
    //!
    //! Set maximum time in milliseconds to wait for a response from the target device.
    void setReadTimeout(uint32_t timeoutMs);

    //! @brief Flush.
    //!
    //! should be called on an open COM port in order to flush any remaining data in the UART RX buffer
    void flushRX();

    //! @brief Read bytes.
    //!
    //! @param buffer Pointer to buffer
    //! @param requestedBytes Number of bytes to read
    virtual status_t read(uint8_t * buffer, uint32_t requestedBytes);

    //! @brief Write bytes.
    //!
    //! @param buffer Pointer to buffer to write
    //! @param byteCount Number of bytes to write
    virtual status_t write(const uint8_t * buffer, uint32_t byteCount);

protected:
    int m_fileDescriptor;                       //!< Port file descriptor.
    uint8_t m_buffer[kDefaultMaxPacketSize];    //!< Buffer for bytes used to build read packet.
};

} // namespace blfwk

#endif // _uart_peripheral_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
