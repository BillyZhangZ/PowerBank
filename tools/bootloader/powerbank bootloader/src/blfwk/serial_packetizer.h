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

#ifndef _serial_packetizer_h_
#define _serial_packetizer_h_

#include "blfwk/host_packetizer.h"
#include "packet/serial_packet.h"
#include <assert.h>

namespace blfwk {

// Forward declarations.
class Peripheral;

/*!
 * @brief Provides source and sink for packets that go over the serial peripherals.
 */
class SerialPacketizer : public Packetizer
{
public:
    //! @brief Constructor.
    SerialPacketizer(Peripheral * peripheral = NULL);

    //! @brief Initialize.
    void init(Peripheral * peripheral);

    //! @brief Peripheral accessor.
    virtual Peripheral * getPeripheral() { return m_peripheral; }

    //! @brief Read a packet.
    //!
    //! Provides the address of a buffer containing the packet.
    //!
    //! @param packet Pointer location to write packet pointer
    //! @param packetLength Number of bytes in returned packet
    virtual status_t readPacket(uint8_t ** packet, uint32_t * packetLength, packet_type_t packetType);

    //! @brief Write a packet.
    //!
    //! @param packet Pointer to packet to write
    //! @param byteCount Number of bytes in packet
    virtual status_t writePacket(const uint8_t * packet, uint32_t byteCount, packet_type_t packetType);

    //! @brief Abort data phase.
    virtual void abortPacket();

    //! @brief Finalize.
    virtual void finalize();

    //! @brief Enable simulator command processor pump.
    virtual void enableSimulatorPump() {}

    //! @brief Pump simulator command processor.
    virtual void pumpSimulator() {}

    //! @brief Set aborted flag.
    virtual void setAborted(bool aborted) {}

    //! @brief Return the max packet size.
    virtual uint32_t getMaxPacketSize();

    //! @brief Send a ping packet and receive an ack.
    //!
    //! This is a method for host only side pinging of the target. The reponse from the
    //! target to a ping packet is a ping response packet. Since the target may or may
    //! not be online there is optionally a series of retries to make the best attempt
    //! at communication possible
    //!
    //! @param retries The number of attempts that should be made.
    //! @param delay The time in milliseconds between each attempt.
    //! @param comSpeed The peripheral baud rate. Used in order to calculate the
    //!     receive delay in the case of low com speeds such as 100 and 300 which need
    //!     nearly a second to complete
    virtual status_t ping(int retries, unsigned int delay, ping_response_t * response, int comSpeed);

protected:
    Peripheral * m_peripheral;          //!< Peripheral to send/receive bytes on.
};

} // namespace blfwk

#endif // _serial_packetizer_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

