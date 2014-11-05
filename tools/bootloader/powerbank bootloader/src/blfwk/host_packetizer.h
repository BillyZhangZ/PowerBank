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

#ifndef _host_packetizer_h_
#define _host_packetizer_h_

#include "blfwk/host_types.h"
#include "bootloader/peripheral.h"
#include <assert.h>

namespace blfwk {

// Forward declarations.
class Peripheral;

/*!
 * @brief Interface class for packetization of commands and data.
 */
class Packetizer
{
public:
    //! @brief Peripheral accessor.
    virtual Peripheral * getPeripheral() = 0;

    //! @brief Read a packet.
    virtual status_t readPacket(uint8_t ** packet, uint32_t * packetLength, packet_type_t packetType) = 0;

    //! @brief Write a packet.
    virtual status_t writePacket(const uint8_t * packet, uint32_t byteCount, packet_type_t packetType) = 0;

    //! @brief Abort data phase.
    virtual void abortPacket() = 0;

    //! @brief Finalize.
    virtual void finalize() = 0;

    //! @brief Enable simulator command processor pump.
    virtual void enableSimulatorPump() = 0;

    //! @brief Pump simulator command processor.
    virtual void pumpSimulator() = 0;

    //! @brief Set aborted flag.
    //!
    //! Used for out-of-band flow control for simulator.
    virtual void setAborted(bool aborted) = 0;

    //! @brief Return the max packet size.
    virtual uint32_t getMaxPacketSize() = 0;
};

} // namespace blfwk

extern const peripheral_packet_interface_t g_hostPacketInterface;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @brief Read a packet.
//!
//! Provides the address of a buffer containing the packet.
//!
//! @param packet Pointer location to write packet pointer
//! @param packetLength Number of bytes in returned packet.
//! @retval kStatus_Success
status_t host_packet_read(const peripheral_descriptor_t * self, uint8_t ** packet, uint32_t * packetLength, packet_type_t packetType);

//! @brief Write a packet.
//!
//! @param packet Pointer to packet to write.
//! @param byteCount Number of bytes in packet.
//! @retval kStatus_Success
status_t host_packet_write(const peripheral_descriptor_t * self, const uint8_t * packet, uint32_t byteCount, packet_type_t packetType);

//! @brief Abort a packet.
void host_packet_abort(const peripheral_descriptor_t * self);

//! @brief Finalize.
status_t host_packet_finalize(const peripheral_descriptor_t * self);

//! @brief Return the max packet size.
uint32_t host_get_max_packet_size(const peripheral_descriptor_t * self);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // _host_packetizer_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

