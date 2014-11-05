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

#include "blfwk/host_bootloader.h"
#include "blfwk/host_packetizer.h"
#include "blfwk/host_peripheral.h"
#include "blfwk/Logging.h"
#include "packet/serial_packet.h"
#include "bootloader/command.h"
#ifdef WIN32
#include <windows.h>
#endif

using namespace blfwk;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Interface to host packet operations.
const peripheral_packet_interface_t g_hostPacketInterface = {
    NULL,                   // init
    host_packet_read,       // readPacket
    host_packet_write,      // writePacket
    host_packet_abort,      // abortPacket
    host_packet_finalize,   // finalize
    host_get_max_packet_size, // getMaxPacketSize
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See host_packetizer.h for documentation of this method.
status_t host_packet_read(const peripheral_descriptor_t * self, uint8_t ** packet, uint32_t * packetLength, packet_type_t packetType)
{
    Bootloader * bl = Bootloader::getBootloader();
    Packetizer * device = bl->getDevice();
    return device->readPacket(packet, packetLength, packetType);
}

// See host_packetizer.h for documentation of this method.
status_t host_packet_write(const peripheral_descriptor_t * self, const uint8_t * packet, uint32_t byteCount, packet_type_t packetType)
{
    Bootloader * bl = Bootloader::getBootloader();
    Packetizer * device = bl->getDevice();
    return device->writePacket(packet, byteCount, packetType);
}

// See host_packetizer.h for documentation of this method.
void host_packet_abort(const peripheral_descriptor_t * self)
{
    Bootloader * bl = Bootloader::getBootloader();
    Packetizer * host = bl->getHost();
    host->setAborted(true);
}

// See host_packetizer.h for documentation of this method.
status_t host_packet_finalize(const peripheral_descriptor_t * self)
{
    Bootloader * bl = Bootloader::getBootloader();
    Packetizer * host = bl->getHost();
    host->finalize();
    return kStatus_Success;
}

uint32_t host_get_max_packet_size(const peripheral_descriptor_t * self)
{
    Bootloader * bl = Bootloader::getBootloader();
    Packetizer * host = bl->getHost();
    return host->getMaxPacketSize();
}

void host_delay(uint32_t milliseconds)
{
    // @todo implement for non-win32
#ifdef WIN32
    Sleep(milliseconds);
#endif
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

