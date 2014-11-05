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
#include "blfwk/serial_packetizer.h"
#include "blfwk/host_peripheral.h"
#include "blfwk/Logging.h"
#include "packet/serial_packet.h"
#include "crc/crc16.h"
#include <assert.h>

using namespace blfwk;

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum {
    kReadRetries = 10,
    kReadDelayMilliseconds = 50
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See serial_packetizer.h for documentation of this method.
SerialPacketizer::SerialPacketizer(Peripheral * peripheral)
    : m_peripheral(peripheral)
{
    serial_packet_init(g_bootloaderContext.activePeripheral);
}

// See serial_packetizer.h for documentation of this method.
void SerialPacketizer::init(Peripheral * peripheral)
{
    m_peripheral = peripheral;

    serial_packet_init(g_bootloaderContext.activePeripheral);
}

// See serial_packetizer.h for documentation of this method.
void SerialPacketizer::finalize()
{
    serial_packet_finalize(g_bootloaderContext.activePeripheral);
}

// See serial_packetizer.h for documentation of this method.
status_t SerialPacketizer::writePacket(const uint8_t * packet, uint32_t byteCount, packet_type_t packetType)
{
    return serial_packet_write(g_bootloaderContext.activePeripheral, packet, byteCount, packetType);
}

// See serial_packetizer.h for documentation of this method.
status_t SerialPacketizer::readPacket(uint8_t ** packet, uint32_t * packetLength, packet_type_t packetType)
{
    return serial_packet_read(g_bootloaderContext.activePeripheral, packet, packetLength, packetType);
}

// See serial_packetizer.h for documentation of this method.
void SerialPacketizer::abortPacket()
{
    serial_packet_abort(g_bootloaderContext.activePeripheral);
}

// See serial_packetizer.h for documentation of this method.
uint32_t SerialPacketizer::getMaxPacketSize()
{
    return serial_packet_get_max_packet_size(g_bootloaderContext.activePeripheral);
}

// See serial_packetizer.h for documentation of this method.
status_t SerialPacketizer::ping(int retries, unsigned int delay, ping_response_t * pingResponse, int comSpeed)
{
    uint8_t startByte;
    const int initialRetries = retries;

    framing_header_t pingPacket;
    pingPacket.startByte = kFramingPacketStartByte;
    pingPacket.packetType = kFramingPacketType_Ping;

    // Send ping until we receive a start byte.
    do
    {
        // Send the ping
        if (m_peripheral->write((uint8_t*)&pingPacket, sizeof(pingPacket)) == kStatus_Success)
        {
            unsigned int reads = 0;

            // Try to get our first byte, wait for it to be a start byte
            while(reads++ < kReadRetries)
            {
                if (m_peripheral->read(&startByte, sizeof(startByte)) == kStatus_Success)
                {
                    if (startByte == kFramingPacketStartByte)
                    {
                        break;
                    }
                }

                host_delay(kReadDelayMilliseconds);
            }

            // If we got our start byte, move on to read the response packet
            if (reads <= kReadRetries)
            {
                break;
            }
        }

        host_delay(delay);

    } while (retries--);

    if (retries < 0)
    {
        return 10; //kStatus_Fail;
    }

    Log::info("Ping responded in %d attempt(s)\n", (initialRetries - retries) + 1);

    // Wait for the rest of the ping bytes
    // In the case of testing low baud rates the target needs time to respond
    // 100 baud rate reply is looking for 9 more bytes = 90 bits with start/stop overhead
    // 90 bits / 100 baud = .9 seconds = 900 milliseconds. The additional 20 milliseconds is to ensure
    // that even high baud rates have some sort of delay and to give a little wiggle room for lower baud rates
    if (comSpeed)
    {
        host_delay(((1000 * 90) / comSpeed) + 20);
    }

    // Read response packet type.
    uint8_t packetType;
    status_t status = m_peripheral->read(&packetType, sizeof(packetType));
    if ((status == kStatus_Success) && (packetType == kFramingPacketType_PingResponse))
    {
        // Read response.
        ping_response_t response;
        status_t status = m_peripheral->read((uint8_t *)&response, sizeof(response));
        if (status == kStatus_Success)
        {
            // Validate reponse CRC.

            // Initialize the CRC16 information.
            uint16_t crc16;
            crc16_data_t crcInfo;
            crc16_init(&crcInfo);

            // Include the start byte and packetType in the CRC.
            crc16_update(&crcInfo, &startByte, sizeof(startByte));
            crc16_update(&crcInfo, &packetType, sizeof(packetType));

            // Run CRC on all other bytes except the CRC field.
            crc16_update(&crcInfo, (uint8_t *)&response, sizeof(response) - sizeof(uint16_t));

            // Finalize the CRC calculations
            crc16_finalize(&crcInfo, &crc16);

            if (response.crc16 != crc16)
            {
                Log::info("Error: ping crc16 failed, received 0x%x, expected 0x%x\n", response.crc16, crc16);
                return 7;//kStatus_Fail;
            }

            Log::debug("Framing protocol version = 0x%x, options = 0x%x\n", response.version, response.options);
            if (pingResponse)
            {
                *pingResponse = response;
            }

            return kStatus_Success;
        }
    }

    return 8;//kStatus_Fail;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

