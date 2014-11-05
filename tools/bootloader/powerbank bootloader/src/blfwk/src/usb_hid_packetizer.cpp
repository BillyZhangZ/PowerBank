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
#include "blfwk/usb_hid_packetizer.h"
#include "blfwk/Logging.h"
#include "bootloader/bootloader.h"
#include "blfwk/smart_ptr.h"
#include <cstring>

using namespace blfwk;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See usb_hid_packetizer.h for documentation of this method.
UsbHidPacketizer::UsbHidPacketizer(UsbHidPeripheral * peripheral)
    : m_peripheral(peripheral), m_device(NULL)
{
    if (!peripheral)
        m_peripheral = new UsbHidPeripheral(kDefault_Vid, kDefault_Pid);
}

// See usb_hid_packetizer.h for documentation of this method.
bool UsbHidPacketizer::init()
{
    // Open the device using the VID, PID,
    // and optionally the Serial number.
    m_device = hid_open(m_peripheral->getVendorId(), m_peripheral->getProductId(), m_peripheral->getSerialNumber());
    if (!m_device)
    {
        Log::error("usbhid: unable to open device vid=0x%x, pid=0x%x, sn=%s.\n",
            m_peripheral->getVendorId(), m_peripheral->getProductId(), m_peripheral->getSerialNumber());
        return false;
    }

    return true;
}

// See usb_hid_packetizer.h for documentation of this method.
UsbHidPacketizer::~UsbHidPacketizer()
{
    if (m_device)
    {
        hid_close(m_device);
        /* Free static HIDAPI objects. */
        hid_exit();
    }
}

// See usb_hid_packetizer.h for documentation of this method.
status_t UsbHidPacketizer::writePacket(const uint8_t * packet, uint32_t byteCount, packet_type_t packetType)
{
    assert(m_device);
    if (byteCount)
    {
        assert(packet);
    }

    // Determine report ID based on packet type.
    uint8_t reportID;
    switch (packetType)
    {
        case kPacketType_Command:
            reportID = kBootloaderReportID_CommandOut;
            break;
        case kPacketType_Data:
            reportID = kBootloaderReportID_DataOut;
            break;
        default:
            Log::error("usbhid: unsupported packet type %d\n", (int)packetType);
            return kStatus_Fail;
    };

    if (reportID == kBootloaderReportID_DataOut)
    {
        // Check if the target has sent an abort report.
        if (pollForAbortPacket())
        {
            Log::info("usb hid detected receiver data abort\n");
            return kStatus_AbortDataPhase;
        }
    }

    // Construct report contents.
    memset(&m_report, 0, sizeof(m_report));
    m_report.header.reportID = reportID;
    m_report.header.packetLengthLsb = byteCount & 0xff;
    m_report.header.packetLengthMsb = (byteCount >> 8) & 0xff;

    // If not a zero-length report, copy in packet data.
    if (byteCount)
    {
        memcpy(m_report.packet, packet, byteCount);
    }

    uint32_t length = sizeof(m_report.header) + byteCount;

    if (Log::getLogger()->getFilterLevel() == Logger::kDebug2)
    {
        uint8_t * buffer = (uint8_t *)&m_report;

        // Log bytes written in hex
        Log::debug2("[");
        for (int i = 0; i < (int)length; i++)
        {
            Log::debug2("%02x", buffer[i]);
            if (i != (length - 1))
            {
                Log::debug2(" ");
            }
        }
        Log::debug2("]\n");
    }

    // Send report.
#ifdef LINUX
    int count = hid_write(m_device, (unsigned char *)&m_report, sizeof(m_report));
#else
    int count = hid_write_timeout(m_device, (unsigned char *)&m_report, sizeof(m_report), kWriteTimeoutMs);
#endif
    if (count < 0)
    {
        const wchar_t * errorMessage = hid_error(m_device);
        if (errorMessage)
        {
            int len = wcslen(errorMessage);
            smart_array_ptr<char> msg = new char[len + 1];
            wcstombs(msg, errorMessage, len + 1);
            Log::error("%s", msg);
        }
        return kStatus_Fail;
    }

    return kStatus_Success;
}

// See usb_hid_packetizer.h for documentation of this method.
status_t UsbHidPacketizer::readPacket(uint8_t ** packet, uint32_t * packetLength, packet_type_t packetType)
{
    assert(m_device);
    assert(packet);
    assert(packetLength);
    *packet = NULL;
    *packetLength = 0;

    // Determine report ID based on packet type.
    uint8_t reportID;
    switch (packetType)
    {
        case kPacketType_Command:
            reportID = kBootloaderReportID_CommandIn;
            break;
        case kPacketType_Data:
            reportID = kBootloaderReportID_DataIn;
            break;
        default:
            Log::error("usbhid: unsupported packet type %d\n", (int)packetType);
            return kStatus_Fail;
    };

    // Read report.
    int count = hid_read_timeout(m_device, (unsigned char *)&m_report, sizeof(m_report), kReadFlushTimeoutMs);

    // Bail if we got an error (-1), or if the number of bytes read was less than
    // the report header.
    if (count < sizeof(m_report.header))
    {
        if (count == -1)
        {
            return kStatus_Fail;
        }
        else
        {
            return kStatus_Timeout;
        }
    }

    // Check the report ID.
    if (m_report.header.reportID != reportID)
    {
        Log::error("usbhid: received unexpected report=%x\n", m_report.header.reportID);
        return kStatus_Fail;
    }

    // Extract the packet length encoded as bytes 1 and 2 of the report. The packet length
    // is transferred in little endian byte order.
    uint16_t lengthInPacket = m_report.header.packetLengthLsb | (m_report.header.packetLengthMsb << 8);

    // See if we received the data abort packet.
    if (lengthInPacket == 0)
    {
        Log::info("usbhid: received data phase abort\n");
        return kStatus_AbortDataPhase;
    }

    // Make sure we got all of the packet. Target will send the maximum
    // report size, so there may be extra trailing bytes.
    if ((count - sizeof(m_report.header)) < lengthInPacket)
    {
        Log::error("usbhid: received only %d bytes of packet with length %d\n", count - sizeof(m_report.header), lengthInPacket);
        return kStatus_Fail;
    }

    // Return results.
    *packet = m_report.packet;
    *packetLength = lengthInPacket;

    if (Log::getLogger()->getFilterLevel() == Logger::kDebug2)
    {
        uint8_t * buffer = (uint8_t *)&m_report;

        // Log bytes read in hex
        Log::debug2("<");
        for (int i = 0; i < (int)count; i++)
        {
            Log::debug2("%02x", buffer[i]);
            if (i != (count - 1))
            {
                Log::debug2(" ");
            }
        }
        Log::debug2(">\n");
    }

    return kStatus_Success;
}

// See usb_hid_packetizer.h for documentation of this method.
void UsbHidPacketizer::flushInput()
{
    int count;
    do {
        count = hid_read_timeout(m_device, (unsigned char *)&m_report, sizeof(m_report), kReadFlushTimeoutMs);
    } while (count > 0);
}

// See usb_hid_packetizer.h for documentation of this method.
void UsbHidPacketizer::abortPacket()
{
    // Abort data phase by writing a zero-length command packet.
    writePacket(NULL, 0, kPacketType_Command);
    flushInput();
}

bool UsbHidPacketizer::pollForAbortPacket()
{
    // Just check to see if there is data to be read from hid device.
    // No reason to wait (milliseconds = 0), because we aren't really expecting anything.
    int count = hid_read_timeout(m_device, (unsigned char *)&m_abortReport, sizeof(m_abortReport), 0);
    if (count == 0)
    {
        // No abort packet
        return false;
    }
    else
    {
        // Got an abort packet
        return true;
    }
}

// See usb_hid_packetizer.h for documentation of this method.
uint32_t UsbHidPacketizer::getMaxPacketSize()
{
    return kMinPacketBufferSize;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

