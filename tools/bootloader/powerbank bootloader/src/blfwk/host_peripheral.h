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

#ifndef _host_peripheral_h_
#define _host_peripheral_h_

#include "blfwk/bus_pal.h"
#include "bootloader/peripheral.h"
#include "fsl_platform_status.h"

namespace blfwk {

/*!
 * @brief Represents a peripheral.
 *
 * Interface class for objects that provide the source for commands or sink for responses.
 */
class Peripheral
{
public:

    enum _host_peripheral_types
    {
        kHostPeripheralType_None,
        kHostPeripheralType_UART,
        kHostPeripheralType_BUSPAL_UART,
        kHostPeripheralType_USB_HID
    };

    struct PeripheralConfigData
    {
        _host_peripheral_types peripheralType;
        std::string comPortName;
        long comPortSpeed;
        uint32_t serialReadTimeoutMs;
        unsigned short usbHidVid;
        unsigned short usbHidPid;
        std::string usbHidSerialNumber;
        BusPal::BusPalConfigData busPalConfig;
    };

    //! @brief Read bytes.
    virtual status_t read(uint8_t * buffer, uint32_t requestedBytes) = 0;

    //! @brief Write bytes.
    virtual status_t write(const uint8_t * buffer, uint32_t byteCount) = 0;
};

/*!
 * @brief Represents a USB HID peripheral.
 *
 * Interface class for objects that provide the source for commands or sink for responses.
 */
class UsbHidPeripheral : public Peripheral
{
public:
    //! @brief Parameterized constructor to hold information for UsbHidPacketizer.
    //!
    //! Dummy implementations of read() and write().
    //!
    //! @param port OS file path for COM port. For example "COM1" on Windows.
    //! @param speed Port speed, e.g. 9600.
    UsbHidPeripheral(unsigned short vendor_id, unsigned short product_id, const char *serial_number = NULL)
    :    m_vendor_id(vendor_id), m_product_id(product_id)
    {
        // Convert to a wchar_t*
        std::string s(serial_number);
        m_serial_number.assign(s.begin(), s.end());

//        size_t origsize = strlen(serial_number) + 1;
//        const size_t newsize = 100;
//        size_t convertedChars = 0;
//        m_serial_number.[newsize];
//        mbstowcs_s(&convertedChars, m_serial_number, origsize, serial_number, _TRUNCATE);
//        wcscat_s(m_serial_number, L" (wchar_t *)");

    }

    //! @brief Dummy read() to satisfy interface.
    //!
    //! @param buffer Pointer to buffer
    //! @param requestedBytes Number of bytes to read
    virtual status_t read(uint8_t * buffer, uint32_t requestedBytes) { return kStatus_Success; }

    //! @brief Dummy write() to satisfy interface
    //!
    //! @param buffer Pointer to buffer to write
    //! @param byteCount Number of bytes to write
    virtual status_t write(const uint8_t * buffer, uint32_t byteCount)  { return kStatus_Success; }

    //! @brief Return USB Vendor ID
    unsigned short getVendorId() { return m_vendor_id; }

    //! @brief Return USB Product ID
    unsigned short getProductId() { return m_product_id; }

    //! @brief Return USB Serial Number
    const wchar_t * getSerialNumber() { return m_serial_number.c_str(); }

private:
    unsigned short m_vendor_id;
    unsigned short m_product_id;
    std::wstring m_serial_number;
};

} // namespace blfwk

extern const peripheral_byte_inteface_t g_hostPeripheralInterface;
extern const peripheral_byte_inteface_t g_devicePeripheralInterface;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

//! @brief Read bytes.
//!
//! @param buffer Pointer to buffer
//! @param requestedBytes Number of bytes to read
//! @param actualBytes Pointer to location to write the number of bytes actually read
//!
//! @retval kStatus_Success
status_t host_peripheral_read(const peripheral_descriptor_t * self, uint8_t * buffer, uint32_t requestedBytes);

//! @brief Write bytes.
//!
//! @param buffer Pointer to buffer to write
//! @param byteCount Number of bytes to write
//! @param actualBytes Pointer to location to write the number of bytes actually written
//!
//! @retval kStatus_Success
status_t host_peripheral_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount);

//! @brief Read bytes.
//!
//! @param buffer Pointer to buffer
//! @param requestedBytes Number of bytes to read
//! @param actualBytes Pointer to location to write the number of bytes actually read
//!
//! @retval kStatus_Success
status_t device_peripheral_read(const peripheral_descriptor_t * self, uint8_t * buffer, uint32_t requestedBytes);

//! @brief Write bytes.
//!
//! @param buffer Pointer to buffer to write
//! @param byteCount Number of bytes to write
//! @param actualBytes Pointer to location to write the number of bytes actually written
//!
//! @retval kStatus_Success
status_t device_peripheral_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount);

#if defined(__cplusplus)
}
#endif // __cplusplus

#endif // _host_peripheral_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

