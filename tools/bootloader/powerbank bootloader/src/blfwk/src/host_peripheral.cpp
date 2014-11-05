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

#include <algorithm>
#include "blfwk/host_bootloader.h"
#include "blfwk/host_peripheral.h"

using namespace blfwk;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Interface to host peripheral operations.
const peripheral_byte_inteface_t g_hostPeripheralInterface = {
    NULL,                       // init
    host_peripheral_read,       // read
    host_peripheral_write       // write
};

//! @brief Interface to device peripheral operations.
const peripheral_byte_inteface_t g_devicePeripheralInterface = {
    NULL,                       // init
    device_peripheral_read,     // read
    device_peripheral_write     // write
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See host_peripheral.h for documentation of this method.
status_t host_peripheral_read(const peripheral_descriptor_t * self, uint8_t * buffer, uint32_t requestedBytes)
{
    Bootloader * bl = Bootloader::getBootloader();
    Peripheral * device = bl->getHost()->getPeripheral();
    return device->read(buffer, requestedBytes);
}

// See host_peripheral.h for documentation of this method.
status_t host_peripheral_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount)
{
    Bootloader * bl = Bootloader::getBootloader();
    Peripheral * device = bl->getHost()->getPeripheral();
    device->write(buffer, byteCount);
    return kStatus_Success;
}

// See host_peripheral.h for documentation of this method.
status_t device_peripheral_read(const peripheral_descriptor_t * self, uint8_t * buffer, uint32_t requestedBytes)
{
    Bootloader * bl = Bootloader::getBootloader();
    Peripheral * device = bl->getDevice()->getPeripheral();
    device->read(buffer, requestedBytes);
    return kStatus_Success;
}

// See host_peripheral.h for documentation of this method.
status_t device_peripheral_write(const peripheral_descriptor_t * self, const uint8_t * buffer, uint32_t byteCount)
{
    Bootloader * bl = Bootloader::getBootloader();
    Peripheral * device = bl->getDevice()->getPeripheral();
    device->write(buffer, byteCount);
    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

