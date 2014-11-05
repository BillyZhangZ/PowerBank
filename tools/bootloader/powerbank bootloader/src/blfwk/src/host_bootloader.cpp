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
#include "blfwk/usb_hid_packetizer.h"
#include "blfwk/host_peripheral.h"
#include "blfwk/uart_peripheral.h"
#include "blfwk/bus_pal_peripheral.h"
#include "blfwk/host_memory.h"
#include "bootloader/context.h"
#include "bootloader/command.h"

using namespace blfwk;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum {
    kBuspalReadRetries = 3
};

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Bootloader global context data.
//!
//! Referenced by both the simulated host and simulated device sides.
bootloader_context_t g_bootloaderContext = {
    &g_memoryInterface,         // Memory interface.
    g_memoryMap,                // Memory map.
    &g_propertyInterface,       // Property store interface.
    &g_commandInterface,        // Command processor interface.
    NULL,                       // Flash driver interface - not used on host
    NULL,                       // Peripheral array - filled in at run-time.
    NULL,                       // Active peripheral - filled in at run-time.
    0                           // Flash driver state - typed to a u32 and unused for host
};

//! @brief Singleton bootloader object.
Bootloader Bootloader::theBootloader;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
// See host_bootloader.h for documentation of this method.
Bootloader * Bootloader::create(Peripheral::PeripheralConfigData& config)
{
    ping_response_t pingResponse;
    Bootloader * bl = getBootloader();
    bl->setPeripheralInterface(&g_hostPeripheralInterface);

    // create logger instance
    if ( Log::getLogger() == NULL )
    {
        bl->m_logger = new StdoutLogger();
        bl->m_logger->setFilterLevel(Logger::kInfo);
        Log::setLogger(bl->m_logger);
    }

    switch (config.peripheralType)
    {
        case Peripheral::kHostPeripheralType_UART:
        {
            UartPeripheral * peripheral = new UartPeripheral(config.comPortName.c_str(), config.comPortSpeed, config.serialReadTimeoutMs);
            SerialPacketizer * packetizer = new SerialPacketizer(peripheral);
            bl->initWithHost(packetizer);
                
            // Send initial ping.
            peripheral->setReadTimeout(10);
            status_t status = packetizer->ping(0, 0, &pingResponse, config.comPortSpeed);
            peripheral->setReadTimeout(config.serialReadTimeoutMs);
            if (status != kStatus_Success)
            {
                bl->flush();
                throw std::runtime_error(format_string("Error: Initial ping failure with status = 0x%08x.", status));
            }

            break;
        }
        case Peripheral::kHostPeripheralType_USB_HID:
        {
            UsbHidPeripheral * peripheral = new UsbHidPeripheral(config.usbHidVid, config.usbHidPid, config.usbHidSerialNumber.c_str());
            UsbHidPacketizer * packetizer = new UsbHidPacketizer((UsbHidPeripheral*)peripheral);
            if (!packetizer->init())
            {
                throw std::runtime_error(format_string("Error: Cannot initialize USB HID device"));
            }
            bl->initWithHost(packetizer);

            break;
        }
        case Peripheral::kHostPeripheralType_BUSPAL_UART:
        {
            BusPalUartPeripheral * peripheral = new BusPalUartPeripheral(config.comPortName.c_str(), config.comPortSpeed, config.busPalConfig);
            SerialPacketizer * packetizer = new SerialPacketizer(peripheral);
            bl->initWithHost(packetizer);

            // Send initial ping.
            // Bus pal peripheral interface will take care of the delays for us.

            uint32_t buspalReadRetries;
#if defined (DEBUG)
            buspalReadRetries = 0;
#else
            buspalReadRetries = kBuspalReadRetries;
#endif
            status_t status = ((SerialPacketizer*)packetizer)->ping(buspalReadRetries, 0, &pingResponse, 0);
            if (status != kStatus_Success)
            {
                bl->flush();
                throw std::runtime_error(format_string("Error: Initial ping failure with status = 0x%08x.", status));
            }

            break;
        }
        default:
            throw std::runtime_error(format_string("Error: Unsupported peripheral type(%d).", config.peripheralType));
    }

    return bl;
}

// See host_bootloader.h for documentation of this method.
Bootloader * Bootloader::getBootloader()
{
    return &theBootloader;
}

// See host_bootloader.h for documentation of this method.
void Bootloader::init(Packetizer * host, Packetizer * device)
{
    m_host = host;
    m_device = device;

    // Initialize the property store component.
    bootloader_property_init();

    g_bootloaderContext.activePeripheral = &m_activePeripheral;

    // Create all memory stores.
    // Stores are never deleted because bootloader is a singleton.
    mem_init();
    FlashMemoryStore * flash = new FlashMemoryStore;
    m_memoryStore.push_back(flash);
    SramMemoryStore * sram = new SramMemoryStore;
    m_memoryStore.push_back(sram);

    // Initialize the command processor component.
    bootloader_command_init();
}

// See host_bootloader.h for documentation of this method.
void Bootloader::run()
{
    // Pump the device state machine.
    while (bootloader_command_pump() == kStatus_Success)
        ;
}

// See host_bootloader.h for documentation of this method.
void Bootloader::setPeripheralInterface(const peripheral_byte_inteface_t * deviceInterface)
{
    m_activePeripheral.byteInterface = deviceInterface;
}

// See host_bootloader.h for documentation of this method.
void Bootloader::setPacketInterface(const peripheral_packet_interface_t * packetInterface)
{
    m_activePeripheral.packetInterface = packetInterface;
}

// See host_bootloader.h for documentation of this method.
bool Bootloader::openStateFiles(const string & pathToDir, bool forceCreate)
{
    // Open all memory stores.
    memory_vector_t::iterator it = m_memoryStore.begin();
    for (; it != m_memoryStore.end(); ++it)
    {
        if (!(*it)->open(pathToDir, forceCreate))
        {
            return false;
        }
    }

    if (!m_optionsStore.init(pathToDir, forceCreate))
    {
        return false;
    }

    //! @todo restore property store
    //m_optionsStore.getProperty(bla);

    m_areStateFilesOpen = true;
    return true;
}

// See host_bootloader.h for documentation of this method.
void Bootloader::flush()
{
    if (m_areStateFilesOpen)
    {
        // Close all memory stores.
        memory_vector_t::iterator it = m_memoryStore.begin();
        for (; it != m_memoryStore.end(); ++it)
        {
            (*it)->close();
        }

        //! @todo restore property store
        //m_optionsStore.setProperty(bla)

        m_optionsStore.persist();

        m_areStateFilesOpen = false;
    }

    // Finalize the packet interface.
    if (m_host)
    {
        m_host->finalize();
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

