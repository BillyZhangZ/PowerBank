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

#ifndef _host_bootloader_h_
#define _host_bootloader_h_

#include "blfwk/host_types.h"
#include "blfwk/host_memory.h"
#include "blfwk/host_command.h"
#include "blfwk/host_packetizer.h"
#include "blfwk/host_peripheral.h"
#include "blfwk/Logging.h"
#include "bootloader/context.h"
#include <string>

//! @brief A vector of memory stores.
typedef std::vector<blfwk::MemoryStore *> memory_vector_t;

namespace blfwk {

//! @brief Command usage string.
const char k_commandUsage[] = "Command:\n\
  reset                        Reset the chip\n\
  get-property <tag>\n\
    1                          Bootloader version\n\
    2                          Available peripherals\n\
    3                          Start of program flash\n\
    4                          Size of program flash\n\
    5                          Size of flash sector\n\
    6                          Blocks in flash array\n\
    7                          Available commands\n\
    8                          CRC check status\n\
    10                         Verify Writes flag\n\
    11                         Max supported packet size\n\
    12                         Reserved regions\n\
    13                         Validate regions flag\n\
    14                         Start of RAM\n\
    15                         Size of RAM\n\
    16                         System device identification\n\
    17                         Flash security state\n\
    18                         Unique device identification\n\
  set-property <tag> <value>\n\
    10                         Verify Writes flag\n\
    13                         Validate regions flag\n\
  flash-erase-region <addr> <byte_count>\n\
                               Erase a region of flash\n\
  flash-erase-all              Erase all flash, excluding protected regions\n\
  flash-erase-all-unsecure     Erase all flash, including protected regions\n\
  read-memory <addr> <byte_count> [<file>]\n\
                               Read memory and write to file or stdout\n\
                                 if no file specified\n\
  write-memory <addr> [<file> | {{<hex-data>}}]\n\
                               Write memory from file or string of hex values,\n\
                               e.g. \"{{11 22 33 44}}\"\n\
  fill-memory <addr> <byte_count> <pattern> [word | short | byte]\n\
                               Fill memory with pattern; size is\n\
                                 word (default), short or byte\n\
  receive-sb-file <file>       Receive SB file\n\
  execute <addr> <arg> <stackpointer>\n\
                               Execute at address with arg and stack pointer\n\
  call <addr> <arg>            Call address with arg\n\
  flash-security-disable <key> Flash Security Disable <8-byte-hex-key>,\n\
                                 e.g. 0102030405060708\n";

/*!
 * @brief Represents the simlation bootloader as a whole.
 *
 * This class provides a convenient way to access other bootloader
 * framework objects.
 *
 * Do not instantiate this class, use the getBootloader() method to
 * access the global bootloader singleton object.
 */
class Bootloader
{
public:
    //! @brief Get the singleton bootloader object.
    static Bootloader * getBootloader();

    //! @brief Initialize the singleton Bootloader and return a reference.
    //!
    //! Populate the the peripheralType member and any other
    //! members relevant to the stated peripheralType.
    //! Caller is responsible for deleting the returned object.
    //!
    //! This factory method is just a shorthand for creating
    //! a subclass directly, for example:
    //! @code
    //!  string_vector_t cmdv;
    //!  cmdv.push_back("my-reset");
    //!  Reset * reset = new Reset(&cmdv);
    //!  reset->init();
    //!  Command * cmd = reset;
    //! @endcode
    //!
    //! @param params Peripheral configuration parameter structure.
    //! @retval Peripheral object
#ifdef WIN32
    __declspec( dllexport ) static Bootloader * create(Peripheral::PeripheralConfigData& config);
#else
    __attribute__((visibility("default"))) static Bootloader * create(Peripheral::PeripheralConfigData& config);
#endif

public:
    //! @brief Default Constructor.
    Bootloader()
        :   m_host(NULL),
            m_device(NULL),
            m_areStateFilesOpen(false),
            m_memoryStore(),
            m_optionsStore(),
            m_activePeripheral()
        {}

    //! @brief Singleton object.
    static Bootloader theBootloader;

public:
    //! @brief Initialize with host and device.
    //!
    //! The host packetizer is the destination for commands and the source of responses.
    //! The device packetizer is the "reverse" object used on the simulated device side.
    //! The packetizers must be initialized with the appropriate peripheral objects.
    //!
    //! @param host Host packetizer
    //! @param device Device packetizer
    void init(Packetizer * host, Packetizer * device);

    //! @brief Initialize with host.
    //!
    //! For use when the host talks directly to hardware instead of
    //! to a simlated device.
    //!
    //! @param host Host packetizer
    void initWithHost(Packetizer * host) { init(host, NULL); }

    //! @brief Initialize with device.
    //!
    //! For use when an application is running only the simlated device.
    //!
    //! @param device Device packetizer
    void initWithDevice(Packetizer * device) { init(NULL, device); }

    //! @brief Inject a command into the bootloader.
    //!
    //! @param cmd The command to send
    void inject(Command & cmd) { cmd.sendTo(*m_host); }

    //! @brief Configure and open state files.
    //!
    //! Must be called to open or create state files.
    //!
    //! @param pathToDir Directory for state files.
    //! @param forceCreate True to re-create state files even if they exist.
    bool openStateFiles(const std::string & pathToDir, bool forceCreate);

    //! @brief Flush state.
    void flush();

    //! @brief Pump the bootloader state machine.
    void run();

    //! @name Accessors.
    //@{

    //! @brief Get the host packetizer.
    Packetizer * getHost() const { return m_host; }

    //! @brief Get the device packetizer.
    Packetizer * getDevice() const { return m_device; }

    //! @brief Get a device state memory store.
    //!
    //! index Index into memory map for the simulated device.
    MemoryStore * getMemoryStore(int index) const { return m_memoryStore[index]; }

    //! @brief Set the peripheral device interface.
    void setPeripheralInterface(const peripheral_byte_inteface_t * deviceInterface);

    //! @brief Set the peripheral packet interface.
    void setPacketInterface(const peripheral_packet_interface_t * packetInterface);

    //@}

protected:
    Packetizer * m_host;                        //!< Packet interface to send commands on.
    Packetizer * m_device;                      //!< Packet interface to receive responses on.
    bool m_areStateFilesOpen;                   //!< True if state files are in use
    memory_vector_t m_memoryStore;              //!< Vector of memory stores, one per map entry.
    OptionsStore m_optionsStore;                //!< Persistent options store.
    peripheral_descriptor_t m_activePeripheral; //!< Descriptor for the active peripheral.
    StdoutLogger * m_logger;                    //!< Singleton logger instance.
};

} // namespace blfwk

#endif // _host_bootloader_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

