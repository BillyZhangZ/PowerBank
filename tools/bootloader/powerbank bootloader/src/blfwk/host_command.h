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

#ifndef _host_command_h_
#define _host_command_h_

#include <string>
#include "blfwk/host_types.h"
#include "blfwk/host_packetizer.h"
#include "blfwk/json.h"
#include "blfwk/format_string.h"
#include "packet/command_packet.h"
#include "blfwk/DataSource.h"

using namespace std;

namespace blfwk {

//! @name Command tags, masks, names.
//@{
struct cmd_t
{ 
    uint8_t tag; 
    uint32_t mask; 
    const char * const name;

    cmd_t(uint8_t tag, uint32_t mask, const char * name)
        : tag(tag), mask(mask), name(name) {}
};
const cmd_t kCommand_FlashEraseAll        (kCommandTag_FlashEraseAll,         0x00000001, "flash-erase-all");
const cmd_t kCommand_FlashEraseRegion     (kCommandTag_FlashEraseRegion,      0x00000002, "flash-erase-region");
const cmd_t kCommand_ReadMemory           (kCommandTag_ReadMemory,            0x00000004, "read-memory");
const cmd_t kCommand_WriteMemory          (kCommandTag_WriteMemory,           0x00000008, "write-memory");
const cmd_t kCommand_FillMemory           (kCommandTag_FillMemory,            0x00000010, "fill-memory");
const cmd_t kCommand_FlashSecurityDisable (kCommandTag_FlashSecurityDisable,  0x00000020, "flash-security-disable");
const cmd_t kCommand_GetProperty          (kCommandTag_GetProperty,           0x00000040, "get-property");
const cmd_t kCommand_ReceiveSbFile        (kCommandTag_ReceiveSbFile,         0x00000080, "receive-sb-file");
const cmd_t kCommand_Execute              (kCommandTag_Execute,               0x00000100, "execute");
const cmd_t kCommand_Call                 (kCommandTag_Call,                  0x00000200, "call");
const cmd_t kCommand_Reset                (kCommandTag_Reset,                 0x00000400, "reset");
const cmd_t kCommand_SetProperty          (kCommandTag_SetProperty,           0x00000800, "set-property");
const cmd_t kCommand_FlashEraseAllUnsecure(kCommandTag_FlashEraseAllUnsecure, 0x00001000, "flash-erase-all-unsecure");
//@}

//! @name Property tags.
//@{
struct property_t
{ 
    uint32_t value; 
    const char * description;

    property_t(uint8_t value, const char * description)
        : value(value), description(description) {}

    std::string str() const { return format_string("0x%02x", value); }
};
const property_t kProperty_CurrentVersion       (0x01, "current-version");
const property_t kProperty_AvailablePeripherals (0x02, "available-peripherals");
const property_t kProperty_FlashStartAddress    (0x03, "flash-start-address");
const property_t kProperty_FlashSizeInBytes     (0x04, "flash-size-in-bytes");
const property_t kProperty_FlashSectorSize      (0x05, "flash-sector-size");
const property_t kProperty_FlashBlockCount      (0x06, "flash-block-count");
const property_t kProperty_AvailableCommands    (0x07, "available-commands");
const property_t kProperty_CrcCheckStatus       (0x08, "crc-check-status");
const property_t kProperty_VerifyWrites         (0x0a, "verify-writes");
const property_t kProperty_MaxPacketSize        (0x0b, "max-packet-size");
const property_t kProperty_ReservedRegions      (0x0c, "reserved-regions");
const property_t kProperty_ValidateRegions      (0x0d, "validate-regions");
const property_t kProperty_RAMStartAddress      (0x0e, "ram-start-address");
const property_t kProperty_RAMSizeInBytes       (0x0f, "ram-size-in-bytes");
const property_t kProperty_SystemDeviceId       (0x10, "system-device-id");
const property_t kProperty_FlashSecurityState   (0x11, "flash-security-state");
const property_t kProperty_UniqueDeviceId       (0x12, "unique-device-id");

const property_t kProperty_Invalid              (0xFF, "invalid");
//@}

//! @brief Entry in a lookup table of status messages.
//!
//! This struct maps a status value to a description of that status.
struct StatusMessageTableEntry
{
    int32_t status; //!< Status code value.
    const std::string message;  //!< Description of the status.
};

//! @brief Status return code descriptions.
//!
//! @warning These strings need to be kept in sync with the platform status codes
//! in the src/include/fsl_platform_status.h file.
extern StatusMessageTableEntry g_statusCodes[];

//! @name Commands
//@{

/*!
 * @brief Represents a bootloader command.
 *
 * Do not instantiate this class. Instead, use the create() method to
 * create the appropriate subclass based on command name in argv[0].
 */
class Command
{
public:
    //! @brief Create an appropriate command subclass.
    //!
    //! Pass the command name in argv[0] and optional
    //! arguments in the rest of the string vector.
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
    //! @param argv Argument vector
    //! @retval Command object
    static Command * create(const string_vector_t * argv);

protected:
    //! @brief Constructor that takes a command name and list of arguments.
    //!
    //! @param argv Argument vector
    Command(const string_vector_t * argv) : m_argv(*argv), m_responseValues() {}

    //! @brief Constructor that takes a command name.
    //!
    //! @param name Name of the command
    Command(const char * const name) : m_argv(1, name), m_responseValues() {}

public:
    //! @brief Destructor.
    virtual ~Command() {}

    //! @brief Initialize.
    //!
    //! Subclasses should implement init() to check for valid arguments.
    virtual bool init() { return true; }

    //! @name String arguments accessors.
    //@{

    //! @brief Get the specified argument.
    virtual std::string getArg(int arg) const { return m_argv.at(arg); }

    //! @brief Get the command name (i.e. argv[0]).
    virtual std::string getName() const { return getArg(0); }

    //! @brief Get the number of arguments, including the command name.
    virtual size_t getArgCount() const { return m_argv.size(); }

    //@}

    //! @brief Send command to packetizer and on to peripheral.
    virtual void sendTo(Packetizer & packetizer) {}

    //! @brief Get response values vector.
    virtual const uint32_vector_t * getResponseValues() const
    {
        return const_cast<uint32_vector_t *>(&m_responseValues);
    }

    //! @brief Get response as JSON string.
    virtual std::string getResponse() const;

    //! @brief Get a status code description.
    virtual std::string getStatusMessage(status_t code) const;

    //! @brief Log the response description.
    void logResponses() const;

protected:
    //! @brief Check generic response packet.
    //!
    //! @param packet Packet received from device
    //! @param commandTag Expected command tag in packet
    virtual bool processResponse(const generic_response_packet_t * packet, uint8_t commandTag);

protected:
    string_vector_t m_argv;             //!< Vector of argument strings.
    uint32_vector_t m_responseValues;   //!< Vector of response values.
};

/*!
 * @brief Command packet operations.
 *
 * Formats command packets and runs the command phase.
 */
class CommandPacket
{
protected:
    //! @brief Constants.
    enum _command_packet_constants {
        kMaxCommandArguments = (kDefaultMaxPacketSize - sizeof(command_packet_t)) / sizeof(uint32_t)    //!< 7 args max for packet size 32 bytes
    };

    //! Format of command packet.
    struct PacketWithArgs
    {
        //! @brief Initialize the command packet.
        void init(uint8_t tag, uint8_t numArguments)
        {
            m_header.commandTag = tag;
            m_header.flags = 0;
            m_header.reserved = 0;
            m_header.parameterCount = numArguments;
        }
        command_packet_t m_header;  //!< Packet header.
        uint32_t m_arguments[kMaxCommandArguments]; //!< Command arguments.
    };

public:
    //! @brief Constructor that takes no command arguments.
    CommandPacket(uint8_t tag)
    {
        m_numArguments = 0;
        m_packet.init(tag, m_numArguments);
    }

    //! @brief Constructor that takes one command argument.
    CommandPacket(uint8_t tag, uint32_t arg)
    {
        m_numArguments = 1;
        m_packet.init(tag, m_numArguments);
        m_packet.m_arguments[0] = arg;
    }

    //! @brief Constructor that takes two command arguments.
    CommandPacket(uint8_t tag, uint32_t arg1, uint32_t arg2)
    {
        m_numArguments = 2;
        m_packet.init(tag, m_numArguments);
        m_packet.m_arguments[0] = arg1;
        m_packet.m_arguments[1] = arg2;
    }

    //! @brief Constructor that takes three command arguments.
    CommandPacket(uint8_t tag, uint32_t arg1, uint32_t arg2, uint32_t arg3)
    {
        m_numArguments = 3;
        m_packet.init(tag, m_numArguments);
        m_packet.m_arguments[0] = arg1;
        m_packet.m_arguments[1] = arg2;
        m_packet.m_arguments[2] = arg3;
    }

    //! @brief Get size of command packet, including arguments.
    uint32_t getSize() const { return sizeof(command_packet_t) + (m_numArguments * sizeof(uint32_t)); }

    //! @brief Get pointer to command packet data.
    const uint8_t * getData() { return reinterpret_cast<uint8_t *>(&m_packet.m_header); }

    //! @brief Send command packet and read response.
    //!
    //! @return Pointer to response packet.
    const uint8_t * sendCommandGetResponse(Packetizer & device);

protected:
    int m_numArguments;         //!< Number of command arguments.
    PacketWithArgs m_packet;    //!< Command packet data.
};

/*!
 * @brief Data packet operations.
 *
 * Formats data packets and runs data phase.
 */
class DataPacket
{
public:
    /*!
     * @brief Abstract class to provide data for data phase.
     */
    class DataProducer
    {
    public:
        //! @brief Query if more data is available.
        virtual bool hasMoreData() const = 0;

        //! @brief Query the total size of the data.
        virtual uint32_t getDataSize() const = 0;

        //! @brief Get the next data chunk.
        //!
        //! Before calling getData(), call moreData() to determine if
        //! data is available.
        virtual uint32_t getData(uint8_t * data, uint32_t size) = 0;
    };

    /*!
     * @brief Provide file data for data phase.
     */
    class FileDataProducer : public DataProducer
    {
    public:
        //! @brief Default constructor.
        FileDataProducer()
        :   m_filePath(),
            m_filePointer(NULL),
            m_fileSize(0)
        {}

        //! @brief Destructor.
        virtual ~FileDataProducer() { if (m_filePointer) fclose(m_filePointer); }

        //! @brief Initialize with a file path.
        bool init(std::string filePath);

        //! \name DataProducer
        //@{
        //! @brief Query if more data is available.
        virtual bool hasMoreData() const
        {
            assert(m_filePointer);
            return (m_fileSize && !feof(m_filePointer));
        }

        //! @brief Query the total size of the data.
        virtual uint32_t getDataSize() const { return (uint32_t)m_fileSize; }

        //! @brief Get the next data chunk.
        //!
        //! Before calling getData(), call moreData() to determine if
        //! data is available.
        virtual uint32_t getData(uint8_t * data, uint32_t size);
        //@}

    protected:
        std::string m_filePath;     //!< Data file path.
        FILE * m_filePointer;       //!< Data file pointer.
        long m_fileSize;            //!< Size in bytes of data file.
    };

    /*!
     * @brief Provide data from hex string for data phase.
     */
    class HexDataProducer : public DataProducer
    {
    public:
        //! @brief Default constructor.
        HexDataProducer()
        :   m_byteIndex(0),
            m_data()
        {}

        //! @brief Constructor that takes a vector<uchar> parameter.
        HexDataProducer(const uchar_vector_t & data)
        :   m_byteIndex(0),
            m_data(data)
        {}

        //! @brief Initialize with a data string.
        //!
        //! @param hexData String with hex digits surrounded by double brackets, e.g. '{{11 22 33}}', white space ignored
        //! @return Number of hex bytes parsed from the string.
        uint32_t initFromString(const std::string hexData);

        //! @brief Destructor.
        virtual ~HexDataProducer() {}

        //! \name DataProducer
        //@{
        //! @brief Query if more data is available.
        virtual bool hasMoreData() const
        {
            return (m_byteIndex < m_data.size());
        }

        //! @brief Query the total size of the data.
        virtual uint32_t getDataSize() const { return m_data.size(); }

        //! @brief Get the next data chunk.
        //!
        //! Before calling getData(), call moreData() to determine if
        //! data is available.
        virtual uint32_t getData(uint8_t * data, uint32_t size);
        //@}

    protected:
        uint32_t m_byteIndex;       //!< Current byte index.
        uchar_vector_t m_data ;     //!< Data byte vector.
    };

    /*!
     * @brief Provide DataSource::Segment data for data phase.
     */
    class SegmentDataProducer : public DataProducer
    {
    public:
        //! @brief Default constructor.
        SegmentDataProducer(blfwk::DataSource::Segment * segment)
        :    m_segment(segment), m_byteIndex(0) {}

        //! @brief Destructor.
        virtual ~SegmentDataProducer() {}

        //! @brief Initialize with a file path.
        bool init(std::string filePath);

        //! \name DataProducer
        //@{
        //! @brief Query if more data is available.
        virtual bool hasMoreData() const
        {
            return (m_byteIndex < m_segment->getLength());
        }

        //! @brief Query the total size of the data.
        virtual uint32_t getDataSize() const { return m_segment->getLength(); }

        //! @brief Get the next data chunk.
        //!
        //! Before calling getData(), call moreData() to determine if
        //! data is available.
        virtual uint32_t getData(uint8_t * data, uint32_t size);
        //@}

    protected:
        blfwk::DataSource::Segment * m_segment;  //!< DataSource::Segment object.
        uint32_t m_byteIndex;             //!< Current byte index.
    };


    /*!
     * @brief Abstract class to consume data from data phase.
     */
    class DataConsumer
    {
    public:
        //! @brief Process the next data chunk.
        virtual void processData(const uint8_t * data, uint32_t size) = 0;

        //! @brief Finalize processing.
        virtual void finalize() = 0;
    };

    /*!
     * @brief Write file data for data phase receive.
     */
    class FileDataConsumer : public DataConsumer
    {
    public:
        //! @brief Default constructor.
        FileDataConsumer()
        :   m_filePath(),
            m_filePointer(NULL)
        {}

        //! @brief Destructor.
        virtual ~FileDataConsumer() { if (m_filePointer) fclose(m_filePointer); }

        //! @brief Initialize with a file path.
        bool init(std::string filePath);

        //! @brief Process the next data chunk.
        virtual void processData(const uint8_t * data, uint32_t size);

        //! @brief Finalize processing.
        virtual void finalize() {}

    protected:
        std::string m_filePath;     //!< Data file path.
        FILE * m_filePointer;       //!< Data file pointer.
    };

    /*!
     * @brief Print data for data phase receive.
     */
    class StdOutDataConsumer : public DataConsumer
    {
    public:
        enum _constants {
            kBytesPerLine = 16  //!< Number of hex bytes to display per line
        };

    public:
        //! @brief Constructor.
        StdOutDataConsumer()
        :   m_currentCount(1)
        {}

        //! @brief Finalize processing.
        virtual void finalize()
        {
            if (((m_currentCount - 1) % kBytesPerLine) != 0)
            {
                printf("\n");
            }
        }

        //! @brief Process the next data chunk.
        virtual void processData(const uint8_t * data, uint32_t size);

    protected:
        uint32_t m_currentCount;    //!< Current byte being processed, starts at 1
    };

public:
    //! @brief Constructor that takes a DataProducer.
    DataPacket(DataProducer * dataProducer)
    {
        assert(dataProducer);
        m_dataProducer = dataProducer;
    }

    //! @brief Constructor that takes a DataConsumer.
    DataPacket(DataConsumer * dataConsumer)
    {
        assert(dataConsumer);
        m_dataConsumer = dataConsumer;
    }

    //! @brief Send data packet to device.
    //!
    //! Calls the data provide to get the data to send.
    uint8_t * sendTo(Packetizer & device);

    //! @brief Receive data packet from device.
    //!
    //! Calls the data consumer to process the receied data.
    uint8_t * receiveFrom(Packetizer & device, uint32_t byteCount);

protected:
    DataProducer * m_dataProducer;      //!< Provides data for the packet.
    DataConsumer * m_dataConsumer;      //!< Process the data in the packet.
    uint8_t m_packet[kMinPacketBufferSize]; //!< The data packet.
};

/*!
 * @brief Represents the bootloader Reset command.
 *
 * The Reset command has no arguments.
 */
class Reset : public Command
{
public:
    //! @brief Constructor that takes argument vector.
    Reset(const string_vector_t * argv) : Command(argv) {}

    //! @brief Default constructor.
    Reset() : Command(kCommand_Reset.name) {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_Reset);
    }
};

/*!
 * @brief Represents the bootloader GetProperty command.
 */
class GetProperty : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    GetProperty(const string_vector_t * argv) : Command(argv) {}

    //! @brief Constructor that takes a property_t argument.
    GetProperty(property_t property)
        :   Command(kCommand_GetProperty.name),
        m_propertyTag(property.value)
    {
        m_argv.push_back(format_string("0x%08x", property.value));
    }

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const get_property_response_packet_t * packet);

    //! @brief Check generic response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_GetProperty);
    }

protected:
    uint32_t m_propertyTag;     //!< Property tag.
};

/*!
 * @brief Represents the bootloader GetProperty command.
 */
class SetProperty : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    SetProperty(const string_vector_t * argv) : Command(argv) {}

    //! @brief Constructor that takes a property_t argument and the value to set.
    SetProperty(property_t property, uint32_t value)
        :   Command(kCommand_SetProperty.name),
        m_propertyTag(property.value), m_propertyValue(value)
    {
        m_argv.push_back(format_string("0x%08x", property.value));
        m_argv.push_back(format_string("0x%08x", value));
    }

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check generic response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_SetProperty);
    }

protected:
    uint32_t m_propertyTag;     //!< Property tag.
    uint32_t m_propertyValue;   //!< Value to set.
};

/*!
 * @brief Represents the bootloader Flash Erase command.
 */
class FlashEraseRegion : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    FlashEraseRegion(const string_vector_t * argv) : Command(argv) {}

    //! @brief Constructor that takes a start and length arguments.
    FlashEraseRegion(uint32_t start, uint32_t length)
        :   Command(kCommand_FlashEraseRegion.name),
        m_startAddress(start), m_byteCount(length)
    {
        m_argv.push_back(format_string("0x%08x", start));
        m_argv.push_back(format_string("0x%08x", length));
    }

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_FlashEraseRegion);
    }

protected:
    uint32_t m_startAddress;    //!< Starting address in flash.
    uint32_t m_byteCount;       //!< Number of bytes to erase.
};

/*!
 * @brief Represents the bootloader Flash Erase All command.
 */
class FlashEraseAll : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    FlashEraseAll(const string_vector_t * argv) : Command(argv) {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_FlashEraseAll);
    }
};

/*!
 * @brief Represents the bootloader Flash Erase All Unsecure command.
 */
class FlashEraseAllUnsecure : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    FlashEraseAllUnsecure(const string_vector_t * argv) : Command(argv) {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_FlashEraseAllUnsecure);
    }
};

/*!
 * @brief Represents the bootloader Read Memory command.
 */
class ReadMemory : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    ReadMemory(const string_vector_t * argv)
    :   Command(argv),
        m_dataFile()
    {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const read_memory_response_packet_t * packet);

    //! @brief Check generic response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_ReadMemory);
    }

protected:
    std::string m_dataFile;     //!< Data file path.
    uint32_t m_startAddress;    //!< Destination memory address.
    uint32_t m_byteCount;       //!< Number of bytes to read.
    uint32_t m_bytesRead;       //!< Number of bytes sent in data phase.
};

/*!
 * @brief Represents the bootloader Write Memory command.
 */
class WriteMemory : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    WriteMemory(const string_vector_t * argv)
    :   Command(argv),
        m_fileOrData(),
        m_segment(NULL),
        m_startAddress(0),
        m_data()
    {}

    //! @brief Constructor that takes an DataSource::Segment argument
    WriteMemory(blfwk::DataSource::Segment * segment)
        :   Command(kCommand_WriteMemory.name),
        m_fileOrData(), m_segment(segment), m_data()
    {
        m_startAddress = segment->getBaseAddress();
        m_argv.push_back(format_string("0x%08x", m_startAddress));
        m_argv.push_back(m_fileOrData);
    }

    //! @brief Constructor that takes an uchar_vector_t argument
    WriteMemory(uint32_t address, const uchar_vector_t & data)
        :   Command(kCommand_WriteMemory.name),
        m_fileOrData(), m_segment(NULL), m_startAddress(address), m_data(data)
    {
        m_argv.push_back(format_string("0x%08x", m_startAddress));
        m_argv.push_back(m_fileOrData);
    }

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_WriteMemory);
    }

protected:
    std::string m_fileOrData;                 //!< Data file path or hex data string.
    blfwk::DataSource::Segment * m_segment;   //!< DataSource segment (instead of file or hex string).
    uint32_t m_startAddress;                  //!< Destination memory address.
    uchar_vector_t m_data;                    //!< The data to write to the device.
};

/*!
 * @brief Represents the bootloader Fill Memory command.
 */
class FillMemory : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    FillMemory(const string_vector_t * argv) : Command(argv) {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_FillMemory);
    }

protected:
    uint32_t m_startAddress;        //!< Destination memory address.
    uint32_t m_byteCount;           //!< Number of bytes to fill.
    uint32_t m_patternWord;         //!< Fill pattern.
};

/*!
 * @brief Represents the bootloader Receive SB File command.
 */
class ReceiveSbFile : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    ReceiveSbFile(const string_vector_t * argv)
    :   Command(argv),
        m_dataFile()
    {}

    //! @brief Constructor that takes a filename argument.
    ReceiveSbFile(const char * const filename)
        :   Command(kCommand_ReceiveSbFile.name),
        m_dataFile(filename)
    {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_ReceiveSbFile);
    }

protected:
    std::string m_dataFile;          //!< SB file path.
};

/*!
 * @brief Represents the bootloader Execute command.
 */
class Execute : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    Execute(const string_vector_t * argv) : Command(argv) {}

    //! @brief Constructor that takes entry_point, param and stack_pointer arguments.
    Execute(uint32_t entry_point, uint32_t param, uint32_t stack_pointer)
        :   Command(kCommand_Execute.name),
        m_jumpAddress(entry_point), m_wordArgument(param),
        m_stackpointer(stack_pointer)
    {
        m_argv.push_back(format_string("0x%08x", entry_point));
        m_argv.push_back(format_string("0x%08x", param));
        m_argv.push_back(format_string("0x%08x", stack_pointer));
    }

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_Execute);
    }

protected:
    uint32_t m_jumpAddress;     //!< Destination memory address to jump to.
    uint32_t m_wordArgument;    //!< Word argument passed to function.
    uint32_t m_stackpointer;    //!< Stack pointer.
};

/*!
 * @brief Represents the bootloader Call command.
 */
class Call : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    Call(const string_vector_t * argv) : Command(argv) {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_Call);
    }

protected:
    uint32_t m_callAddress;     //!< Destination memory address to call.
    uint32_t m_wordArgument;    //!< Word argument passed to function.
};

/*!
 * @brief Represents the bootloader Flash Security Disable command.
 */
class FlashSecurityDisable : public Command
{
public:
    //! @brief Constructor that takes an argument vector.
    FlashSecurityDisable(const string_vector_t * argv) : Command(argv) {}

    //! @brief Initialize.
    virtual bool init();

    //! @brief Send command to packetizer.
    virtual void sendTo(Packetizer & packetizer);

protected:
    //! @brief Check response packet.
    virtual bool processResponse(const uint8_t * packet)
    {
        return Command::processResponse(reinterpret_cast<const generic_response_packet_t *>(packet), kCommandTag_FlashSecurityDisable);
    }

protected:
    uint32_t m_keyLow;    //!< Bytes 0-3 of the flash backdoor key.
    uint32_t m_keyHigh;   //!< Bytes 4-7 of the flash backdoor key.
};

//@}

} // namespace blfwk

#endif // _host_command_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
