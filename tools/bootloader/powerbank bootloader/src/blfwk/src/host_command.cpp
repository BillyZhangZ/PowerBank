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
#include "blfwk/host_command.h"
#include "blfwk/host_packetizer.h"
#include "blfwk/Logging.h"
#include "packet/command_packet.h"
#include "property/property.h"
#include "blfwk/format_string.h"
#include "blfwk/utils.h"
#include "sbloader/loader.h"
#include "bootloader/bootloader.h"
#include "memory/memory.h"
#include <cstring>
#include <algorithm>

using namespace blfwk;
using namespace utils;
using namespace std;

//#define TEST_SENDER_ABORT
//#define TEST_RECEIVER_ABORT

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Value of the terminator code in the g_statusCodes table.
const int32_t kStatusMessageTableTerminatorValue = 0x7fffffff;

//! @brief Status return code descriptions.
StatusMessageTableEntry blfwk::g_statusCodes[] =
{
    // Generic statuses.
    { kStatus_Success, "Success." },
    { kStatus_Fail, "Failure." },
    { kStatus_ReadOnly, "kStatus_ReadOnly" },
    { kStatus_OutOfRange, "kStatus_OutOfRange" },
    { kStatus_InvalidArgument, "kStatus_InvalidArgument" },

    // Flash driver errors.
    { 100 /*kStatus_FlashSizeError*/, "kStatus_FlashSizeError" },
    { 101 /*kStatus_FlashAlignmentError*/, "kStatus_FlashAlignmentError" },
    { 102 /*kStatus_FlashAddressError*/, "kStatus_FlashAddressError" },
    { 103 /*kStatus_FlashAccessError*/, "kStatus_FlashAccessError" },
    { 104 /*kStatus_FlashProtectionViolation*/, "kStatus_FlashProtectionViolation" },
    { 105 /*kStatus_FlashCommandFailure*/, "kStatus_FlashCommandFailure" },
    { 106 /*kStatus_FlashUnknownProperty*/, "kStatus_FlashUnknownProperty" },

    // I2C driver errors.
    { 200 /*kStatus_I2C_SlaveTxUnderrun*/, "I2C Slave TX Underrun error." },
    { 201 /*kStatus_I2C_SlaveRxOverrun*/, "I2C Slave RX Overrun error." },
    { 202 /*kStatus_I2C_AribtrationLost*/, "I2C Arbitration Lost error." },

    // SPI driver errors.
    { 300 /*kStatus_SPI_SlaveTxUnderrun*/, "SPI Slave TX Underrun error." },
    { 301 /*kStatus_SPI_SlaveRxOverrun*/, "SPI Slave RX Overrun error." },

    // Bootloader errors.
    { kStatus_UnknownCommand, "kStatus_UnknownCommand" },
    { kStatus_SecurityViolation, "Command disallowed when security is enabled." },
    { kStatus_AbortDataPhase, "kStatus_AbortDataPhase" },
    { kStatus_Ping, "kStatus_Ping" },
    { kStatus_NoResponse, "No response packet from target device." },

    // SB loader errors.
    { kStatusRomLdrSectionOverrun, "kStatusRomLdrSectionOverrun" },
    { kStatusRomLdrSignature, "kStatusRomLdrSignature" },
    { kStatusRomLdrSectionLength, "kStatusRomLdrSectionLength" },
    { kStatusRomLdrUnencryptedOnly, "kStatusRomLdrUnencryptedOnly" },
    { kStatusRomLdrEOFReached, "kStatusRomLdrEOFReached" },
    { kStatusRomLdrChecksum, "kStatusRomLdrChecksum" },
    { kStatusRomLdrCrc32Error, "kStatusRomLdrCrc32Error" },
    { kStatusRomLdrUnknownCommand, "kStatusRomLdrUnknownCommand" },
    { kStatusRomLdrIdNotFound, "kStatusRomLdrIdNotFound" },
    { kStatusRomLdrDataUnderrun, "kStatusRomLdrDataUnderrun" },
    { kStatusRomLdrJumpReturned, "kStatusRomLdrJumpReturned" },
    { kStatusRomLdrCallFailed, "kStatusRomLdrCallFailed" },

    // Memory interface errors.
    { kStatusMemoryRangeInvalid, "kStatusMemoryRangeInvalid" },
    { kStatusMemoryReadFailed, "kStatusMemoryReadFailed" },
    { kStatusMemoryWriteFailed, "kStatusMemoryWriteFailed" },

    // Property store errors.
    { kStatus_UnknownProperty, "Unknown property." },
    { kStatus_ReadOnlyProperty, "Property is read-only." },
    { kStatus_InvalidPropertyValue, "Invalid property value." },

    // Terminator
    { kStatusMessageTableTerminatorValue, "" }
};


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
Command * Command::create(const string_vector_t * argv)
{
    Command * cmd;

    if (argv->at(0) == kCommand_Reset.name)
    {
        cmd = new Reset(argv);
    }
    else if (argv->at(0) == kCommand_GetProperty.name)
    {
        cmd = new GetProperty(argv);
    }
    else if (argv->at(0) == kCommand_SetProperty.name)
    {
        cmd = new SetProperty(argv);
    }
    else if (argv->at(0) == kCommand_FlashEraseRegion.name)
    {
        cmd = new FlashEraseRegion(argv);
    }
    else if (argv->at(0) == kCommand_FlashEraseAll.name)
    {
        cmd = new FlashEraseAll(argv);
    }
    else if (argv->at(0) == kCommand_FlashEraseAllUnsecure.name)
    {
        cmd = new FlashEraseAllUnsecure(argv);
    }
    else if (argv->at(0) == kCommand_ReadMemory.name)
    {
        cmd = new ReadMemory(argv);
    }
    else if (argv->at(0) == kCommand_WriteMemory.name)
    {
        cmd = new WriteMemory(argv);
    }
    else if (argv->at(0) == kCommand_FillMemory.name)
    {
        cmd = new FillMemory(argv);
    }
    else if (argv->at(0) == kCommand_ReceiveSbFile.name)
    {
        cmd = new ReceiveSbFile(argv);
    }
    else if (argv->at(0) == kCommand_Execute.name)
    {
        cmd = new Execute(argv);
    }
    else if (argv->at(0) == kCommand_Call.name)
    {
        cmd = new Call(argv);
    }
    else if (argv->at(0) == kCommand_FlashSecurityDisable.name)
    {
        cmd = new FlashSecurityDisable(argv);
    }
    else
    {
        return NULL;
    }

    // Validate arguments.
    if (!cmd->init())
    {
        delete cmd;
        return NULL;
    }

    return cmd;
}

// See host_command.h for documentation of this method.
std::string Command::getResponse() const
{
    Json::Value root;
    root["command"] = getName();
    root["status"] = Json::Value(Json::objectValue);
    root["status"]["value"] = static_cast<int32_t>(m_responseValues.at(0));
    root["status"]["description"] = format_string("%d (0x%X) %s",
        m_responseValues.at(0), m_responseValues.at(0), getStatusMessage(m_responseValues.at(0)).c_str());
    root["response"] = Json::Value(Json::arrayValue);
    for (int i = 1; i < (int)m_responseValues.size(); ++i)
    {
        root["response"].append(Json::Value(m_responseValues.at(i)));
    }

    Json::StyledWriter writer;
    return writer.write(root);
}

std::string Command::getStatusMessage(status_t code) const
{
    int i;
    for (i=0; g_statusCodes[i].status != kStatusMessageTableTerminatorValue; ++i)
    {
        if (code == g_statusCodes[i].status)
        {
            return g_statusCodes[i].message;
        }
    }

    return format_string("Unknown error code (%d)", code);
}

// See host_command.h for documentation of this method.
void Command::logResponses() const
{
    const uint32_vector_t * respv = getResponseValues();
    size_t count = respv->size();

    Log::info("Response status = %d (0x%x) %s\n", respv->at(0), respv->at(0), getStatusMessage(respv->at(0)).c_str());

    for (int i = 1; i < (int)count; ++i)
    {
        Log::info("Response word %d = %d (0x%x)\n", i, respv->at(i), respv->at(i));
    }

    Log::json(getResponse().c_str());
}

// See host_command.h for documentation of this method.
bool Command::processResponse(const generic_response_packet_t * packet, uint8_t commandTag)
{
    if (!packet)
    {
        Log::debug("processResponse: null packet\n");
        m_responseValues.push_back(kStatus_NoResponse);
        return false;
    }

    if (packet->commandPacket.commandTag != kCommandTag_GenericResponse)
    {
        Log::error("Error: expected kCommandTag_GenericResponse (0x%x), received 0x%x\n", kCommandTag_GenericResponse, packet->commandPacket.commandTag);
        m_responseValues.push_back(kStatus_UnknownCommand);
        return false;
    }
    if (packet->commandTag != commandTag)
    {
        Log::error("Error: expected commandTag 0x%x, received 0x%x\n", commandTag, packet->commandTag);
        m_responseValues.push_back(kStatus_UnknownCommand);
        return false;
    }

    // Set the status in the response vector.
    m_responseValues.push_back(packet->status);

    if (packet->status != kStatus_Success)
    {
        return false;
    }

    Log::info("Successful generic response to command '%s'\n", getName().c_str());
    return true;
}

//! See host_command.h for documentation on this function.
bool blfwk::DataPacket::FileDataProducer::init(string filePath)
{
    m_filePointer = fopen(filePath.c_str(), "rb");
    if (!m_filePointer)
    {
        Log::error("Error: cannot open input data file '%s'\n", filePath.c_str());
        return false;
    }

    // Get the file size.
    ::fseek(m_filePointer, 0, SEEK_END);
    m_fileSize = ftell(m_filePointer);
    ::fseek(m_filePointer, 0, SEEK_SET);

    Log::info("Preparing to send %d (0x%x) bytes to the target.\n", m_fileSize, m_fileSize);
    return true;
}

//! See host_command.h for documentation on this function.
uint32_t blfwk::DataPacket::FileDataProducer::getData(uint8_t * data, uint32_t size)
{
    assert(m_filePointer);
    assert(data);
    if ((size == 0) || !hasMoreData())
    {
        return 0;
    }

    return (uint32_t)fread(data, 1, size, m_filePointer);
}

//! See host_command.h for documentation on this function.
uint32_t blfwk::DataPacket::HexDataProducer::initFromString(const string hexData)
{
    // Remove everything from string except for hex digits.
    string hexString = string_hex(hexData);

    // Clear any existing data.
    m_data.clear();

    // Load data byte array from hex string.
    // Two hex characters equals one byte.
    // Any trailing character is ignored.
    for (uint32_t i = 0; i < hexString.size(); i += 2)
    {
        string hexByte = hexString.substr(i, 2);
        long int val = strtol(hexByte.c_str(), NULL, 16);
        m_data.push_back(static_cast<uint8_t>(val));
    }

    return m_data.size();
}

//! See host_command.h for documentation on this function.
uint32_t blfwk::DataPacket::HexDataProducer::getData(uint8_t * data, uint32_t size)
{
    assert(data);

    if (!hasMoreData())
    {
        return 0;
    }

    uint32_t remaining = m_data.size() - m_byteIndex;
    size = min(size, remaining);
    memcpy(data, &m_data[m_byteIndex], size);
    m_byteIndex += size;
    return size;
}

//! See host_command.h for documentation on this function.
uint32_t blfwk::DataPacket::SegmentDataProducer::getData(uint8_t * data, uint32_t size)
{
    assert(data);

    if (!hasMoreData())
    {
        return 0;
    }

    size = m_segment->getData(m_byteIndex, size, data);
    m_byteIndex += size;
    return size;
}

//! See host_command.h for documentation on this function.
bool blfwk::DataPacket::FileDataConsumer::init(string filePath)
{
    m_filePointer = fopen(filePath.c_str(), "wb");
    if (!m_filePointer)
    {
        Log::error("Error: cannot open output data file '%s'\n", filePath.c_str());
        return false;
    }
    return true;
}

//! See host_command.h for documentation on this function.
void blfwk::DataPacket::FileDataConsumer::processData(const uint8_t * data, uint32_t size)
{
    fwrite(data, 1, size, m_filePointer);
}

//! See host_command.h for documentation on this function.
void blfwk::DataPacket::StdOutDataConsumer::processData(const uint8_t * data, uint32_t size)
{
    for (int i = 0; i < (int)size; ++i)
    {
        printf("%02x", data[i]);
        if ((m_currentCount++ % kBytesPerLine) == 0)
        {
            printf("\n");
        }
        else
        {
            printf(" ");
        }
    }
}

//! See host_command.h for documentation on this function.
uint8_t * blfwk::DataPacket::sendTo(Packetizer & device)
{
    if (!m_dataProducer->hasMoreData())
    {
        device.pumpSimulator();
    }

    while (m_dataProducer->hasMoreData())
    {
        uint32_t count = m_dataProducer->getData(m_packet, sizeof(m_packet));
        if (count)
        {
            status_t status = device.writePacket((const uint8_t *)m_packet, count, kPacketType_Data);
            if (status != kStatus_Success)
            {
                Log::error("Data phase write aborted by status 0x%x\n", status);
                break;
            }

#ifdef TEST_SENDER_ABORT
            // Send zero length packet to abort data phase.
            Log::info("Testing data phase abort\n");
            device.writePacket((const uint8_t *)&m_packet, 0, kPacketType_Data);
            break;
#endif
        }
    }

    // Read final command status
    uint8_t * responsePacket;
    uint32_t responseLength;
    if (device.readPacket(&responsePacket, &responseLength, kPacketType_Command) != kStatus_Success)
    {
        return NULL;
    }
    return responsePacket;
}

//! See host_command.h for documentation on this function.
uint8_t * blfwk::DataPacket::receiveFrom(Packetizer & device, uint32_t byteCount)
{
    // If byte count is zero, need to pump the simulator to get the final response
    if (byteCount == 0)
    {
        device.pumpSimulator();
    }

    while (byteCount > 0)
    {
        uint8_t * dataPacket;
        uint32_t length;

        // Pump the simulator command state machine, if it is enabled.
        device.pumpSimulator();

        device.readPacket(&dataPacket, &length, kPacketType_Data);

        // Check for sender abort of data phase.
        if (length == 0)
        {
            Log::info("Data phase aborted by sender\n");
            break;
        }

        m_dataConsumer->processData(dataPacket, length);
        byteCount -= length;

#ifdef TEST_RECEIVER_ABORT
        Log::info("Testing data phase abort\n");
        device.abortPacket();
        break;
#endif
    }

    m_dataConsumer->finalize();

    // Read the final generic response packet.
    uint8_t * responsePacket;
    uint32_t responseLength;
    if (device.readPacket(&responsePacket, &responseLength, kPacketType_Command) != kStatus_Success)
    {
        return NULL;
    }
    return responsePacket;
}

//! See host_command.h for documentation on this function.
const uint8_t * blfwk::CommandPacket::sendCommandGetResponse(Packetizer & device)
{
    uint8_t * responsePacket = NULL;
    uint32_t responseLength = 0;
    uint32_t retries = 2;

    while (retries)
    {
        status_t status = device.writePacket(getData(), getSize(), kPacketType_Command);
        if (status != kStatus_Success)
        {
            Log::error("sendCommandGetResponse.writePacket error %d.\n", status);
            return NULL;
        }
        status = device.readPacket(&responsePacket, &responseLength, kPacketType_Command);
        if (status == kStatus_Success)
        {
            return responsePacket;
        }
        else if ( status == kStatus_Timeout)
        {
            --retries;
            Log::error("Retry command %d.\n", 2-retries);
            continue;
        }
        else
        {
            Log::error("sendCommandGetResponse.readPacket error %d.\n", status);
            return NULL;
        }
    }

    // Failed retries + 1
    Log::error("Failed to read packet after %d tries.\n", retries+1);
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Reset command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool Reset::init()
{
    if (getArgCount() != 1)
    {
        return false;
    }
    return true;
}

// See host_command.h for documentation of this method.
void Reset::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_Reset);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// GetProperty command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool GetProperty::init()
{
    if (getArgCount() != 2)
    {
        return false;
    }

    // Save property tag number.
    m_propertyTag = strtoul(getArg(1).c_str(), NULL, 0);

    return true;
}

// See host_command.h for documentation of this method.
void GetProperty::sendTo(Packetizer & device)
{
    // Command Phase
    blfwk::CommandPacket cmdPacket(kCommandTag_GetProperty, m_propertyTag);
    const uint8_t * responsePacket = cmdPacket.sendCommandGetResponse(device);

    const get_property_response_packet_t * packet = reinterpret_cast<const get_property_response_packet_t *>(responsePacket);
    processResponse(packet);
}

bool GetProperty::processResponse(const get_property_response_packet_t * packet)
{
    if (!packet)
    {
        Log::debug("processResponse: null packet\n");
        m_responseValues.push_back(kStatus_NoResponse);
        return false;
    }

    // Handle generic response, which would be returned if command is not supported.
    if (packet->commandPacket.commandTag == kCommandTag_GenericResponse)
    {
        return processResponse((const uint8_t *)packet);
    }

    if (packet->commandPacket.commandTag != kCommandTag_GetPropertyResponse)
    {
        Log::error("Error: expected kCommandTag_GetPropertyResponse (0x%x), received 0x%x\n", kCommandTag_GetPropertyResponse, packet->commandPacket.commandTag);
        m_responseValues.push_back(kStatus_UnknownCommand);
        return false;
    }

    // Set the status in the response vector.
    m_responseValues.push_back(packet->status);

    if (packet->status != kStatus_Success)
    {
        return false;
    }

    Log::debug("Successful response to command '%s'\n", getName().c_str());

    // Currently, no properties have a data phase.
    assert(!(packet->commandPacket.flags & kCommandFlag_HasDataPhase));

    // All properties have at least one response word.
    // Attention: parameterCount = 1(response status) + response words
    for (uint8_t i = 0; i < (packet->commandPacket.parameterCount-1); ++i)
    {
        m_responseValues.push_back(packet->propertyValue[i]);
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// SetProperty command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool SetProperty::init()
{
    if (getArgCount() != 3)
    {
        return false;
    }

    // Save property tag number and value.
    m_propertyTag = strtoul(getArg(1).c_str(), NULL, 0);
    m_propertyValue = strtoul(getArg(2).c_str(), NULL, 0);
    return true;
}

// See host_command.h for documentation of this method.
void SetProperty::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_SetProperty, m_propertyTag, m_propertyValue);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// FlashEraseRegion command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool FlashEraseRegion::init()
{
    if (getArgCount() != 3)
    {
        return false;
    }

    m_startAddress = strtoul(getArg(1).c_str(), NULL, 0);
    m_byteCount = strtoul(getArg(2).c_str(), NULL, 0);

    return true;
}

// See host_command.h for documentation of this method.
void FlashEraseRegion::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_FlashEraseRegion, m_startAddress, m_byteCount);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// FlashEraseAll command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool FlashEraseAll::init()
{
    if (getArgCount() != 1)
    {
        return false;
    }
    return true;
}

// See host_command.h for documentation of this method.
void FlashEraseAll::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_FlashEraseAll);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// FlashEraseAllUnsecure command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool FlashEraseAllUnsecure::init()
{
    if (getArgCount() != 1)
    {
        return false;
    }
    return true;
}

// See host_command.h for documentation of this method.
void FlashEraseAllUnsecure::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_FlashEraseAllUnsecure);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// ReadMemory command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool ReadMemory::init()
{
    if ((getArgCount() < 3) || (getArgCount() > 4))
    {
        return false;
    }

    m_startAddress = strtoul(getArg(1).c_str(), NULL, 0);
    m_byteCount = strtoul(getArg(2).c_str(), NULL, 0);

    // File name argument is optional - will use stdout if missing.
    if (getArgCount() == 4)
    {
        m_dataFile = getArg(3);
    }

    return true;
}

// See host_command.h for documentation of this method.
void ReadMemory::sendTo(Packetizer & device)
{
    DataPacket::DataConsumer * dataConsumer;
    DataPacket::FileDataConsumer fileDataConsumer;
    DataPacket::StdOutDataConsumer stdoutDataConsumer;

    // Setup to write to file or stdout
    if (m_dataFile.size() > 0)
    {
        if (!fileDataConsumer.init(m_dataFile))
        {
            return;
        }
        dataConsumer = &fileDataConsumer;
    }
    else
    {
        dataConsumer = &stdoutDataConsumer;
    }

    // Send command packet.
    blfwk::CommandPacket cmdPacket(kCommandTag_ReadMemory, m_startAddress, m_byteCount);
    const uint8_t * responsePacket = cmdPacket.sendCommandGetResponse(device);

    const read_memory_response_packet_t * packet = reinterpret_cast<const read_memory_response_packet_t *>(responsePacket);
    if (processResponse(packet))
    {
        // Receive data packets.
        blfwk::DataPacket dataPacket(dataConsumer);
        uint8_t * finalResponsePacket = dataPacket.receiveFrom(device, packet->dataByteCount);
        if (finalResponsePacket && processResponse(finalResponsePacket))
        {
            // Push the number of bytes transferred response value.
            m_responseValues.push_back(m_bytesRead);
        }
    }
}

bool ReadMemory::processResponse(const read_memory_response_packet_t * packet)
{
    if (!packet)
    {
        Log::debug("processResponse: null packet\n");
        return false;
    }

    // Handle generic response, which would be returned if command is not supported.
    if (packet->commandPacket.commandTag == kCommandTag_GenericResponse)
    {
        return processResponse((const uint8_t *)packet);
    }

    if (packet->commandPacket.commandTag != kCommandTag_ReadMemoryResponse)
    {
        Log::error("Error: expected kCommandTag_ReadMemoryResponse (0x%x), received 0x%x\n", kCommandTag_ReadMemoryResponse, packet->commandPacket.commandTag);
        return false;
    }
    if (packet->status != kStatus_Success)
    {
        // Set the status in the response vector.
        // If status is OK, this push will be done by final response processing
        m_responseValues.push_back(packet->status);
        return false;
    }

	Log::error("Successful response to command '%s'\n", getName().c_str());

    // Save the byte count.
    // This will be pushed to the response values vectors after the data phase.
    m_bytesRead = packet->dataByteCount;

    return true;
}

////////////////////////////////////////////////////////////////////////////////
// WriteMemory command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool WriteMemory::init()
{
    if (getArgCount() != 3)
    {
        return false;
    }

    m_startAddress = strtoul(getArg(1).c_str(), NULL, 0);
    m_fileOrData = getArg(2);

    return true;
}

// See host_command.h for documentation of this method.
void WriteMemory::sendTo(Packetizer & device)
{
    DataPacket::HexDataProducer hexProducer(m_data);
    DataPacket::FileDataProducer fileProducer;
    DataPacket::SegmentDataProducer segmentProducer(m_segment);
    DataPacket::DataProducer * dataProducer;

    if (m_segment)
    {
        dataProducer = &segmentProducer;
    }
    else if ((m_fileOrData[0] == '{') && (m_fileOrData[1] == '{'))
    {
        // Argument string is hex data, so use hex data producer.
        if (hexProducer.initFromString(m_fileOrData) == 0)
        {
            return;
        }
        dataProducer = &hexProducer;
    }
    else if ( m_data.size() > 0 )
    {
        dataProducer = &hexProducer;
    }
    else
    {
        // Argument string is file name, so use file data producer.
        if (!fileProducer.init(m_fileOrData))
        {
            return;
        }
        dataProducer = &fileProducer;
    }

    // Send command packet.
    blfwk::CommandPacket cmdPacket(kCommandTag_WriteMemory, m_startAddress, dataProducer->getDataSize());
    if (!processResponse(cmdPacket.sendCommandGetResponse(device)))
    {
        return;
    }

    // Pop the initial (successful) generic response value.
    if (m_responseValues.size())
    {
        m_responseValues.pop_back();
    }

    // Send data packets.
    blfwk::DataPacket dataPacket(dataProducer);
    processResponse(dataPacket.sendTo(device));
}

////////////////////////////////////////////////////////////////////////////////
// FillMemory command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool FillMemory::init()
{
    if ((getArgCount() < 4) || (getArgCount() > 5))
    {
        return false;
    }

    m_startAddress = strtoul(getArg(1).c_str(), NULL, 0);
    m_byteCount = strtoul(getArg(2).c_str(), NULL, 0);
    m_patternWord = strtoul(getArg(3).c_str(), NULL, 0);

    if ((getArgCount() == 5) && (getArg(4) != "word"))
    {
        unsigned char b1 = (unsigned char)((m_patternWord >>  8) & 0xff);
        unsigned char b0 = (unsigned char)( m_patternWord        & 0xff);

        if (getArg(4) == "byte")
        {
            // Replicate byte pattern in word.
            m_patternWord = b0 + (b0 << 8) + (b0 << 16) + (b0 << 24);
        }
        else if (getArg(4) == "short")
        {
            // Replicate short pattern in word.
            m_patternWord = b0 + (b1 << 8) + (b0 << 16) + (b1 << 24);
        }
        else
        {
            return false; // unknown pattern size argument
        }
    }

    return true;
}

// See host_command.h for documentation of this method.
void FillMemory::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_FillMemory, m_startAddress, m_byteCount, m_patternWord);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// ReceiveSbFile command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool ReceiveSbFile::init()
{
    if (getArgCount() != 2)
    {
        return false;
    }
    m_dataFile = getArg(1);
    return true;
}

// See host_command.h for documentation of this method.
void ReceiveSbFile::sendTo(Packetizer & device)
{
    DataPacket::FileDataProducer dataProducer;
    if (!dataProducer.init(m_dataFile))
    {
        return;
    }

    // Send command packet.
    blfwk::CommandPacket cmdPacket(kCommandTag_ReceiveSbFile, (uint32_t)dataProducer.getDataSize());
    if (!processResponse(cmdPacket.sendCommandGetResponse(device)))
    {
        return;
    }

    // Pop the initial (successful) generic response value.
    if (m_responseValues.size())
    {
        m_responseValues.pop_back();
    }

    // Send data packets.
    blfwk::DataPacket dataPacket(&dataProducer);
    processResponse(dataPacket.sendTo(device));
}

////////////////////////////////////////////////////////////////////////////////
// Execute command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool Execute::init()
{
    if (getArgCount() != 4)
    {
        return false;
    }
    m_jumpAddress = strtoul(getArg(1).c_str(), NULL, 0);
    m_wordArgument = strtoul(getArg(2).c_str(), NULL, 0);
	m_stackpointer = strtoul(getArg(3).c_str(), NULL, 0);
    return true;
}

// See host_command.h for documentation of this method.
void Execute::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_Execute, m_jumpAddress, m_wordArgument, m_stackpointer);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// Call command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool Call::init()
{
    if (getArgCount() != 3)
    {
        return false;
    }
    m_callAddress = strtoul(getArg(1).c_str(), NULL, 0);
    m_wordArgument = strtoul(getArg(2).c_str(), NULL, 0);
    return true;
}

// See host_command.h for documentation of this method.
void Call::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_Call, m_callAddress, m_wordArgument);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// FlashSecurityDisable command
////////////////////////////////////////////////////////////////////////////////

// See host_command.h for documentation of this method.
bool FlashSecurityDisable::init()
{
    if (getArgCount() != 2)
    {
        return false;
    }
    if (getArg(1).length() != 16)
    {
        return false;
    }

    // String must be hex digits with no leading 0x.
    m_keyLow = strtoul(getArg(1).substr(0, 8).c_str(), NULL, 16);
    m_keyHigh = strtoul(getArg(1).substr(8, 8).c_str(), NULL, 16);
    return true;
}

// See host_command.h for documentation of this method.
void FlashSecurityDisable::sendTo(Packetizer & device)
{
    blfwk::CommandPacket cmdPacket(kCommandTag_FlashSecurityDisable, m_keyLow, m_keyHigh);
    processResponse(cmdPacket.sendCommandGetResponse(device));
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

