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

#include "blfwk/bus_pal.h"
#include "blfwk/Logging.h"
#include "blfwk/serial.h"
#include <vector>
#include <cstring>
#include <algorithm>

using namespace blfwk;
using namespace std;

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Bit Bang Mode commands.
enum {
    kBitBang_Reset     = 0x00, //!< Reset, responds "BBIO1"
    kBitBang_SpiMode   = 0x01, //!< Enter binary SPI mode, responds "SPI1"
    kBitBang_I2cMode   = 0x02, //!< Enter binary I2C mode, responds "I2C1"
    kBitBang_UartMode  = 0x03, //!< Enter binary UART mode, responds "ART1"
    kBitBang_1Wire     = 0x04, //!< Enter binary 1-Wire mode, responds "1W01"
    kBitBang_RawWire   = 0x05, //!< Enter binary raw-wire mode, responds "RAW1"
    kBitBang_Jtag      = 0x06, //!< Enter OpenOCD JTAG mode
    kBitBang_HardReset = 0x0f, //!< Reset Bus Pal
    kBitBang_SelfTest  = 0x10, //!< Bus Pal self-tests
    kBitBang_SetupPwm  = 0x12, //!< Setup pulse-width modulation (requires 5 byte setup)
    kBitBang_ClearPwm  = 0x13, //!< Clear/disable PWM
    kBitBang_Probe     = 0x14, //!< Take voltage probe measurement (returns 2 bytes)
    kBitBang_ContProbe = 0x15, //!< Continuous voltage probe measurement
    kBitBang_FreqAux   = 0x16, //!< Frequency measurement on AUX pin
    kBitBang_CfgPins   = 0x40, //!< Configure pins as input(1) or output(0): AUX|MOSI|CLK|MISO|CS
    kBitBang_SetPins   = 0x80  //!< Set on (1) or off (0): POWER|PULLUP|AUX|MOSI|CLK|MISO|CS
};

//! @brief Uart Mode commands.
enum {
    kUart_Version      = 0x01, //!< Display mode version string, responds "ARTx"
    kUart_EchoRx       = 0x02, //!< Start (0)/stop(1) echo UART RX
    kUart_SetBaud      = 0x07, //!< Manual baud rate configuration, send 2 bytes
    kUart_Bridge       = 0x0f, //!< UART bridge mode (reset to exit)
    kUart_BulkWrite    = 0x10, //!< Bulk UART write, send 1-16 bytes (0=1byte!)
    kUart_ConfigPeriph = 0x40, //!< Configure peripherals w=power, x=pullups, y=AUX, z=CS
    kUart_SetSpeed     = 0x60, //!< Set UART speed
    kUart_ConfigUart   = 0x80  //!< Configure UART settings
};

//! @brief Spi mode commands.
enum {
    kSpi_Exit          = 0x00, //!< 00000000 - Exit to bit bang mode
    kSpi_Version       = 0x01, //!< 00000001 - Enter raw SPI mode, display version string
    kSpi_ChipSelect    = 0x02, //!< 0000001x - CS high (1) or low (0)
    kSpi_Sniff         = 0x0c, //!< 000011XX - Sniff SPI traffic when CS low(10)/all(01)
    kSpi_BulkTransfer  = 0x10, //!< 0001xxxx - Bulk SPI transfer, send/read 1-16 bytes (0=1byte!)
    kSpi_ConfigPeriph  = 0x40, //!< 0100wxyz - Configure peripherals w=power, x=pull-ups, y=AUX, z=CS
    kSpi_SetSpeed      = 0x60, //!< 01100xxx - SPI speed
    kSpi_ConfigSpi     = 0x80, //!< 1000wxyz - SPI config, w=HiZ/3.3v, x=CKP idle, y=CKE edge, z=SMP sample
    kSpi_WriteThenRead = 0x04  //!< 00000100 - Write then read extended command
};

//! @brief Spi configuration shifts for the mask
enum {
    kSpiConfigShift_Direction = 0,
    kSpiConfigShift_Phase     = 1,
    kSpiConfigShift_Polarity  = 2
};

#pragma pack(1)
struct SpiWriteThenReadCommand
{
    uint8_t command;
    uint16_t numBytesToWrite;
    uint16_t numBytesToRead;
};

struct SpiSetSpeedCommand
{
    uint8_t command;
    uint32_t speed;
};

struct I2cWriteThenReadCommand
{
    uint8_t command;
    uint16_t numBytesToWrite;
    uint16_t numBytesToRead;
};
#pragma pack()

//! @brief I2c mode commands.
enum {
    kI2c_Exit            = 0x00, //!< 00000000 - Exit to bit bang mode
    kI2c_Version         = 0x01, //!< 00000001 - Display mode version string, responds "I2Cx"
    kI2c_StartBit        = 0x02, //!< 00000010 - I2C start bit
    kI2c_StopBit         = 0x03, //!< 00000011 - I2C stop bit
    kI2c_ReadByte        = 0x04, //!< 00000100 - I2C read byte
    kI2c_AckBit          = 0x06, //!< 00000110 - ACK bit
    kI2c_NackBit         = 0x07, //!< 00000111 - NACK bit
    kI2c_BusSniff        = 0x0F, //!< 00001111 - Start bus sniffer
    kI2c_BulkWrite       = 0x10, //!< 0001xxxx - Bulk I2C write, send 1-16 bytes (0=1byte!)
    kI2c_ConfigurePeriph = 0x40, //!< 0100wxyz - Configure peripherals w=power, x=pullups, y=AUX, z=CS
    kI2c_PullUpSelect    = 0x50, //!< 010100xy - Pull up voltage select (BPV4 only)- x=5v y=3.3v
    kI2c_SetSpeed        = 0x60, //!< 011000xx - Set I2C speed, 3=~400kHz, 2=~100kHz, 1=~50kHz, 0=~5kHz (updated in v4.2 firmware)
    kI2c_SetAddress      = 0x70, //!< 11100000 - Set I2C address
    kI2c_WriteThenRead   = 0x08  //!< Write then read
};

enum {
    kSerialReadTimeoutMs = 3000 //!< 3 second timeout
};

//! @name Expected mode enter command responses.
//@{

//! @brief Response to enter bit bang mode.
const char * k_responseBitBangMode = "BBIO1";
const char * k_responseReset       = "BBIO1";
const char * k_responseSpiMode     = "SPI1";
const char * k_responseI2cMode     = "I2C1";
const char * k_responseUartMode    = "ART1";

//@}

//! @brief UART settings.
//!
//! Arguments (lower 5 bits) are wxxyz:
//! w=pin output HiZ(0)/3.3v(1)
//! xx=databits and parity 8/N(0), 8/E(1), 8/O(2), 9/N(3)
//! y=stop bits 1(0)/2(1)
//! z=RX polarity idle 1 (0), idle 0 (1)
enum {
    kUartSetting_33 = 0x10,
    kUartSettings = kUartSetting_33
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////
void BusPal::flushRX(int fd)
{
    char readBuf[100];
    Log::info("Flushing data...\n");
    int test;
    while ((test = buspal_serial_read(fd, readBuf, sizeof(readBuf))))
    {
        if (test)
        {
            Log::info("Flushed %d bytes\n", test);
        }
    }

    Log::info("Done flushing.\n");
}

// See See bus_pal.h for documentation on this method.
BusPal::BusPal(void)
    :   m_fileDescriptor(-1),
        m_mode(kBusPalModeBitBang)
{}

bool BusPal::parse(const string_vector_t& params, BusPal::BusPalConfigData& config)
{
    if (!params[0].compare(0, 3, "spi"))
    {
        config.transport = BusPal::kBusPalTransport_SPI;

        if ((params.size() > 1))
        {
            config.spiSpeedKHz = atoi(params[1].c_str());

            if (params.size() > 2)
            {
                config.spiPolarity = (BusPal::spi_clock_polarity_t)atoi(params[2].c_str());

                if (params.size() > 3)
                {
                    config.spiPhase = (BusPal::spi_clock_phase_t)atoi(params[3].c_str());

                    if (params.size() > 4)
                    {
                        if (!strcmp(params[4].c_str(), "lsb"))
                        {
                            config.spiDirection = BusPal::kSpiLsbFirst;
                        }
                        else if (!strcmp(params[4].c_str(), "msb"))
                        {
                            config.spiDirection = BusPal::kSpiMsbFirst;
                        }
                    }
                }
            }
        }
    } // SPI

    else if (!params[0].compare(0, 3, "i2c"))
    {
        config.transport = BusPal::kBusPalTransport_I2C;

        if (params.size() > 1)
        {
            config.i2cAddress = (uint8_t)strtoul(params[1].c_str(), NULL, 16);

            if (params.size() > 2)
            {
                config.i2cSpeedKHz = atoi(params[2].c_str());
            }
        }
    } // I2C

    else
    {
        // Error: Invalid BusPal transport.
        return false;
    }

    return true;
}

bool BusPal::init(const char * busPalPort, long speed)
{
    assert(busPalPort);

    // Open the port.
    m_fileDescriptor = serial_open(const_cast<char *>(busPalPort));
    if (m_fileDescriptor != -1)
    {
        serial_setup(m_fileDescriptor, speed);

        if(enterBitBangMode())
        {
            // Set our default read timeout, not doing this will result in lost data
            serial_set_read_timeout(m_fileDescriptor, kSerialReadTimeoutMs);

            return true;
        }
    }

    return false;
}

bool BusPal::enterSpiMode()
{
    bool retVal = writeCommand(kBitBang_SpiMode, k_responseSpiMode);

    if (retVal)
    {
        m_mode = kBusPalModeSpi;
    }

    return retVal;
}

bool BusPal::enterI2cMode()
{
    bool retVal = writeCommand(kBitBang_I2cMode, k_responseI2cMode);

    if (retVal)
    {
        m_mode = kBusPalModeI2c;
    }

    return retVal;
}

BusPal::~BusPal()
{
    if (m_fileDescriptor != -1)
    {
        serial_close(m_fileDescriptor);
    }
}

// See See bus_pal.h for documentation on this method.
bool BusPal::enterBitBangMode()
{
    Log::info("Entering bit bang mode...\n");

    // Set serial read timeout to 0 otherwise we will get a delay when clearing this out
    serial_set_read_timeout(m_fileDescriptor, 0);

    flushRX(m_fileDescriptor);

    // Set the timeout back to default
    serial_set_read_timeout(m_fileDescriptor, kSerialReadTimeoutMs);

    if (reset())
    {
        Log::info("Entered BB mode\n");
        return true;
    }
    else
    {
        Log::error("Error: enter bit bang mode failed\n");
        Log::error("Dumping any remaining data...\n");

        char blah;
        while(buspal_serial_read(m_fileDescriptor, &blah, 1));

        return false;
    }
}

// See See bus_pal.h for documentation on this method.
bool BusPal::writeCommand(uint8_t commandByte, const char * expectedResponse)
{
    assert(expectedResponse);
    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&commandByte), 1);
    if (actualBytes != 1)
    {
        Log::error("Error: Bus Pal write command failed\n");
        return false;
    }

    int count = strlen(expectedResponse);

    char * str = 0;

    while(!str)
    {
        str = reinterpret_cast<char *>(response(count));
        if (str)
        {
            Log::debug("write command response = ");
            for (int i = 0; i < count; i++)
            {
                Log::debug("%c", str[i]);
            }
            Log::debug("\n");
        }
    }

    return (strncmp(str, expectedResponse, count) == 0);
}

// See See bus_pal.h for documentation on this method.
uint8_t BusPal::writeCommand(uint8_t commandByte)
{
    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&commandByte), 1);
    if (actualBytes != 1)
    {
        Log::error("Error: Bus Pal write command failed\n");
        return false;
    }

    uint8_t * data = 0;

    while(data == NULL)
    {
        data = response();
    }

    return data ? *data : 0;
}

// See See bus_pal.h for documentation on this method.
uint8_t * BusPal::response(size_t byteCount)
{
    int remainingBytes = std::min<int>(byteCount, sizeof(m_responseBuffer));
    char * buffer = reinterpret_cast<char *>(m_responseBuffer);

    while (remainingBytes > 0)
    {
        int actualBytes = buspal_serial_read(m_fileDescriptor, buffer, remainingBytes);
        if (actualBytes > 0)
        {
            remainingBytes -= actualBytes;
            buffer += actualBytes;
        }
        else
        {
            // If no bytes received, it is a timeout error.
            return NULL;
        }
    }
    return m_responseBuffer;
}

void BusPal::flushInput()
{
    char dummy;
    do {
    } while (buspal_serial_read(m_fileDescriptor, &dummy, 1) == 1);
}

// See See bus_pal.h for documentation on this method.
bool BusPal::resetHardware()
{
    uint8_t rc = writeCommand(kBitBang_HardReset);
    if (rc != kResponseOk)
    {
        Log::error("Error: bad response from hard reset\n");
        return false;
    }
    return true;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::reset()
{
    // We need to do a non wait forever loop here to verify that bus pal is running/connected,
    // so that is why we are doing this manually instead of using writeCommand
    // if bus pal is not active this will return failure after the serial read timeout instead of blocking forever
    // Reset is always the first command run when talking to bus pal
    uint8_t commandByte = kBitBang_Reset;
    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&commandByte), sizeof(commandByte));
    bool retVal = false;
    if (actualBytes == sizeof(commandByte))
    {
        std::vector<uint8_t> rxBytes;

        // Set our serial timeout to the minimum so there are no additional delays
        serial_set_read_timeout(m_fileDescriptor, 0);

        // Read all of the data available from bus pal out, we will only be looking at the last strlen(k_responseBitBangMode) bytes
        // There is an issue with OpenSDA on the KL25 when communicating to some PCs, very rarely, between the previous call to flushRX
        // and here, calling serial_read will return a bunch of old data from either the PC drivers buffer or even possibly on
        // the KL25's OpenSDA chip buffer. To combat this we read out everything available and just check the last bytes for what
        // we're looking for
        uint8_t byte;
        while(buspal_serial_read(m_fileDescriptor, reinterpret_cast<char*>(&byte), sizeof(byte)))
        {
            rxBytes.push_back(byte);
        }

        if (rxBytes.size() >= strlen(k_responseBitBangMode))
        {
            unsigned int responseIndex = rxBytes.size() - strlen(k_responseBitBangMode);

            if (!strncmp((const char*)&rxBytes[responseIndex], k_responseBitBangMode, strlen(k_responseBitBangMode)))
            {
                retVal = true;
            }
        }

        // Set back to default
        serial_set_read_timeout(m_fileDescriptor, kSerialReadTimeoutMs);
    }

    return retVal;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::setSpiConfig(spi_clock_polarity_t polarity, spi_clock_phase_t phase, spi_shift_direction_t direction)
{
    uint8_t mask = (polarity & 1) << kSpiConfigShift_Polarity;
    mask |= (phase & 1) << kSpiConfigShift_Phase;
    mask |= (direction & 1) << kSpiConfigShift_Direction;

    uint8_t rc = writeCommand(kSpi_ConfigSpi | mask);

    if (rc != kResponseOk)
    {
        Log::error("Error: bad response from Set Spi Config\n");
        return false;
    }

    return true;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::setSpiSpeed(unsigned int speed)
{
    SpiSetSpeedCommand command = {kSpi_SetSpeed, speed};

    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&command), sizeof(command));
    if (actualBytes != sizeof(command))
    {
        Log::error("Error: Bus Pal Spi Set Speed command failed\n");
        return false;
    }

    uint8_t * respData = 0;

    while(respData == NULL)
    {
        respData = response(1);
    }

    if (respData[0] != kResponseOk)
    {
        Log::error("Error: bad response to Spi Set Speed, response byte = 0x%x\n", respData[0]);
        return false;
    }

    return true;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::setI2cAddress(uint8_t address)
{
    uint8_t command[] = {kI2c_SetAddress, address};

    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&command), sizeof(command));
    if (actualBytes != sizeof(command))
    {
        Log::error("Error: Bus Pal I2c Set Address command failed\n");
        return false;
    }

    uint8_t * respData = 0;

    while(respData == NULL)
    {
        respData = response(1);
    }

    if (respData[0] != kResponseOk)
    {
        Log::error("Error: bad response to Set I2c Address, response byte = 0x%x\n", respData[0]);
        return false;
    }

    return true;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::setI2cSpeed(uint32_t speed)
{
    uint8_t speedMask = 0;

    switch(speed)
    {
        case 400:
            speedMask = 3;
            break;
        case 100:
            speedMask = 2;
            break;
        case 50:
            speedMask = 1;
            break;
        case 5:
            speedMask = 0;
            break;
    }

    uint8_t rc = writeCommand(kI2c_SetSpeed | speedMask);

    if (rc != kResponseOk)
    {
        Log::error("Error: bad response from Set I2c Speed\n");
        return false;
    }
    return true;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::rawSetPins(uint8_t config)
{
    uint8_t rc = writeCommand(kBitBang_SetPins | config);
    if (rc != kResponseOk)
    {
        Log::error("Error: bad response from raw set pins\n");
        return false;
    }
    return true;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::bulkTransfer(const uint8_t * data, size_t byteCount)
{
    switch(m_mode)
    {
        case kBusPalModeSpi:
            return bulkTransferSpi(data, byteCount);
        case kBusPalModeI2c:
            return bulkTransferI2c(data, byteCount);
        default:
            return false;
    }
}

// See See bus_pal.h for documentation on this method.
int BusPal::read(uint8_t * data, size_t byteCount)
{
    switch(m_mode)
    {
        case kBusPalModeSpi:
            return readSpi(data, byteCount);
        case kBusPalModeI2c:
            return readI2c(data, byteCount);
        default:
            return false;
    }
}

// See See bus_pal.h for documentation on this method.
bool BusPal::bulkTransferSpi(const uint8_t * data, size_t byteCount)
{
    assert(data);

    SpiWriteThenReadCommand command;
    command.command = kSpi_WriteThenRead;
    command.numBytesToWrite = std::min<int>(byteCount, kBulkTransferMax);
    command.numBytesToRead = 0;

    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&command), sizeof(command));
    if (actualBytes != sizeof(command))
    {
        Log::error("Error: Bus Pal spi bulk transfer command failed\n");
        return false;
    }

    actualBytes = buspal_serial_write(m_fileDescriptor, (char*)data, byteCount);
    if (actualBytes != byteCount)
    {
        Log::error("Error: Bus Pal spi bulk transfer command failed\n");
        return false;
    }

    uint8_t * respData = 0;

    while(respData == NULL)
    {
        respData = response(1);
    }

    if (respData[0] != kResponseOk)
    {
        Log::error("Error: bad response to spi bulk transfer, response byte = 0x%x\n", respData[0]);
        return false;
    }

    return true;
}

// See See bus_pal.h for documentation on this method.
int BusPal::readSpi(uint8_t * data, size_t byteCount)
{
    assert(data);

    SpiWriteThenReadCommand command;
    command.command = kSpi_WriteThenRead;
    command.numBytesToWrite = 0;
    command.numBytesToRead = std::min<int>(byteCount, kBulkTransferMax);

    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&command), sizeof(command));
    if (actualBytes != sizeof(command))
    {
        Log::error("Error: Bus Pal read spi command failed\n");
        return false;
    }

    uint8_t * respData = 0;

    while(respData == NULL)
    {
        respData = response(1);
    }

    if (respData[0] != kResponseOk)
    {
        Log::error("Error: bad response to spi bulk transfer, response byte 0 = 0x%x\n", respData[0]);
        return false;
    }

    int remainingBytes = byteCount;
    char * buffer = reinterpret_cast<char *>(data);

    while (remainingBytes > 0)
    {
        actualBytes = buspal_serial_read(m_fileDescriptor, buffer, remainingBytes);
        if (actualBytes > 0)
        {
            remainingBytes -= actualBytes;
            buffer += actualBytes;
        }
        else
        {
            // If no bytes received, it is a timeout error.
            return 0;
        }
    }

    return byteCount;
}

// See See bus_pal.h for documentation on this method.
bool BusPal::bulkTransferI2c(const uint8_t * data, size_t byteCount)
{
    assert(data);

    I2cWriteThenReadCommand command;
    command.command = kI2c_WriteThenRead;
    command.numBytesToWrite = std::min<int>(byteCount, kBulkTransferMax);
    command.numBytesToRead = 0;

    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&command), sizeof(command));
    if (actualBytes != sizeof(command))
    {
        Log::error("Error: Bus Pal i2c bulk transfer command failed\n");
        return false;
    }

    actualBytes = buspal_serial_write(m_fileDescriptor, (char*)data, byteCount);
    if (actualBytes != byteCount)
    {
        Log::error("Error: Bus Pal i2c bulk transfer command failed\n");
        return false;
    }

    uint8_t * respData = 0;

    while(respData == NULL)
    {
        respData = response(1);
    }

    if (respData[0] != kResponseOk)
    {
        Log::error("Error: bad response to i2c bulk transfer, response byte = 0x%x\n", respData[0]);
        return false;
    }

    return true;
}

// See See bus_pal.h for documentation on this method.
int BusPal::readI2c(uint8_t * data, size_t byteCount)
{
    assert(data);

    I2cWriteThenReadCommand command;
    command.command = kI2c_WriteThenRead;
    command.numBytesToWrite = 0;
    command.numBytesToRead = std::min<int>(byteCount, kBulkTransferMax);

    int actualBytes = buspal_serial_write(m_fileDescriptor, reinterpret_cast<char *>(&command), sizeof(command));
    if (actualBytes != sizeof(command))
    {
        Log::error("Error: Bus Pal i2c read command failed\n");
        return false;
    }

    uint8_t * respData = 0;

    while(respData == NULL)
    {
        respData = response(1);
    }

    if (respData[0] != kResponseOk)
    {
        Log::error("Error: bad response to i2c read, response byte 0 = 0x%x\n", respData[0]);
        return false;
    }

    int remainingBytes = byteCount;
    char * buffer = reinterpret_cast<char *>(data);

    while (remainingBytes > 0)
    {
        actualBytes = buspal_serial_read(m_fileDescriptor, buffer, remainingBytes);
        if (actualBytes > 0)
        {
            remainingBytes -= actualBytes;
            buffer += actualBytes;
        }
        else
        {
            // If no bytes received, it is a timeout error.
            return 0;
        }
    }

    return byteCount;
}

int BusPal::buspal_serial_read(int fd, char *buf, int size)
{
    int retVal = serial_read(fd, buf, size);

    // If serial_read returned a 0 it is a timeout failure but doing this double
    // read fixes an intermittent issue when talking to bus pal the byte is actually sent from
    // the bus pal but serial_read returns empty however the byte is available on the next read
    if (retVal == 0)
    {
        retVal = serial_read(fd, buf, size);
    }

    if (Log::getLogger()->getFilterLevel() == Logger::kDebug2)
    {
        // Log bytes read in hex
        Log::debug2("<");
        for (int i = 0; i < retVal; i++)
        {
            Log::debug2("%02x", (uint8_t)buf[i]);
            if (i != (retVal - 1))
            {
                Log::debug2(" ");
            }
        }
        Log::debug2(">\n");
    }

    return retVal;
}

int BusPal::buspal_serial_write(int fd, char *buf, int size)
{
    if (Log::getLogger()->getFilterLevel() == Logger::kDebug2)
    {
        // Log bytes written in hex
        Log::debug2("[");
        for (int i = 0; i < size; i++)
        {
            Log::debug2("%02x", (uint8_t)buf[i]);
            if (i != (size - 1))
            {
                Log::debug2(" ");
            }
        }
        Log::debug2("]\n");
    }

    return serial_write(fd, buf, size);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

