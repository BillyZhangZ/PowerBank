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

#ifndef _bus_pal_h_
#define _bus_pal_h_

#include "blfwk/host_types.h"

namespace blfwk {

/*!
 * @brief Interface with the BusPal.
 *
 * For hardware documentation, see http://dangerousprototypes.com/docs/Bus_Pal.
 * This class is based on the pyBusPalLite python package.
 */
class BusPal
{
    public:
        //! @brief Pin configurations.
        enum {
            kPinCfg_Power = 0x08,
            kPinCfg_Pullups = 0x04,
            kPinCfg_Aux = 0x02,
            kPinCfg_Cs = 0x01
        };

        //! @brief Bit Bang IO pins.
        enum {
            kBbioPin_Mosi = 0x01,
            kBbioPin_Clk = 0x02,
            kBbioPin_Miso = 0x04,
            kBbioPin_Cs = 0x08,
            kBbioPin_Aux = 0x10,
            kBbioPin_Pullup = 0x20,
            kBbioPin_Power = 0x40
        };

        //! @brief Constants.
        enum {
            kResetCount = 20,       //!< Max number of nulls to send to enter BBIO mode
            kResponseOk = 0x01,     //!< Successful command response
            kBulkTransferMax = 4096,  //!< Max number of bytes per bulk transfer
            kMaxResponse = kBulkTransferMax //!< Max number of bytes in command response, including bulk transfer response, plus some padding
        };

        //! @brief BusPal Transports.
        enum bus_pal_transport_t {
            kBusPalTransport_None,
            kBusPalTransport_SPI,
            kBusPalTransport_I2C
        };

        //! @brief SPI clock polarity configuration.
        enum spi_clock_polarity_t {
            kSpiClockPolarity_ActiveHigh = 0,   //!< Active-high SPI clock (idles low).
            kSpiClockPolarity_ActiveLow = 1     //!< Active-low SPI clock (idles high).
        };

        //! @brief SPI clock phase configuration.
        enum spi_clock_phase_t {
            kSpiClockPhase_FirstEdge = 0,       //!< First edge on SPSCK occurs at the middle of the first cycle of a data transfer.
            kSpiClockPhase_SecondEdge = 1       //!< First edge on SPSCK occurs at the start of the first cycle of a data transfer.
        };

        //! @brief SPI data shifter direction options.
        enum spi_shift_direction_t {
            kSpiMsbFirst = 0,    //!< Data transfers start with most significant bit.
            kSpiLsbFirst = 1    //!< Data transfers start with least significant bit.
        };

        //! @brief I2C default address.
        static const uint8_t kBusPalDefaultI2cSlaveAddress = 0x10;  // I2C Slave 7-bit address

    public:
        //! @brief BusPal configuration data.
        struct BusPalConfigData
        {
            bus_pal_transport_t transport;
            uint32_t spiSpeedKHz;
            spi_clock_polarity_t spiPolarity;
            spi_clock_phase_t spiPhase;
            spi_shift_direction_t spiDirection;
            uint8_t i2cAddress;
            uint32_t i2cSpeedKHz;

            BusPalConfigData(bus_pal_transport_t bus = BusPal::kBusPalTransport_None)
                : transport(bus)
            {
                spiSpeedKHz  = 100;
                spiPolarity  = BusPal::kSpiClockPolarity_ActiveLow;
                spiPhase     = BusPal::kSpiClockPhase_SecondEdge;
                spiDirection = BusPal::kSpiMsbFirst;
                i2cAddress   = BusPal::kBusPalDefaultI2cSlaveAddress;
                i2cSpeedKHz  = 100;
            }
        };

        //! @brief Constructor.
        BusPal(void);

        //! @brief Destructor.
        virtual ~BusPal();

        //! @brief parse the passed in options into the config structure.
        static bool parse(const string_vector_t& params, BusPal::BusPalConfigData& config);

        //! @brief initializes the com port used for Bus Pal
        virtual bool init(const char * busPalPort, long uartSpeed);

        //! @brief Reset to bit bang mode from another peripheral mode.
        virtual bool reset();

        //! @brief Reset the bus pal, comes up in terminal mode.
        virtual bool resetHardware();

        //! @brief Enter bit bang (binary scripting) mode.
        //!
        //! Call this first before entering other peripheral modes.
        virtual bool enterBitBangMode();

        //! @brief Enter Spi mode.
        virtual bool enterSpiMode();

        //! @brief Enter I2c mode.
        virtual bool enterI2cMode();

        //! @brief Enter Uart mode. Not currently supported for bus pal
        virtual bool enterUartMode() { return false; }

        //! @brief Enter 1wire mode. Not currently supported for bus pal
        virtual bool enter1WireMode() { return false; }

        //! @brief Enter raw wire mode. Not currently supported for bus pal
        virtual bool enterRawWireMode() { return false; }

        //! @brief Raw configure pins.
        virtual bool rawConfigurePins(uint8_t config) { return true; }

        //! @brief Raw set pins.
        virtual bool rawSetPins(uint8_t config);

        //! @brief Configure pins.
        virtual bool configurePins(uint8_t config = 0) { return true; }

        //! @brief Read pins.
        virtual uint8_t readPins() { return 0; }

        //! @brief Set SPI speed.
        virtual bool setSpiSpeed(unsigned int speed);

        //! @brief Sets the SPI configuration
        virtual bool setSpiConfig(spi_clock_polarity_t polarity, spi_clock_phase_t phase, spi_shift_direction_t direction);

        //! @brief Set I2c address
        virtual bool setI2cAddress(uint8_t address);

        //! @brief Set I2c speed
        virtual bool setI2cSpeed(uint32_t speed);

        //! @brief Read response.
        //!
        //! @retval NULL No response from device
        //! @retval Non-NULL Pointer to internal array of bytes at least size byteCount
        virtual uint8_t * response(size_t byteCount = 1);

        //! @brief Bulk write transfer.
        virtual bool bulkTransfer(const uint8_t * data, size_t byteCount = 1);

        //! @brief Read data.
        virtual int read(uint8_t * data, size_t byteCount);

    protected:
        //! @brief Write command, check string response.
        virtual bool writeCommand(uint8_t commandByte, const char * expectedResponse);

        //! @brief Write command, return 1 byte response.
        virtual uint8_t writeCommand(uint8_t commandByte);

        //! @brief bulkTransfer via Spi
        virtual bool bulkTransferSpi(const uint8_t * data, size_t byteCount = 1);
        //! @brief bulkTransfer via I2c
        virtual bool bulkTransferI2c(const uint8_t * data, size_t byteCount = 1);

        //! @brief read via Spi
        virtual int readSpi(uint8_t * data, size_t byteCount);
        //! @brief read via I2c
        virtual int readI2c(uint8_t * data, size_t byteCount);

        //! @brief Flush serial input.
        virtual void flushInput();

        //! @brief Overriden serial_read for logging
        int buspal_serial_read(int fd, char *buf, int size);

        //! @brief Overriden serial_write for logging
        int buspal_serial_write(int fd, char *buf, int size);

        //! @brief Flushes the serial port of any RX data
        void flushRX(int fd);

    protected:
        enum bus_pal_mode_t {
            kBusPalModeBitBang,
            kBusPalModeSpi,
            kBusPalModeI2c
        };
        int m_fileDescriptor;                      //!< PC COM port file descriptor.
        uint8_t m_responseBuffer[kMaxResponse];    //!< Command response buffer.
        bus_pal_mode_t m_mode;
};

} // namespace blfwk

#endif // _bus_pal_h_

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

