/*
 * Copyright (c) 2013, Freescale Semiconductor, Inc.
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

#ifndef _property_h
#define _property_h

#include <stdint.h>
#include "bootloader_common.h"
#include "packet/command_packet.h"
#if !defined(BOOTLOADER_HOST)
#include "device/fsl_device_registers.h"
#endif

//! @addtogroup property
//! @{

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

//! @name Command Availability
//@{

//! Sets a bit in the available commands property value to indicate a command with
//! the given tag is available.
#define HAS_CMD(tag) (1 << ((tag)-kFirstCommandTag))

//! Checks whether a command with the specified tag is present in the provided command
//! availability mask.
#define IS_CMD_AVAILABLE(mask, tag) (((mask) & HAS_CMD(tag)) != 0)

//@}

//! @brief Property store status codes.
enum _property_errors
{
    kStatus_UnknownProperty         = MAKE_STATUS(kStatusGroup_PropertyStore, 0),
    kStatus_ReadOnlyProperty        = MAKE_STATUS(kStatusGroup_PropertyStore, 1),  //!< Property is read-only.
    kStatus_InvalidPropertyValue    = MAKE_STATUS(kStatusGroup_PropertyStore, 2)   //!< Property value is out of range.
};

//! @brief Property tags.
//! @note Do not change any tag values. Add tags at the end.
enum _property_tag
{
    kPropertyTag_CurrentVersion         = 0x01,
    kPropertyTag_AvailablePeripherals   = 0x02,
    kPropertyTag_FlashStartAddress      = 0x03,
    kPropertyTag_FlashSizeInBytes       = 0x04,
    kPropertyTag_FlashSectorSize        = 0x05,
    kPropertyTag_FlashBlockCount        = 0x06,
    kPropertyTag_AvailableCommands      = 0x07,
    kPropertyTag_CrcCheckStatus         = 0x08,
    kPropertyTag_VerifyWrites           = 0x0a,
    kPropertyTag_MaxPacketSize          = 0x0b,
    kPropertyTag_ReservedRegions        = 0x0c,
    kPropertyTag_ValidateRegions        = 0x0d,
    kPropertyTag_RAMStartAddress        = 0x0e,
    kPropertyTag_RAMSizeInBytes         = 0x0f,
    kPropertyTag_SystemDeviceId         = 0x10,
    kPropertyTag_FlashSecurityState     = 0x11,
    kPropertyTag_UniqueDeviceId         = 0x12,
};

//! @brief Property constants.
enum _property_constants {
    kProperty_ReservedRegionsCount = 2,
    kProperty_FlashReservedRegionIndex = 0,
    kProperty_RamReservedRegionIndex = 1
};

//! @brief Structure of version property.
typedef union BootloaderVersion
{
    struct {
        uint32_t bugfix : 8;     //!< bugfix version [7:0]
        uint32_t minor : 8;      //!< minor version [15:8]
        uint32_t major : 8;      //!< major version [23:16]
        uint32_t name : 8;       //!< name [31:24]
    };
    uint32_t version;   //!< combined version numbers
} bootloader_version_t;

//! @brief Bit positions for clock flags in configuration data.
enum _clock_flags {
    kClockFlag_HighSpeed = (1<<0)
};

//! @brief Format of bootloader configuration data on Flash.
typedef struct BootloaderConfigurationData
{
    uint32_t tag;                          //!< [00:03] Tag value used to validate the bootloader configuration data. Must be set to 'kcfg'.
    uint32_t crcStartAddress;              //!< [04:07]
    uint32_t crcByteCount;                 //!< [08:0b]
    uint32_t crcExpectedValue;             //!< [0c:0f]
    uint8_t enabledPeripherals;            //!< [10:10]
    uint8_t i2cSlaveAddress;               //!< [11:11]
    uint16_t peripheralDetectionTimeoutMs; //!< [12:13] Timeout in milliseconds for peripheral detection before jumping to application code
    uint16_t usbVid;                       //!< [14:15]
    uint16_t usbPid;                       //!< [16:17]
    uint32_t usbStringsPointer;            //!< [18:1b]
    uint8_t clockFlags;                    //!< [1c:1c] High Speed and other clock options
    uint8_t clockDivider;                  //!< [1d:1d] One's complement of clock divider, zero divider is divide by 1
} bootloader_configuration_data_t;

//! @brief Structure of a reserved regions entry.
typedef struct ReservedRegion
{
    uint32_t startAddress;
    uint32_t endAddress;
} reserved_region_t;

//! @brief Structure of a unique device id.
typedef struct UniqueDeviceId
{
    uint32_t uidl;
    uint32_t uidml;
    uint32_t uidmh;
#if defined(BOOTLOADER_HOST) | defined(HW_SIM_UIDH)
    uint32_t uidh;
#endif
} unique_device_id_t;

//! @brief Structure of property store.
typedef struct PropertyStore
{
    bootloader_version_t currentVersion; //!< Current bootloader version.
    bootloader_version_t serialProtocolVersion; //!< Serial protocol version number.
    uint32_t availablePeripherals;      //!< The set of peripherals supported available on this chip. See enum _peripheral_types in peripheral.h.
    uint32_t flashStartAddress;         //!< Start address of program flash.
    uint32_t flashSizeInBytes;          //!< Size in bytes of program flash.
    uint32_t flashSectorSize;           //!< The size in bytes of one sector of program flash. This is the minimum erase size.
    uint32_t flashBlockSize;            //!< The size in bytes of one block of program flash.
    uint32_t flashBlockCount;           //!< Number of blocks in the flash array.
    uint32_t ramStartAddress;           //!< Start address of RAM
    uint32_t ramSizeInBytes;            //!< Size in bytes of RAM
    uint32_t crcCheckStatus;            //!< Status code from the last CRC check operation.
    uint32_t verifyWrites;              //!< Boolean controlling whether the bootloader will verify writes to flash. Non-zero enables verificaton. Writable by host.
    uint32_t validateRegions;           //!< Boolean controlling whether the bootloader will validate that memory regions are not reserved. Non-zero enables verificaton. Writable by host.
    uint32_t availableCommands;         //!< Bit mask of the available commands.
    unique_device_id_t UniqueDeviceId;  //!< Unique identification for the device.
    reserved_region_t reservedRegions[kProperty_ReservedRegionsCount]; //!< Flash and Ram reserved regions.
    bootloader_configuration_data_t configurationData; //!< Configuration data from flash address 0x3e0-0x3ff in sector 0 (32 bytes max)
} property_store_t;

enum _property_store_tags
{
    //! @brief Tag value used to validate the bootloader configuration data.
    kPropertyStoreTag = FOUR_CHAR_CODE('k', 'c', 'f', 'g')
};

//! @brief Interface to property operations.
typedef struct PropertyInterface
{
    status_t (*load_user_config)(void);                                         //!< Load the user configuration data
    status_t (*init)(void);                                                     //!< Initialize
    status_t (*get)(uint8_t tag, const void ** value, uint32_t * valueSize);    //!< Get property
    status_t (*set_uint32)(uint8_t tag, uint32_t value);                        //!< Set uint32_t property
    property_store_t * store;                                                   //!< The property store
} property_interface_t;

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

//! @brief Property interface.
extern const property_interface_t g_propertyInterface;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if __cplusplus
extern "C" {
#endif

//! @name Property Store
//@{

//! @brief Early initialization function to get user configuration data
status_t bootloader_property_load_user_config(void);

//! @brief Initialize the property store.
status_t bootloader_property_init(void);

//! @brief Get a property.
//!
//! Example calling sequence for uint32_t property:
//! @code
//! void * value;
//! uint32_t valueSize;
//! status_t status = bootloader_property_get(sometag, &value, &valueSize);
//! uint32_t result = *(uint32_t *)value;
//! @endcode
//!
//! @param tag Tag of the requested property
//! @param value Pointer to where to write a pointer to the result, may be NULL
//! @param valueSize Size in bytes of the property value, may be NULL
//!
//! @retval kStatus_Success
//! @retval kStatus_UnknownProperty
status_t bootloader_property_get(uint8_t tag, const void ** value, uint32_t * valueSize);

//! @brief Set a property.
//!
//! Only uint32_t properties can be set with this function.
//!
//! @param tag Tag of the property to set
//! @param value New property value
//!
//! @retval kStatus_Success
//! @retval kStatus_UnknownProperty
//! @retval kStatus_ReadOnlyProperty
status_t bootloader_property_set_uint32(uint8_t tag, uint32_t value);

//@}

#if __cplusplus
}
#endif

//! @}

#endif // _property_h
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
