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
#if !defined(__CONTEXT_H__)
#define __CONTEXT_H__

#include "fsl_platform_types.h"
#include "fsl_platform_status.h"
#include "bootloader/peripheral.h"
#include "memory/memory.h"
#include "packet/command_packet.h"
#include "bootloader/command.h"
#include "property/property.h"

#if !BOOTLOADER_HOST
#include "flash/flash.h"
#endif

//! @addtogroup bl_context
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

#if !BOOTLOADER_HOST

//! @brief Interface for the flash driver.
typedef struct FlashDriverInterface {
    status_t (*flash_init)(flash_driver_t * driver);
    status_t (*flash_erase_all)(flash_driver_t * driver);
    status_t (*flash_erase_all_unsecure)(flash_driver_t * driver);
    status_t (*flash_erase)(flash_driver_t * driver, uint32_t start, uint32_t lengthInBytes);
    status_t (*flash_program)(flash_driver_t * driver, uint32_t start, uint32_t * src, uint32_t lengthInBytes);
    status_t (*flash_get_security_state)(flash_driver_t * driver, flash_security_state_t * state);
    status_t (*flash_security_bypass)(flash_driver_t * driver, const uint8_t * backdoorKey);
    status_t (*flash_verify_erase_all)(flash_driver_t * driver, flash_margin_value_t margin);
    status_t (*flash_verify_erase)(flash_driver_t * driver, uint32_t start, uint32_t lengthInBytes, flash_margin_value_t margin);
    status_t (*flash_verify_program)(flash_driver_t * driver, uint32_t start, uint32_t lengthInBytes,
                                  const uint8_t * expectedData, flash_margin_value_t margin,
                                  uint32_t * failedAddress, uint8_t * failedData);
    status_t (*flash_get_property)(flash_driver_t * driver, flash_property_t whichProperty, uint32_t * value);
} flash_driver_interface_t;

#else // !BOOTLOADER_HOST

// Provide stub definitions for flash driver types for the host.
typedef uint32_t flash_driver_interface_t;
typedef uint32_t flash_driver_t;

#endif // !BOOTLOADER_HOST

//! @brief Structure of bootloader global context.
typedef struct _bootloaderContext {
    //! @name API tree
    //@{
    const memory_interface_t * memoryInterface; //!< Abstract interface to memory operations.
    const memory_map_entry_t * memoryMap;       //!< Memory map used by abstract memory interface.
    const property_interface_t * propertyInterface; //!< Interface to property store.
    const command_interface_t * commandInterface;   //!< Interface to command processor operations.
    const flash_driver_interface_t * flashDriverInterface; //!< Flash driver interface.
    const peripheral_descriptor_t * allPeripherals; //!< Array of all peripherals.
    //@}

    //! @name Runtime state
    //@{
    const peripheral_descriptor_t * activePeripheral;   //!< The currently active peripheral.
    flash_driver_t flashState;                      //!< Flash driver instance.
    //@}
} bootloader_context_t;

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

extern bootloader_context_t g_bootloaderContext;

//! @}

#endif // __CONTEXT_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

