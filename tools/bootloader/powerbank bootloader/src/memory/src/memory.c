/*
 * Copyright (c) 2013-2014, Freescale Semiconductor, Inc.
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

#include "bootloader_common.h"
#include "bootloader/context.h"
#include "memory/memory.h"
#if !defined(BOOTLOADER_HOST)
#include "drivers/flash/flash.h"
#include "device/fsl_device_registers.h"
#endif // BOOTLOADER_HOST
#include "flash_memory.h"
#include <assert.h>
#include "sram_init.h"


////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

// Forward function declarations.
static status_t find_map_entry(uint32_t address, uint32_t length, const memory_map_entry_t ** map);
bool mem_is_block_reserved(uint32_t address, uint32_t length);


////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Interface to generic memory operations.
const memory_interface_t g_memoryInterface = {
    mem_init,
    mem_read,
    mem_write,
#if !BL_MIN_PROFILE
    mem_fill
#endif // !BL_MIN_PROFILE
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See memory.h for documentation on this function.
status_t mem_read(uint32_t address, uint32_t length, uint8_t * buffer)
{
    if (length == 0)
    {
        return kStatus_Success;
    }

    const memory_map_entry_t * mapEntry;
    status_t status = find_map_entry(address, length, &mapEntry);
    if (status == kStatus_Success)
    {
        status = mapEntry->memoryInterface->read(address, length, buffer);
    }
    return status;
}

// See memory.h for documentation on this function.
status_t mem_write(uint32_t address, uint32_t length, const uint8_t * buffer)
{
    if (length == 0)
    {
        return kStatus_Success;
    }

    if (mem_is_block_reserved(address, length))
    {
        return kStatusMemoryRangeInvalid;
    }

    const memory_map_entry_t * mapEntry;
    status_t status = find_map_entry(address, length, &mapEntry);
    if (status == kStatus_Success)
    {
        status = mapEntry->memoryInterface->write(address, length, buffer);
    }
    return status;
}

// See memory.h for documentation on this function.
status_t mem_fill(uint32_t address, uint32_t length, uint32_t pattern)
{
    if (length == 0)
    {
        return kStatus_Success;
    }

    if (mem_is_block_reserved(address, length))
    {
        return kStatusMemoryRangeInvalid;
    }

    const memory_map_entry_t * mapEntry;
    status_t status = find_map_entry(address, length, &mapEntry);
    if (status == kStatus_Success)
    {
        status = mapEntry->memoryInterface->fill(address, length, pattern);
    }
    return status;
}

//! @brief Find a map entry that matches address and length.
//!
//! @param address Start address for the memory operation.
//! @param length Number of bytes on which the operation will act.
//! @param map The matching map entry is returned through this pointer if the return status
//!     is #kStatus_Success.
//!
//! @retval #kStatus_Success A valid map entry was found and returned through @a map.
//! @retval #kStatusMemoryRangeInvalid The requested address range does not match an entry, or
//!     the length extends past the matching entry's end address.
static status_t find_map_entry(uint32_t address, uint32_t length, const memory_map_entry_t ** map)
{
    status_t status = kStatusMemoryRangeInvalid;

    // Set starting entry.
    assert(map);
    *map = &g_bootloaderContext.memoryMap[0];

    // Scan memory map array looking for a match.
    while ((length > 0) && map && *map && (*map)->memoryInterface)
    {
        // Check if the start address is within this entry's address range.
        if ((address >= (*map)->startAddress) && (address <= (*map)->endAddress))
        {
            // Check that the length fits in this entry's address range.
            if ((address + length - 1) <= (*map)->endAddress)
            {
                status = kStatus_Success;
            }
            break;
        }
        ++(*map);
    }

    return status;
}

// See memory.h for documentation on this function.
bool mem_is_block_reserved(uint32_t address, uint32_t length)
{
    if (!g_bootloaderContext.propertyInterface->store->validateRegions)
    {
        return false;
    }

    uint32_t end = address + length - 1;
    for (int i = 0; i < kProperty_ReservedRegionsCount; ++i)
    {
        reserved_region_t * region = &g_bootloaderContext.propertyInterface->store->reservedRegions[i];

        if ((region->startAddress == 0) && (region->endAddress == 0))
        {
            // Special case, empty region
            continue;
        }

        if ((address <= region->endAddress) && (end >= region->startAddress))
        {
            return true;
        }
    }
    return false;
}


// See memory.h for documentation on this function.
status_t mem_init(void)
{
    status_t status = kStatus_Success;



#if !defined(BOOTLOADER_HOST)

    // Update address range of flash
    memory_map_entry_t * map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexFlashArray];
    flash_get_property(&g_bootloaderContext.flashState, kFlashProperty_FlashBlockBaseAddr, &map->startAddress);
    uint32_t tmp;
    flash_get_property(&g_bootloaderContext.flashState, kFlashProperty_TotalFlashSize, &tmp);
    map->endAddress = map->startAddress + tmp - 1;

    // Update address range of sram
    status = sram_init();

#endif // BOOTLOADER_HOST

    return status;
}


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
