/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
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
#include "device/fsl_device_registers.h"
#endif // BOOTLOADER_HOST
#include "sram_init.h"
#include <assert.h>


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////


//! @brief Initialize address ranges of SRAM for chips belongs to cm4 family
status_t sram_init(void)
{
#if defined (__CORE_CM4_H_GENERIC)

    uint32_t ram_size = 0;

    switch(HW_SIM_SOPT1.B.RAMSIZE)
    {
        case kRamSize32kIndex:
            ram_size = 32 * 1024UL;
            break;
        case kRamSize64kIndex:
            ram_size = 64 * 1024UL;
            break;
        case kRamSize96kIndex:
            ram_size = 96 * 1024UL;
            break;
        case kRamSize128kIndex:
            ram_size = 128 * 1024UL;
            break;
        case kRamSize256kIndex:
            ram_size = 256 * 1024UL;
            break;
        default:
            ram_size = 0;
            break;
    }

    assert(ram_size > 0);

    if(ram_size)
    {
        // Update  address range of SRAM
        memory_map_entry_t* map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexSRAM];

#if (defined (K64F12_SERIES)|| defined (K66F18_SERIES))

        uint32_t tmp = ram_size / 4;
        map->startAddress = kSRAMSeparatrix - tmp;  // start of SRAM
        map->endAddress = kSRAMSeparatrix + (tmp * 3) - 1;    // end of SRAM

#elif (defined(K22F51212_SERIES)) || \
      (defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FX512VMJ12))

        uint32_t tmp = ram_size / 2;
        map->startAddress = kSRAMSeparatrix - tmp;  // start of SRAM
        map->endAddress = kSRAMSeparatrix + tmp - 1;    // end of SRAM

#endif // K64F12_SERIES
    }

#else
    #error "This function only applies to cm4 family"
#endif // __CORE_CM4_H_GENERIC

    return kStatus_Success;
}




////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
