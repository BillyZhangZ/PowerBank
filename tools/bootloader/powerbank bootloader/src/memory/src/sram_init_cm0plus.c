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
#include <assert.h>
#include "sram_init.h"


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////



//! @brief Initialize address ranges of SRAM for chips belongs to cm0plus family
status_t sram_init(void)
{
#if defined (__CORE_CM0PLUS_H_GENERIC)
    /* Acording to current chip spec, All chips belonging CM0+ family have the same feature of calculating address range of SRAM, but maybe it is not suitable for NPI
     * So now this function only supports following series.
     */
#if ( defined (KL02Z4_SERIES)  || \
      defined (KL03Z4_SERIES)  || \
      defined (KL05Z4_SERIES)  || \
      defined (KL25Z4_SERIES)  || \
      defined (KL27Z4_SERIES)  || \
      defined (KL43Z4_SERIES)  || \
      defined (KL46Z4_SERIES) )
    // For CM0+ family RAM is dividved as [kSRAMSeparatrix -1/4 RAMSIZE, kSRAMSeparatrix-1], [kSRAMSeparatrix, kSRAMSeparatrix + 3/4 RAMSIZE -1]
    uint32_t tmp = HW_SIM_SDID.B.SRAMSIZE;

    uint32_t ram_size = 0;
    if (tmp <= kMaxRamIndex)
    {
        ram_size = kMinKlRamSize << tmp;
    }

    assert(ram_size > 0);

    if (ram_size > 0)
    {
        // Update address range of SRAM
        memory_map_entry_t * map = (memory_map_entry_t *)&g_bootloaderContext.memoryMap[kIndexSRAM];
        tmp = ram_size / 4;
        map->startAddress = kSRAMSeparatrix - tmp;          // start of SRAM
        map->endAddress = kSRAMSeparatrix + tmp * 3 - 1;    // end of SRAM
    }

#endif // CPU_MKL05Z32LF4

#else
    #error "This function only applies to cm0plus family"
#endif // __CORE_CM0PLUS_H_GENERIC

    return kStatus_Success;
}



////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
