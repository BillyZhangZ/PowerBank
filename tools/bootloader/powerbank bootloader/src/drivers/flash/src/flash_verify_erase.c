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

#include "SSD_FTFx_Common.h"
#include "flash/flash.h"
#include "fsl_platform_common.h"

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

// Make a mark here, Waiting for PEx team's update
// -------------------------------------------------
// This feature should be defined in fls_flash_features.h
//! @brief Flash verify unit of FTFx_VERIFY_SECTION cmd.
enum _flash_verify_unit
{
#if FSL_FEATURE_FLASH_IS_FTFE
    kFlashVerifyUnitInBytes = 16
#else
    kFlashVerifyUnitInBytes = FSL_FEATURE_FLASH_PFLASH_BLOCK_WRITE_UNIT_SIZE
#endif
};
// -------------------------------------------------

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

// See flash.h for documentation of this function.
status_t flash_verify_erase(flash_driver_t * driver, uint32_t start, uint32_t lengthInBytes, flash_margin_value_t margin)
{
    // Check arguments.
    status_t returnCode = flash_check_range(driver, start, lengthInBytes);
    if (returnCode)
    {
        return returnCode;
    }

    uint32_t blockSize = driver->PFlashTotalSize / driver->PFlashBlockCount;
    uint32_t nextBlockStartAddress = ALIGN_UP(start, blockSize);
    if (nextBlockStartAddress == start)
    {
        nextBlockStartAddress += blockSize;
    }


    uint32_t remainingBytes = lengthInBytes;

    while (remainingBytes)
    {
        uint32_t verifyLength = nextBlockStartAddress - start;
        if (verifyLength > remainingBytes)
        {
            verifyLength = remainingBytes;
        }

        uint32_t numberOfPhrases = verifyLength / kFlashVerifyUnitInBytes;

        // Fill in verify section command parameters.
        kFCCOBx[0] = start;
        HW_FTFx_FCCOBx_WR(0, FTFx_VERIFY_SECTION);
        HW_FTFx_FCCOBx_WR(4, numberOfPhrases >> 8);
        HW_FTFx_FCCOBx_WR(5, numberOfPhrases & 0xFF);
        HW_FTFx_FCCOBx_WR(6, margin);

        // calling flash command sequence function to execute the command
        returnCode = flash_command_sequence();
        if (returnCode)
        {
            return returnCode;
        }

        remainingBytes -= verifyLength;
        start += verifyLength;
        nextBlockStartAddress += blockSize;
    }

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

