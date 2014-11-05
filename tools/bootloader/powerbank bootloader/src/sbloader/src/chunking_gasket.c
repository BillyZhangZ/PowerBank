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

#include "bootloader_common.h"
#include "sbloader/chunking_gasket.h"
#include <assert.h>

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! Driver global data.
static chunk_data_t g_chunkData;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! Initializes chunking gasket driver.
//!
//! @param initData Initialization data.
int chunk_init(rom_BootInit_t * initData)
{
    // Initialize global data.
    //! \todo use passed initData mem for global data
    g_chunkData.pullData = (chunk_pull_data_callback_t)initData->bootParam;
    g_chunkData.chunkFillPosition = 0;
    g_chunkData.dataPacket = NULL;
    g_chunkData.dataPacketSize = 0;
    g_chunkData.dataDrainPosition = 0;
    g_chunkData.skipCount = 0;
    return 0;
}

//! Builds up one chunk by calling peripheral packet read interface.
//! Returns at most one chunk, may return zero chunks.
//!
//! @param count Pointer to requested and returned count. Requested count is ignored (at most 1 chunk is returned).
//! @param underrun Pointer to returned flag indicating that data was required but not available.
//! @return pointer to internal chunk buffer or NULL.
chunk_t * chunk_next(int * count, bool * underrun)
{
    assert(count);
    assert(underrun);
    unsigned required = sizeof(g_chunkData.chunkBuf) - g_chunkData.chunkFillPosition;
    unsigned available = g_chunkData.dataPacketSize - g_chunkData.dataDrainPosition;
    *underrun = false;

    if (available == 0)
    {
        g_chunkData.dataDrainPosition = 0;
        assert(g_chunkData.pullData);
        g_chunkData.pullData(&g_chunkData.dataPacket, &g_chunkData.dataPacketSize);
        if (g_chunkData.dataPacketSize == 0)
        {
            // No data available.
            *underrun = true;
            *count = 0;
            return NULL;
        }
        available = g_chunkData.dataPacketSize;
    }

    while ((required > 0) && (available > 0))
    {
        g_chunkData.chunkBuf[g_chunkData.chunkFillPosition++] = g_chunkData.dataPacket[g_chunkData.dataDrainPosition++];
        --required;
        --available;
    };

    if (required == 0)
    {
        // Got one chunk, reset fill postition.
        g_chunkData.chunkFillPosition = 0;
        // If skipping, reduce skip count and throw away chunk.
        if (g_chunkData.skipCount > 0)
        {
            --g_chunkData.skipCount;
            *count = 0;
            return NULL;
        }
        // Return chunk.
        *count = 1;
        return &g_chunkData.chunkBuf;
    }
    else
    {
        // Still working on a chunk, return a zero count.
        *count = 0;
        return NULL;
    }
}

//! Causes chunks to be skipped.
//!
//! @param count Number of chunks to skip.
//! @retval 0
int chunk_skip(int count)
{
    // Set global count of number of chunks to skip on next read.
    g_chunkData.skipCount = (unsigned)count;
    return 0;
}

//! Shut down chunking gasket driver.
//!
//! @retval 0
int chunk_stop()
{
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
