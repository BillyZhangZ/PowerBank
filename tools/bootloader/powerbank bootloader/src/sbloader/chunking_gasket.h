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

#ifndef _chunking_gasket_h_
#define _chunking_gasket_h_

#include "itf_define.h"

//! @addtogroup chunking_gasket
//! @{

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @brief Type for the pull data callback routine.
typedef void (*chunk_pull_data_callback_t)(uint8_t ** data, uint32_t * count);

//! @brief Driver data definition.
typedef struct _chunk_data
{
    chunk_pull_data_callback_t pullData;    //!< Pull data routine.
    chunk_t     chunkBuf;                   //!< 16-byte chunk buffer.
    unsigned    chunkFillPosition;          //!< Current position to fill in chunkBuf.
    uint8_t *   dataPacket;                 //!< Pointer to current data packet.
    uint32_t    dataPacketSize;             //!< Size of current data packet.
    unsigned    dataDrainPosition;          //!< Current position to drain in dataPacket.
    unsigned    skipCount;                  //!< Number of chunks to skip on next read.
} chunk_data_t;

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

////////////////////////////////////////////////////////////////////////////////
// API
////////////////////////////////////////////////////////////////////////////////

//! @brief Initialize chunking gasket driver.
int chunk_init(rom_BootInit_t * pInit);

//! @brief Read chunks.
chunk_t * chunk_next(int * pCount, bool * underrun);

//! @brief Skip chunks.
int chunk_skip(int count);

//! @brief Stop reading chunks.
int chunk_stop(void);

#if defined(__cplusplus)
}
#endif // __cplusplus

//! @}

#endif // _chunking_gasket_h_
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
