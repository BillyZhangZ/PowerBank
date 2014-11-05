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

#ifndef _itf_define_h
#define _itf_define_h

#include "bootloader_common.h"

//! @addtogroup sbloader
//! @{

//! Defines the number of bytes in a cipher block (chunk). This is dictated by
//! the encryption algorithm.
#define BYTES_PER_CHUNK 16

//! Defines the unit of data that is returned by the rom_BootNext function.
//! From the boot driver's view, an image file looks like an array of these
//! data units (chunks). The size of a chunk is dictated by the cipher algorithm.
//! The Encore ROM will use AES-128 block encryption. Thus a chunk is defined as
//! 128 bits or 16 bytes. While any alignment is supported, performance is best
//! if chunks are word (32-bit) aligned.
typedef uint8_t chunk_t[BYTES_PER_CHUNK];

//! This structure defines the parameter passed to the rom_BootInit function.
//! \note   The memory region pMem[size] is word aligned. Boot drivers must
//!         handle stricter alignment requirements internally.
typedef struct
{
    void *pMem;         //!< Pointer to memory region for use by the driver.
    int size;           //!< Size of the memory region.
    uint32_t mode;      //!< Current boot mode.
    uint32_t id;        //!< ID of the boot image. Always 0 on the first call.
    void * bootParam;   //!< Parameter passed from loader init to boot driver.
} rom_BootInit_t;

//! This enumeration defines the first parameter passed to the rom_BootControl
//! function. It specifies the requested driver control action.
typedef enum
{
    BOOT_PAUSE,         //!< Pause all active operations
    BOOT_RESUME         //!< Update driver settings and resume operations
} rom_BootAction_t;

//! Standard boot driver API. All boot devices \b must include an instance of
//! this structure as the first element of it's interface structure.
typedef struct _rom_BootItf
{
    int (*Init)(rom_BootInit_t *);
    chunk_t * (*Next)(int *, bool * underrun);
    int (*Skip)(int);
    int (*Stop)(void);
} rom_BootItf_t;

//! Chunking gasket interface definition.
typedef struct _chunk_BootItf
{
    rom_BootItf_t Boot;                 //!< First member must be the standard API
} chunk_BootItf_t;

//! ROM interface table definition.
typedef struct _rom_ItfTbl
{
    const void      *pLdr;      //!< Boot loader
	chunk_BootItf_t *pChunk;    //!< Chunking gastet boot driver
} rom_ItfTbl_t;

//! @}

#endif // _itf_define_h

