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

#ifndef _loader_h
#define _loader_h

#include <stdint.h>
#include "fsl_platform_types.h"
#include "fsl_platform_status.h"
#include "itf_rom.h"
#include "crc/crc32.h"

//! @addtogroup sbloader
//! @{

//! @brief SB loader status codes.
enum _sbloader_status
{
    kStatusRomLdrSectionOverrun = MAKE_STATUS(kStatusGroup_SBLoader, 0),
    kStatusRomLdrSignature = MAKE_STATUS(kStatusGroup_SBLoader, 1),
    kStatusRomLdrSectionLength = MAKE_STATUS(kStatusGroup_SBLoader, 2),
    kStatusRomLdrUnencryptedOnly = MAKE_STATUS(kStatusGroup_SBLoader, 3),
    kStatusRomLdrEOFReached = MAKE_STATUS(kStatusGroup_SBLoader, 4),
    kStatusRomLdrChecksum = MAKE_STATUS(kStatusGroup_SBLoader, 5),
    kStatusRomLdrCrc32Error = MAKE_STATUS(kStatusGroup_SBLoader, 6),
    kStatusRomLdrUnknownCommand = MAKE_STATUS(kStatusGroup_SBLoader, 7),
    kStatusRomLdrIdNotFound = MAKE_STATUS(kStatusGroup_SBLoader, 8),
    kStatusRomLdrDataUnderrun = MAKE_STATUS(kStatusGroup_SBLoader, 9),
    kStatusRomLdrJumpReturned = MAKE_STATUS(kStatusGroup_SBLoader, 10),
    kStatusRomLdrCallFailed = MAKE_STATUS(kStatusGroup_SBLoader, 11)
};


//! Boot image signature in 32-bit little-endian format "PMTS"
#define BOOT_SIGNATURE  0x504d5453

//! Boot image signature in 32-bit little-endian format "ltgs"
#define BOOT_SIGNATURE2  0x6c746773

//! These define file header flags
#define FFLG_DISPLAY_PROGRESS   0x0001

//! These define section header flags
#define SFLG_SECTION_BOOTABLE   0x0001

//! These define boot command flags
#define CFLG_LAST_TAG   0x01

//! ROM_ERASE_CMD flags
#define ROM_ERASE_ALL_MASK 0x01

//! These define the boot command tags
#define ROM_NOP_CMD     0x00
#define ROM_TAG_CMD     0x01
#define ROM_LOAD_CMD    0x02
#define ROM_FILL_CMD    0x03
#define ROM_JUMP_CMD    0x04
#define ROM_CALL_CMD    0x05
#define ROM_MODE_CMD    0x06
#define ROM_ERASE_CMD   0x07

//! Plugin return codes
#define ROM_BOOT_SECTION_ID     1
#define ROM_BOOT_IMAGE_ID       2

//! Boot command definition
typedef struct _boot_cmd
{
    uint8_t checksum;           //!< 8-bit checksum over command chunk
    uint8_t tag;                //!< command tag (identifier)
    uint16_t flags;             //!< command flags (modifier)
    uint32_t address;           //!< address argument
    uint32_t count;             //!< count argument
    uint32_t data;              //!< data argument
} boot_cmd_t;


//! Definition for boot image file header chunk 1
typedef struct _boot_hdr1
{
    uint32_t hash;              //!< last 32-bits of SHA-1 hash
    uint32_t signature;         //!< must equal "STMP"
    uint8_t major;              //!< major file format version
    uint8_t minor;              //!< minor file format version
    uint16_t fileFlags;         //!< global file flags
    uint32_t fileChunks;        //!< total chunks in the file
} boot_hdr1_t;


//! Definition for boot image file header chunk 2
typedef struct _boot_hdr2
{
    uint32_t bootOffset;        //!< chunk offset to the first boot section
    uint32_t bootSectID;        //!< section ID of the first boot section
    uint16_t keyCount;          //!< number of keys in the key dictionary
    uint16_t keyOffset;         //!< chunk offset to the key dictionary
    uint16_t hdrChunks;         //!< number of chunks in the header
    uint16_t sectCount;         //!< number of sections in the image
} boot_hdr2_t;

// Provides forward reference to the loader context definition.
typedef struct _ldr_Context ldr_Context_t;

//! Function pointer definition for all loader state and action functions.
typedef int (*pLdrFnc_t)(ldr_Context_t *);

//! Jump command function pointer definition.
typedef int (*pJumpFnc_t)(uint32_t);

//! Call command function pointer definition.
typedef int (*pCallFnc_t)(uint32_t, uint32_t *);

//! Loader context definition.
struct _ldr_Context
{
    void * bootParam;           //!< Arbitrary parameter passed to boot driver.
    uint32_t bootMode;          //!< current boot mode
    rom_BootItf_t *pBootItf;    //!< pointer to boot driver interface
    pLdrFnc_t State;            //!< pointer to loader state function
    pLdrFnc_t Action;           //!< pointer to loader action function
    int getCnt;                 //!< number of chunks to get
    int gotCnt;                 //!< number of chunks received
    uint32_t fileChunks;        //!< chunks remaining in file
    uint32_t sectChunks;        //!< chunks remaining in section
    uint16_t fileFlags;         //!< file header flags
    uint16_t keyCount;          //!< number of keys in the key dictionary
    uint32_t objectID;          //!< ID of the current boot section or image
    crc32_data_t crc32;         //!< crc calculated over load command payload
    uint8_t *pSrc;              //!< source buffer address
    chunk_t initVector;         //!< decryption initialization vector
    chunk_t scratchPad;         //!< chunk size scratch pad area
    boot_cmd_t bootCmd;         //!< current boot command
    bool bIsImageAuthenticated; //!< state variable to track if first call cmd executed
    bool skipToEnd;             //!< true if skipping to end of file
};

//! Loader interface definition.
typedef struct _loader_Itf
{
    ldr_Context_t *pCtx;                    //!< pointer to loader context
    int (*StatePump)(void);                 //!< state pump function
    int (*StateInit)(ldr_Context_t *);      //!< initialization state function
    int (*StateMove)(ldr_Context_t *);      //!< move state function
    int (*StateAction)(ldr_Context_t *);    //!< action state function
    int (*DoHeader)(ldr_Context_t *);       //!< header action function
    int (*DoCommand)(ldr_Context_t *);      //!< command action function
} loader_Itf_t;

extern ldr_Context_t g_ldrCtx;
extern const loader_Itf_t rom_LoaderItf;

#if defined(__cplusplus)
extern "C" {
#endif // __cplusplus

status_t ldr_DoLoadInit(void * bootParam);
status_t ldr_DoLoadPump(void);
status_t ldr_FinalizeJumpCmd(void);

#if defined(__cplusplus)
}
#endif // __cplusplus

//! @}

#endif // _loader_h

