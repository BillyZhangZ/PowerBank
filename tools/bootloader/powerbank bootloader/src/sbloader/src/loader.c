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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bootloader_common.h"
#include "sbloader/loader.h"
#include "bootloader/context.h"
#include "bootloader/bootloader.h"
#include "sbloader/itf_rom.h"
#include "sbloader/ldr_config.h"
#include "bootloader/shutdown_cleanup.h"

//#define DEBUG_ENTRY

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! Loader utilities.
rom_BootItf_t *ldr_GetBootItf(uint32_t mode);
int ldr_GetHeader(ldr_Context_t *);
int ldr_GetBootCmd(ldr_Context_t *);
int ldr_GetNextTag(ldr_Context_t *);

//! Loader action (ldrCtx.Action) functions for header processing.
int ldr_DoHeader(ldr_Context_t *);
int ldr_DoHeader1(ldr_Context_t *);
int ldr_DoHeader2(ldr_Context_t *);

//! Loader action (ldrCtx.Action) functions for command processing.
int ldr_DoCommand(ldr_Context_t *);
int ldr_DoLoadBytes(ldr_Context_t *);
int ldr_DoLoadChunks(ldr_Context_t *);
int ldr_DoLoadCmd(ldr_Context_t *);
int ldr_DoFillCmd(ldr_Context_t *);
int ldr_DoJumpCmd(ldr_Context_t *);
int ldr_DoCallCmd(ldr_Context_t *);
int ldr_DoTagCmd(ldr_Context_t *);
int ldr_DoEraseCmd(ldr_Context_t *);

//! Loader state (ldrCtx.State) functions
int ldr_StatePump(void);
int ldr_StateInit(ldr_Context_t *);
int ldr_StateMove(ldr_Context_t *);
int ldr_StateAction(ldr_Context_t *);

////////////////////////////////////////////////////////////////////////////////
// Data
////////////////////////////////////////////////////////////////////////////////

//! Global loader context data.
ldr_Context_t g_ldrCtx;

//! Loader interface implementation.
const loader_Itf_t rom_LoaderItf =
{
    &g_ldrCtx,
    ldr_StatePump,
    ldr_StateInit,
    ldr_StateMove,
    ldr_StateAction,
    ldr_DoHeader,
    ldr_DoCommand
};

////////////////////////////////////////////////////////////////////////////////
//! \brief  Get boot driver interface
//!
//! Maps a boot mode the associated boot driver interface.
//!
//! \param[in]  mode    Boot mode.
//!
//! \return Pointer to the boot driver interface for passed in mode.
////////////////////////////////////////////////////////////////////////////////
rom_BootItf_t *ldr_GetBootItf(uint32_t mode)
{
    return (rom_BootItf_t *)g_RomItf.pChunk;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Sets the loader context to get the next boot command
//!
//! Initializes the loader context, so the next state machine sequence will
//! get the next command from the boot image.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \post   *pCtx is setup to get the next boot command.
//!
//! \note   pCtx->pDcp->pkt1 and pkt2 are setup outside this routine
//!
//! \retval SUCCESS
////////////////////////////////////////////////////////////////////////////////
int ldr_GetBootCmd(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_GetBootCmd\r\n");
#endif

    // Go get one chunk, then call the boot command action function
    pCtx->getCnt = 1;
    pCtx->Action = g_pLdr->DoCommand;

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Sets the loader context to get the boot image header
//!
//! Initializes the loader context, so the next state machine sequence will
//! get the first header chunk from the boot image.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->initVector must be zero.
//!
//! \post   *pCtx is setup to get the first image header chunk.
//!
//! \retval kStatus_Success
////////////////////////////////////////////////////////////////////////////////
int ldr_GetHeader(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_GetHeader\r\n");
#endif

    // Assume the header section has at least three chunks
    pCtx->sectChunks = 3;

    // Go get one chunk, then call the header action function
    pCtx->getCnt = 1;
    pCtx->Action = g_pLdr->DoHeader;

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Skip ahead to the next "tag" command
//!
//! Calls the boot driver skip function to skip over the chunks remaining in
//! the current section. It also initializes the loader context to get the
//! next tag command.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->fileChunks indicates remaining chunks in image file.
//! \pre    pCtx->sectChunks indicates remaining chunks in current section.
//!
//! \post   *pCtx is setup to get the next "tag" command.
//!
//! \return Return value from boot driver skip function or the following values.
//! \retval kStatus_Success
//! \retval kStatusRomLdrEOFReached if the end of the image file is reached.
////////////////////////////////////////////////////////////////////////////////
int ldr_GetNextTag(ldr_Context_t *pCtx)
{
    int rc = kStatus_Success;

#ifdef DEBUG_ENTRY
    debug_printf("ldr_GetNextTag\r\n");
#endif

    // If we are not in the last section of the image file, setup to get a
    // "tag" command then skip the remaining chunks in the section. Otherwise,
    // return an error.
    if (pCtx->fileChunks > pCtx->sectChunks)
    {
        // Skip over any remaining chunks in section, making sure to adjust
        // the file chunk count.
        if (pCtx->sectChunks != 0)
        {
            pCtx->fileChunks -= pCtx->sectChunks;
            rc = pCtx->pBootItf->Skip(pCtx->sectChunks);
        }

        // Setup the loader context to get the next command
        ldr_GetBootCmd(pCtx);

        // Assume for now the new section has at least one chunk
        pCtx->sectChunks = 1;
    }
    else
    {
        // No where to skip to, so return an error
        rc = kStatusRomLdrEOFReached;
    }
    return rc;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader action function processes the first header chunk
//!
//! Processes the first header chunk of the boot image file, which
//! is ignored.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->pSrc points at the current header chunk.
//!
//! \post   pCtx->Action is set to process the next header chunk.
//!
//! \retval SUCCESS
////////////////////////////////////////////////////////////////////////////////
int ldr_DoHeader(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoHeader\r\n");
#endif

    // Ignore the first chunk since encryption not supported.
    // This stage could be optimized out.

    // Setup to process the next header chunk
    pCtx->Action = ldr_DoHeader1;
    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader action function processes header chunk 1
//!
//! Processes the second header chunk of the boot image file, which contains
//! the file signature, version, flags and chunk count.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->pSrc points at the current header chunk.
//!
//! \post   pCtx->Action is set to process the next header chunk.
//! \post   pCtx->fileChunks and fileFlags are set to the header values.
//!
//! \retval kStatus_Success
//! \retval kStatusRomLdrSignature if the signature or version are incorrect.
////////////////////////////////////////////////////////////////////////////////
int ldr_DoHeader1(ldr_Context_t *pCtx)
{
    boot_hdr1_t *pHdr1 = (boot_hdr1_t *)pCtx->pSrc;

#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoHeader1\r\n");
#endif

    // Copy the file flags and chunk count
    pCtx->fileFlags = pHdr1->fileFlags;
    pCtx->fileChunks = pHdr1->fileChunks - 2;

    //// Display progress on the debug uart if enabled
    //if (pHdr1->fileFlags & FFLG_DISPLAY_PROGRESS)
    //    uart_write("H");

    //debug_printf("sig = 0x%x, major = 0x%x\r\n", pHdr1->signature, pHdr1->major);

    // Check the file signature and version
    if ((pHdr1->signature != BOOT_SIGNATURE) || (pHdr1->major > SB_FILE_MAJOR_VERSION))
    {
        return kStatusRomLdrSignature;
    }
    else
    {
        // Setup to process the next header chunk
        pCtx->Action = ldr_DoHeader2;
        return kStatus_Success;
    }
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader action function processes header chunk 2
//!
//! Processes the third header chunk of the boot image file, which contains
//! parameters for the first boot section and the key dictionary.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->pSrc points at the current header chunk.
//!
//! \post   pCtx->Action is set to process the next header chunk.
//! \post   pCtx->sectChunks, objectID and keyCount are set to the header values.
//!
//! \retval kStatus_Success
//! \retval kStatusRomLdrSectionLength if the bootOffset is out of range.
//! \retval kStatusRomLdrUnencryptedOnly if the unencrypted image is disabled.
////////////////////////////////////////////////////////////////////////////////
int ldr_DoHeader2(ldr_Context_t *pCtx)
{
    boot_hdr2_t *pHdr2 = (boot_hdr2_t *)pCtx->pSrc;

#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoHeader2\r\n");
#endif

    // Save first boot section parameters
    pCtx->objectID = pHdr2->bootSectID;
    pCtx->sectChunks = pHdr2->bootOffset - 3;

    // Sanity check the section chunk count
    if (pCtx->sectChunks >= pCtx->fileChunks)
        return kStatusRomLdrSectionLength;

    // Check whether the image is encrypted
    if (pHdr2->keyCount != 0)
    {
        debug_printf("Error: image is encrypted\r\n");
        return kStatusRomLdrUnencryptedOnly;
    }
    else
    {
        // Skip the rest of the header
        return ldr_GetNextTag(pCtx);
    }
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader action function loads the trailing "ragged edge bytes"
//!
//! Implements the second of two action functions for the boot "load" command.
//! The last move sequence for the load command moves the final
//! payload chunk. This function checks the payload
//! CRC, then copies the final load bytes to the load
//! destination.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//! \pre    pCtx->scratchPad contains the final decrypted payload chunk.
//! \pre    pCtx->crc32 holds the CRC calculated over the entire payload.
//!
//! \post   *pCtx is setup to get the next boot command.
//!
//! \retval Return value from ldr_GetBootCmd()
//! \retval ERROR_ROM_LDR_PAYLOAD_CRC if the data payload CRC was incorrect
////////////////////////////////////////////////////////////////////////////////
int ldr_DoLoadBytes(ldr_Context_t *pCtx)
{
    uint32_t crc32Result;

#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoLoadBytes\r\n");
#endif

    //DBG_PRINTF(".LBy.%d", pCtx->bootCmd.count);

    // See if the calculated CRC matches the expected result
    //if (pCtx->crc32 == pCtx->bootCmd.data)
    {
        // Copy the trailing edge payload bytes to the load destination.
        status_t status = g_bootloaderContext.memoryInterface->write(pCtx->bootCmd.address, pCtx->bootCmd.count, pCtx->pSrc);
        if (status != kStatus_Success)
        {
            return status;
        }

        // update the crc running value then finalize
        crc32_update(&pCtx->crc32, pCtx->pSrc, pCtx->gotCnt * sizeof(chunk_t));
        crc32_finalize(&pCtx->crc32, &crc32Result);

        if (crc32Result != pCtx->bootCmd.data)
        {
            return kStatusRomLdrCrc32Error;
        }

        // Setup to get the next boot command.
        return ldr_GetBootCmd(pCtx);
    }
    //else
    //{
    //    // CRC did not match, return an error
    //    DBG_PRINTF("!=%x", pCtx->crc32);
    //    DBG_DUMP_OCRAM(0);
    //    return ERROR_ROM_LDR_PAYLOAD_CRC;
    //}
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader action function loads data chunks
//!
//! Implements the first of two action functions for the boot "load" command.
//! Loads one or more complete "chunk size" blocks of data. It
//! does this by setting the loader context so the next state machine sequence
//! will move the data.
//!
//! If the amount of data remaining to load is one chunk or less, it sets up
//! a state sequence to move the next chunk. The
//! action function is changed to \ref ldr_DoLoadBytes, which will check the
//! payload CRC and memcopy to the correct location.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//! \pre    pCtx->gotCnt has the number of chunks transfered by the
//!         last move sequence. Must be 0 when starting a new load command.
//!
//! \post   pCtx->bootCmd is adjusted to reflect the previous move.
//! \post   *pCtx is setup to move the load command payload.
//!
//! \retval kStatus_Success
////////////////////////////////////////////////////////////////////////////////
int ldr_DoLoadChunks(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoLoadChunks\r\n");
#endif

    if (pCtx->gotCnt > 0)
    {
        // update the crc running value
        crc32_update(&pCtx->crc32, pCtx->pSrc, pCtx->gotCnt * sizeof(chunk_t));
        status_t status = g_bootloaderContext.memoryInterface->write(pCtx->bootCmd.address, pCtx->gotCnt * sizeof(chunk_t), pCtx->pSrc);
        if (status != kStatus_Success)
        {
            return status;
        }
    }

    // Adjust the boot command parameters to reflect the last move
    pCtx->bootCmd.address += pCtx->gotCnt * sizeof(chunk_t);
    pCtx->bootCmd.count -= pCtx->gotCnt * sizeof(chunk_t);

    //DBG_PRINTF(".LCh.%d", pCtx->bootCmd.count);

    // Set the destination address, chunk count and Action function based on
    // the remaining number of bytes to load.
    if (pCtx->bootCmd.count > sizeof(chunk_t))
    {
        pCtx->getCnt = (pCtx->bootCmd.count - 1) / sizeof(chunk_t);
        pCtx->Action = ldr_DoLoadChunks;
    }
    else
    {
        // This is the last data chunk.
        // Change the action function to LoadBytes.
        pCtx->getCnt = 1;
        pCtx->Action = ldr_DoLoadBytes;
    }
    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader "load" command handler
//!
//! Implements the first command handler called for the boot "load" command.
//! The load action is split into two functions:
//! - \ref ldr_DoLoadChunks, loads chunks using the DCP
//! - \ref ldr_DoLoadBytes, loads the trailing "ragged edge bytes" using memcopy
//! This function initializes the context before calling \ref ldr_DoLoadChunks.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
////////////////////////////////////////////////////////////////////////////////
int ldr_DoLoadCmd(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoLoadCmd\r\n");
#endif

    // initialize crc32
    crc32_init(&pCtx->crc32);

    // ldr_DoLoadChunks() expects pCtx->gotCnt to reflect the number of chunks
    // moved prior to calling it. So when starting a new load command
    // gotCnt must be set to 0.
    pCtx->gotCnt = 0;
    return ldr_DoLoadChunks(pCtx);
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader "fill" command handler
//!
//! Implements the action function for the boot "fill" command.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//!
//! \post   *pCtx is left setup to get the next boot command.
//!
//! \retval kStatus_Success
////////////////////////////////////////////////////////////////////////////////
int ldr_DoFillCmd(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoFillCmd\r\n");
#endif

    status_t status = g_bootloaderContext.memoryInterface->fill(pCtx->bootCmd.address, pCtx->bootCmd.count, pCtx->bootCmd.data);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Context is already setup to get the next boot command
    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader "jump" command handler
//!
//! Implements the action function for the boot "jump" command.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//!
//! \retval kStatusRomLdrJumpReturned if the plugin returns
////////////////////////////////////////////////////////////////////////////////
int ldr_DoJumpCmd(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoJumpCmd\r\n");
#endif

    // Stop the current boot driver
    pCtx->pBootItf->Stop();

    // Actual jump is implemented in ldr_FinalizeJumpCmd().
    return kStatus_AbortDataPhase;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader "call" command handler
//!
//! Implements the action function for the boot "call" command.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//!
//! \post   *pCtx is setup to get the next boot command.
//! \post   pCtx->objectID is set to the next boot section or image ID.
//!
//! \return Return code from plugin or ldr_GetNextTag.
////////////////////////////////////////////////////////////////////////////////
int ldr_DoCallCmd(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoCallCmd\r\n");
#endif

#if defined(BOOTLOADER_HOST)
    return kStatus_Success;
#else
    pCallFnc_t entry_fun = (pCallFnc_t)pCtx->bootCmd.address;
    status_t rc = kStatusRomLdrCallFailed;

    // todo: need common impl with bootloader call cmd

    // Flush I$ to purge any previous boot calls
    //cpu_invalidate_ICache();
    //cpu_invalidate_ITLB();

    // Call the plugin entry point with the specified parameter. The plugin
    // can start a new section or image by returning the appropriate code and
    // updating the object id pointed to by the second parameter.
    if (entry_fun != NULL)
    {
       rc = entry_fun(pCtx->bootCmd.data, &pCtx->objectID);
    }

    if (rc == ROM_BOOT_SECTION_ID)
    {
        // The plugin returned a section ID, skip ahead to the next section
        return ldr_GetNextTag(pCtx);
    }
    else if (rc == ROM_BOOT_IMAGE_ID)
    {
        // The plugin returned an image ID, restart the loader state machine
        pCtx->State = g_pLdr->StateInit;
        return kStatus_Success;
    }
    else
    {
        // Otherwise, just pass on the plugin return code. Context is already
        // setup to get the next boot command.
        return rc;
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader "tag" command handler
//!
//! Implements the action function for the boot "tag" command.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//!
//! \post   *pCtx is setup to get the next boot command.
//! \post   pCtx->sectChunks is updated to the next section.
//!
//! \retval kStatusRomLdrSectionLength if new section count is out of range.
//! \retval kStatusRomLdrIdNotFound if no match and no more sections.
//! \retval Return code from ldr_GetNextTag.
////////////////////////////////////////////////////////////////////////////////
int ldr_DoTagCmd(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoTagCmd\r\n");
#endif

    // Get the section chunk count from the boot command
    pCtx->sectChunks = pCtx->bootCmd.count;

    // Sanity check the new section count
    if (pCtx->sectChunks > pCtx->fileChunks)
        return kStatusRomLdrSectionLength;

    if((pCtx->bootCmd.data & SFLG_SECTION_BOOTABLE)
        && (pCtx->bootCmd.address == pCtx->objectID))
    {
        // This section is bootable and matches the ID we are looking for, so
        // just continue getting commands from this point
        return kStatus_Success;
    }
    else if (pCtx->bootCmd.flags & CFLG_LAST_TAG)
    {
        // This isn't the right boot section and it's the last one, so return
        // an error
        return kStatusRomLdrIdNotFound;
    }
    else
    {
        // This isn't the boot section we are looking for and there are more, so
        // skip to the next one
        return ldr_GetNextTag(pCtx);
    }
}


////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader "erase" command handler
//!
//! Implements the action function for the boot "erase" command.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//!
//! \post   *pCtx is setup to get the next boot command.
//! \post   pCtx->sectChunks is updated to the next section.
//!
//! \retval kStatus_Success
////////////////////////////////////////////////////////////////////////////////
int ldr_DoEraseCmd(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoEraseCmd\r\n");
#endif

    if (pCtx->bootCmd.flags & ROM_ERASE_ALL_MASK)
    {
#ifdef BOOTLOADER_HOST
        host_flash_erase_all();
#else
        flash_mem_erase_all();
#endif
    }
    else
    {
#ifdef BOOTLOADER_HOST
        host_flash_erase_region(pCtx->bootCmd.address, pCtx->bootCmd.count);
#else
        flash_mem_erase(pCtx->bootCmd.address, pCtx->bootCmd.count);
#endif
    }

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader boot command parsing function
//!
//! Implements the action function for parsing a boot command. Performs generic
//! integrity tests and flag handling, then calls the appropriate command
//! handler function based on the command tag.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootCmd contains the boot command.
//!
//! \retval kStatusRomLdrChecksum if command checksum failed.
//! \retval kStatusRomLdrUnknownCommand if command tag is not recognized.
//! \retval Return code from the boot command handler function.
////////////////////////////////////////////////////////////////////////////////
int ldr_DoCommand(ldr_Context_t *pCtx)
{
//    static const char tagSymbol[] = "N\0T\0L\0F\0J\0C\0M\0";
    boot_cmd_t *pCmd = &pCtx->bootCmd;
    int i, sum = 0x5a;

#ifdef DEBUG_ENTRY
    debug_printf("ldr_DoCommand\r\n");
#endif

    // Save the boot commmand.
    pCtx->bootCmd = *((boot_cmd_t *)pCtx->pSrc);

    // Compute and test the boot command checksum
    for (i = 1; i < sizeof(boot_cmd_t); i++)
    {
        sum += ((uint8_t *)pCmd)[i];
    }
    if (((uint8_t *)pCmd)[0] != (sum & 0xFF))
    {
        debug_printf("Error: invalid boot command checksum\r\n");
        return kStatusRomLdrChecksum;
    }

    // Display command for debugging
    //DBG_PRINTF(".Cmd.%s", &tagSymbol[2 * pCmd->tag]);
    //DBG_DUMPCHUNK(pCmd);

    // If enabled, display progress on the debug uart
    //if (pCtx->fileFlags & FFLG_DISPLAY_PROGRESS)
    //    uart_write(&tagSymbol[2 * pCmd->tag]);

    // Switch to the appropriate command handler function
    switch (pCmd->tag)
    {
    case ROM_NOP_CMD:    return kStatus_Success;
    case ROM_TAG_CMD:    return ldr_DoTagCmd(pCtx);
    case ROM_LOAD_CMD:   return ldr_DoLoadCmd(pCtx);
    case ROM_FILL_CMD:   return ldr_DoFillCmd(pCtx);
    case ROM_JUMP_CMD:   return ldr_DoJumpCmd(pCtx);
    case ROM_CALL_CMD:   return ldr_DoCallCmd(pCtx);
    case ROM_ERASE_CMD:  return ldr_DoEraseCmd(pCtx);
    case ROM_MODE_CMD:   return kStatus_Success;        // ignored for Kinetis
    }
    return kStatusRomLdrUnknownCommand;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader Init State
//!
//! Implements the loader initialization state handler. This state initializes
//! the loader context, then calls the Init() function of the boot driver.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->bootMode contains the desired boot mode.
//!
//! \post   *pCtx is initialized.
//!
//! \return Returns the result from the boot driver Init() function.
////////////////////////////////////////////////////////////////////////////////
int ldr_StateInit(ldr_Context_t *pCtx)
{
    rom_BootInit_t init = {NULL, 0, 0, 0};

#ifdef DEBUG_ENTRY
    debug_printf("ldr_StateInit\r\n");
#endif

    // Clear the loader context (remember to save the boot mode)
    init.mode = pCtx->bootMode;
    init.id = pCtx->objectID;
    init.bootParam = pCtx->bootParam;
    memset(pCtx, 0, sizeof(ldr_Context_t));

    // Now initialize the context
    pCtx->bootParam = init.bootParam;
    pCtx->bootMode = init.mode;
    pCtx->pBootItf = ldr_GetBootItf(init.mode);
    pCtx->State = g_pLdr->StateMove;
    pCtx->skipToEnd = false;

    // Setup context to get the image header
    ldr_GetHeader(pCtx);

    // Initialize the boot driver
    return pCtx->pBootItf->Init(&init);
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader Move State
//!
//! Implements the loader move state handler. This state calls the boot driver
//! Next() function.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->getCnt contains the number of chunks to request.
//!
//! \post   pCtx->State is set to ldr_StateAction().
//!
//! \retval kStatusRomLdrSectionOverrun if section count is exceeded.
//! \retval Result from the boot driver Next() function.
////////////////////////////////////////////////////////////////////////////////
int ldr_StateMove(ldr_Context_t *pCtx)
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_StateMove\r\n");
#endif

    int got = pCtx->getCnt;
    bool underrun = false;

    // When we hit the end of a section, skip to the end of the file
    if (pCtx->skipToEnd || ((uint32_t)got > pCtx->sectChunks))
    {
        if (pCtx->fileChunks == 0)
        {
            return kStatusRomLdrSectionOverrun;
        }
        pCtx->skipToEnd = true;
        got = pCtx->fileChunks;
        pCtx->pSrc = (uint8_t *)(pCtx->pBootItf->Next(&got, &underrun));
        if (got > 0)
        {
            pCtx->fileChunks -= got;
            if (pCtx->fileChunks == 0)
            {
                return kStatusRomLdrSectionOverrun;
            }
        }
        pCtx->getCnt = 1;
        return underrun ? kStatusRomLdrDataUnderrun : kStatus_Success;
    }

    // Request the next boot image chunk(s) from the driver
    pCtx->pSrc = (uint8_t *)(pCtx->pBootItf->Next(&got, &underrun));

    // If the driver returned data, adjust counts
    if (got > 0)
    {
        pCtx->sectChunks -= got;
        pCtx->fileChunks -= got;
        pCtx->gotCnt = got;

        // go to the action state next
        pCtx->State = g_pLdr->StateAction;
    }

    // Return underrun condition if loader needs data.
    return underrun ? kStatusRomLdrDataUnderrun : kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader Action State
//!
//! Implements the loader action state handler.
//!
//! \param[in]  pCtx    Pointer to a loader context structure.
//!
//! \pre    pCtx->Action will be called when the DCP operation completes.
//!
//! \post   pCtx->State is set to ldr_StateMove().
//!
//! \retval Return code of the current Action function.
////////////////////////////////////////////////////////////////////////////////
int ldr_StateAction(ldr_Context_t *pCtx)
{
    pCtx->State = g_pLdr->StateMove;
    return pCtx->Action(pCtx);
}

////////////////////////////////////////////////////////////////////////////////
//! \brief  Loader State Pump
//!
//! Calls the current loader state handler. The purpose of this function is
//! to provide an entry point that a rom plugin / patch could use to hook
//! the loader state machine.
//!
//! \return Returns the result from state handler function.
////////////////////////////////////////////////////////////////////////////////
int ldr_StatePump()
{
    return g_ldrCtx.State(&g_ldrCtx);
}

////////////////////////////////////////////////////////////////////////////////
//! @brief Initialize the loader state machine.
//!
//! Sets the initial state to ldr_StateInit().
//!
//! @param bootParam Arbitrary value passed to the boot driver.
////////////////////////////////////////////////////////////////////////////////
status_t ldr_DoLoadInit(void * bootParam)
{
    // Init the loader context
    g_ldrCtx.bootParam = bootParam;
    g_ldrCtx.bootMode = 0;
    g_ldrCtx.objectID = 0;
    g_ldrCtx.State = g_pLdr->StateInit;

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
//! @brief Pump the loader state machine.
////////////////////////////////////////////////////////////////////////////////
status_t ldr_DoLoadPump()
{
    // Pump the state machine.
    return (status_t)g_pLdr->StatePump();
}

////////////////////////////////////////////////////////////////////////////////
//! @brief Implement jump if last command was ROM_JUMP_CMD.
////////////////////////////////////////////////////////////////////////////////
status_t ldr_FinalizeJumpCmd()
{
#ifdef DEBUG_ENTRY
    debug_printf("ldr_FinalizeJumpCmd\r\n");
#endif

#if !defined(BOOTLOADER_HOST)
    if (g_ldrCtx.bootCmd.tag == ROM_JUMP_CMD)
    {
        pJumpFnc_t entry_fun = (pJumpFnc_t)g_ldrCtx.bootCmd.address;

        // Jump to the entry point with the specified parameter
        if (entry_fun != NULL)
        {
            // Clean up prior to calling user code.
            shutdown_cleanup(true);

            entry_fun(g_ldrCtx.bootCmd.data);
        }

        // We should never get here, so return an error if we do
        return kStatusRomLdrJumpReturned;
    }
#endif // BOOOTLOADER_HOST

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

