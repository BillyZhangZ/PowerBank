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

#include "bootloader/bootloader.h"
#include "flash/flash.h"

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

//! @brief Function table for flash driver.
const flash_driver_interface_t g_flashDriverInterface = {
        .flash_init = flash_init,
        .flash_erase_all = flash_erase_all,
        .flash_erase_all_unsecure = flash_erase_all_unsecure,
        .flash_erase = flash_erase,
        .flash_program = flash_program,
        .flash_get_security_state = flash_get_security_state,
        .flash_security_bypass = flash_security_bypass,
        .flash_verify_erase_all = flash_verify_erase_all,
        .flash_verify_erase = flash_verify_erase,
        .flash_verify_program = flash_verify_program,
        .flash_get_property = flash_get_property
    };

//! @brief Copyright string for the bootloader.
const char bootloaderCopyright[] = "Copyright (c) 2013 Freescale Semiconductor, Inc. All rights reserved.";

//! @brief Static API tree.
const bootloader_tree_t g_bootloaderTree = {
        .runBootloader = bootloader_user_entry,
        .version = {
                .name = kBootloader_Version_Name,
                .major = kBootloader_Version_Major,
                .minor = kBootloader_Version_Minor,
                .bugfix = kBootloader_Version_Bugfix
            },
        .copyright = bootloaderCopyright,
        .runtimeContext = &g_bootloaderContext,
        .flashDriver = &g_flashDriverInterface
    };

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

