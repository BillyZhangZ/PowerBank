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
 
*/
#include "sbloader/itf_define.h"
#include "sbloader/loader.h"
#include "sbloader/chunking_gasket.h"

//! Chunking gasket interface implementation.
const chunk_BootItf_t rom_ChunkBootItf =
{
    // Standard boot driver API
    {
        chunk_init,
        chunk_next,
        chunk_skip,
        chunk_stop
    }
};

rom_ItfTbl_t g_RomItf =
{
    &rom_LoaderItf,                         //!< Boot loader
    (chunk_BootItf_t *)&rom_ChunkBootItf    //!< Chunking gasket boot driver
};
