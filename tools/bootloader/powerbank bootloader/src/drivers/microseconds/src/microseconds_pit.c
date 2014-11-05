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
/*
 * @file microseconds.c
 * @brief Microseconds timer driver source file
 *
 * Notes: The driver configure PIT as lifetime timer
 */
#include "fsl_platform_common.h"
#include "microseconds/microseconds.h"
#include <stdarg.h>

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////
static uint32_t s_pitClockMhz;

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

//! @brief Initialize timer facilities.
//!
//! It is initialize the timer to lifetime timer by chained channel 0
//! and channel 1 together, and set b0th channels to maximum counting period
void microseconds_init(void)
{
    uint32_t busClockDivider;

    //PIT clock gate control ON
    SIM->SCGC6 |= SIM_SCGC6_PIT_MASK;

    //Turn on PIT: MDIS = 0, FRZ = 0
    PIT->MCR = 0x00;

    //Set up timer 1 to max value
    PIT->CHANNEL[1].LDVAL = 0xFFFFFFFF;            //setup timer 1 for maximum counting period
    PIT->CHANNEL[1].TCTRL = 0;                     //Disable timer 1 interrupts
    PIT->CHANNEL[1].TFLG = 1;                      //clear the timer 1 flag
    PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_CHN_MASK;   //chain timer 1 to timer 0
    PIT->CHANNEL[1].TCTRL |= PIT_TCTRL_TEN_MASK;   //start timer 1

    //Set up timer 0 to max value
    PIT->CHANNEL[0].LDVAL = 0xFFFFFFFF;            //setup timer 0 for maximum counting period
    PIT->CHANNEL[0].TFLG = 1;                      //clear the timer 0 flag
    PIT->CHANNEL[0].TCTRL = PIT_TCTRL_TEN_MASK;   //start timer 0

    busClockDivider = ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV4_MASK) >> SIM_CLKDIV1_OUTDIV4_SHIFT) + 1;
    s_pitClockMhz = ((SystemCoreClock / busClockDivider) / 1000000);
}

//! @brief Shutdown the microsecond timer
void microseconds_shutdown(void)
{
    //Turn off PIT: MDIS = 1, FRZ = 0
    PIT->MCR |= PIT_MCR_MDIS_MASK;
}

//! @brief Read back running tick count
uint64_t microseconds_get_ticks(void)
{
    uint32_t valueH = PIT->LTMR64H;
    uint32_t valueL = PIT->LTMR64L;

    // Invert to turn into an up counter
    return ~(((uint64_t)valueH << 32) + (uint64_t)(valueL));
}

//! @brief Returns the conversion of ticks to actual microseconds
//!        This is used to seperate any calculations from getting a timer
//         value for speed critical scenarios
uint32_t microseconds_convert_to_microseconds(uint32_t ticks)
{
    // return the total ticks divided by the number of Mhz the system clock is at to give microseconds
    return (ticks / s_pitClockMhz);
}

//! @brief Returns the conversion of microseconds to ticks
uint32_t microseconds_convert_to_ticks(uint32_t microseconds)
{
    return (microseconds * s_pitClockMhz);
}

//! @brief Delay specified time
//!
//! @param us Delay time in microseconds unit
void microseconds_delay(uint32_t us)
{
    uint32_t currentTicks = microseconds_get_ticks();

    //! The clock value in Mhz = ticks/microsecond
    uint64_t ticksNeeded = (us * s_pitClockMhz) + currentTicks;

    while(microseconds_get_ticks() > ticksNeeded)
    {
        ;
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////

