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

#include <assert.h>
#include "bootloader/context.h"
#include "bootloader/peripheral.h"
#include "bootloader/shutdown_cleanup.h"
#include "bootloader_common.h"
#include "microseconds/microseconds.h"
#include "bootloader/bootloader.h"
#include "flash/flash.h"
#include "microseconds/microseconds.h"
#include "property/property.h"
#include "utilities/vector_table_info.h"
#include "utilities/fsl_rtos_abstraction.h"

//! @addtogroup bl_core
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if DEBUG
static const char * get_peripheral_name(uint32_t peripheralTypeMask);
#endif

static void get_user_application_entry(uint32_t *appEntry, uint32_t *appStack);
static void jump_to_application(uint32_t applicationAddress, uint32_t stackPointer);
static peripheral_descriptor_t const * get_active_peripheral(void);
static void bootloader_init(void);
static void bootloader_run(void);

void main(void);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

#if DEBUG
static const char * const kPeripheralNames[] =
    {
        "UART", // kPeripheralType_UART
        "I2C", // kPeripheralType_I2CSlave
        "SPI", // kPeripheralType_SPISlave
        "CAN", // kPeripheralType_CAN
        "HID", // kPeripheralType_USB_HID
        "CDC", // kPeripheralType_USB_CDC
        "DFU", // kPeripheralType_USB_DFU
        "MSD" // kPeripheralType_USB_MSC
    };
#endif // DEBUG

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

#if DEBUG
//! @brief Returns the name of a peripheral given its type mask.
const char * get_peripheral_name(uint32_t peripheralTypeMask)
{
    unsigned i;
    for (i=0; i < ARRAY_SIZE(kPeripheralNames); ++i)
    {
        if (peripheralTypeMask & (1 << i))
        {
            return kPeripheralNames[i];
        }
    }

    return "Unknown peripheral";
}
#endif // DEBUG

//! @brief Returns the user application address and stack pointer.
//!
//! For flash-resident and rom-resident target, gets the user application address
//! and stack pointer from the APP_VECTOR_TABLE.
//! Ram-resident version does not support jumping to application address.
static void get_user_application_entry(uint32_t *appEntry, uint32_t *appStack)
{
    assert(appEntry);
    assert(appStack);

#if BL_TARGET_RAM
    *appEntry = NULL;
    *appStack = NULL;
#else
    *appEntry = APP_VECTOR_TABLE[kInitialPC];
    *appStack = APP_VECTOR_TABLE[kInitialSP];
#endif
}

//! @brief Exits bootloader and jumps to the user application.
static void jump_to_application(uint32_t applicationAddress, uint32_t stackPointer)
{
    shutdown_cleanup(true);

    // Create the function call to the user application.
    // Static variables are needed since changed the stack pointer out from under the compiler
    // we need to ensure the values we are using are not stored on the previous stack
    static uint32_t s_stackPointer = 0;
    s_stackPointer = stackPointer;
    static void (*farewellBootloader)(void) = 0;
    farewellBootloader = (void (*)(void))applicationAddress;

    // Set the VTOR to the application vector table address.
    SCB->VTOR = (uint32_t)APP_VECTOR_TABLE;

    // Set stack pointers to the application stack pointer.
    __set_MSP(s_stackPointer);
    __set_PSP(s_stackPointer);

    // Jump to the application.
    farewellBootloader();
}

//! A given jump address is considered valid if:
//! - Not 0x00000000
//! - Not 0xffffffff
//! - Not the reset handler entry point for the bootloader
//! - Is in flash or is in RAM
bool is_valid_application_location(uint32_t applicationAddress)
{
    property_store_t * store = g_bootloaderContext.propertyInterface->store;

    // Verify that the jumpLocation is non zero and then either within flash or RAM, both calculations are:
    // (jumpLocation >= startAddress) && (jumpLocation < (startAddress + size))
    return (  applicationAddress &&                                     // address is not null AND
             (applicationAddress != 0xffffffff) &&                      // address is not blank Flash (0xff) AND
 //! @todo - change the following line to ensure address is not in the range of bootloader (see KBL-283)
             (applicationAddress != (uint32_t)&Reset_Handler) &&        // address is not bootloader AND
            ((applicationAddress >= store->flashStartAddress) &&        // address is in Flash OR
             (applicationAddress < (store->flashStartAddress +
                                    store->flashSizeInBytes))) ||
            ((applicationAddress >= store->ramStartAddress) &&          // address is in Ram
             (applicationAddress < (store->ramStartAddress +
                                    store->ramSizeInBytes))));
}

//! @brief Determines the active peripheral.
//!
//! This function has several stages:
//! - Init enabled peripherals.
//! - Compute timeout.
//! - Wait for peripheral activity with timeout.
//! - Shutdown inactive peripherals.
//!
//! If peripheral detection times out, then this function will call jump_to_application() to
//! directly enter the user application.
//!
//! The timeout value comes from the BCA if set, or the #BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT
//! configuration macro. If the boot pin is asserted, or if there is not a valid user application
//! in flash, then the timeout is disabled and peripheral detection will continue infinitely.
static peripheral_descriptor_t const * get_active_peripheral(void)
{
    peripheral_descriptor_t const * peripheral;
    peripheral_descriptor_t const * activePeripheral = NULL;
    bootloader_configuration_data_t * configurationData = &g_bootloaderContext.propertyInterface->store->configurationData;

    // Bring up all the peripherals
    for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
    {
        // Check that the peripheral is enabled in the user configuration data
        if (configurationData->enabledPeripherals & peripheral->typeMask)
        {
            assert(peripheral->controlInterface->init);

            debug_printf("Initing %s\r\n", get_peripheral_name(peripheral->typeMask));
            peripheral->controlInterface->init(peripheral, peripheral->packetInterface->byteReceivedCallback);
        }
    }

    uint64_t lastTicks;          // Value of our last recorded ticks second marker
    uint32_t timeoutTicks = 0;   // The number of ticks we will wait for timeout, 0 means no timeout
    const uint32_t ticksPerMillisecond = microseconds_convert_to_ticks(1000);

    // Get the user application entry point and stack pointer.
    uint32_t applicationAddress, stackPointer;
    get_user_application_entry(&applicationAddress, &stackPointer);

    // If the boot to rom option is not set AND there is a valid jump location determine the timeout value
    if (!is_boot_pin_asserted() && is_valid_application_location(applicationAddress))
    {
        // Calculate how many ticks we need to wait based on the bootloader config. Check to see if
        // there is a valid configuration data value for the timeout. If there's not, use the
        // default timeout value.
        uint32_t milliseconds;
        if (configurationData->peripheralDetectionTimeoutMs != 0xFFFF)
        {
            milliseconds = configurationData->peripheralDetectionTimeoutMs;
        }
        else
        {
            milliseconds = BL_DEFAULT_PERIPHERAL_DETECT_TIMEOUT;
        }
        timeoutTicks = milliseconds * ticksPerMillisecond;

        // save how many ticks we're currently at before the detection loop starts
        lastTicks = microseconds_get_ticks();
    }

    // Wait for a peripheral to become active
    while (activePeripheral == NULL)
    {
       
        // If timeout is enabled, check to see if we've exceeded it.
        if (timeoutTicks)
        {
            // Note that we assume that the tick counter won't overflow and wrap back to 0.
            // The timeout value is only up to 65536 milliseconds, and the tick count starts
            // at zero when when inited the microseconds driver just a few moments ago.
            uint64_t elapsedTicks = microseconds_get_ticks() - lastTicks;

            // Check if the elapsed time is longer than the timeout.
            if (elapsedTicks >= timeoutTicks)
            {
                jump_to_application(applicationAddress, stackPointer);
            }
        }

        // Traverse through all the peripherals
        for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
        {
            assert(peripheral->controlInterface->pollForActivity);

            if (peripheral->controlInterface->pollForActivity(peripheral))
            {
                debug_printf("%s is active\r\n", get_peripheral_name(peripheral->typeMask));

                activePeripheral = peripheral;
                break;
            }
        }
    }

    // Shut down all non active peripherals
    for (peripheral = g_peripherals; peripheral->typeMask != 0; ++peripheral)
    {
        if (activePeripheral != peripheral)
        {
            debug_printf("Shutting down %s\r\n", get_peripheral_name(peripheral->typeMask));

            assert(peripheral->controlInterface->shutdown);
            peripheral->controlInterface->shutdown(peripheral);
        }
    }

    return activePeripheral;
}

//! @brief Initialize the bootloader and peripherals.
//!
//! This function initializes hardware and clocks, loads user configuration data, and initialzes
//! a number of drivers. It then enters the active peripheral detection phase by calling
//! get_active_peripheral(). Once the peripheral is detected, the packet and comand interfaces
//! are initialized.
//!
//! Note that this routine may not return if peripheral detection times out and the bootloader
//! jumps directly to the user application in flash.
static void bootloader_init(void)
{
    // Init the global irq lock
    lock_init();

    // Init pinmux and other hardware setup.
    init_hardware();

    // Load the user configuration data so that we can configure the clocks
    g_bootloaderContext.propertyInterface->load_user_config();

    // Configure clocks.
    configure_clocks();

    // Start the lifetime counter
    microseconds_init();

    // Init flash driver.
    flash_init(&g_bootloaderContext.flashState);

    // Init address range of flash array, SRAM_L and SRAM U.
    g_bootloaderContext.memoryInterface->init();

    // Fully init the property store.
    g_bootloaderContext.propertyInterface->init();

#if DEBUG
    // Initialize any debug functionality
    debug_init();
#endif

    // Message so python instantiated debugger can tell the
    // bootloader application is running on the target.
    debug_printf("\r\n\r\nRunning bootloader...\r\n");

#if DEBUG && !DEBUG_PRINT_DISABLE
    bootloader_version_t version = g_bootloaderContext.propertyInterface->store->currentVersion;
    debug_printf("Bootloader version %c%d.%d.%d\r\n",
        version.name, version.major, version.minor, version.bugfix);
#endif

    // Wait for a peripheral to become active.
    g_bootloaderContext.activePeripheral = get_active_peripheral();
    assert(g_bootloaderContext.activePeripheral);

    // Validate required active peripheral interfaces.
    assert(g_bootloaderContext.activePeripheral->controlInterface);

    // Init the active peripheral.
    if (g_bootloaderContext.activePeripheral->byteInterface
        && g_bootloaderContext.activePeripheral->byteInterface->init)
    {
        g_bootloaderContext.activePeripheral->byteInterface->init(g_bootloaderContext.activePeripheral);
    }
    if (g_bootloaderContext.activePeripheral->packetInterface
        && g_bootloaderContext.activePeripheral->packetInterface->init)
    {
        g_bootloaderContext.activePeripheral->packetInterface->init(g_bootloaderContext.activePeripheral);
    }

    // Initialize the command processor component.
    g_bootloaderContext.commandInterface->init();
}

//! @brief Bootloader outer loop.
//!
//! Infinitely calls the command interface and active peripheral control interface pump routines.
static void bootloader_run(void)
{
    const peripheral_descriptor_t * activePeripheral = g_bootloaderContext.activePeripheral;

    assert(g_bootloaderContext.commandInterface->pump);

    // Read and execute commands.
    while (1)
    {
        g_bootloaderContext.commandInterface->pump();
      
        // Pump the active peripheral.
        if (activePeripheral->controlInterface->pump)
        {
            activePeripheral->controlInterface->pump(activePeripheral);
        }
      
    }
}

//! @brief Entry point for the bootloader.
void main(void)
{
  
#if 1
    SIM_SCGC5 = SIM_SCGC5_PORTC_MASK;
    //led1
    GPIOC_PDDR |= GPIO_PDDR_PDD(1<<2);
    GPIOC_PDOR |= GPIO_PDOR_PDO(1<<2);
    PORTC_PCR2 |= PORT_PCR_MUX(1);
    
    //led2
    GPIOC_PDDR |= GPIO_PDDR_PDD(1<<3);
    GPIOC_PDOR |= GPIO_PDOR_PDO(1<<3);
    PORTC_PCR3 |= PORT_PCR_MUX(1);

#endif
    bootloader_init();
    bootloader_run();

    // Should never end up here.
    debug_printf("Warning: reached end of main()\r\n");
}

//! Since we never exit this gets rid of the C standard functions that cause
//! extra ROM size usage.
void exit(int arg)
{
}

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
