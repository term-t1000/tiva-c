//*****************************************************************************
//
// hello.c - Simple hello world example.
//
// Copyright (c) 2013-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "drivers/pinout.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


#include "inc/hw_ccm.h"
#include "driverlib/udma.h"

#include "driverlib/crc.h"



//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! Open a terminal with 115,200 8-N-1 to see the output for this demo.
//
//*****************************************************************************

//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}



#define CCM_LOOP_TIMEOUT        500000

//*****************************************************************************
//
// The DMA control structure table.
//
//*****************************************************************************
#if defined(ewarm)
#pragma data_alignment=1024
tDMAControlTable g_psDMAControlTable[64];
#elif defined(ccs)
#pragma DATA_ALIGN(g_psDMAControlTable, 1024)
tDMAControlTable g_psDMAControlTable[64];
#else
tDMAControlTable g_psDMAControlTable[64] __attribute__((aligned(1024)));
#endif

//*****************************************************************************
//
// Random data for use with the test vectors.
//
//*****************************************************************************
uint32_t g_pui32RandomData[] =
{
    0x8a5f1b22, 0xcb935d29, 0xcc1ac092, 0x5dad8c9e, 0x6a83b39f, 0x8607dc60,
    0xda0ba4d2, 0xf49b0fa2, 0xaf35d524, 0xffa8001d, 0xbcc931e8, 0x4a2c99ef,
    0x7fa297ab, 0xab943bae, 0x07c61cc4, 0x47c8627d
};

//*****************************************************************************
//
// Test Vector Structure
//
//*****************************************************************************
typedef struct CRCTestVectorStruct
{
    uint32_t ui32TestNumber;
    uint32_t ui32Seed;
    uint32_t ui32Result;
}
tCRCTestVector;

//*****************************************************************************
//
// CRC-32 Test Vectors
//
//*****************************************************************************
tCRCTestVector g_psCRC4C11DB7TestVectors[3] =
{
    {
        0,
        0x00000000,
        0xbcc90d0d
    },
    {
        1,
        0xffffffff,
        0x2ff0435c
    },
    {
        2,
        0xa5a5a5a5,
        0x75fd6f5c
    }
};



bool CRCInit(void)
{
    uint32_t ui32Loop;

    //
    // Check that the CCM peripheral is present.
    //
    if(!ROM_SysCtlPeripheralPresent(SYSCTL_PERIPH_CCM0))
    {
        UARTprintf("No CCM peripheral found!\n");

        //
        // Return failure.
        //
        return(false);
    }

    //
    // The hardware is available, enable it.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);

    //
    // Wait for the peripheral to be ready.
    //
    ui32Loop = 0;
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
        //
        // Increment our poll counter.
        //
        ui32Loop++;

        if(ui32Loop > CCM_LOOP_TIMEOUT)
        {
            //
            // Timed out, notify and spin.
            //
            UARTprintf("Time out on CCM ready after enable.\n");

            //
            // Return failure.
            //
            return(false);
        }
    }

    //
    // Reset the peripheral to ensure we are starting from a known condition.
    //
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_CCM0);

    //
    // Wait for the peripheral to be ready again.
    //
    ui32Loop = 0;
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
        //
        // Increment our poll counter.
        //
        ui32Loop++;

        if(ui32Loop > CCM_LOOP_TIMEOUT)
        {
            //
            // Timed out, spin.
            //
            UARTprintf("Time out on CCM ready after reset.\n");

            //
            // Return failure.
            //
            return(false);
        }
    }

    //
    // Return initialization success.
    //
    return(true);
}




// Process data to produce a CRC-32 checksum.
//
//*****************************************************************************
uint32_t
CRC32DataProcess(uint32_t *pui32Data, uint32_t ui32Length, uint32_t ui32Seed,
                 bool bUseDMA)
{
    uint32_t ui32Result;

    //
    // Perform a soft reset.
    //
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_CCM0);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
    }

    //
    // Configure the CRC engine.
    //
    ROM_CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_SEED | CRC_CFG_TYPE_P4C11DB7 |
                                 CRC_CFG_SIZE_32BIT));

    //
    // Write the seed.
    //
    ROM_CRCSeedSet(CCM0_BASE, ui32Seed);

    //
    // Generate CRC using uDMA to copy data.
    //
    if(bUseDMA)
    {
        //
        // Setup the DMA module to copy data in.
        //
        ROM_uDMAChannelAssign(UDMA_CH30_SW);
        ROM_uDMAChannelAttributeDisable(UDMA_CH30_SW,
                                        UDMA_ATTR_ALTSELECT |
                                        UDMA_ATTR_USEBURST |
                                        UDMA_ATTR_HIGH_PRIORITY |
                                        UDMA_ATTR_REQMASK);
        ROM_uDMAChannelControlSet(UDMA_CH30_SW | UDMA_PRI_SELECT,
                                  UDMA_SIZE_32 | UDMA_SRC_INC_32 |
                                  UDMA_DST_INC_NONE | UDMA_ARB_1 |
                                  UDMA_DST_PROT_PRIV);
        ROM_uDMAChannelTransferSet(UDMA_CH30_SW | UDMA_PRI_SELECT,
                                   UDMA_MODE_AUTO, (void *)g_pui32RandomData,
                                   (void *)(CCM0_BASE + CCM_O_CRCDIN),
                                   ui32Length);
        ROM_uDMAChannelEnable(UDMA_CH30_SW);
        UARTprintf(" Data in DMA request enabled.\n");

        //
        // Start the uDMA.
        //
        ROM_uDMAChannelRequest(UDMA_CH30_SW | UDMA_PRI_SELECT);

        //
        // Wait for the transfer to finish.
        //
        while(ROM_uDMAChannelIsEnabled(UDMA_CH30_SW))
        {
        }

        //
        // Read the result.
        //
        ui32Result = ROM_CRCResultRead(CCM0_BASE, false);
    }

    //
    // Generate CRC using CPU to copy data.
    //
    else
    {
        ui32Result = ROM_CRCDataProcess(CCM0_BASE, g_pui32RandomData,
                                        ui32Length, false);
    }

    return(ui32Result);
}




//*****************************************************************************
//
// Print "Hello World!" to the UART on the Intelligent UART Module.
//
//*****************************************************************************
int
main(void)
{
    //
    // Run from the PLL at 120 MHz.
    //
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                SYSCTL_CFG_VCO_480), 120000000);

    //
    // Configure the device pins.
    //
    PinoutSet(false, false);


    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    ROM_uDMAEnable();
    ROM_uDMAControlBaseSet(g_psDMAControlTable);


    uint32_t ui32Errors;
    if(!CRCInit())
    {
        UARTprintf("Initialization of the CRC  module failed.\n");
        ui32Errors |= 0x00000001;
    }


    //
    // Enable the GPIO pins for the LED D1 (PN1).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Hello!
    //
    UARTprintf("Hello, world!\n");

    //
    // We are finished.  Hang around flashing D1.
    //
    while(1)
    {
        uint32_t ui32Result;

        for(uint32_t ui32Idx = 0;
                ui32Idx < (sizeof(g_psCRC4C11DB7TestVectors) / sizeof(tCRCTestVector));
                ui32Idx++)
        {
            UARTprintf("Starting vector #%d\n", ui32Idx);

            //
            // Generate the checksum without uDMA.
            //
            UARTprintf("Generating CRC-32 checksum without uDMA.\n");
            ui32Result =
                CRC32DataProcess(g_pui32RandomData,
                        (sizeof(g_pui32RandomData) / 4),
                        g_psCRC4C11DB7TestVectors[ui32Idx].ui32Seed,
                        false);

            if(ui32Result != g_psCRC4C11DB7TestVectors[ui32Idx].ui32Result)
            {
                UARTprintf("CRC result mismatch - Exp: 0x%08x, Act: 0x%08x\n",
                        g_psCRC4C11DB7TestVectors[ui32Idx].ui32Result,
                        ui32Result);
            }

            UARTprintf("Generating CRC-32 checksum with uDMA.\n");
            ui32Result =
                CRC32DataProcess(g_pui32RandomData,
                        (sizeof(g_pui32RandomData) / 4),
                        g_psCRC4C11DB7TestVectors[ui32Idx].ui32Seed,
                        true);

            if(ui32Result != g_psCRC4C11DB7TestVectors[ui32Idx].ui32Result)
            {
                UARTprintf("CRC result mismatch - Exp: 0x%08x, Act: 0x%08x\n",
                        g_psCRC4C11DB7TestVectors[ui32Idx].ui32Result,
                        ui32Result);
            }


        }




        //
        // Turn on D1.
        //
        LEDWrite(CLP_D1, 1);

        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 10 / 3);

        //
        // Turn off D1.
        //
        LEDWrite(CLP_D1, 0);

        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 10 / 3);
    }
}
