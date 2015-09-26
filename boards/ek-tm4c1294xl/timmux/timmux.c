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
#include "inc/hw_ints.h"
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

#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

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


void setup_timmux_gpio(void);
void setup_low_accuracy_timer(void);
void timmux_insert(int id, uint32_t tim_val);


extern volatile uint32_t probe_timer_interval;

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
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    //
    // Configure the device pins.
    //
    PinoutSet(false, false);

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
    UARTprintf("Starting timers multiplexor example\n");

    setup_timmux_gpio();
    setup_low_accuracy_timer();

    //
    // We are finished.  Hang around flashing D1.
    //
    while(1)
    {
        //
        // Turn on D1.
        //
        LEDWrite(CLP_D1, 1);


        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 4);

        //
        // Turn off D1.
        //
        LEDWrite(CLP_D1, 0);

        //
        // Delay for a bit.
        //

        SysCtlDelay(g_ui32SysClock / 4);

#if 0
        timmux_insert(/*id*/ 0, 1000 ); // 2us
        SysCtlDelay(50000);
        UARTprintf("Prev delta: %d\n", probe_timer_interval);
        timmux_insert(/*id*/ 2, 3000 );
        timmux_insert(/*id*/ 1, 2000 ); // 2+x us
        timmux_insert(/*id*/ 3, 4000 );
#endif
        timmux_insert(/*id*/ 0, 1000 );
        timmux_insert(/*id*/ 1, 1000 );
        timmux_insert(/*id*/ 2, 1000 );
        timmux_insert(/*id*/ 3, 1000 );
        timmux_insert(/*id*/ 4, 1000 );
        timmux_insert(/*id*/ 5, 1000 );
        timmux_insert(/*id*/ 6, 1000 );
        timmux_insert(/*id*/ 7, 1000 );

    }
}
