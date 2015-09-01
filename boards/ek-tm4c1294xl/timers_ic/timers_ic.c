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


#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "inc/tm4c1294ncpdt.h"


#define GPIO_PORTQ_DATA_R       (*((volatile uint32_t *)0x400663FC))

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



void Timer7aIntHandler(void) {
    ROM_TimerIntClear(TIMER7_BASE, TIMER_TIMA_TIMEOUT);
    GPIO_PORTQ_DATA_R ^= (1 << 0); // TOGGLE PQ0
}


volatile uint32_t timer_high = 0;
volatile uint64_t last_timer_prev = 0;
volatile uint32_t last_timer_value = 0;

#define TIM0_PRESCALER 256


void Timer0OvfHandler(void) {
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // this should have higher interrupt priority
    ++timer_high;
}

// prev values 30725122
// C1 6 Capture PL5 86 - - - T0CCP1 
void Timer0Capture1Handler(void) {
    volatile uint32_t high_cnt = 0;
    ROM_IntMasterDisable();
    high_cnt = timer_high;
    ROM_IntMasterEnable();

    ROM_TimerIntClear(TIMER0_BASE, TIMER_CAPB_EVENT);

    //uint32_t t = TIMER0_TBPS_R; // what this counter is doing?
    uint64_t t = ( ((uint64_t)high_cnt) << 24) | TIMER0_TBR_R;// | TIMER0_TBPS_R;

    if (last_timer_prev < t) {
        last_timer_value = t - last_timer_prev;
    } else {
        last_timer_value = 0xFFFFFFFFFFFFFFFFUL + t - last_timer_prev;
    }

    last_timer_prev = t;

    GPIO_PORTQ_DATA_R ^= (1 << 1); // TOGGLE PQ1
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

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    GPIO_PORTQ_DATA_R &= ~(1 << 0);
    GPIO_PORTQ_DATA_R &= ~(1 << 1);

    // Enable processor interrupts.
    ROM_IntMasterEnable();

    // TIMER7 Is only used to generate appropriate test signal
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7);
    ROM_TimerConfigure(TIMER7_BASE, TIMER_CFG_A_PERIODIC 
            | TIMER_CFG_SPLIT_PAIR);
    ROM_TimerLoadSet(TIMER7_BASE, TIMER_A, 60000); // 120M / 60000 / 10 = 200 times per second 
    ROM_IntEnable(INT_TIMER7A);
    ROM_TimerIntEnable(TIMER7_BASE, TIMER_TIMA_TIMEOUT);

    TimerPrescaleSet(TIMER7_BASE, TIMER_A , 10-1);

    ROM_TimerEnable(TIMER7_BASE, TIMER_A);


    // We need two 24bit timers (16bit + 8 bit prescaler) to handle overflow
    // This allows to measure periods longer than 128ms

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    ROM_GPIOPinConfigure(GPIO_PL5_T0CCP1);
    GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_5);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC_UP |
            TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_CAP_TIME_UP );
    TimerControlEvent(TIMER0_BASE, TIMER_B, TIMER_EVENT_POS_EDGE);
    ROM_TimerMatchSet(TIMER0_BASE, TIMER_B, 0);

    ROM_TimerMatchSet(TIMER0_BASE, TIMER_A, 0);

    ROM_IntEnable(INT_TIMER0B);
    ROM_IntEnable(INT_TIMER0A);

    ROM_TimerIntEnable(TIMER0_BASE, TIMER_CAPB_EVENT);
    ROM_TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerPrescaleSet(TIMER0_BASE, TIMER_A , TIM0_PRESCALER-1); // 25M / 2^24 = 1.49 times per second
    TimerPrescaleSet(TIMER0_BASE, TIMER_B , TIM0_PRESCALER-1); // 25M / 2^24 = 1.49 times per second

    ROM_TimerEnable(TIMER0_BASE, TIMER_BOTH); 

    // We are finished.  Hang around flashing D1.
    while(1)
    {
        //
        // Turn on D1.
        //
        LEDWrite(CLP_D1, 1);

        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 10);

        //
        // Turn off D1.
        //
        LEDWrite(CLP_D1, 0);

        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 10);

        UARTprintf("%3d %u\r\n", timer_high, last_timer_value);
    }
}
