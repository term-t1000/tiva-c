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

#include "driverlib/adc.h"

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



static uint32_t g_pui32ADCData[8];

#define Convert10Bit(ui32Value)  ((int16_t)(ui32Value >> 2))

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

//*****************************************************************************
//
// Initialize the ADC inputs used by the game pad device.  This example uses
// the ADC pins on Port E pins 1, 2, and 3(AIN0-2).
//
//*****************************************************************************

/*        X8 (from top)
 * 3.3V         5.0V
 *              GND
 *              PE0 AIN3
 *              PE1 AIN2
 *              PE2 AIN1
 *              PE3 AIN0
 *              PD7 AIN4
 *
 *
 *              Not on main headers:
 *              AIN5 PD6
 *              AIN6 PD5
 *              AIN7 PD4
 *              AIN8 PE5
 *              AIN9 PE4
 */
void
ADCInit(void)
{
    int32_t ui32Chan;

    //
    // Enable the GPIOs and the ADC used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_ADC0);

    //
    // Select the external reference for greatest accuracy.
    //
    ROM_ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);

    //
    // Configure the pins which are used as analog inputs.
    //
    ROM_GPIOPinTypeADC(GPIO_PORTE_AHB_BASE, GPIO_PIN_3 | GPIO_PIN_2 |
                                            GPIO_PIN_1 | GPIO_PIN_0);

    ROM_GPIOPinTypeADC(GPIO_PORTD_AHB_BASE, GPIO_PIN_7);
    ROM_GPIOPinTypeADC(GPIO_PORTD_AHB_BASE, GPIO_PIN_6);
    ROM_GPIOPinTypeADC(GPIO_PORTD_AHB_BASE, GPIO_PIN_5);
    ROM_GPIOPinTypeADC(GPIO_PORTD_AHB_BASE, GPIO_PIN_4);

    // Need second ADC to get > 8 channels
    //ROM_GPIOPinTypeADC(GPIO_PORTE_AHB_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    //
    // Configure the sequencer for 4 steps.
    //
    for(ui32Chan = 0; ui32Chan < 7; ui32Chan++)
    {
        //
        // Configure the sequence step
        //
        ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, ui32Chan, ui32Chan);
    }

    ROM_ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH7 | ADC_CTL_IE |
                                                  ADC_CTL_END);
    //
    // Enable the sequence but do not start it yet.
    //
    ROM_ADCSequenceEnable(ADC0_BASE, 0);
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

    ADCInit();

    ADCProcessorTrigger(ADC0_BASE, 0);

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
        SysCtlDelay(g_ui32SysClock / 10 );

        //
        // Turn off D1.
        //
        LEDWrite(CLP_D1, 0);

        //
        // Delay for a bit.
        //
        SysCtlDelay(g_ui32SysClock / 10 );

        uint16_t adc_val[sizeof(g_pui32ADCData)/sizeof(g_pui32ADCData[0])];
        // See if the ADC updated.
        //
        if(ADCIntStatus(ADC0_BASE, 0, false) != 0)
        {
            //
            // Clear the ADC interrupt.
            //
            ADCIntClear(ADC0_BASE, 0);

            //
            // Read the data and trigger a new sample request.
            //
            ADCSequenceDataGet(ADC0_BASE, 0, &g_pui32ADCData[0]);
            ADCProcessorTrigger(ADC0_BASE, 0);

            //
            // Update the report.
            //
            UARTprintf("ADC:");
            for (int i = 0; i < sizeof(g_pui32ADCData)/sizeof(g_pui32ADCData[0]); ++i) {
                adc_val[i] = Convert10Bit(g_pui32ADCData[i]);
                UARTprintf(" %4d", adc_val[i]);
            }
            UARTprintf("\r\n");
        }


    }
}
