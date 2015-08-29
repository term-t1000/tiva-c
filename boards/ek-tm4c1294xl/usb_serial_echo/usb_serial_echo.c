//****************************************************************************
//
// usb_dev_cserial.c - Main routines for the USB CDC composite serial example.
//
// Copyright (c) 2010-2014 Texas Instruments Incorporated.  All rights reserved.
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
//****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/usb.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcomp.h"
#include "usblib/device/usbdcdc.h"
#include "utils/ustdlib.h"
#include "drivers/pinout.h"
#include "usb_structs.h"

//****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB Composite Serial Device (usb_dev_cserial)</h1>
//!
//! This example application turns the evaluation kit into a multiple virtual
//! serial ports when connected to the USB host system.  The application
//! supports the USB Communication Device Class, Abstract Control Model to
//! redirect UART0 traffic to and from the USB host system.  For this example,
//! the evaluation kit will enumerate as a composite device with two virtual
//! serial ports. Including the physical UART0 connection with the ICDI, this
//! means that three independent virtual serial ports will be visible to the
//! USB host.
//!
//! The first virtual serial port will echo data to the physical UART0 port on
//! the device which is connected to the virtual serial port on the ICDI device
//! on this board. The physical UART0 will also echo onto the first virtual
//! serial device provided by the Stellaris controller.
//!
//! The second Stellaris virtual serial port will provide a console that can
//! echo data to both the ICDI virtual serial port and the first Stellaris
//! virtual serial port.  It will also allow turning on, off or toggling the
//! boards led status.  Typing a "?" and pressing return should echo a list of
//! commands to the terminal, since this board can show up as possibly three
//! individual virtual serial devices.
//!
//! Assuming you installed TivaWare in the default directory, a driver
//! information (INF) file for use with Windows XP, Windows Vista and Windows7
//! can be found in C:/TivaWare_C_Series-x.x/windows_drivers. For Windows 2000,
//! the required INF file is in C:/TivaWare_C_Series-x.x/windows_drivers/win2K.
//
//*****************************************************************************

//****************************************************************************
//
// Note:
//
// This example is intended to run on Stellaris evaluation kit hardware
// where the UARTs are wired solely for TX and RX, and do not have GPIOs
// connected to act as handshake signals.  As a result, this example mimics
// the case where communication is always possible.  It reports DSR, DCD
// and CTS as high to ensure that the USB host recognizes that data can be
// sent and merely ignores the host's requested DTR and RTS states.  "TODO"
// comments in the code indicate where code would be required to add support
// for real handshakes.
//
//****************************************************************************


//****************************************************************************
//
// The system tick rate expressed both as ticks per second and a millisecond
// period.
//
//****************************************************************************
#define SYSTICKS_PER_SECOND 100
#define SYSTICK_PERIOD_MS (1000 / SYSTICKS_PER_SECOND)

//*****************************************************************************
//
// Variable to remember our clock frequency
//
//*****************************************************************************
uint32_t g_ui32SysClock = 0;

//****************************************************************************
//
// Default line coding settings for the redirected UART.
//
//****************************************************************************
#define DEFAULT_BIT_RATE        115200
#define DEFAULT_UART_CONFIG     (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | \
                                 UART_CONFIG_STOP_ONE)

//****************************************************************************
//
// GPIO peripherals and pins muxed with the redirected UART.  These will
// depend upon the IC in use and the UART selected in UART0_BASE.  Be careful
// that these settings all agree with the hardware you are using.
//
//****************************************************************************
#define TX_GPIO_BASE            GPIO_PORTA_BASE
#define TX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOA
#define TX_GPIO_PIN             GPIO_PIN_1

#define RX_GPIO_BASE            GPIO_PORTA_BASE
#define RX_GPIO_PERIPH          SYSCTL_PERIPH_GPIOA
#define RX_GPIO_PIN             GPIO_PIN_0

//****************************************************************************
//
// The LED control macros.
//
//****************************************************************************
#define LEDOn()  ROM_GPIOPinWrite(CLP_D1_PORT, CLP_D1_PIN, CLP_D1_PIN);

#define LEDOff() ROM_GPIOPinWrite(CLP_D1_PORT, CLP_D1_PIN, 0)
#define LEDToggle()                                                         \
        ROM_GPIOPinWrite(CLP_D1_PORT, CLP_D1_PIN,                           \
                         (ROM_GPIOPinRead(CLP_D1_PORT, CLP_D1_PIN) ^        \
                          CLP_D1_PIN));

//****************************************************************************
//
// Flag indicating whether or not we are currently sending a Break condition.
//
//****************************************************************************
static bool g_bSendingBreak = false;

//****************************************************************************
//
// Global system tick counter
//
//****************************************************************************
volatile uint32_t g_ui32SysTickCount = 0;

volatile uint32_t last_time_usb_accessed = 0;

//****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002
#define COMMAND_RECEIVED        0x00000004

volatile uint32_t g_ui32Flags = 0;
char *g_pcStatus;

//****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//****************************************************************************
static volatile bool g_bUSBConfigured = false;

//****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1)
    {
    }
}
#endif

//****************************************************************************
//
// Interrupt handler for the system tick counter.
//
//****************************************************************************
void
SysTickIntHandler(void)
{
    //
    // Update our system time.
    //
    g_ui32SysTickCount++;
}

//****************************************************************************
//
// Set the state of the RS232 RTS and DTR signals.  Handshaking is not
// supported so this request will be ignored.
//
//****************************************************************************
static void
SetControlLineState(unsigned short usState)
{
}

//****************************************************************************
//
// Set the communication parameters to use on the UART.
//
//****************************************************************************
static bool
SetLineCoding(tLineCoding *psLineCoding)
{
    uint32_t ui32Config;
    bool bRetcode;

    //
    // Assume everything is OK until we detect any problem.
    //
    bRetcode = true;

    //
    // Word length.  For invalid values, the default is to set 8 bits per
    // character and return an error.
    //
    switch(psLineCoding->ui8Databits)
    {
        case 5:
        {
            ui32Config = UART_CONFIG_WLEN_5;
            break;
        }

        case 6:
        {
            ui32Config = UART_CONFIG_WLEN_6;
            break;
        }

        case 7:
        {
            ui32Config = UART_CONFIG_WLEN_7;
            break;
        }

        case 8:
        {
            ui32Config = UART_CONFIG_WLEN_8;
            break;
        }

        default:
        {
            ui32Config = UART_CONFIG_WLEN_8;
            bRetcode = false;
            break;
        }
    }

    //
    // Parity.  For any invalid values, we set no parity and return an error.
    //
    switch(psLineCoding->ui8Parity)
    {
        case USB_CDC_PARITY_NONE:
        {
            ui32Config |= UART_CONFIG_PAR_NONE;
            break;
        }

        case USB_CDC_PARITY_ODD:
        {
            ui32Config |= UART_CONFIG_PAR_ODD;
            break;
        }

        case USB_CDC_PARITY_EVEN:
        {
            ui32Config |= UART_CONFIG_PAR_EVEN;
            break;
        }

        case USB_CDC_PARITY_MARK:
        {
            ui32Config |= UART_CONFIG_PAR_ONE;
            break;
        }

        case USB_CDC_PARITY_SPACE:
        {
            ui32Config |= UART_CONFIG_PAR_ZERO;
            break;
        }

        default:
        {
            ui32Config |= UART_CONFIG_PAR_NONE;
            bRetcode = false;
            break;
        }
    }

    //
    // Stop bits.  Our hardware only supports 1 or 2 stop bits whereas CDC
    // allows the host to select 1.5 stop bits.  If passed 1.5 (or any other
    // invalid or unsupported value of ui8Stop, we set up for 1 stop bit but
    // return an error in case the caller needs to Stall or otherwise report
    // this back to the host.
    //
    switch(psLineCoding->ui8Stop)
    {
        //
        // One stop bit requested.
        //
        case USB_CDC_STOP_BITS_1:
        {
            ui32Config |= UART_CONFIG_STOP_ONE;
            break;
        }

        //
        // Two stop bits requested.
        //
        case USB_CDC_STOP_BITS_2:
        {
            ui32Config |= UART_CONFIG_STOP_TWO;
            break;
        }

        //
        // Other cases are either invalid values of ui8Stop or values that we
        // cannot support so set 1 stop bit but return an error.
        //
        default:
        {
            ui32Config = UART_CONFIG_STOP_ONE;
            bRetcode |= false;
            break;
        }
    }

    //
    // Let the caller know if we had a problem or not.
    //
    return(bRetcode);
}

//****************************************************************************
//
// Get the communication parameters in use on the UART.
//
//****************************************************************************
static void
GetLineCoding(tLineCoding *psLineCoding)
{
    uint32_t ui32Config;

    psLineCoding->ui32Rate = DEFAULT_BIT_RATE;
    ui32Config = DEFAULT_UART_CONFIG;

    //
    // Translate the configuration word length field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_WLEN_MASK)
    {
        case UART_CONFIG_WLEN_8:
        {
            psLineCoding->ui8Databits = 8;
            break;
        }

        case UART_CONFIG_WLEN_7:
        {
            psLineCoding->ui8Databits = 7;
            break;
        }

        case UART_CONFIG_WLEN_6:
        {
            psLineCoding->ui8Databits = 6;
            break;
        }

        case UART_CONFIG_WLEN_5:
        {
            psLineCoding->ui8Databits = 5;
            break;
        }
    }

    //
    // Translate the configuration parity field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_PAR_MASK)
    {
        case UART_CONFIG_PAR_NONE:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_NONE;
            break;
        }

        case UART_CONFIG_PAR_ODD:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_ODD;
            break;
        }

        case UART_CONFIG_PAR_EVEN:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_EVEN;
            break;
        }

        case UART_CONFIG_PAR_ONE:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_MARK;
            break;
        }

        case UART_CONFIG_PAR_ZERO:
        {
            psLineCoding->ui8Parity = USB_CDC_PARITY_SPACE;
            break;
        }
    }

    //
    // Translate the configuration stop bits field into the format expected
    // by the host.
    //
    switch(ui32Config & UART_CONFIG_STOP_MASK)
    {
        case UART_CONFIG_STOP_ONE:
        {
            psLineCoding->ui8Stop = USB_CDC_STOP_BITS_1;
            break;
        }

        case UART_CONFIG_STOP_TWO:
        {
            psLineCoding->ui8Stop = USB_CDC_STOP_BITS_2;
            break;
        }
    }
}

//****************************************************************************
//
// This function sets or clears a break condition on the redirected UART RX
// line.  A break is started when the function is called with \e bSend set to
// \b true and persists until the function is called again with \e bSend set
// to \b false.
//
//****************************************************************************
static void
SendBreak(bool bSend)
{
    //
    // Are we being asked to start or stop the break condition?
    //
    if(!bSend)
    {
        //
        // Remove the break condition on the line.
        //
        //UARTBreakCtl(UART0_BASE, false);
        g_bSendingBreak = false;
    }
    else
    {
        //
        // Start sending a break condition on the line.
        //
        //UARTBreakCtl(UART0_BASE, true);
        g_bSendingBreak = true;
    }
}

//****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the
// device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//****************************************************************************
uint32_t
ControlHandler(void *pvCBData, uint32_t ui32Event,
               uint32_t ui32MsgValue, void *pvMsgData)
{
    uint32_t ui32IntsOff;

    //
    // Which event are we being asked to process?
    //
    switch(ui32Event)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bUSBConfigured = true;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_psTxBuffer);
            USBBufferFlush(&g_psRxBuffer);

            //
            // Tell the main loop to update the display.
            //
            ui32IntsOff = IntMasterDisable();
            g_pcStatus = "Host connected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
            {
                IntMasterEnable();
            }
            break;
        }

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bUSBConfigured = false;
            ui32IntsOff = IntMasterDisable();
            g_pcStatus = "Host disconnected.";
            g_ui32Flags |= COMMAND_STATUS_UPDATE;
            if(!ui32IntsOff)
            {
                IntMasterEnable();
            }
            break;
        }

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
        {
            GetLineCoding(pvMsgData);
            break;
        }

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
        {
            SetLineCoding(pvMsgData);
            break;
        }

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
        {
            SetControlLineState((unsigned short)ui32MsgValue);
            break;
        }

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
        {
            SendBreak(true);
            break;
        }

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
        {
            SendBreak(false);
            break;
        }

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
        {
            break;
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
        {
            break;
        }
    }

    return(0);
}

//****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ui32CBData is the client-supplied callback pointer for this channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//****************************************************************************
uint32_t USBCDC_TxHandler(void *pvCBData, uint32_t ui32Event,
        uint32_t ui32MsgValue, void *pvMsgData) {
    // Which event have we been sent?
    switch(ui32Event)
    {
        case USB_EVENT_TX_COMPLETE:
        {
            //
            // Since we are using the USBBuffer, we don't need to do anything
            // here.
            //
            break;
        }

        //
        // We don't expect to receive any other events.  Ignore any that show
        // up in a release build or hang in a debug build.
        //
        default:
        {
            break;
        }
    }
    return(0);
}

//****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ui32CBData is the client-supplied callback data value for this
// channel.
// \param ui32Event identifies the event we are being notified about.
// \param ui32MsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//****************************************************************************
uint32_t
USBCDC_RxHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgValue,
             void *pvMsgData)
{
    const tUSBDCDCDevice *psCDCDevice;
    const tUSBBuffer *pBufferRx;
    const tUSBBuffer *pBufferTx;

    //
    // Which event are we being sent?
    //
    switch(ui32Event)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
            {
                static uint8_t rbuf[UART_BUFFER_SIZE];

                psCDCDevice = (const tUSBDCDCDevice *)pvCBData;
                pBufferRx = (const tUSBBuffer *)psCDCDevice->pvRxCBData;
                pBufferTx = (const tUSBBuffer *)psCDCDevice->pvTxCBData;

                int rxsize = USBBufferRead(pBufferRx,rbuf, UART_BUFFER_SIZE);
                USBBufferWrite(pBufferTx,rbuf, rxsize);

                last_time_usb_accessed = g_ui32SysTickCount;

                break;
            }

            //
            // We are being asked how much unprocessed data we have still to
            // process. We return 0 if the UART is currently idle or 1 if it is
            // in the process of transmitting something. The actual number of
            // bytes in the UART FIFO is not important here, merely whether or
            // not everything previously sent to us has been transmitted.
            //
        case USB_EVENT_DATA_REMAINING:
            {
                //
                // Get the number of bytes in the buffer and add 1 if some data
                // still has to clear the transmitter.
                //
                return(0);
            }

            //
            // We are being asked to provide a buffer into which the next packet
            // can be read. We do not support this mode of receiving data so let
            // the driver know by returning 0. The CDC driver should not be
            // sending this message but this is included just for illustration and
            // completeness.
            //
        case USB_EVENT_REQUEST_BUFFER:
            {
                return(0);
            }

            //
            // We don't expect to receive any other events.  Ignore any that show
            // up in a release build or hang in a debug build.
            //
        default:
            {
                break;
            }
    }

    return(0);
}

//****************************************************************************
//
// This is the main application entry function.
//
//****************************************************************************
int main(void)
{
    // Run from the PLL at 120 MHz.
    g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    // Not configured initially.
    g_bUSBConfigured = false;

    // Enable the peripherals used in this example.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);

    // Configure the device pins.
    // Enable pins for USB and disable for ethernet
    PinoutSet(false, true);

    // Turn off the LED.
    LEDOff();

    // Enable the system tick.
    // g_ui32SysTickCount contains a number of 10ms ticks since start.
    SysTickPeriodSet(g_ui32SysClock / SYSTICKS_PER_SECOND);
    SysTickIntEnable();
    SysTickEnable();

    // Initialize the transmit and receive buffers for first serial device.
    USBBufferInit(&g_psTxBuffer);
    USBBufferInit(&g_psRxBuffer);
    // Full Speed usb uses 64 bytes per packet.
    // If last packet in transmission has exact 64 bytes size
    // usb device/host should notify remote side about transmission end
    // for this purpose usb stack sents Zero Length Packet (ZLP)
    // I have patched Tiva C library to avoid stales like described in
    // https://github.com/olegv142/stm32tivc_usb_cdc
    USBBufferZeroLengthPacketInsert(&g_psTxBuffer, true);

    USBDCDCInit(0, &g_psCDCDevice);

    //
    // Main application loop.
    //
    while(1) {
        // burn the light if last usb access was < 50 ms ago
        if (last_time_usb_accessed + 5 > g_ui32SysTickCount) {
            LEDOn();
        } else {
            LEDOff();
        }
    }
}
