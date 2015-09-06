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
#include "driverlib/ssi.h"
#include "utils/uartstdio.h"

#include "driverlib/udma.h"
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"

#include <string.h>

#include "driverlib/crc.h"
#include "inc/hw_ccm.h"

#include "spi_data_flash.h"


#define DEBUG_SPI 1

#ifdef DEBUG_SPI
#   define dbgprint(...)  UARTprintf(__VA_ARGS__)
#else
#   define dbgprint(...)
#endif

// The control table used by the uDMA controller.  This table must be aligned
// to a 1024 byte boundary.
#if defined(ewarm)
#pragma data_alignment=1024
uint8_t pui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(pui8ControlTable, 1024)
uint8_t pui8ControlTable[1024];
#else
uint8_t pui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif


static uint32_t g_ui32uDMAErrCount = 0;


volatile uint32_t last_written_crc = 0;

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
    // Enable the GPIO Peripheral used by the UART.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, g_ui32SysClock);
}

/*
 * PD0 MISO
 * PD1 MOSI
 * PD2 SS
 * PD3 CLK
*/
void ConfigureSPI(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SSIDisable(SSI2_BASE);


    MAP_GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // MISO

    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 1<<2);

    ROM_GPIOPinConfigure(GPIO_PD3_SSI2CLK); // dark blue
    //ROM_GPIOPinConfigure(GPIO_PD2_SSI2FSS); // SLAVE SELECT
    ROM_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0); // MOSI : PURPLE 
    ROM_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1); // MISO : GREY
    ROM_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3 /* | GPIO_PIN_2 */ | GPIO_PIN_1 | GPIO_PIN_0);

    SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);
    SSIConfigSetExpClk(SSI2_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 12000000, 8);

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 1<<2); // deselect SPI slave (disable nSS)
    ROM_SSIEnable(SSI2_BASE);


    MAP_GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // MISO
}


void spi_receive_finished() {
        SSIIntClear(SSI2_BASE, SSI_DMARX);

        GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 1<<2); // deselect SPI slave (disable nSS)

        SSIIntDisable(SSI2_BASE, SSI_DMARX);
        flash_ctl.in_progress = 0;
}

void SSI2IntHandler(void) {
    uint32_t status = SSIIntStatus(SSI2_BASE, true);
    if (status & SSI_RXTO) {
        SSIIntClear(SSI2_BASE, SSI_RXTO);
    } else if (status & SSI_TXEOT) {
        SSIIntClear(SSI2_BASE, SSI_TXEOT);
    } else if (status & SSI_TXFF) {
        SSIIntClear(SSI2_BASE, SSI_TXFF);
    } else if (status & SSI_RXFF) {
        SSIIntClear(SSI2_BASE, SSI_RXFF);
    } else if (status & SSI_RXOR) {
        SSIIntClear(SSI2_BASE, SSI_RXOR);
        ++(flash_ctl.rcv_err_cnt);
    } else if (status & SSI_DMATX) {
        SSIIntClear(SSI2_BASE, SSI_DMATX);
        SSIIntDisable(SSI2_BASE, SSI_DMATX);
    } else if (status & SSI_DMARX) {
        spi_receive_finished();
    }
}


uint32_t calc_crc32(uint8_t* data, int sz) {
    // Write the seed.
    uint32_t ui32Seed = 0x00000000;
    ROM_CRCSeedSet(CCM0_BASE, ui32Seed);
    return ROM_CRCDataProcess(CCM0_BASE, (uint32_t*)data, (sz+3)/4, false);
}

// I'm not using function pointers to simplify correctness verification
void spi_send_receive_handler(void) {
    if (flash_ctl.cmd == 0 || flash_ctl.in_progress != 0)
        return;

    int spi_received_bytes = flash_ctl.total_sz - flash_ctl.snd_sz; 

    switch (flash_ctl.cmd) {
        case 0x9F:
            dbgprint("Received cmd: %4x sz: %d:", flash_ctl.cmd, spi_received_bytes);
            for (int i = flash_ctl.snd_sz; i < flash_ctl.total_sz; ++i) {
                dbgprint(" %02X", flash_ctl.rbuf[i]); 
            }
            dbgprint("\r\n");
            break;
        case 0x84:
            dbgprint("write Done\r\n");
            break;
        case AT45D_BUFFER1_READ:
            { 
                uint32_t rcv_crc = calc_crc32(flash_ctl.rbuf + flash_ctl.snd_sz, flash_ctl.rcv_sz);
                if (!flash_ctl.ignore_crc_check &&  rcv_crc != last_written_crc) {
                    flash_ctl.crc_err ++;
                } 

                dbgprint("Received cmd: %4x sz: %d, crc32: %08X\r\n", flash_ctl.cmd, spi_received_bytes, rcv_crc);
                /*
                for (int i = 0; i < flash_ctl.rcv_sz; ++i) {
                    dbgprint(" %02X", flash_ctl.rbuf[flash_ctl.snd_sz+i]); 
                }
                */

                break;
            }
        case AT45D_MMPAGE_TO_BUFFER1_CMD:
            dbgprint("read from MM to buffer1: pageid: 0x%08x\r\n", flash_ctl.pageid);
            break;
        case AT45D_STATUS_REGISTER_READ:
            dbgprint("Status reg: 0x%02X%02X\r\n", flash_ctl.rbuf[1], flash_ctl.rbuf[2]);
            // BIT7: RDY/BUSY Ready/Busy Status
            // 0 Device is busy with an internal operation.
            // 1 Device is ready.
            if (flash_ctl.status_busy_wait && ((flash_ctl.rbuf[1] & 0x80) == 0x00 ) ) {
                at45d_read_status_reg(1);
                return;
            }
            break;
        case AT45D_BUFFER1_TO_MMPAGE:
            dbgprint("Memory programming page addr: 0x%08X\r\n", flash_ctl.pageid);
            break;
        default:
            break;
    }
    flash_ctl.cmd = 0;
}

int spi_send_and_receive(int cmd, uint8_t *wbuf, uint8_t *rbuf, int snd_sz, int rcv_sz) {
    if (flash_ctl.in_progress)
        return 0;
    flash_ctl.cmd = cmd;
    flash_ctl.in_progress = 1;

    flash_ctl.snd_sz = snd_sz;
    flash_ctl.rcv_sz = rcv_sz;
    flash_ctl.total_sz = snd_sz + rcv_sz;


    memcpy((void*)flash_ctl.buf, (void*)wbuf, snd_sz);
    memset((void*)(flash_ctl.buf+snd_sz), 0x00, rcv_sz);
    memset((void*)(rbuf), 0x00, flash_ctl.total_sz);


    flash_ctl.rbuf = rbuf;

    if (flash_ctl.rbuf == NULL)
        return 0;


    dbgprint("Sending %4x, sz: %d ", cmd, flash_ctl.total_sz);


    uDMAChannelAssign(UDMA_CH12_SSI2RX);
    ROM_uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_UART2RX_12,
                                    UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);
    ROM_uDMAChannelControlSet(UDMA_SEC_CHANNEL_UART2RX_12 | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_NONE | UDMA_DST_INC_8 |
                              UDMA_ARB_4);
    ROM_uDMAChannelTransferSet(UDMA_SEC_CHANNEL_UART2RX_12 | UDMA_PRI_SELECT,
                               UDMA_MODE_PINGPONG,
                               (void *)(SSI2_BASE + SSI_O_DR),
                               flash_ctl.rbuf,
                               flash_ctl.total_sz);

    uDMAChannelAssign(UDMA_CH13_SSI2TX);
    ROM_uDMAChannelAttributeDisable(UDMA_SEC_CHANNEL_UART2TX_13,
                                    UDMA_ATTR_ALTSELECT |
                                    UDMA_ATTR_HIGH_PRIORITY |
                                    UDMA_ATTR_REQMASK);
    ROM_uDMAChannelAttributeEnable(UDMA_SEC_CHANNEL_UART2TX_13, UDMA_ATTR_USEBURST);
    ROM_uDMAChannelControlSet(UDMA_SEC_CHANNEL_UART2TX_13 | UDMA_PRI_SELECT,
                              UDMA_SIZE_8 | UDMA_SRC_INC_8 |
                              UDMA_DST_INC_NONE |
                              UDMA_ARB_4);
    ROM_uDMAChannelTransferSet(UDMA_SEC_CHANNEL_UART2TX_13 | UDMA_PRI_SELECT,
                               UDMA_MODE_BASIC, (void*)flash_ctl.buf,
                               (void *)(SSI2_BASE + SSI_O_DR),
                               flash_ctl.total_sz);

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 0);

    ROM_uDMAChannelEnable(UDMA_SEC_CHANNEL_UART2RX_12);
    ROM_uDMAChannelEnable(UDMA_SEC_CHANNEL_UART2TX_13);

    SSIDMAEnable(SSI2_BASE, SSI_DMA_TX | SSI_DMA_RX);

    SSIIntEnable(SSI2_BASE, SSI_DMATX | SSI_DMARX | SSI_RXOR);

    return 1;
}

void uDMAErrorHandler(void) {
    uint32_t ui32Status;

    //
    // Check for uDMA error bit
    //
    ui32Status = ROM_uDMAErrorStatusGet();

    //
    // If there is a uDMA error, then clear the error and increment
    // the error counter.
    //
    if(ui32Status)
    {
        ROM_uDMAErrorStatusClear();
        g_ui32uDMAErrCount++;
    }
}


void uDMAIntHandler(void) {
}


#define CCM_LOOP_TIMEOUT        500000
bool CRCInit(void) {
    uint32_t ui32Loop;

    // Check that the CCM peripheral is present.
    if(!ROM_SysCtlPeripheralPresent(SYSCTL_PERIPH_CCM0))
    {
        UARTprintf("No CCM peripheral found!\n");

        //
        // Return failure.
        //
        return(false);
    }

    // The hardware is available, enable it.
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_CCM0);

    // Wait for the peripheral to be ready.
    ui32Loop = 0;
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
        // Increment our poll counter.
        ui32Loop++;

        if(ui32Loop > CCM_LOOP_TIMEOUT)
        {
            // Timed out, notify and spin.
            UARTprintf("Time out on CCM ready after enable.\n");

            // Return failure.
            return(false);
        }
    }

    // Reset the peripheral to ensure we are starting from a known condition.
    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_CCM0);

    // Wait for the peripheral to be ready again.
    ui32Loop = 0;
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
        // Increment our poll counter.
        ui32Loop++;

        if(ui32Loop > CCM_LOOP_TIMEOUT)
        {
            // Timed out, spin.
            UARTprintf("Time out on CCM ready after reset.\n");

            // Return failure.
            return(false);
        }
    }


    ROM_SysCtlPeripheralReset(SYSCTL_PERIPH_CCM0);
    while(!ROM_SysCtlPeripheralReady(SYSCTL_PERIPH_CCM0))
    {
    }

    // Configure the CRC engine.
    ROM_CRCConfigSet(CCM0_BASE, (CRC_CFG_INIT_SEED | CRC_CFG_TYPE_P4C11DB7 |
                CRC_CFG_SIZE_32BIT));


    // Return initialization success.
    return(true);
}


int at45d_main_memory_to_buffer1(uint32_t addr) {
    // 0 1 0 1 0 0 1 1 X X X X X A A A A A A A A A A A X X X X X X X X
    // <CMD><reserved[7:3], A[18:16]><A[15:8]><A[7:0] - dont care>
    static uint8_t read_buffer_cmd[4] = { AT45D_MMPAGE_TO_BUFFER1_CMD, 0x00, 0x00, 0x00};
    read_buffer_cmd[1] = 0xF & (addr >> 6); // <XXXX, PA10-7>
    read_buffer_cmd[2] = 0xFF & (addr << 2); // <PA6-0, XX>
    read_buffer_cmd[3] = 0x00; // <XXXX XXXX>

    static uint8_t serno_response[300] = { 0 }; 
    flash_ctl.pageid = addr;
    return spi_send_and_receive(AT45D_MMPAGE_TO_BUFFER1_CMD, read_buffer_cmd, serno_response, 4, 0);
}

int at45d_buffer1_to_main_memory(uint32_t addr) {
    //   0        1          2           3
    // <CMD><XXXX, PA10-7><PA6-0, XX><XXXX XXXX>
    static uint8_t read_buffer_cmd[4] = { AT45D_BUFFER1_TO_MMPAGE, 0x00, 0x00, 0x00};
    read_buffer_cmd[1] = 0xF & (addr >> 6); // <XXXX, PA10-7>
    read_buffer_cmd[2] = 0xFF & (addr << 2); // <PA6-0, XX>
    read_buffer_cmd[3] = 0x00; // <XXXX XXXX>
    static uint8_t serno_response[300] = { 0 }; 
    flash_ctl.pageid = addr;
    return spi_send_and_receive(AT45D_BUFFER1_TO_MMPAGE, read_buffer_cmd, serno_response, 4, 0);
}

int at45d_buffer1_read(uint8_t *rbuf, uint16_t bfa, int sz, int ignore_crc) {
    //  1 0 1 0 1 0 0 X X X X X X X X X X X X X X X B B B B B B B B B
    //  <CMD><X><X-BFA8><BFA7-0><X>
    static uint8_t read_buffer_cmd[5] = { AT45D_BUFFER1_READ, 0x00, 0x00, 0x00, 0x00 };
    read_buffer_cmd[2] = (bfa & 0x0100) >> 8;
    read_buffer_cmd[3] = 0xFF & (bfa);

    flash_ctl.ignore_crc_check = ignore_crc;
    return spi_send_and_receive(AT45D_BUFFER1_READ, read_buffer_cmd, rbuf, 5, sz);
}


int at45d_write_to_buffer1(uint8_t *data, int sz, uint16_t bfa) {
    //   0   1    2     3
    // <CMD><X><X-BFA8><BFA7-0>
    static uint8_t write_buffer_cmd[4+264] = { AT45D_BUFFER1_WRITE_CMD, 0x00, 0x00, 0x00, 0x00 };
    write_buffer_cmd[2] = (bfa & 0x0100) >> 8;
    write_buffer_cmd[3] = 0xFF & (bfa);

    memcpy(write_buffer_cmd+4, data, sz);

    static uint8_t serno_response[300] = { 0}; 

    last_written_crc = calc_crc32(data, sz);
    dbgprint("About to write data with crc32: 0x%08X\r\n", last_written_crc);
    return spi_send_and_receive(AT45D_BUFFER1_WRITE_CMD, write_buffer_cmd, serno_response, 4+sz, 0);
}

int at45d_read_status_reg(int busy_wait) {
    flash_ctl.status_busy_wait = busy_wait;
    static uint8_t write_buffer_cmd[3] = { AT45D_STATUS_REGISTER_READ, 0x00, 0x00};
    static uint8_t serno_response[3] = { 0}; 
    return spi_send_and_receive(AT45D_STATUS_REGISTER_READ, write_buffer_cmd, serno_response, 1, 2);
}


void fill_page_with_pattern(uint8_t* data, int sz, uint8_t ch) {
    for (int i = 0; i < sz; ++i) {
        data[i] = ch;
    }
}


uint8_t bigbuffer[65536];

void fill_with_inc_pattern(uint8_t *buf, int sz) {
    for (int i = 0; i < sz; i+=2) {
        buf[i] =  0xFF & ((i/2) >> 8);
        buf[i+1] = 0xFF & (i/2);
    }
}


void read_big_buffer(uint8_t *buf, int sz) {
    for (int pageid = 0; pageid < sz/256; ++pageid) {
        at45d_main_memory_to_buffer1(pageid);

        while(flash_ctl.cmd) spi_send_receive_handler();
        at45d_read_status_reg(1); // wait until MSB will be zero (status reg)
        while(flash_ctl.cmd) spi_send_receive_handler();

        spi_send_receive_handler();

        static uint8_t page_data[SPI_DATAFLASH_BUF_MAX_SIZE];
        at45d_buffer1_read(page_data, 0, SPI_FLASH_OP_BLOCK_SIZE, 1 /* ignore crc problems */);

        while(flash_ctl.in_progress);
        spi_send_receive_handler();

        uint8_t *page_ptr = page_data+flash_ctl.snd_sz; 
        uint32_t page_crc = page_ptr[SPI_FLASH_OP_BLOCK_SIZE-4];
        page_ptr[SPI_FLASH_OP_BLOCK_SIZE-4] = 0;
        page_crc <<= 8;
        page_crc |= page_ptr[SPI_FLASH_OP_BLOCK_SIZE-3];
        page_ptr[SPI_FLASH_OP_BLOCK_SIZE-3] = 0;
        page_crc <<= 8;
        page_crc |= page_ptr[SPI_FLASH_OP_BLOCK_SIZE-2];
        page_ptr[SPI_FLASH_OP_BLOCK_SIZE-2] = 0;
        page_crc <<= 8;
        page_crc |= page_ptr[SPI_FLASH_OP_BLOCK_SIZE-1];
        page_ptr[SPI_FLASH_OP_BLOCK_SIZE-1] = 0;

        uint32_t page_real_crc = calc_crc32(page_ptr, SPI_FLASH_OP_BLOCK_SIZE);

        if (page_real_crc != page_crc) {
            dbgprint("Invalid crc at: %d\r\n", pageid);
        } else {
            memcpy(buf+pageid*256, page_ptr, 256);
            dbgprint("Got a valid page %d\r\n", pageid);
        }

    }
}


void write_big_buffer_page(uint8_t *buf, int sz, int pageid) {
    static uint8_t page_data[SPI_DATAFLASH_BUF_MAX_SIZE];
    memcpy(page_data, buf+pageid*256, 256);
    memset(page_data+256, 0, SPI_FLASH_OP_BLOCK_SIZE - 256);

    page_data[SPI_FLASH_OP_BLOCK_SIZE-5] = 0xFF & pageid;
    page_data[SPI_FLASH_OP_BLOCK_SIZE-6] = 0xFF & (pageid >> 8);
    uint32_t page_crc = calc_crc32(page_data, SPI_FLASH_OP_BLOCK_SIZE);
    page_data[SPI_FLASH_OP_BLOCK_SIZE-1] = 0xFF & page_crc;
    page_data[SPI_FLASH_OP_BLOCK_SIZE-2] = 0xFF & (page_crc >> 8);
    page_data[SPI_FLASH_OP_BLOCK_SIZE-3] = 0xFF & (page_crc >> 16);
    page_data[SPI_FLASH_OP_BLOCK_SIZE-4] = 0xFF & (page_crc >> 24);

    at45d_write_to_buffer1(page_data, SPI_FLASH_OP_BLOCK_SIZE, 0);

    while(flash_ctl.cmd) spi_send_receive_handler();

    at45d_buffer1_to_main_memory(pageid);
    while(flash_ctl.cmd) spi_send_receive_handler();

    at45d_read_status_reg(1); // wait until MSB will be zero (status reg)
    while(flash_ctl.cmd) spi_send_receive_handler();
}

int main(void) {
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

    ConfigureSPI();


    ROM_IntMasterEnable();


    SSIIntRegister(SSI2_BASE, SSI2IntHandler);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UDMA);

    ROM_uDMAEnable();
    ROM_uDMAControlBaseSet(pui8ControlTable);
    ROM_IntEnable(INT_UDMAERR);


    CRCInit();

    int tick_no = 0;
    //
    // We are finished.  Hang around flashing D1.
    //
    while(1)
    {
        // Turn on D1.
        LEDWrite(CLP_D1, 1);

        // Delay for a bit.
        SysCtlDelay(g_ui32SysClock / 3 / 2);

        // Turn off D1.
        LEDWrite(CLP_D1, 0);

        // Delay for a bit.
        SysCtlDelay(g_ui32SysClock / 10 /3);



        /*
        fill_with_inc_pattern(bigbuffer, sizeof(bigbuffer));

        for (int i = 0; i < 65536/256; ++i) {
            write_big_buffer_page(bigbuffer, sizeof(bigbuffer), i);
        }
        */

        read_big_buffer(bigbuffer, sizeof(bigbuffer));

        dbgprint("Dumping first page\r\n");
        for (int i = 0; i < 256; ++i) {
            dbgprint("%02X ", bigbuffer[i]); 
        }
        dbgprint("\r\n");



        //infinite cycle: stop MCU here
        while(1);

        uint8_t write_buf[SPI_FLASH_OP_BLOCK_SIZE];
        fill_page_with_pattern(write_buf, SPI_FLASH_OP_BLOCK_SIZE, 0xCC);
        at45d_write_to_buffer1(write_buf, SPI_FLASH_OP_BLOCK_SIZE, 0);

        while(flash_ctl.cmd) spi_send_receive_handler();

        at45d_buffer1_to_main_memory(3);

        while(flash_ctl.cmd) spi_send_receive_handler();

        at45d_read_status_reg(1); // wait until MSB will be zero (status reg)
        while(flash_ctl.cmd) spi_send_receive_handler();

        fill_page_with_pattern(write_buf, SPI_FLASH_OP_BLOCK_SIZE, 0x00);
        at45d_write_to_buffer1(write_buf, SPI_FLASH_OP_BLOCK_SIZE, 0);

        dbgprint("Reading first 10 pages\r\n");
        for (uint32_t pageid = 0; pageid < 5; ++pageid) {
            at45d_main_memory_to_buffer1(pageid);

            while(flash_ctl.cmd) spi_send_receive_handler();
            at45d_read_status_reg(1); // wait until MSB will be zero (status reg)
            while(flash_ctl.cmd) spi_send_receive_handler();

            spi_send_receive_handler();
            static uint8_t page_data[SPI_DATAFLASH_BUF_MAX_SIZE];
            at45d_buffer1_read(page_data, 0, SPI_FLASH_OP_BLOCK_SIZE, 1 /* ignore crc problems */);
            while(flash_ctl.in_progress);
            spi_send_receive_handler();

            for (int i = 0; i < SPI_FLASH_OP_BLOCK_SIZE; ++i) {
                dbgprint("%02X ", page_data[flash_ctl.snd_sz+i]); 
            }
            dbgprint("\r\n");

        }




        //infinite cycle: stop MCU here
        while(1);


#if 0

        for (int it = 0; it < 1 /* 3*10 */; ++it) {
            spi_send_receive_handler();

            ++tick_no;

#define SPI_FLASH_OP_BLOCK_SIZE 264

#if 0 // SIMPLE_TEST
            static uint8_t get_serno_cmd[1] = { 0x9F };
            static uint8_t serno_response[6] = { 0, 0, 0, 0, 0, 0 }; 
            spi_send_and_receive(0x9F, get_serno_cmd, serno_response, 1, 6);
#else
            if ( (tick_no % 4)  == 0) {
                static uint8_t get_serno_cmd[1] = { 0x9F };
                static uint8_t serno_response[6] = { 0, 0, 0, 0, 0, 0 }; 
                spi_send_and_receive(0x9F, get_serno_cmd, serno_response, 1, 6);
            } else if ( (tick_no % 4)  == 1) {
                static uint8_t write_buffer_cmd[300];
                for (int i = 0; i < 264/2; ++i) {
                    int out = i + tick_no;
                    write_buffer_cmd[4+i*2] = (out>>8) & 0xFF;
                    write_buffer_cmd[4+i*2+1] = (out) & 0xFF;
                }
                at45d_write_to_buffer1(write_buffer_cmd, SPI_FLASH_OP_BLOCK_SIZE, 0 /*bfa*/);
            } else if ( (tick_no % 4)  == 1) {
               at45d_main_memory_to_buffer1(tick_no);
                /*
                   static uint8_t read_buffer_cmd[4] = { 0x53, 0x00, 0x00, 0x00};
                   static uint8_t serno_response[300] = { 0}; 
                   spi_send_and_receive(0x53, read_buffer_cmd, serno_response, 4, 0);
                   */

                at45d_main_memory_to_buffer1();

            } else {
                static uint8_t read_buffer_cmd[5] = { 0xD4, 0x00, 0x00, 0x00, 0x00 };
                static uint8_t serno_response[300] = { 0}; 
                spi_send_and_receive(0xD4, read_buffer_cmd, serno_response, 5, SPI_FLASH_OP_BLOCK_SIZE);
            }
        }

        UARTprintf("# crc errors: %d\r\n", flash_ctl.crc_err);
#endif

#endif

    }
}
