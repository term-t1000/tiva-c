# Tiva C

A ready-made repository for writing, compiling and flashing code for the TI Tiva C.

Tested on Mac OS X 10.6.8, should work on most Macs and Linuxes.

## Requirements

 - [ARM EABI Toolchain Builder](https://github.com/jsnyder/arm-eabi-toolchain)
 - [lm4tools](https://github.com/utzig/lm4tools)
 - TI Tiva C TM4C123x or TM4C129x series dev board


## Usage

Assuming you're using the Tiva C Connected Launchpad dev board (`ek-tm4c1294xl`):

```bash
$ cd boards/ek-tm4c1294xl/usb_serial_echo
$ make
$ lm4flash gcc/usb_serial_echo.bin
# Great success!
```

## Changes

 - Implemented simple USB CDC serial echo for EK-TM4C1294XL Connected LaunchPad for gcc
 - applied USB stack fixes from [stm32tivc_usb_cdc](https://github.com/olegv142/stm32tivc_usb_cdc)
 - Implemented simple ADC example boards/ek-tm4c1294xl/adc
 - Added frequency meter boards/ek-tm4c1294xl/timers_ic . This is an example for Input capture + Timer overflow handling. It can measure frequences from 0.5 Hz to 100 kHz
 - Implemented crc32 example: boards/ek-tm4c1294xl/crc32

