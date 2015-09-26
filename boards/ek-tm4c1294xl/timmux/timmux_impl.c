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

#include "inc/tm4c1294ncpdt.h"

#define TIMMUX_CNT 4
struct timmix_data_t {
    volatile uint32_t at[TIMMUX_CNT*2];
    volatile uint8_t en[TIMMUX_CNT*2];
} static timmix_data;



#define TIMER_GAP_3US 3

volatile uint32_t probe_timer_interval;


inline static void set_low_accuracy_timer(uint32_t delta);
inline static void set_low_accuracy_timer_b(uint32_t delta);

__attribute__((optimize("unroll-loops")))
inline static void timmux_sub(uint32_t val, int spos) {
    for (int i = spos; i < spos+TIMMUX_CNT; ++i) {
            timmix_data.at[i] -= val;
    }
}

__attribute__((optimize("unroll-loops")))
inline static void timmux_add(uint32_t val, int spos) {
    if (val > 65000 || val < TIMER_GAP_3US)
        return;

    for (int i = spos; i < spos+TIMMUX_CNT; ++i) {
            timmix_data.at[i] += val;
    }
}


inline static void handle_timmux_event(int id) {
    switch(id) {
        case 0:
            GPIO_PORTK_DATA_R &= ~(1 << 0);
            //GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0);
            break;
        case 1:
            GPIO_PORTK_DATA_R &= ~(1 << 1);
            //GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0);
            break;
        case 2:
            GPIO_PORTK_DATA_R &= ~(1 << 2);
            //GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0);
            break;
        case 3:
            GPIO_PORTK_DATA_R &= ~(1 << 3);
            //GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_3, 0);
            break;
        case 4:
            GPIO_PORTK_DATA_R &= ~(1 << 6);
            //GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_6, 0);
            break;
        case 5:
            GPIO_PORTK_DATA_R &= ~(1 << 7);
            //GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_7, 0);
            break;
        case 6:
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
            break;
        case 7:
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
            break;
        default:
            break;
    }
}



inline static void timmux_get_next(int spos) {
    uint32_t tim_min = 0xFFFFFFFF;
    int min_id;
    for (int i = spos; i < spos+TIMMUX_CNT; ++i) {
        if (timmix_data.at[i] < 2*TIMER_GAP_3US && timmix_data.en[i]) {
            timmix_data.en[i] = 0;
            timmix_data.at[i] = 0xFFFFFFFF;
            handle_timmux_event(i);
        } else if (timmix_data.en[i] && timmix_data.at[i] && tim_min > timmix_data.at[i]) {
            tim_min = timmix_data.at[i] - (TIMER_GAP_3US -1);
            min_id = i;
        }
    }

    if (tim_min == 0xFFFFFFFF)
        return;

    timmux_sub(tim_min, spos);

    tim_min -= TIMER_GAP_3US;


    if (spos == 0)
        set_low_accuracy_timer(tim_min);
    else
        set_low_accuracy_timer_b(tim_min);
}

inline static void timmux_insert_unlocked(int id, uint32_t tim_val) {
    uint32_t ts = TIMER5_TAR_R; 
    timmux_add(ts, 0);
    timmix_data.en[id] = 1;
    timmix_data.at[id] = tim_val;
    timmux_get_next(0);
}

inline static void timmux_insert_unlocked_b(int id, uint32_t tim_val) {
    timmux_add(TIMER5_TBR_R, TIMMUX_CNT);
    timmix_data.en[id] = 1;
    timmix_data.at[id] = tim_val;
    timmux_get_next(TIMMUX_CNT);
}

inline static void timmux_enable_output(int id) {
    switch(id) {
        case 0:
            GPIO_PORTK_DATA_R |= (1 << 0);
            break;
        case 1:
            GPIO_PORTK_DATA_R |= (1 << 1);
            break;
        case 2:
            GPIO_PORTK_DATA_R |= (1 << 2);
            break;
        case 3:
            GPIO_PORTK_DATA_R |= (1 << 3);
            break;
        case 4:
            GPIO_PORTK_DATA_R |= (1 << 6);
            break;
        case 5:
            GPIO_PORTK_DATA_R |= (1 << 7);
            break;
        case 6:
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, -1);
            break;
        case 7:
            GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, -1);
            break;
        default:
        break;
    }

}

inline static void set_low_accuracy_timer(uint32_t delta) {
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerLoadSet(TIMER5_BASE, TIMER_A, delta);
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER5_BASE, TIMER_A);
}

inline static void set_low_accuracy_timer_b(uint32_t delta) {
    TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT);
    TimerLoadSet(TIMER5_BASE, TIMER_B, delta);
    TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT);
    TimerEnable(TIMER5_BASE, TIMER_B);
}



/// exported funcitons
void setup_timmux_gpio(void) {
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7);

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_6 | GPIO_PIN_7, 0);

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);
}

void setup_low_accuracy_timer(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
    TimerConfigure(TIMER5_BASE, TIMER_CFG_A_ONE_SHOT | TIMER_CFG_B_ONE_SHOT | TIMER_CFG_SPLIT_PAIR);
    TimerPrescaleSet(TIMER5_BASE, TIMER_A , 120-1);
    TimerPrescaleSet(TIMER5_BASE, TIMER_B , 120-1);

    ROM_IntEnable(INT_TIMER5A);
    ROM_IntEnable(INT_TIMER5B);
    ROM_TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
    ROM_TimerIntEnable(TIMER5_BASE, TIMER_TIMB_TIMEOUT);
}

void Timer5TimAMuxHandler(void) {
    TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT); // we will not have next event in 3us
    timmux_get_next(0);
}

void Timer5TimBMuxHandler(void) {
    TimerIntClear(TIMER5_BASE, TIMER_TIMB_TIMEOUT); // we will not have next event in 6us
    timmux_get_next(TIMMUX_CNT);
}


void timmux_insert(int id, uint32_t tim_val) {
    timmux_enable_output(id);
    TimerIntDisable(TIMER5_BASE, TIMER_A);
    if (id < TIMMUX_CNT)
        timmux_insert_unlocked(id, tim_val);
    else
        timmux_insert_unlocked_b(id, tim_val);
    TimerIntEnable(TIMER5_BASE, TIMER_A);
}


