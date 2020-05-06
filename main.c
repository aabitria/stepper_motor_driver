#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include "dwt.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"


#include "generic_stepper.h"

// call sysctldelay corresponding here
#define CHECK_MSEC          5

#define PRESS_MSEC          15
#define RELEASE_MSEC        100

#define CHECK_MSEC_DELAY    SysCtlDelay(80000000 / 200);

bool debounced_key_press = false;
bool count = false;

void stop_motor (void);
void set_profile (struct motion_profile *prof);

// All units in rad/sec
struct motion_profile test_profile =
{
     .mode  =   QUARTER,
     .dir   =   0,
     .speed =   200,
     .disp  =   9600,     // 200 turns
     .acc   =   300,
     .dec   =   300,
};


void init_button (void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlDelay(3);

    // Set PF4/USR_SW1 as input
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
    GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
}

bool raw_key_pressed (void)
{
    // return status on SW1 or PF4
    //return !(GPIO_PORTF_DATA_R & 0b10000);
    return (!(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4) & (1 << 4)));
}

// Ganssle debouncing - no interrupt
// from http://www.ganssle.com/debouncing-pt2.htm
void debounce_switch(bool *key_changed, bool *key_pressed)
{
    static uint32_t Count;
    uint32_t raw_state;

    *key_changed = false;
    *key_pressed = debounced_key_press;

    raw_state = raw_key_pressed();

    if (raw_state == debounced_key_press)
    {
        // Reset the timer
        if (debounced_key_press)
            Count = RELEASE_MSEC / CHECK_MSEC;
        else
            Count = PRESS_MSEC / CHECK_MSEC;
    }
    else
    {
        // key has changed - wait for new state to become stable
        if (--Count == 0)
        {
            debounced_key_press = raw_state;
            *key_changed = true;
            *key_pressed = debounced_key_press;

            // Reset the timer
            if (debounced_key_press)
                Count = RELEASE_MSEC / CHECK_MSEC;
            else
                Count = PRESS_MSEC / CHECK_MSEC;
        }

    }

}

/**
 * main.c
 */



int main(void)
{
    bool kc = false, kp = false;

    // Set SysClk to 80MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    init_button();

    generic_stepper_init();

    // Enable interrupts
    IntMasterEnable();

    while(1)
    {
        CHECK_MSEC_DELAY
        debounce_switch(&kc, &kp);

        if (kc && kp) {
            if (!generic_stepper_move_ongoing()) {
                /* motor will only start if it's not currently running */
                generic_stepper_start_motor(&test_profile);
            }
        }
    }

    return 0;
}

