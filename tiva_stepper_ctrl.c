#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"

#include "generic_stepper.h"
#include "tiva_stepper_ctrl.h"

static uint16_t ssi_read_write(uint32_t reg, uint16_t dir, uint16_t value);

struct drv8711_reg_map
{
    uint16_t reg[8];
};

struct drv8711_reg_map drv8711;

void read_ctlr_registers (void)
{
    uint32_t idx;

    for (idx = 0; idx < 8; idx++)
    {
        drv8711.reg[idx] = ssi_read_write(idx, 1, 0);
    }
}

static void tiva_sm_index (void)
{
    // Generate step pulse
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, (1 << 6));
    SysCtlDelay(10);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, (0));
}

void tiva_sm_set_max_current (void)
{
    volatile uint16_t data;

    // All changes to the default settings will be placed here, then written
    // Lower max current/torque to 1/16th
    drv8711.reg[1] &= ~(0xFF << 0);
    drv8711.reg[1] |= (0x10 << 0);

    // Write it
    ssi_read_write(0x1, 0, drv8711.reg[1]);

    // Verify by re-reading
    data = ssi_read_write(0x1, 1, 0);
    if (data != drv8711.reg[1])
    {
        while(1);
    }
}

static void ssi_init(void)
{
    // Enable SSI2 Clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlDelay(3);



    // Configure GPIOA pin 2 as gpio output (SCS)
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);

    // Enable AFSEL, set DEN and AMSEL for PB4-7
    GPIOPinTypeSSI(GPIO_PORTB_BASE, (GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7));

    // Clear PCTL and assign PB4-7 to SSI2 functions
    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB5_SSI2FSS);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);

    // Set SSI2 registers
    // Disable SSI2 and set to Master
    SSIDisable(SSI2_BASE);

    // Set Clock source to Sysclk and baud rate to 100kHz (SPR = 99; CPDVSR = 8) 1MHz (SPR = 7; CPDVSR = 10)
    // CPH=0;CPO=0 ; Freescale SPI
    // Data size = 16
    SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);
    SSIConfigSetExpClk(SSI2_BASE, 80000000,
                       SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER,
                       1000000, 16);

    // Enable SSI
    SSIEnable(SSI2_BASE);

    return;
}

static uint16_t ssi_read_write(uint32_t reg, uint16_t dir, uint16_t value)
{
    uint16_t data = 0;
    uint32_t read_data = 0;

    // write bit or read bit from direction
    if (dir) {              // read
        data |= (1 << 15);
    } else {                // write
        data &= ~(1 << 15);
    }

    // address bits
    data |= ((reg & 0x7) << 12);

    // value
    data |= (value & 0xFFF);

    // Enable SCS and send data
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, (1 << 2));
    SSIDataPutNonBlocking(SSI2_BASE, (data & 0xFFFF));

    // Wait for transmission to finish
    while(SSIBusy(SSI2_BASE));
    SSIDataGetNonBlocking(SSI2_BASE, &read_data);

    // Turn off SCS
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, 0x0);

    return ((uint16_t)read_data & 0x0FFF);
}

static void tiva_sm_start_motor (struct speed_ramp_data *rd)
{
    // enable driver and allow current to the windings
    drv8711.reg[0] |= (1 << 0);
    ssi_read_write(0x0, 0, drv8711.reg[0]);

    /* Load initial step delay into timer */
    TimerLoadSet(TIMER0_BASE, TIMER_A, rd->step_delay);

    /* Enable timer 0 */
    TimerEnable(TIMER0_BASE, TIMER_A);
}

static void tiva_sm_stop_motor (void)
{
    /* Disable timer 0 */
    TimerDisable(TIMER0_BASE, TIMER_A);

    /* Disable the driver */
    drv8711.reg[0] &= ~(1 << 0);
    ssi_read_write(0x0, 0, drv8711.reg[0]);
}

static void timer0_init(void)
{
    // Enable Timer clock for timer 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    // Disable timer 0
    TimerDisable(TIMER0_BASE, TIMER_A);

    // Configure for 32-bit timer
    // Periodic counter, count down mode
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    // Load default for 60rpm
    TimerLoadSet(TIMER0_BASE, TIMER_A, 416667 - 1);

    // Disable prescaler for 32bit ctr in 16/32 counter
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, 0x0);

    // Unmask timeout interrupts and clear it
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Set timer interrupt priority to 0x4
    // Enable ints in NVIC (Timer 0 is 35 to 19???)
    IntPrioritySet(INT_TIMER0A, 0x80);
    IntEnable(INT_TIMER0A);
}



// init'ing gpios and ssi should be outside of this.  Its function is exclusively to
// initialize the motor driver chip
static void drv8711_init_driver (void)
{
    // Reset nSLEEP and RESET pins - first as low and high respectively
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, (0 << 5));
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, (1 << 5));
    SysCtlDelay(100);

    // then pull them hi and lo, respectively
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, (1 << 5));
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, (0 << 5));

    // Reset step pin to low
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_6, (0 << 6));

    // Read Ctlr registers specific to this driver
    read_ctlr_registers();

    // Modify driver settings like reduced peak current
    tiva_sm_set_max_current();

    return;
}

static void tiva_sm_set_direction (struct speed_ramp_data *rd)
{
    if (rd->dir) {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, (1 << 7));
    } else {
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_7, (0 << 7));
    }
}

static uint32_t tiva_sm_init (void)
{
    // Initialize all required peripherals for stepper driver chip initialization.
    // Then call the chip initialization fn.
    // Enable GPIOB and GPIOA (SCS) with GPIOE for PE5 reset
    // Enable SSI2 Clock
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlDelay(3);

    // Direction pin, index pin and reset pin setup, both from port A
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7));

    // Configure GPIOE pin 5 as gpio output (SLEEP)
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);

    // Initialize timer
    timer0_init();

    // Initialize SSI
    ssi_init();

    // Call the implementation-specific chip driver init function
    drv8711_init_driver();

    return 0;
}

struct sm_ctrl tiva_sm = {
    .name           =   "Tiva-C Stepper",
    .init           =   tiva_sm_init,
    .set_direction  =   tiva_sm_set_direction,
    .start_motor    =   tiva_sm_start_motor,
    .stop_motor     =   tiva_sm_stop_motor,
    .index          =   tiva_sm_index,
};

void Timer0_Handler (void)
{
    struct generic_sm *sm = (struct generic_sm *)tiva_sm.priv;

    /* Implementation-specific clearing of interrupt */
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    /* Implementation-specific reloading of timer counter */
    TimerLoadSet(TIMER0_BASE, TIMER_A, sm->srd->step_delay);

    /* While counter is counting down, call callback function to update ramp and state */
    sm->callback(sm);
}



