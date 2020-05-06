#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "mathfw.h"

#include "generic_stepper.h"

struct generic_sm stepper;

/* Ported controller */
extern struct sm_ctrl tiva_sm;

struct sm_ctrl *ctrl = &tiva_sm;

struct speed_ramp_data srd = {0};

void set_speed_ramp (struct motion_profile *prof)
{
    float c;        // temporary float calculation holder
    float accel = (float)prof->acc;
    float decel = (float)prof->dec;
    float spd = (float)prof->speed;
    float step = (float)srd.total_step_count;

    // Compute minimum step delay
    c = (ALPHA * 80000000) / spd;
    srd.min_delay = (uint32_t)(c + 0.5);

    // Compute c0 step delay
    srd.c_prev = 0;
    srd.c_curr = T0_FREQ * root2_s((2 * ALPHA)/spd);

    // Compute accel_lim
    c = (spd * spd) / (ALPHA * accel);
    srd.max_s_lim = (uint32_t)(c + 0.5);

    // Find out steps to reach accel limit
    c = (step * decel) / (decel + accel);
    srd.accel_lim = (uint32_t)(c + 0.5);

    // TODO: handle case when accel_lim is zero

    // get decel_val
    if (srd.accel_lim <= srd.max_s_lim)
    {
        srd.decel_val = srd.accel_lim - srd.total_step_count;
    }
    else
    {
        srd.decel_val = -((float)srd.max_s_lim * accel) / decel;
    }

    srd.decel_start = srd.total_step_count + srd.decel_val;

    // TODO: Handle case where max speed is so low than step delay

    srd.state_ = ACCEL;

    srd.last_accel_delay = 0;
    srd.accel_count = 0;
    srd.step_count = 0;
    srd.step_delay = (uint32_t)(srd.c_curr + 0.5);

    return;
}

void set_profile (struct motion_profile *prof)
{
    stepper.curr_profile = prof;

    // Check for continuous rotation; if finite, multiply with step mode for actual steps
    srd.total_step_count = stepper.curr_profile->disp;
    if (srd.total_step_count != CONTNS_ROTATION)
        srd.total_step_count *= stepper.curr_profile->mode;

    set_speed_ramp(prof);
}

uint8_t generic_stepper_move_ongoing (void)
{
    return stepper.sm_status;
}

void generic_stepper_start_motor (struct motion_profile *mf)
{
    // Setup the profile first
    set_profile(mf);

    // Call ctrl start motor
    stepper.ctrl->start_motor(stepper.srd);

    stepper.sm_status = 1;
}

void generic_stepper_stop_motor (void)
{
    // Start timer
    stepper.ctrl->stop_motor();

    stepper.sm_status = 0;
}

/* Generic layer ramp update */
void generic_stepper_cb (struct generic_sm *gsm)
{
    switch(srd.state_) {

    case ACCEL:
        /* A delay has passed, so move the motor one step & update counters */
        stepper.ctrl->index();
        srd.step_count++;
        srd.accel_count++;
        srd.c_prev = srd.c_curr;
        srd.c_curr = srd.c_prev - ((2 * srd.c_prev) / (4 * srd.accel_count + 1));
        srd.step_delay = (uint32_t)(srd.c_curr + 0.5);

        if (srd.step_count >= srd.decel_start) {
            // Check if we should start deceleration
            srd.accel_count = srd.decel_val;
            srd.state_ = DECEL;
        } else if (srd.step_delay <= srd.min_delay) {
            // Check if we hit max speed
            srd.last_accel_delay = srd.step_delay;
            srd.step_delay = srd.min_delay;
            srd.state_ = RUN;
        }
        break;

    case RUN:
        /* A delay has passed, so move the motor one step & update counters */
        stepper.ctrl->index();
        srd.step_count++;
        srd.step_delay = srd.min_delay;

        if (srd.step_count >= srd.decel_start) {
            // Check if we should start deceleration
            srd.accel_count = srd.decel_val;        // decel_val is negative
            srd.step_delay = srd.last_accel_delay;
            srd.state_ = DECEL;
        }
        break;

    case DECEL:
        /* A delay has passed, so move the motor one step & update counters */
        stepper.ctrl->index();
        srd.step_count++;
        srd.accel_count++;
        srd.c_prev = srd.c_curr;
        srd.c_curr = srd.c_prev - ((2 * srd.c_prev) / (4 * srd.accel_count + 1));
        srd.step_delay = (uint32_t)(srd.c_curr + 0.5);

        if (srd.accel_count >= 0) {
            srd.state_ = STOP;
        }
        break;

    case STOP:
        generic_stepper_stop_motor();
        break;

    default:
        break;
    }

}

uint32_t generic_stepper_init (void)
{
    uint8_t ret = 0;

    memset(&stepper, 0, sizeof(struct generic_sm));
    stepper.srd = &srd;
    stepper.callback = generic_stepper_cb;
    stepper.curr_profile = NULL;
    stepper.ctrl = ctrl;
    ctrl->priv = (void *)&stepper;

    /* Call controller init */
    if (stepper.ctrl->init)
        ret = stepper.ctrl->init();

    return ret;
}
