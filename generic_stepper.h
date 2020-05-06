
#ifndef __GENERIC_STEPPER_H__
#define __GENERIC_STEPPER_H__

#define CONTNS_ROTATION     0xffffffff

#define ALPHA       (2*3.14159 / (48 * 4))              // 2 x pi / spr
#define T0_FREQ     (0.676 * 80000000)


enum step_mode
{
    FULL    =   1,
    HALF    =   2,
    QUARTER =   4,
    EIGHTH  =   8,
    S16TH   =   16,
    S32ND   =   32,
};

enum state
{
    STOP    =   0,
    ACCEL   =   1,
    RUN     =   2,
    DECEL   =   3,
};

struct speed_ramp_data {
    /* Current state in the ramp */
    enum state  state_;

    /* step delay while accelerating - the c[i] */
    uint32_t    step_delay;
    float       c_curr;
    float       c_prev;

    /* These two values will determine the case scenario we have */
    int32_t     accel_lim;
    int32_t     decel_start;
    int32_t     decel_val;
    uint32_t    max_s_lim;
    uint32_t    last_accel_delay;

    /* Counter of accel/decel steps while in those states */
    int32_t     accel_count;
    uint32_t    step_count;

    /* total step count including accel/decel */
    uint32_t    total_step_count;

    /* Min step delay during max speed */
    uint32_t    min_delay;

    /* direction */
    uint32_t    dir;
};

struct motion_profile {
    enum step_mode  mode;
    bool            dir;
    uint32_t        speed;  // in what?  timer value
    uint32_t        disp;   // in steps * mode
    uint32_t        acc;
    uint32_t        dec;
};

struct sm_ctrl {
    char        *name;
    void        *priv;
    uint32_t    (*init)(void);
    void        (*set_direction)(struct speed_ramp_data *rd);
    void        (*start_motor)(struct speed_ramp_data *rd);
    void        (*stop_motor)(void);
    void        (*index)(void);
};


struct generic_sm {
    uint8_t                 sm_status;
    struct speed_ramp_data  *srd;
    struct motion_profile   *curr_profile;
    struct sm_ctrl          *ctrl;

    void                    (*callback)(struct generic_sm *);
};


uint32_t generic_stepper_init (void);
void generic_stepper_start_motor (struct motion_profile *mf);
void generic_stepper_stop_motor (void);
uint8_t generic_stepper_move_ongoing (void);

#endif /* __GENERIC_STEPPER_H__ */
