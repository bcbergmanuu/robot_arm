#ifndef motor_pid_
#define motor_pid_

#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "pid_ctrl.h"


enum pid_controls {    
    pid_torque,
    pid_velocity,
    pid_position,    
    pid_control_count,
};

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_controls[pid_control_count];  
    
    int position_measured;
    uint16_t torque_measured;
    float position_target, velocity_target, target_torque, pwm_speedvalue, velocity_measured;    
} motor_control_context_t;



typedef struct
{
    int position;
    float velocity;
    float current;

    //debug, remove after
    int pwmspeed;
} controls;

void set_controls(controls c);
void executePid(uint16_t adc_value);
int motor_pid_control_init();

#endif