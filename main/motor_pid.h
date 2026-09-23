#ifndef motor_pid_
#define motor_pid_

#include "bdc_motor.h"
#include "driver/pulse_cnt.h"
#include "pid_ctrl.h"

void init_motor(void *arg);
int update_pid_params();

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
    
    int position_measured, velocity_measured;

    float position_target, velocity_target, target_torque, pwm_speedvalue, torque_measured;
    
} motor_control_context_t;



typedef struct
{
    int position;
    float velocity;
    float current;
} controls;

void set_controls(controls c);
void updateAdcValue(float value);

#endif