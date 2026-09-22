#ifndef motor_pid_
#define motor_pid_

void init_motor(void *arg);
int update_pid_params();



enum pid_controls {    
    pid_velocity,
    pid_position,
    pid_control_count,
};


#endif