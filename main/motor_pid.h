#ifndef motor_pid_
#define motor_pid_

int init_motor();
int update_pid_params();



enum pid_controls {    
    velocity,
    position,
    pid_control_count,
};


#endif