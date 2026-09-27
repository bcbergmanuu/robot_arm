#pragma once

#include <stdint.h>
#include "axis/pid.h"

#define AXIS_TICK_HZ 1000
#define AXIS_DT (1.0f / (float)AXIS_TICK_HZ)

typedef struct {
    uint8_t node_id;
    const char *name;
    float counts_per_rad;        /* encoder counts per joint radian (positive) */
    float max_duty;              /* motor nominal voltage / supply voltage, <= 1 */
    int32_t pos_min, pos_max;    /* soft limits, counts relative to home zero */
    float max_vel;               /* counts/s */
    float max_acc;               /* counts/s^2 */
    float max_current_ma;        /* sustained above this for overcurrent_ms -> fault */
    uint32_t overcurrent_ms;
    int32_t max_following_error; /* counts */
    pid_gains_t pos_pid;         /* error counts -> velocity correction counts/s */
    pid_gains_t vel_pid;         /* error counts/s -> duty */
    float vel_ff;                /* duty per counts/s (feed-forward) */
    int8_t home_dir;             /* -1 or +1: direction of the homing end stop */
    float home_vel;              /* counts/s, positive */
    int32_t home_pos;            /* joint position (counts) at the end stop */
    float home_current_ma;       /* stall threshold */
    uint32_t home_timeout_ms;
    int8_t motor_sign, encoder_sign;
    uint32_t watchdog_ms;
} axis_config_t;
