#pragma once
#include "axis/axis_config.h"

/* Small, round numbers so the tests are easy to reason about. */
static const axis_config_t TEST_CFG = {
    .node_id = 3, .name = "test",
    .counts_per_rad = 1000.0f, .max_duty = 0.8f,
    .pos_min = -20000, .pos_max = 20000,
    .max_vel = 10000.0f, .max_acc = 50000.0f,
    .max_current_ma = 2000.0f, .overcurrent_ms = 200,
    .max_following_error = 3000,
    .pos_pid = {20.0f, 0.0f, 0.0f, -10000.0f, 10000.0f, -2000.0f, 2000.0f},
    .vel_pid = {0.0002f, 0.004f, 0.0f, -0.8f, 0.8f, -0.3f, 0.3f},
    .vel_ff = 1.0f / 20000.0f,
    .home_dir = -1, .home_vel = 2000.0f, .home_pos = -21000,
    .home_current_ma = 1500.0f, .home_timeout_ms = 20000,
    .motor_sign = 1, .encoder_sign = 1, .watchdog_ms = 200,
};
