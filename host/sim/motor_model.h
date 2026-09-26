#pragma once
#include <stdint.h>

typedef struct {
    float R, L, kt;             /* ohm, henry, Nm/A (ke == kt in SI) */
    float supply_v;
    float gear_ratio, gear_efficiency;
    float counts_per_motor_rev; /* quadrature counts = 4 x encoder lines */
    float sense_mv_per_a, adc_max_mv;
} motor_params_t;

typedef struct {
    motor_params_t p;
    float i;
} motor_t;

void    motor_init(motor_t *m, const motor_params_t *p);
float   motor_step(motor_t *m, float duty, float omega_motor, float dt);
int32_t motor_encoder_count(const motor_t *m, double motor_angle_rad);
float   motor_sensed_current_ma(const motor_t *m, float duty);
