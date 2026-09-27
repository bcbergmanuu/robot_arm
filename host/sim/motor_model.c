#include "motor_model.h"

#include <math.h>

void motor_init(motor_t *m, const motor_params_t *p) {
    m->p = *p;
    m->i = 0.0f;
}

float motor_step(motor_t *m, float duty, float omega_motor, float dt) {
    float v = duty * m->p.supply_v;
    float i_ss = (v - m->p.kt * omega_motor) / m->p.R;
    m->i = i_ss + (m->i - i_ss) * expf(-dt * m->p.R / m->p.L);
    return m->p.kt * m->i;
}

int32_t motor_encoder_count(const motor_t *m, double motor_angle_rad) {
    double counts = motor_angle_rad / (2.0 * M_PI) * (double)m->p.counts_per_motor_rev;
    return (int32_t)floor(counts);
}

float motor_sensed_current_ma(const motor_t *m, float duty) {
    if (fabsf(duty) < 1e-3f) {
        return 0.0f;
    }
    float mv = fabsf(m->i) * m->p.sense_mv_per_a;
    if (mv > m->p.adc_max_mv) {
        mv = m->p.adc_max_mv;
    }
    return mv / m->p.sense_mv_per_a * 1000.0f;
}
