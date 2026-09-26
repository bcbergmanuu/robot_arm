#include "bench.h"

#include <math.h>

#define BENCH_SUBSTEPS 10
#define STICTION_OMEGA 1e-3f

void bench_init(bench_t *b, const bench_params_t *p) {
    b->p = *p;
    motor_init(&b->m, &p->motor);
    b->angle = 0.0;
    b->omega = 0.0f;
}

static float sgn(float x) { return (x > 0.0f) - (x < 0.0f); }

/* J domega = T - b omega - Tc sign(omega); stuck while |omega| is tiny and the
 * motor torque cannot break the Coulomb friction. Friction may bring the rotor
 * to rest but never reverses it within one substep. */
static void mech_substep(bench_t *b, float torque, float h) {
    const bench_params_t *p = &b->p;
    if (fabsf(b->omega) < STICTION_OMEGA && fabsf(torque) <= p->tau_coulomb) {
        b->omega = 0.0f;
        return;
    }
    float dir = b->omega != 0.0f ? sgn(b->omega) : sgn(torque);
    float accel = (torque - p->b_viscous * b->omega - p->tau_coulomb * dir) / p->j_total;
    float next = b->omega + accel * h;
    if (b->omega != 0.0f && sgn(next) != dir) {
        next = 0.0f;  /* crossing zero: stop here, stiction decides next substep */
    }
    b->angle += 0.5 * (double)(b->omega + next) * (double)h;
    b->omega = next;
}

void bench_step(bench_t *b, float duty, float dt) {
    float h = dt / BENCH_SUBSTEPS;
    for (int k = 0; k < BENCH_SUBSTEPS; k++) {
        float torque = motor_step(&b->m, duty, b->omega, h);
        mech_substep(b, torque, h);
    }
}

int bench_run(const bench_params_t *p, const float *duty, int n, float dt,
              int32_t *pos_out, float *omega_out, float *current_ma_out) {
    if (!p || !duty || n < 0 || !(dt > 0.0f) || !(p->j_total > 0.0f) || !pos_out || !omega_out ||
        !current_ma_out) {
        return -1;
    }
    bench_t b;
    bench_init(&b, p);
    for (int k = 0; k < n; k++) {
        pos_out[k] = motor_encoder_count(&b.m, b.angle);
        omega_out[k] = b.omega;
        current_ma_out[k] = motor_sensed_current_ma(&b.m, duty[k]);
        bench_step(&b, duty[k], dt);
    }
    return 0;
}
