#pragma once
/* Bench model: one DC motor driving an inertia with viscous + Coulomb friction,
 * everything referred to the motor shaft. Used to fit/validate the motor model
 * against a recorded open-loop step response (see docs/simulator.md). */
#include <stdint.h>
#include "motor_model.h"

typedef struct {
    motor_params_t motor;
    float j_total;     /* kg m^2 at the motor shaft */
    float b_viscous;   /* Nm s/rad at the motor shaft */
    float tau_coulomb; /* Nm at the motor shaft */
} bench_params_t;

typedef struct {
    bench_params_t p;
    motor_t m;
    double angle; /* rad at the motor shaft */
    float omega;  /* rad/s at the motor shaft */
} bench_t;

void bench_init(bench_t *b, const bench_params_t *p);
/* Advance by dt (10 substeps internally); Coulomb friction with stiction. */
void bench_step(bench_t *b, float duty, float dt);
/* Run a whole duty profile. Sample k is recorded, then duty[k] is applied for dt.
 * Returns 0 on success, -1 on invalid arguments. */
int bench_run(const bench_params_t *p, const float *duty, int n, float dt,
              int32_t *pos_out, float *omega_out, float *current_ma_out);
