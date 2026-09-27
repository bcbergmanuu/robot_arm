#pragma once
#include <math.h>
#include <stdint.h>

/* First-order velocity plant: tau * dv/dt = gain * duty - v. Optional hard stop. */
typedef struct {
    double pos; float vel; float gain, tau;
    int has_stop; double stop_pos; int stop_dir;   /* stop_dir -1: stop below, +1: stop above */
    float current_ma;
} plant_t;

static void plant_init(plant_t *p, float gain, float tau) {
    p->pos = 0; p->vel = 0; p->gain = gain; p->tau = tau; p->has_stop = 0; p->stop_pos = 0; p->stop_dir = 0; p->current_ma = 0;
}

static void plant_step(plant_t *p, float duty, float dt) {
    p->vel += (p->gain * duty - p->vel) * dt / p->tau;
    p->pos += p->vel * dt;
    p->current_ma = fabsf(duty) * 1000.0f;
    if (p->has_stop && ((p->stop_dir < 0 && p->pos <= p->stop_pos) || (p->stop_dir > 0 && p->pos >= p->stop_pos))) {
        p->pos = p->stop_pos; p->vel = 0;
        if (duty * p->stop_dir > 0.05f) p->current_ma = 2500.0f;   /* stalled against the stop */
    }
}

static int32_t plant_count(const plant_t *p) { return (int32_t)floor(p->pos); }
