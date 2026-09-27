#include "axis/pid.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void pidc_init(pidc_t *p, const pid_gains_t *g) { p->g = *g; pidc_reset(p); }

void pidc_reset(pidc_t *p) { p->integ = 0.0f; p->prev_err = 0.0f; p->has_prev = 0; }

float pidc_update(pidc_t *p, float err, float dt) {
    const pid_gains_t *g = &p->g;
    float d = 0.0f;
    if (p->has_prev && dt > 0.0f) d = g->kd * (err - p->prev_err) / dt;
    p->prev_err = err;
    p->has_prev = 1;

    float integ_new = clampf(p->integ + g->ki * err * dt, g->i_min, g->i_max);
    float unclamped = g->kp * err + integ_new + d;
    /* Anti-windup: refuse integral growth that pushes further into saturation. */
    int winding_up = (unclamped > g->out_max && err > 0.0f) || (unclamped < g->out_min && err < 0.0f);
    if (!winding_up) p->integ = integ_new;
    return clampf(g->kp * err + p->integ + d, g->out_min, g->out_max);
}
