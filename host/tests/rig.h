#pragma once

/*
 * Shared axis+plant test rig: drives an axis_t against a first-order test
 * plant over simulated time, tracking following-error and position extremes.
 * Used by test_axis_motion.c (Task 6) and the homing tests (Task 7).
 */

#include <math.h>

#include "axis/axis.h"
#include "plant.h"
#include "test_config.h"

typedef struct { axis_t a; plant_t p; float max_err; float max_pos; } rig_t;

/* static inline: this header may be included by test files that don't use
 * every helper, and -Werror would otherwise trip on -Wunused-function. */
static inline void rig_init(rig_t *r, const axis_config_t *cfg) {
    axis_init(&r->a, cfg); plant_init(&r->p, 20000.0f, 0.02f); r->max_err = 0; r->max_pos = -1e9f;
}
static inline void cmd(rig_t *r, uint8_t c) { can_frame_t f; proto_encode_command(&f, 3, c); axis_on_frame(&r->a, &f); }
static inline void sp(rig_t *r, uint8_t kind, int32_t v) { can_frame_t f; proto_encode_setpoint(&f, 3, kind, v); axis_on_frame(&r->a, &f); }
static inline void run_ms(rig_t *r, int ms) {
    for (int i = 0; i < ms; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&r->a, &f); }
        axis_inputs_t in = {plant_count(&r->p), r->p.current_ma};
        axis_outputs_t out; axis_tick(&r->a, &in, &out);
        plant_step(&r->p, out.duty, AXIS_DT);
        float err = fabsf(r->a.sp_pos - (float)r->a.pos);
        if (err > r->max_err) r->max_err = err;
        if ((float)r->p.pos > r->max_pos) r->max_pos = (float)r->p.pos;
        can_frame_t f; while (axis_pop_tx(&r->a, &f)) {}
    }
}
