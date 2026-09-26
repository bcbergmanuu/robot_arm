#include "simaxis.h"

#include <stdlib.h>
#include <string.h>

#include "axis/axis.h"
#include "axis/config_table.h"

#define SIMAXIS_SUBSTEPS 10
#define SIMAXIS_SUBSTEP_DT 1e-4 /* 100 us: AXIS_DT (1 ms) / SIMAXIS_SUBSTEPS */

struct simaxis {
    axis_t axis;
    motor_t motor;
    const axis_config_t *cfg;
    float current_ma;     /* sensed current, averaged over the last tick's substeps;
                            * fed as axis_inputs_t.current_ma on the *next* tick */
    float last_duty;      /* out.duty from the most recent axis_tick, for debug */
    double last_torque;   /* mean joint torque returned by the most recent simaxis_step */
    int encoder_started;  /* the incremental encoder counts from the pose at the first step */
    int32_t encoder_count0;
};

simaxis_t *simaxis_create_with_config(const axis_config_t *cfg, const motor_params_t *motor) {
    if (!cfg || !motor) return NULL;

    simaxis_t *s = (simaxis_t *)calloc(1, sizeof(*s));
    if (!s) return NULL;

    s->cfg = cfg;
    axis_init(&s->axis, cfg);
    motor_init(&s->motor, motor);
    return s;
}

simaxis_t *simaxis_create(uint8_t node_id, const motor_params_t *motor) {
    return simaxis_create_with_config(axis_config_for_node(node_id), motor);
}

void simaxis_destroy(simaxis_t *s) { free(s); }

double simaxis_step(simaxis_t *s, int n_ticks, double joint_q, double joint_qd) {
    if (!s || n_ticks <= 0) return 0.0;

    const axis_config_t *cfg = s->cfg;
    const float gear_ratio = s->motor.p.gear_ratio;
    const float gear_efficiency = s->motor.p.gear_efficiency;
    const float motor_sign = (float)cfg->motor_sign;
    /* motor_sign models a motor wired in reverse: axis_tick already pre-multiplies
     * its output duty by motor_sign to compensate (axis.c:257), so the "hardware"
     * duty out.duty *is* the terminal voltage command as-is (no extra flip here).
     * The same wiring reversal flips which physical rotation direction produces
     * which back-EMF sign, so the electrical-frame speed motor_step sees for its
     * back-EMF term also needs the motor_sign flip. Both cancel in the resulting
     * joint torque (motor_sign^2 == 1), which is the point: whatever the wiring,
     * a positive commanded duty must produce positive joint torque. */
    const float omega_electrical = motor_sign * (float)(joint_qd * (double)gear_ratio);

    if (!s->encoder_started) {
        /* Incremental encoder, like the real board: it reads 0 at power-up wherever the
         * joint is; only homing gives positions an absolute meaning. */
        s->encoder_count0 = motor_encoder_count(&s->motor, joint_q * (double)gear_ratio);
        s->encoder_started = 1;
    }

    double torque_sum = 0.0;
    long substep_count = 0;

    for (int tick = 0; tick < n_ticks; tick++) {
        double t = (double)tick * (double)AXIS_DT;
        double motor_angle = (joint_q + joint_qd * t) * (double)gear_ratio;
        int32_t encoder_raw =
            (int32_t)cfg->encoder_sign * (motor_encoder_count(&s->motor, motor_angle) - s->encoder_count0);

        axis_inputs_t in = {.encoder_raw = encoder_raw, .current_ma = s->current_ma};
        axis_outputs_t out;
        axis_tick(&s->axis, &in, &out);
        s->last_duty = out.duty;

        float duty_hw = out.duty; /* terminal voltage command; already carries motor_sign once */

        float current_sum = 0.0f;
        for (int k = 0; k < SIMAXIS_SUBSTEPS; k++) {
            float motor_torque = motor_step(&s->motor, duty_hw, omega_electrical, (float)SIMAXIS_SUBSTEP_DT);
            float joint_torque = motor_sign * motor_torque * gear_ratio * gear_efficiency;
            torque_sum += (double)joint_torque;
            substep_count++;
            current_sum += motor_sensed_current_ma(&s->motor, duty_hw);
        }
        s->current_ma = current_sum / (float)SIMAXIS_SUBSTEPS;
    }

    s->last_torque = substep_count ? torque_sum / (double)substep_count : 0.0;
    return s->last_torque;
}

void simaxis_rx(simaxis_t *s, uint16_t id, uint8_t len, const uint8_t *data) {
    if (!s || len > 8) return; /* oversized frame: not a valid can_frame_t, silently dropped */
    can_frame_t f = {0};
    f.id = id;
    f.len = len;
    memcpy(f.data, data, len);
    axis_on_frame(&s->axis, &f);
}

int simaxis_tx(simaxis_t *s, uint16_t *id, uint8_t *len, uint8_t *data) {
    if (!s) return 0;
    can_frame_t f;
    if (!axis_pop_tx(&s->axis, &f)) return 0;
    *id = f.id;
    *len = f.len;
    memcpy(data, f.data, f.len);
    return 1;
}

void simaxis_get_debug(const simaxis_t *s, simaxis_debug_t *out) {
    if (!s || !out) return;
    out->duty = s->last_duty;
    out->current_ma = s->current_ma;
    out->motor_torque = (float)s->last_torque;
    out->pos = s->axis.pos;
    out->vel = s->axis.vel;
    out->state = (uint8_t)s->axis.state;
    out->faults = s->axis.faults;
    out->homed = s->axis.homed ? 1u : 0u;
}
