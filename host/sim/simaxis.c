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
};

simaxis_t *simaxis_create(uint8_t node_id, const motor_params_t *motor) {
    const axis_config_t *cfg = axis_config_for_node(node_id);
    if (!cfg || !motor) return NULL;

    simaxis_t *s = (simaxis_t *)calloc(1, sizeof(*s));
    if (!s) return NULL;

    s->cfg = cfg;
    axis_init(&s->axis, cfg);
    motor_init(&s->motor, motor);
    return s;
}

void simaxis_destroy(simaxis_t *s) { free(s); }

double simaxis_step(simaxis_t *s, int n_ticks, double joint_q, double joint_qd) {
    if (!s || n_ticks <= 0) return 0.0;

    const axis_config_t *cfg = s->cfg;
    const float gear_ratio = s->motor.p.gear_ratio;
    const float gear_efficiency = s->motor.p.gear_efficiency;
    const float omega_motor = (float)(joint_qd * (double)gear_ratio);

    double torque_sum = 0.0;
    long substep_count = 0;

    for (int tick = 0; tick < n_ticks; tick++) {
        double t = (double)tick * (double)AXIS_DT;
        double motor_angle = (joint_q + joint_qd * t) * (double)gear_ratio;
        int32_t encoder_raw = (int32_t)cfg->encoder_sign * motor_encoder_count(&s->motor, motor_angle);

        axis_inputs_t in = {.encoder_raw = encoder_raw, .current_ma = s->current_ma};
        axis_outputs_t out;
        axis_tick(&s->axis, &in, &out);
        s->last_duty = out.duty;

        float applied_duty = (float)cfg->motor_sign * out.duty;

        float current_sum = 0.0f;
        for (int k = 0; k < SIMAXIS_SUBSTEPS; k++) {
            float motor_torque = motor_step(&s->motor, applied_duty, omega_motor, (float)SIMAXIS_SUBSTEP_DT);
            float joint_torque = motor_torque * gear_ratio * gear_efficiency * (float)cfg->motor_sign;
            torque_sum += (double)joint_torque;
            substep_count++;
            current_sum += motor_sensed_current_ma(&s->motor, applied_duty);
        }
        s->current_ma = current_sum / (float)SIMAXIS_SUBSTEPS;
    }

    s->last_torque = substep_count ? torque_sum / (double)substep_count : 0.0;
    return s->last_torque;
}

void simaxis_rx(simaxis_t *s, uint16_t id, uint8_t len, const uint8_t *data) {
    if (!s || len > 8) return;
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
