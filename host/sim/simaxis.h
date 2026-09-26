#pragma once
/* simaxis: one simulated CAN node -- an axis_core axis_t plus a DC motor model,
 * driven by a physics engine that supplies the joint angle/velocity each tick.
 * See docs/simulator.md for the sign conventions and the per-tick pipeline. */
#include <stdint.h>

#include "axis/axis_config.h"
#include "motor_model.h"

typedef struct simaxis simaxis_t;

/* NULL if node_id has no entry in the generated axis_config_table (see
 * axis/config_table.h). motor describes the physical motor + gearbox for this
 * axis (built by the caller from config/arm.yaml, see robotarm.sim.native). */
simaxis_t *simaxis_create(uint8_t node_id, const motor_params_t *motor);

/* Same as simaxis_create, but takes the axis_config_t directly instead of
 * looking it up by node id. Not part of the ctypes API (Python always goes
 * through simaxis_create by node); exposed for host tests that need axis
 * configs -- e.g. non-default motor_sign/encoder_sign -- outside the
 * generated table. simaxis_create is implemented in terms of this. */
simaxis_t *simaxis_create_with_config(const axis_config_t *cfg, const motor_params_t *motor);

void simaxis_destroy(simaxis_t *s);

/* Advance n 1 kHz ticks. joint_q (rad) / joint_qd (rad/s) come from the physics
 * engine and are held constant across the call (q is extrapolated with qd
 * inside, for each tick's simulated encoder read). The encoder is incremental like
 * the real board: it reads 0 at the first simaxis_step, wherever the joint is. Returns the mean joint
 * torque (Nm) produced over the call, for the physics engine to apply. */
double simaxis_step(simaxis_t *s, int n_ticks, double joint_q, double joint_qd);

/* Deliver one CAN frame (len <= 8) to this node's axis_on_frame. */
void simaxis_rx(simaxis_t *s, uint16_t id, uint8_t len, const uint8_t *data);

/* Pop one queued outbound CAN frame into (*id, *len, data[0..*len)). Returns 1
 * if a frame was popped, 0 if the node's tx queue was empty. */
int simaxis_tx(simaxis_t *s, uint16_t *id, uint8_t *len, uint8_t *data);

typedef struct {
    float duty;         /* last commanded duty, as sent to the simulated H-bridge */
    float current_ma;   /* last sensed current (mean over the tick's substeps) */
    float motor_torque; /* mean joint torque (Nm) returned by the last simaxis_step call */
    int32_t pos;         /* axis_t.pos: counts relative to home zero */
    float vel;           /* axis_t.vel: counts/s */
    uint8_t state;       /* axis_state_t */
    uint8_t faults;      /* AXIS_FAULT_* bitmask */
    uint8_t homed;
} simaxis_debug_t;

void simaxis_get_debug(const simaxis_t *s, simaxis_debug_t *out);
