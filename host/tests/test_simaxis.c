#include "simaxis.h"

#include "axis/config_table.h"
#include "axis/protocol.h"
#include "tinytest.h"

/* shoulder (node 2): faulhaber_2657cr_12v @ 24 V supply, 370:1 gearbox, 64 cpr. */
static motor_params_t shoulder_motor(void) {
    motor_params_t p = {
        .R = 1.14f, .L = 0.00012f, .kt = 0.0191f, .supply_v = 24.0f,
        .gear_ratio = 370.0f, .gear_efficiency = 0.7f,
        .counts_per_motor_rev = 256.0f, .sense_mv_per_a = 528.0f, .adc_max_mv = 2500.0f,
    };
    return p;
}

static void send_cmd(simaxis_t *s, uint8_t node, uint8_t cmd) {
    can_frame_t f;
    proto_encode_command(&f, node, cmd);
    simaxis_rx(s, f.id, f.len, f.data);
}

static void send_sp(simaxis_t *s, uint8_t node, uint8_t kind, int32_t v) {
    can_frame_t f;
    proto_encode_setpoint(&f, node, kind, v);
    simaxis_rx(s, f.id, f.len, f.data);
}

static void send_hb(simaxis_t *s, uint8_t seq) {
    can_frame_t f;
    proto_encode_heartbeat(&f, seq);
    simaxis_rx(s, f.id, f.len, f.data);
}

static void test_unknown_node_returns_null(void) {
    motor_params_t p = shoulder_motor();
    TT_CHECK(simaxis_create(0, &p) == NULL);  /* broadcast, not an axis */
    TT_CHECK(simaxis_create(9, &p) == NULL);  /* out of range */
}

/* ENABLE + DUTY 0.3 against a pure-inertia joint (J_joint = j_rotor*N^2 + 0.05, explicit
 * Euler at 1 ms) for 1 s; heartbeats every 100 ticks. Used both for the baseline shoulder
 * config and (fix round 1) for wiring-sign variants: whatever motor_sign/encoder_sign are
 * configured, a positive DUTY command must still move the joint -- and the axis's own
 * reported position -- in the positive direction. */
static void run_duty_scenario(const axis_config_t *cfg, const motor_params_t *motor,
                              double *out_q, double *out_qd, simaxis_debug_t *out_dbg,
                              int *out_status_frames, int *out_other_node_frames) {
    simaxis_t *s = simaxis_create_with_config(cfg, motor);
    TT_CHECK(s != NULL);

    const double j_rotor = 2e-6;
    const double gear_ratio = 370.0;
    const double j_joint = j_rotor * gear_ratio * gear_ratio + 0.05; /* pure inertia, referred to the joint */

    double q = 0.0, qd = 0.0;
    const double dt = 1e-3;
    uint8_t node = cfg->node_id;

    send_cmd(s, node, PROTO_CMD_ENABLE);
    send_sp(s, node, PROTO_SP_DUTY, 3000); /* 0.3 */

    int status_frames = 0, other_node_frames = 0;
    for (int i = 0; i < 1000; i++) {
        if (i % 100 == 0) send_hb(s, (uint8_t)i);

        double torque = simaxis_step(s, 1, q, qd);
        double qdd = torque / j_joint;
        qd += qdd * dt;
        q += qd * dt;

        uint16_t id;
        uint8_t len, data[8];
        while (simaxis_tx(s, &id, &len, data)) {
            if (proto_node(id) != node) other_node_frames++;
            if (proto_type(id) == PROTO_MSG_STATUS) status_frames++;
        }
    }

    *out_q = q;
    *out_qd = qd;
    simaxis_get_debug(s, out_dbg);
    *out_status_frames = status_frames;
    *out_other_node_frames = other_node_frames;

    simaxis_destroy(s);
}

static void test_duty_command_moves_joint_positive_within_a_second(void) {
    motor_params_t motor = shoulder_motor();
    const axis_config_t *cfg = axis_config_for_node(2);
    TT_CHECK(cfg != NULL);

    double q, qd;
    simaxis_debug_t dbg;
    int status_frames, other_node_frames;
    run_duty_scenario(cfg, &motor, &q, &qd, &dbg, &status_frames, &other_node_frames);

    TT_CHECK(qd > 0.0);
    TT_CHECK(q > 0.0);
    TT_CHECK(status_frames > 0);
    TT_CHECK(other_node_frames == 0);
    TT_CHECK(dbg.state == 2 /* AXIS_READY */);
    TT_CHECK(dbg.vel > 0.0f);
}

/* Fix round 1 (Ruling R12): motor_sign was applied an odd number of times end-to-end
 * (axis_tick's own out.duty*motor_sign, then simaxis re-applying it to the drive voltage,
 * then again to the joint torque), so a motor_sign=-1 axis moved backwards on a positive
 * command. Same bug shape for encoder_sign if it fed back into the wrong place. Run the
 * same scenario with each sign flipped (and both) and require the joint -- and the axis's
 * own position -- to still move positive. */
static void test_wiring_sign_variants_still_move_positive(void) {
    static const struct { int8_t motor_sign, encoder_sign; } variants[] = {
        {-1, 1}, {1, -1}, {-1, -1},
    };

    for (size_t i = 0; i < sizeof(variants) / sizeof(variants[0]); i++) {
        motor_params_t motor = shoulder_motor();
        axis_config_t cfg = *axis_config_for_node(2);
        cfg.motor_sign = variants[i].motor_sign;
        cfg.encoder_sign = variants[i].encoder_sign;

        double q, qd;
        simaxis_debug_t dbg;
        int status_frames, other_node_frames;
        run_duty_scenario(&cfg, &motor, &q, &qd, &dbg, &status_frames, &other_node_frames);

        TT_CHECK(qd > 0.0);
        TT_CHECK(q > 0.0);
        TT_CHECK(dbg.pos > 0);
        TT_CHECK(dbg.state == 2 /* AXIS_READY */);
        TT_CHECK(dbg.vel > 0.0f);
    }
}

/* Ruling R16b: the simulated encoder is incremental like the real board -- it reads 0 at the
 * first simaxis_step wherever the joint is, and counts relative to that pose afterwards. */
static void test_encoder_is_incremental_from_first_step(void) {
    motor_params_t motor = shoulder_motor();
    const float counts_per_rad = 256.0f * 370.0f / 6.2831853f;
    for (int sign = -1; sign <= 1; sign += 2) {
        axis_config_t cfg = *axis_config_for_node(2);
        cfg.encoder_sign = (int8_t)sign;
        simaxis_t *s = simaxis_create_with_config(&cfg, &motor);
        simaxis_debug_t dbg;

        simaxis_step(s, 1, 1.0, 0.0);                /* arm powered up at q = 1 rad */
        simaxis_get_debug(s, &dbg);
        TT_CHECK(dbg.pos == 0);

        simaxis_step(s, 1, 1.1, 0.0);                /* joint moved +0.1 rad */
        simaxis_get_debug(s, &dbg);
        TT_NEAR((float)dbg.pos, 0.1f * counts_per_rad, 2.0f);   /* encoder_sign cancels in pos */
        simaxis_destroy(s);
    }
}

int main(void) {
    TT_RUN(test_unknown_node_returns_null);
    TT_RUN(test_duty_command_moves_joint_positive_within_a_second);
    TT_RUN(test_wiring_sign_variants_still_move_positive);
    TT_RUN(test_encoder_is_incremental_from_first_step);
    return TT_DONE();
}
