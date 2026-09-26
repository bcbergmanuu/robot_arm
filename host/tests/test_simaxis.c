#include "simaxis.h"

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

static void test_duty_command_moves_joint_positive_within_a_second(void) {
    motor_params_t motor = shoulder_motor();
    simaxis_t *s = simaxis_create(2, &motor);
    TT_CHECK(s != NULL);

    const double j_rotor = 2e-6;
    const double gear_ratio = 370.0;
    const double j_joint = j_rotor * gear_ratio * gear_ratio + 0.05; /* pure inertia, referred to the joint */

    double q = 0.0, qd = 0.0;
    const double dt = 1e-3;

    send_cmd(s, 2, PROTO_CMD_ENABLE);
    send_sp(s, 2, PROTO_SP_DUTY, 3000); /* 0.3 */

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
            if (proto_node(id) != 2) other_node_frames++;
            if (proto_type(id) == PROTO_MSG_STATUS) status_frames++;
        }
    }

    TT_CHECK(qd > 0.0);
    TT_CHECK(q > 0.0);
    TT_CHECK(status_frames > 0);
    TT_CHECK(other_node_frames == 0);

    simaxis_debug_t dbg;
    simaxis_get_debug(s, &dbg);
    TT_CHECK(dbg.state == 2 /* AXIS_READY */);
    TT_CHECK(dbg.vel > 0.0f);

    simaxis_destroy(s);
}

int main(void) {
    TT_RUN(test_unknown_node_returns_null);
    TT_RUN(test_duty_command_moves_joint_positive_within_a_second);
    return TT_DONE();
}
