#include "axis/axis.h"
#include "test_config.h"
#include "tinytest.h"

static axis_inputs_t in0 = {0, 0.0f};

static void send_cmd(axis_t *a, uint8_t node, uint8_t cmd) { can_frame_t f; proto_encode_command(&f, node, cmd); axis_on_frame(a, &f); }
static void send_sp(axis_t *a, uint8_t kind, int32_t v) { can_frame_t f; proto_encode_setpoint(&f, 3, kind, v); axis_on_frame(a, &f); }
static float tick(axis_t *a, const axis_inputs_t *in) { axis_outputs_t o; axis_tick(a, in, &o); return o.duty; }
static void drain(axis_t *a) { can_frame_t f; while (axis_pop_tx(a, &f)) {} }

static void test_starts_disabled_with_zero_duty(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    TT_CHECK(a.state == AXIS_DISABLED);
    TT_NEAR(tick(&a, &in0), 0.0f, 1e-9);
}

static void test_enable_and_duty_setpoint_clamped(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_READY);
    send_sp(&a, PROTO_SP_DUTY, 5000);
    TT_NEAR(tick(&a, &in0), 0.5f, 1e-6);
    send_sp(&a, PROTO_SP_DUTY, -10000);
    TT_NEAR(tick(&a, &in0), -0.8f, 1e-6);            /* max_duty */
}

static void test_motor_sign_applied(void) {
    axis_config_t cfg = TEST_CFG; cfg.motor_sign = -1;
    axis_t a; axis_init(&a, &cfg);
    send_cmd(&a, 3, PROTO_CMD_ENABLE); send_sp(&a, PROTO_SP_DUTY, 2500);
    TT_NEAR(tick(&a, &in0), -0.25f, 1e-6);
}

static void test_frames_for_other_nodes_ignored(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 4, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_DISABLED);
    send_cmd(&a, PROTO_NODE_BROADCAST, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_READY);
}

static void test_watchdog_trips_without_master(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE); send_sp(&a, PROTO_SP_DUTY, 5000);
    for (int i = 0; i < 200; i++) tick(&a, &in0);
    TT_CHECK(a.state == AXIS_READY);
    tick(&a, &in0);
    TT_CHECK(a.state == AXIS_FAULT && (a.faults & AXIS_FAULT_WATCHDOG));
    TT_NEAR(tick(&a, &in0), 0.0f, 1e-9);
}

static void test_heartbeat_keeps_axis_alive(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    for (int i = 0; i < 2000; i++) {
        if (i % 100 == 0) { can_frame_t f; proto_encode_heartbeat(&f, (uint8_t)i); axis_on_frame(&a, &f); }
        tick(&a, &in0);
    }
    TT_CHECK(a.state == AXIS_READY);
}

static void test_watchdog_ignored_while_disabled(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    for (int i = 0; i < 1000; i++) tick(&a, &in0);
    TT_CHECK(a.state == AXIS_DISABLED && a.faults == 0);
}

static void test_estop_and_clear(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE); send_sp(&a, PROTO_SP_DUTY, 5000);
    can_frame_t f; proto_encode_estop(&f); axis_on_frame(&a, &f);
    TT_CHECK(a.state == AXIS_FAULT && (a.faults & AXIS_FAULT_ESTOP));
    TT_NEAR(tick(&a, &in0), 0.0f, 1e-9);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_FAULT);                  /* cannot enable while faulted */
    send_cmd(&a, 3, PROTO_CMD_CLEAR_FAULT);
    TT_CHECK(a.state == AXIS_DISABLED && a.faults == 0);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_READY);
}

static void test_overcurrent_fault(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    axis_inputs_t hot = {0, 2500.0f};
    for (int i = 0; i < 150; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&a, &f); }
        tick(&a, &hot);
    }
    TT_CHECK(a.state == AXIS_READY);                  /* short spikes tolerated */
    for (int i = 0; i < 100; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&a, &f); }
        tick(&a, &hot);
    }
    TT_CHECK(a.state == AXIS_FAULT && (a.faults & AXIS_FAULT_OVERCURRENT));
}

static void test_status_frames_at_100hz(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    drain(&a);
    axis_inputs_t in = {1234, 321.0f};
    int status = 0, telem = 0;
    for (int i = 0; i < 100; i++) {
        tick(&a, &in);
        can_frame_t f;
        while (axis_pop_tx(&a, &f)) {
            int32_t pos, vel; uint8_t st, fl, fg; int16_t cur;
            if (proto_decode_status(&f, &pos, &st, &fl, &fg)) { status++; TT_CHECK(pos == 1234 && st == AXIS_DISABLED && fg == 0); }
            if (proto_decode_telemetry(&f, &vel, &cur)) { telem++; TT_CHECK(cur == 321); }
            TT_CHECK(proto_node(f.id) == 3);
        }
    }
    TT_CHECK(status == 10 && telem == 10);
}

static void test_tx_queue_drops_oldest_when_full(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    for (int i = 0; i < 1000; i++) tick(&a, &in0);   /* never drained */
    int n = 0; can_frame_t f;
    while (axis_pop_tx(&a, &f)) n++;
    TT_CHECK(n == AXIS_TX_QUEUE_LEN);
}

int main(void) {
    TT_RUN(test_starts_disabled_with_zero_duty);
    TT_RUN(test_enable_and_duty_setpoint_clamped);
    TT_RUN(test_motor_sign_applied);
    TT_RUN(test_frames_for_other_nodes_ignored);
    TT_RUN(test_watchdog_trips_without_master);
    TT_RUN(test_heartbeat_keeps_axis_alive);
    TT_RUN(test_watchdog_ignored_while_disabled);
    TT_RUN(test_estop_and_clear);
    TT_RUN(test_overcurrent_fault);
    TT_RUN(test_status_frames_at_100hz);
    TT_RUN(test_tx_queue_drops_oldest_when_full);
    return TT_DONE();
}
