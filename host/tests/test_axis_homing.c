#include "axis/axis.h"
#include "rig.h"
#include "tinytest.h"

static void test_homes_against_stop_and_backs_off(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.pos = 3000.0; r.p.has_stop = 1; r.p.stop_pos = -2500.0; r.p.stop_dir = -1;  /* home_dir = -1 */
    cmd(&r, PROTO_CMD_HOME);
    TT_CHECK(r.a.state == AXIS_HOMING);
    run_ms(&r, 5000);
    TT_CHECK(r.a.homed && r.a.state == AXIS_READY);
    /* the stop defines home_pos (-21000); soft limit pos_min = -20000 is 1000 counts further in */
    run_ms(&r, 2000);
    TT_NEAR(r.a.pos, TEST_CFG.pos_min, 10);
    TT_NEAR(r.p.pos, -2500.0 + 1000.0, 10);
    TT_CHECK(r.a.faults == 0);
}

static void test_homing_times_out_without_stop(void) {
    axis_config_t cfg = TEST_CFG; cfg.home_timeout_ms = 1000;
    rig_t r; rig_init(&r, &cfg);
    cmd(&r, PROTO_CMD_HOME);
    run_ms(&r, 1100);
    TT_CHECK(r.a.state == AXIS_FAULT && (r.a.faults & AXIS_FAULT_HOMING) && !r.a.homed);
}

static void test_home_rejected_while_faulted(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    can_frame_t f; proto_encode_estop(&f); axis_on_frame(&r.a, &f);
    cmd(&r, PROTO_CMD_HOME);
    TT_CHECK(r.a.state == AXIS_FAULT);
}

static void test_stall_ignored_during_settle_window(void) {
    /* No end stop: the axis free-runs, so velocity is still ramping (well below
     * 0.2*home_vel) for the first several ms - stall_ms must stay 0 throughout
     * the settle window regardless, since AXIS_HOME_SETTLE_MS gates it. */
    rig_t r; rig_init(&r, &TEST_CFG);
    cmd(&r, PROTO_CMD_HOME);
    for (int i = 0; i < AXIS_HOME_SETTLE_MS; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&r.a, &f); }
        axis_inputs_t in = {plant_count(&r.p), r.p.current_ma};
        axis_outputs_t out; axis_tick(&r.a, &in, &out);
        plant_step(&r.p, out.duty, AXIS_DT);
        TT_CHECK(r.a.stall_ms == 0);
        can_frame_t f; while (axis_pop_tx(&r.a, &f)) {}
    }
}

static void test_rehoming_from_ready(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.has_stop = 1; r.p.stop_pos = -2500.0; r.p.stop_dir = -1;
    cmd(&r, PROTO_CMD_HOME); run_ms(&r, 5000);
    TT_CHECK(r.a.homed);
    cmd(&r, PROTO_CMD_HOME);
    TT_CHECK(r.a.state == AXIS_HOMING && !r.a.homed);
    run_ms(&r, 5000);
    TT_CHECK(r.a.homed && r.a.state == AXIS_READY);
}

int main(void) {
    TT_RUN(test_homes_against_stop_and_backs_off);
    TT_RUN(test_homing_times_out_without_stop);
    TT_RUN(test_home_rejected_while_faulted);
    TT_RUN(test_stall_ignored_during_settle_window);
    TT_RUN(test_rehoming_from_ready);
    return TT_DONE();
}
