#include <math.h>

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

/* R16: the zero offset jumps when home is found (raw at the stop 37000 -> pos -21000).
 * The velocity window must be re-primed, or the next ticks see a ~-7e6 counts/s spike. */
static void test_no_velocity_spike_when_home_is_set(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.pos = 40000.0; r.p.has_stop = 1; r.p.stop_pos = 37000.0; r.p.stop_dir = -1;
    cmd(&r, PROTO_CMD_HOME);
    int ms = 0;
    while (!r.a.homed && ms < 5000) { run_ms(&r, 1); ms++; }
    TT_CHECK(r.a.homed);
    float max_vel = 0.0f, max_duty = 0.0f;
    for (int i = 0; i < 20; i++) {
        run_ms(&r, 1);
        if (fabsf(r.a.vel) > max_vel) max_vel = fabsf(r.a.vel);
        if (fabsf(r.a.duty) > max_duty) max_duty = fabsf(r.a.duty);
    }
    TT_CHECK(max_vel < 2000.0f);
    TT_CHECK(max_duty < 0.2f);
    run_ms(&r, 2000);
    TT_NEAR(r.a.pos, TEST_CFG.pos_min, 10);
    TT_CHECK(r.a.state == AXIS_READY && r.a.faults == 0);
}

/* R20: an axis already pressed against its home stop when HOME starts draws stall
 * current from the first ms. Stall detection only starts after the settle window,
 * so the overcurrent check must not fault it first. */
static void test_homes_when_already_pressed_against_stop(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.pos = -2500.0; r.p.has_stop = 1; r.p.stop_pos = -2500.0; r.p.stop_dir = -1;
    cmd(&r, PROTO_CMD_HOME);
    run_ms(&r, 5000);
    TT_CHECK(r.a.homed && r.a.state == AXIS_READY);
    TT_CHECK((r.a.faults & AXIS_FAULT_OVERCURRENT) == 0);
    TT_CHECK(r.a.faults == 0);
}

/* R20: the overcurrent suspension during homing is bounded. A jammed axis drawing
 * 2500 mA whose stall is never recognised (current below home_current_ma, encoder
 * still counting as if moving) must still fault once the grace window has passed. */
static void test_jammed_homing_still_faults_overcurrent(void) {
    axis_config_t cfg = TEST_CFG; cfg.home_current_ma = 3000.0f; /* 2500 mA never counts as stall */
    axis_t a; axis_init(&a, &cfg);
    can_frame_t f; proto_encode_command(&f, 3, PROTO_CMD_HOME); axis_on_frame(&a, &f);
    int32_t raw = 0;
    int ms = 0;
    for (; ms < 2000 && a.state == AXIS_HOMING; ms++) {
        if (ms % 50 == 0) { proto_encode_heartbeat(&f, 0); axis_on_frame(&a, &f); }
        raw -= 3; /* 3000 counts/s toward the stop: > 0.2 * home_vel, so no stall evidence */
        axis_inputs_t in = {raw, 2500.0f};
        axis_outputs_t out; axis_tick(&a, &in, &out);
        while (axis_pop_tx(&a, &f)) {}
    }
    TT_CHECK(a.state == AXIS_FAULT && (a.faults & AXIS_FAULT_OVERCURRENT) && !a.homed);
    TT_CHECK(ms <= AXIS_HOME_OC_GRACE_MS + (int)cfg.overcurrent_ms + 2);
}

int main(void) {
    TT_RUN(test_homes_against_stop_and_backs_off);
    TT_RUN(test_homing_times_out_without_stop);
    TT_RUN(test_home_rejected_while_faulted);
    TT_RUN(test_stall_ignored_during_settle_window);
    TT_RUN(test_rehoming_from_ready);
    TT_RUN(test_no_velocity_spike_when_home_is_set);
    TT_RUN(test_homes_when_already_pressed_against_stop);
    TT_RUN(test_jammed_homing_still_faults_overcurrent);
    return TT_DONE();
}
