#include <math.h>

#include "axis/axis.h"
#include "rig.h"
#include "tinytest.h"

static void test_velocity_mode_tracks_target(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 5000);
    run_ms(&r, 50);
    TT_CHECK(r.a.sp_vel < 5000.0f);                         /* accel-limited ramp: 50000 * 0.05 = 2500 */
    run_ms(&r, 950);
    TT_NEAR(r.a.vel, 5000.0f, 250.0f);
    TT_CHECK(r.a.state == AXIS_READY);
}

static void test_velocity_clamped_to_max(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 1000000);
    run_ms(&r, 1000);
    TT_NEAR(r.a.sp_vel, 10000.0f, 1.0f);
}

static void test_position_move_settles_without_big_overshoot(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_POSITION, 10000);
    run_ms(&r, 3000);
    TT_NEAR(r.a.pos, 10000, 5);
    TT_CHECK(r.max_pos < 10050.0f);
    TT_CHECK(r.a.state == AXIS_READY);
}

static void test_position_target_clamped_to_soft_limit(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_POSITION, 50000);
    run_ms(&r, 6000);
    TT_NEAR(r.a.pos, 20000, 10);
    TT_CHECK(r.a.state == AXIS_READY && r.a.faults == 0);
}

static void test_velocity_jog_stops_at_soft_limit(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 10000);
    run_ms(&r, 5000);                                       /* keeps pushing into the limit */
    TT_CHECK(r.a.sp_pos <= 20000.5f);
    TT_CHECK(r.max_pos <= 20060.0f);
    TT_CHECK(r.a.state == AXIS_READY && r.a.faults == 0);
    sp(&r, PROTO_SP_VELOCITY, -5000);                       /* can drive back out */
    run_ms(&r, 500);
    TT_CHECK(r.a.pos < 19000);
}

static void test_unhomed_position_setpoint_ignored_and_jog_is_slow(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_POSITION, 10000);
    run_ms(&r, 500);
    TT_NEAR(r.a.pos, 0, 5);
    sp(&r, PROTO_SP_VELOCITY, 10000);
    run_ms(&r, 1000);
    TT_NEAR(r.a.sp_vel, TEST_CFG.home_vel, 1.0f);
}

static void test_following_error_fault_when_stalled(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    r.p.gain = 0.0f;                                        /* motor does not move */
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 5000);
    run_ms(&r, 2000);
    TT_CHECK(r.a.state == AXIS_FAULT && (r.a.faults & AXIS_FAULT_FOLLOWING));
}

static void test_mode_switch_is_bumpless(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_DUTY, 2000);
    run_ms(&r, 300);
    int32_t p0 = r.a.pos;
    sp(&r, PROTO_SP_POSITION, p0);
    run_ms(&r, 5);
    TT_CHECK(fabsf(r.a.sp_pos - (float)p0) < 200.0f);
    run_ms(&r, 1000);
    TT_CHECK(r.a.state == AXIS_READY);
}

int main(void) {
    TT_RUN(test_velocity_mode_tracks_target);
    TT_RUN(test_velocity_clamped_to_max);
    TT_RUN(test_position_move_settles_without_big_overshoot);
    TT_RUN(test_position_target_clamped_to_soft_limit);
    TT_RUN(test_velocity_jog_stops_at_soft_limit);
    TT_RUN(test_unhomed_position_setpoint_ignored_and_jog_is_slow);
    TT_RUN(test_following_error_fault_when_stalled);
    TT_RUN(test_mode_switch_is_bumpless);
    return TT_DONE();
}
