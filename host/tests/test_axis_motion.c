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

static void test_duty_stays_clamped_while_stalled(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    r.p.gain = 0.0f;                                        /* motor does not move: stalled */
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 5000);
    int i;
    for (i = 0; i < 3000 && r.a.state != AXIS_FAULT; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&r.a, &f); }
        axis_inputs_t in = {plant_count(&r.p), r.p.current_ma};
        axis_outputs_t out; axis_tick(&r.a, &in, &out);
        TT_CHECK(fabsf(out.duty) <= TEST_CFG.max_duty + 1e-6f);
        plant_step(&r.p, out.duty, AXIS_DT);
        can_frame_t f; while (axis_pop_tx(&r.a, &f)) {}
    }
    TT_CHECK(i < 3000);                                     /* actually faulted, not just ran out */
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

/* Start-up regression (R14): the encoder is already far from 0 at power-up and a
 * command arrives before the first tick. The core must not see a velocity spike or
 * a stale sp_pos (false FOLLOWING fault). */
static void test_enable_before_first_tick_at_large_encoder_count(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.pos = 15000.0;                                      /* 5x max_following_error away from 0 */
    cmd(&r, PROTO_CMD_ENABLE);                              /* before any axis_tick */
    run_ms(&r, 1);
    TT_NEAR(r.a.vel, 0.0f, 1.0f);                           /* no start-up velocity spike */
    run_ms(&r, 99);
    TT_CHECK(r.a.state == AXIS_READY);
    TT_CHECK(r.a.faults == 0);
    TT_CHECK(fabsf(r.a.duty) < 0.02f);
    TT_NEAR(r.a.pos, 15000, 5);
}

static void test_home_before_first_tick_at_large_encoder_count(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.pos = 15000.0;
    cmd(&r, PROTO_CMD_HOME);
    run_ms(&r, 1);
    TT_NEAR(r.a.vel, 0.0f, 1.0f);
    TT_CHECK(r.a.state == AXIS_HOMING);
    TT_CHECK(fabsf(r.a.duty) <= TEST_CFG.vel_ff * TEST_CFG.home_vel + 0.01f); /* no kick from a bogus velocity */
}

int main(void) {
    TT_RUN(test_velocity_mode_tracks_target);
    TT_RUN(test_velocity_clamped_to_max);
    TT_RUN(test_position_move_settles_without_big_overshoot);
    TT_RUN(test_position_target_clamped_to_soft_limit);
    TT_RUN(test_velocity_jog_stops_at_soft_limit);
    TT_RUN(test_unhomed_position_setpoint_ignored_and_jog_is_slow);
    TT_RUN(test_following_error_fault_when_stalled);
    TT_RUN(test_duty_stays_clamped_while_stalled);
    TT_RUN(test_mode_switch_is_bumpless);
    TT_RUN(test_enable_before_first_tick_at_large_encoder_count);
    TT_RUN(test_home_before_first_tick_at_large_encoder_count);
    return TT_DONE();
}
