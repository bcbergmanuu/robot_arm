#include <math.h>
#include "bench.h"
#include "tinytest.h"

/* faulhaber_2657cr_12v-class motor on its own shaft, 256 counts/rev */
static bench_params_t base(void) {
    bench_params_t p = {
        .motor = {1.14f, 0.00012f, 0.0191f, 12.0f, 1.0f, 1.0f, 256.0f, 528.0f, 2500.0f},
        .j_total = 5e-6f, .b_viscous = 0.0f, .tau_coulomb = 0.0f,
    };
    return p;
}

static void test_coulomb_above_stall_torque_never_moves(void) {
    bench_params_t p = base();
    p.tau_coulomb = 1.2f * p.motor.kt * p.motor.supply_v / p.motor.R;
    bench_t b; bench_init(&b, &p);
    for (int i = 0; i < 2000; i++) bench_step(&b, 1.0f, 1e-3f);
    TT_CHECK(b.omega == 0.0f);
    TT_CHECK(b.angle == 0.0);
}

static void test_frictionless_steady_speed_is_supply_over_kt(void) {
    bench_params_t p = base();
    bench_t b; bench_init(&b, &p);
    for (int i = 0; i < 2000; i++) bench_step(&b, 1.0f, 1e-3f);
    float expected = p.motor.supply_v / p.motor.kt;
    TT_NEAR(b.omega, expected, 0.01f * expected);
}

static void test_after_switch_off_rotor_stops_and_never_reverses(void) {
    bench_params_t p = base();
    p.b_viscous = 1e-6f;
    p.tau_coulomb = 2e-3f;
    bench_t b; bench_init(&b, &p);
    for (int i = 0; i < 500; i++) bench_step(&b, 1.0f, 1e-3f);
    TT_CHECK(b.omega > 100.0f);
    double prev = b.angle;
    int reversed = 0;
    for (int i = 0; i < 2000; i++) {
        bench_step(&b, 0.0f, 1e-3f);
        if (b.omega < 0.0f || b.angle < prev) reversed = 1;
        prev = b.angle;
    }
    TT_CHECK(!reversed);
    TT_CHECK(b.omega == 0.0f);
}

static void test_bench_run_matches_stepping(void) {
    bench_params_t p = base();
    p.tau_coulomb = 1e-3f;
    enum { N = 300 };
    float duty[N];
    for (int i = 0; i < N; i++) duty[i] = (i >= 50 && i < 200) ? 1.0f : 0.0f;
    int32_t pos[N]; float omega[N], cur[N];
    TT_CHECK(bench_run(&p, duty, N, 2e-4f, pos, omega, cur) == 0);
    bench_t b; bench_init(&b, &p);
    for (int i = 0; i < 150; i++) bench_step(&b, duty[i], 2e-4f);
    TT_CHECK(pos[150] == motor_encoder_count(&b.m, b.angle));
    TT_NEAR(omega[150], b.omega, 1e-3);
    TT_CHECK(pos[0] == 0 && pos[50] == 0 && pos[N - 1] > pos[199]);
    TT_CHECK(cur[150] > 0.0f && cur[N - 1] == 0.0f);
}

int main(void) {
    TT_RUN(test_coulomb_above_stall_torque_never_moves);
    TT_RUN(test_frictionless_steady_speed_is_supply_over_kt);
    TT_RUN(test_after_switch_off_rotor_stops_and_never_reverses);
    TT_RUN(test_bench_run_matches_stepping);
    return TT_DONE();
}
