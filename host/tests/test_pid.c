#include "axis/pid.h"
#include "tinytest.h"

static pid_gains_t gains(float kp, float ki, float kd) {
    pid_gains_t g = {kp, ki, kd, -1.0f, 1.0f, -0.5f, 0.5f};
    return g;
}

static void test_proportional_only(void) {
    pidc_t p; pid_gains_t g = gains(0.1f, 0, 0); pidc_init(&p, &g);
    TT_NEAR(pidc_update(&p, 2.0f, 0.001f), 0.2f, 1e-6);
    TT_NEAR(pidc_update(&p, -3.0f, 0.001f), -0.3f, 1e-6);
}

static void test_integral_accumulates_with_dt(void) {
    pidc_t p; pid_gains_t g = gains(0, 10.0f, 0); pidc_init(&p, &g);
    float out = 0;
    for (int i = 0; i < 10; i++) out = pidc_update(&p, 1.0f, 0.001f);
    TT_NEAR(out, 0.1f, 1e-5);   /* 10 * 1.0 * 0.001 * 10 */
}

static void test_integral_clamped(void) {
    pidc_t p; pid_gains_t g = gains(0, 100.0f, 0); pidc_init(&p, &g);
    float out = 0;
    for (int i = 0; i < 1000; i++) out = pidc_update(&p, 1.0f, 0.001f);
    TT_NEAR(out, 0.5f, 1e-6);   /* i_max */
}

static void test_output_clamped(void) {
    pidc_t p; pid_gains_t g = gains(10.0f, 0, 0); pidc_init(&p, &g);
    TT_NEAR(pidc_update(&p, 5.0f, 0.001f), 1.0f, 1e-6);
    TT_NEAR(pidc_update(&p, -5.0f, 0.001f), -1.0f, 1e-6);
}

static void test_anti_windup_recovers_quickly(void) {
    pid_gains_t g = {2.0f, 50.0f, 0, -1.0f, 1.0f, -10.0f, 10.0f};
    pidc_t p; pidc_init(&p, &g);
    for (int i = 0; i < 2000; i++) pidc_update(&p, 5.0f, 0.001f);   /* saturated high */
    /* error flips sign: output must leave saturation immediately, not after unwinding 10 units */
    TT_CHECK(pidc_update(&p, -0.6f, 0.001f) < 0.0f);
}

static void test_derivative_skips_first_sample(void) {
    pidc_t p; pid_gains_t g = gains(0, 0, 0.01f); pidc_init(&p, &g);
    TT_NEAR(pidc_update(&p, 1.0f, 0.001f), 0.0f, 1e-6);
    TT_NEAR(pidc_update(&p, 1.02f, 0.001f), 0.2f, 1e-4);   /* 0.01 * 0.02 / 0.001 */
}

static void test_reset_clears_state(void) {
    pidc_t p; pid_gains_t g = gains(0, 10.0f, 0); pidc_init(&p, &g);
    for (int i = 0; i < 10; i++) pidc_update(&p, 1.0f, 0.001f);
    pidc_reset(&p);
    TT_NEAR(pidc_update(&p, 0.0f, 0.001f), 0.0f, 1e-6);
}

int main(void) {
    TT_RUN(test_proportional_only);
    TT_RUN(test_integral_accumulates_with_dt);
    TT_RUN(test_integral_clamped);
    TT_RUN(test_output_clamped);
    TT_RUN(test_anti_windup_recovers_quickly);
    TT_RUN(test_derivative_skips_first_sample);
    TT_RUN(test_reset_clears_state);
    return TT_DONE();
}
