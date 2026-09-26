#include <math.h>
#include "motor_model.h"
#include "tinytest.h"

static motor_params_t P = {4.0f, 0.0004f, 0.035f, 24.0f, 100.0f, 0.7f, 512.0f, 528.0f, 2500.0f};

static void test_stall_current_and_torque(void) {
    motor_t m; motor_init(&m, &P);
    float tq = 0; for (int i = 0; i < 100; i++) tq = motor_step(&m, 1.0f, 0.0f, 1e-4f);
    TT_NEAR(m.i, 6.0f, 1e-3);            /* 24 V / 4 ohm */
    TT_NEAR(tq, 0.21f, 1e-4);
}

static void test_no_load_speed_zero_current(void) {
    motor_t m; motor_init(&m, &P);
    float omega = 24.0f / 0.035f;       /* back-EMF equals supply */
    for (int i = 0; i < 100; i++) motor_step(&m, 1.0f, omega, 1e-4f);
    TT_NEAR(m.i, 0.0f, 1e-3);
}

static void test_electrical_time_constant(void) {
    motor_t m; motor_init(&m, &P);
    motor_step(&m, 1.0f, 0.0f, 0.0001f);  /* tau = L/R = 100 us */
    TT_NEAR(m.i, 6.0f * (1.0f - expf(-1.0f)), 1e-3);
}

static void test_large_dt_is_stable(void) {
    motor_t m; motor_init(&m, &P);
    motor_step(&m, 1.0f, 0.0f, 0.01f);
    TT_NEAR(m.i, 6.0f, 1e-3);
}

static void test_brake_current_opposes_motion(void) {
    motor_t m; motor_init(&m, &P);
    for (int i = 0; i < 100; i++) motor_step(&m, 0.0f, 100.0f, 1e-4f);
    TT_CHECK(m.i < 0.0f);
    TT_NEAR(motor_sensed_current_ma(&m, 0.0f), 0.0f, 1e-6);   /* not visible on OCM */
}

static void test_sensed_current_magnitude_and_clip(void) {
    motor_t m; motor_init(&m, &P);
    m.i = -1.0f; TT_NEAR(motor_sensed_current_ma(&m, -0.5f), 1000.0f, 0.5f);
    m.i = 10.0f; TT_NEAR(motor_sensed_current_ma(&m, 1.0f), 2500.0f / 528.0f * 1000.0f, 0.5f);
}

static void test_encoder_quantization(void) {
    motor_t m; motor_init(&m, &P);
    TT_CHECK(motor_encoder_count(&m, 2.0 * M_PI) == 512);
    TT_CHECK(motor_encoder_count(&m, -0.001) == -1);
}

int main(void) {
    TT_RUN(test_stall_current_and_torque);
    TT_RUN(test_no_load_speed_zero_current);
    TT_RUN(test_electrical_time_constant);
    TT_RUN(test_large_dt_is_stable);
    TT_RUN(test_brake_current_opposes_motion);
    TT_RUN(test_sensed_current_magnitude_and_clip);
    TT_RUN(test_encoder_quantization);
    return TT_DONE();
}
