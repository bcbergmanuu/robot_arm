#include <math.h>

#include "axis/current_sense.h"
#include "tinytest.h"

static void test_divides_by_duty_above_threshold(void) {
    TT_NEAR(axis_motor_current_from_avg(300.0f, 0.3f), 1000.0f, 1e-3);
    TT_NEAR(axis_motor_current_from_avg(300.0f, -0.3f), 1000.0f, 1e-3); /* sign of duty irrelevant */
    TT_NEAR(axis_motor_current_from_avg(100.0f, 0.1f), 1000.0f, 1e-3);  /* threshold inclusive */
    TT_NEAR(axis_motor_current_from_avg(800.0f, 1.0f), 800.0f, 1e-3);
}

static void test_raw_average_below_threshold(void) {
    TT_NEAR(axis_motor_current_from_avg(50.0f, 0.05f), 50.0f, 1e-6);
    TT_NEAR(axis_motor_current_from_avg(50.0f, -0.0999f), 50.0f, 1e-6);
    TT_NEAR(axis_motor_current_from_avg(20.0f, 0.0f), 20.0f, 1e-6);
}

static void test_clamped(void) {
    TT_NEAR(axis_motor_current_from_avg(-5.0f, 0.5f), 0.0f, 1e-6);
    TT_NEAR(axis_motor_current_from_avg(1.0e6f, 0.2f), AXIS_CURRENT_MAX_MA, 1e-3);
    TT_NEAR(axis_motor_current_from_avg(NAN, 0.5f), 0.0f, 1e-6);
}

int main(void) {
    TT_RUN(test_divides_by_duty_above_threshold);
    TT_RUN(test_raw_average_below_threshold);
    TT_RUN(test_clamped);
    return TT_DONE();
}
