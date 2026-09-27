#include "axis/current_sense.h"

#include <math.h>

float axis_motor_current_from_avg(float avg_ma, float duty) {
    float mag = fabsf(duty);
    float i = (mag >= AXIS_CURRENT_MIN_DUTY) ? avg_ma / mag : avg_ma;
    if (!(i > 0.0f)) return 0.0f; /* also maps NaN to 0 */
    return fminf(i, AXIS_CURRENT_MAX_MA);
}
