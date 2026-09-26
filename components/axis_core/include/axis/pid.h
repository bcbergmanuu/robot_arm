#pragma once

#include <stdint.h>

/*
 * PID controller with anti-windup.
 *
 * Gains are continuous-time:
 *  - kp: proportional gain
 *  - ki: integral gain (1/s)
 *  - kd: derivative gain (s)
 *
 * Clamps:
 *  - out_min, out_max: output saturation limits
 *  - i_min, i_max: integral term limits (in output units)
 */
typedef struct {
    float kp;        /* Proportional gain */
    float ki;        /* Integral gain (1/s) */
    float kd;        /* Derivative gain (s) */
    float out_min;   /* Output minimum */
    float out_max;   /* Output maximum */
    float i_min;     /* Integral term minimum */
    float i_max;     /* Integral term maximum */
} pid_gains_t;

/*
 * PID controller state.
 *
 * Holds gains and internal state (integral accumulator, previous error,
 * derivative initialization flag).
 */
typedef struct {
    pid_gains_t g;       /* Gains */
    float integ;         /* Integral accumulator */
    float prev_err;      /* Previous error (for derivative) */
    int has_prev;        /* Flag: has_prev is valid */
} pidc_t;

/*
 * Initialize PID controller with gains.
 *
 * @param p: pointer to controller state
 * @param g: pointer to gain struct
 */
void pidc_init(pidc_t *p, const pid_gains_t *g);

/*
 * Reset controller state (integral, derivative history).
 *
 * @param p: pointer to controller state
 */
void pidc_reset(pidc_t *p);

/*
 * Update PID controller.
 *
 * Computes output = kp * err + integral_term + kd * d(err)/dt.
 * Implements anti-windup: integral does not grow if it pushes further into saturation.
 *
 * @param p: pointer to controller state
 * @param err: error signal
 * @param dt: time step (seconds)
 * @return: clamped output
 */
float pidc_update(pidc_t *p, float err, float dt);
