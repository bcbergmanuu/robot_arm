#pragma once

/*
 * Motor current from the TB9051 OCM average (R31).
 *
 * The OCM pin mirrors the motor current only while the H-bridge is in its PWM on-phase; during
 * the off-phase it reads ~0. The firmware (and the simulator, which models exactly that) averages
 * the sensor over a whole 1 ms control tick, so the average is |i| * |duty|. This converts that
 * average back to the motor-current magnitude, given the duty that was applied during the window:
 *  - |duty| >= AXIS_CURRENT_MIN_DUTY: avg / |duty|
 *  - below that (including duty 0) the division would only amplify ADC noise, so the raw average
 *    is returned unscaled -- it under-reads the true current by up to 10x at small duties.
 * The result is clamped to [0, AXIS_CURRENT_MAX_MA].
 */

#define AXIS_CURRENT_MIN_DUTY 0.1f
#define AXIS_CURRENT_MAX_MA 30000.0f /* well above any max_current_ma; below the int16 telemetry clamp */

float axis_motor_current_from_avg(float avg_ma, float duty);
