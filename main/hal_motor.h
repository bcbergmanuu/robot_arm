#pragma once

#include <stdbool.h>

/* TB9051 H-bridge driven by two MCPWM generators (PWM1/PWM2 inputs). */

void hal_motor_init(void);            /* outputs start at duty 0 (brake) */
void hal_motor_set_duty(float duty);  /* [-1,1], sign = direction, 0 = brake; task context, non-blocking */
void hal_motor_trip_from_isr(void);   /* ISR/IRAM-safe: brake now and ignore all further duties until reboot */
bool hal_motor_tripped(void);
