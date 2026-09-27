#pragma once

/* Motor current from the TB9051 OCM pin (current mirror -> resistor), ADC continuous mode. */

void hal_current_init(void);   /* starts sampling and a small averaging task */
float hal_current_ma(void);    /* latest ~1 ms average in mA (magnitude; OCM has no sign), lock-free */
