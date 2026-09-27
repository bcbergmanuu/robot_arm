#pragma once

#include <stdint.h>

/* Quadrature encoder on PCNT, x4 decoding. */

void hal_encoder_init(void);      /* count starts at 0 (incremental encoder) */
int32_t hal_encoder_read(void);   /* accumulated, never wraps at the ±limit */
