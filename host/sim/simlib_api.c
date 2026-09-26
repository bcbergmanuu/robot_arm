/* Exported C API of libsimaxis, loaded from Python via ctypes (robotarm.sim.native). */
#include <stdint.h>

#include "bench.h"

#define SIMAXIS_API __attribute__((visibility("default")))

SIMAXIS_API int simaxis_bench_run(const bench_params_t *p, const float *duty, int n, float dt,
                                  int32_t *pos_out, float *omega_out, float *current_ma_out) {
    return bench_run(p, duty, n, dt, pos_out, omega_out, current_ma_out);
}
