/* Exported C API of libsimaxis, loaded from Python via ctypes (robotarm.sim.native).
 *
 * simaxis_create/destroy/step/rx/tx/get_debug (host/sim/simaxis.h) are exported
 * under their own names -- they aren't wrapped here like simaxis_bench_run below,
 * because those names must match exactly what NativeAxis loads via ctypes.
 * Getting them into this shared library still requires the whole-archive link
 * of libsimcore in host/CMakeLists.txt: a plain link only pulls in the object
 * files an already-linked symbol references, and nothing in this translation
 * unit references simaxis_create et al. by name. */
#include <stdint.h>

#include "bench.h"
#include "simaxis.h"

#define SIMAXIS_API __attribute__((visibility("default")))

SIMAXIS_API int simaxis_bench_run(const bench_params_t *p, const float *duty, int n, float dt,
                                  int32_t *pos_out, float *omega_out, float *current_ma_out) {
    return bench_run(p, duty, n, dt, pos_out, omega_out, current_ma_out);
}
