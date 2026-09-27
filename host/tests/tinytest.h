#pragma once
#include <math.h>
#include <stdio.h>

static int tt_fail = 0, tt_count = 0;

#define TT_CHECK(cond) do { tt_count++; if (!(cond)) { tt_fail++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define TT_NEAR(a, b, eps) do { double _a = (double)(a), _b = (double)(b); tt_count++; \
    if (fabs(_a - _b) > (double)(eps)) { tt_fail++; \
    printf("  FAIL %s:%d: %s = %g, expected %g (eps %g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(eps)); } } while (0)
#define TT_RUN(fn) do { printf("%s\n", #fn); fn(); } while (0)
#define TT_DONE() (printf("%d checks, %d failed\n", tt_count, tt_fail), tt_fail ? 1 : 0)
