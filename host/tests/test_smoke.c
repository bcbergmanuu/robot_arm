#include <string.h>
#include "axis/version.h"
#include "tinytest.h"

static void test_version(void) { TT_CHECK(strcmp(axis_core_version(), "0.1.0") == 0); }

int main(void) { TT_RUN(test_version); return TT_DONE(); }
