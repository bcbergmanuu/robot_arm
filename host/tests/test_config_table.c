#include "axis/config_table.h"
#include "tinytest.h"

static void test_lookup(void) {
    TT_CHECK(AXIS_CONFIG_COUNT == 6);
    const axis_config_t *c = axis_config_for_node(2);
    TT_CHECK(c != NULL && c->node_id == 2);
    TT_NEAR(c->max_duty, 0.5f, 1e-6);
    TT_CHECK(c->pos_min < 0 && c->pos_max > 0);
    TT_CHECK(c->home_dir == 1 && c->home_pos > c->pos_max);
    TT_CHECK(axis_config_for_node(0) == NULL && axis_config_for_node(7) == NULL);
}

int main(void) { TT_RUN(test_lookup); return TT_DONE(); }
