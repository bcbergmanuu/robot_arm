#pragma once

#include "axis/axis_config.h"

extern const axis_config_t AXIS_CONFIGS[];
extern const unsigned AXIS_CONFIG_COUNT;

const axis_config_t *axis_config_for_node(uint8_t node);
