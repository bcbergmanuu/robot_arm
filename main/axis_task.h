#pragma once

#include "axis/axis_config.h"

/* Starts the 1 kHz control task (gptimer-paced, core 1, stall-guarded) and a 1 Hz status-log task.
 * The HAL modules must be initialised first. */
void axis_task_start(const axis_config_t *cfg);
