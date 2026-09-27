#pragma once

#include <stdint.h>

/* Starts the 1 kHz control task (gptimer-paced, core 1) and a 1 Hz status-log task.
 * The HAL modules must be initialised first. node_id must have an entry in the axis config table. */
void axis_task_start(uint8_t node_id);
