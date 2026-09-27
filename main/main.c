#include <stdlib.h>

#include "axis/config_table.h"
#include "axis_task.h"
#include "esp_log.h"
#include "hal_can.h"
#include "hal_current.h"
#include "hal_encoder.h"
#include "hal_motor.h"
#include "sdkconfig.h"

static const char *TAG = "main";

void app_main(void)
{
    const uint8_t node = CONFIG_AXIS_NODE_ID;
    if (axis_config_for_node(node) == NULL) {
        ESP_LOGE(TAG, "no axis config for node %u (CONFIG_AXIS_NODE_ID)", node);
        abort();
    }

    hal_motor_init(); /* first, so the bridge inputs sit at brake while the rest comes up */
    hal_encoder_init();
    hal_current_init();
    hal_can_init();
    axis_task_start(node);
}
