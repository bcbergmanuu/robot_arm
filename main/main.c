#include <stdio.h>
#include "motor_pid.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "adc_continuous_read.h"

void app_main(void)
{
      xTaskCreate(
            adc_run,
            "current_adc",
            4096,
            NULL,
            10,
            NULL
      );

      xTaskCreate(
            init_motor,
            "current_adc",
            4096,
            NULL,
            10,
            NULL
      );      
}