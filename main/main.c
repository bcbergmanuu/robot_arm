#include <stdio.h>
#include "motor_pid.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adc_continuous_read.h"
#include "main.h"
#include "motion_control.h"

TaskHandle_t taskHandle_adc = NULL, taskHandle_pid = NULL;  

void app_main(void)
{
      xTaskCreate(
            adc_run,
            "current_adc",
            4096,
            NULL,
            10,
            &taskHandle_adc
      );

      xTaskCreate(
            init_motor,
            "current_adc",
            4096,
            NULL,
            10,
            &taskHandle_pid
      );      

      xTaskCreate(
            start_motor,
            "motor_ctrl",
            4096,
            NULL,
            10,
            NULL
      ); 
      vTaskDelay(portMAX_DELAY);
      //Never call vTaskStartScheduler() in ESP32 applications because the Espressif ESP-IDF startup process starts the FreeRTOS scheduler automatically
}