#include <stdio.h>
#include "motor_pid.h"
#include "esp_log.h"
#include "driver/gpio.h"   
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "task_handles.h"
#include "motion_control.h"
#include "ads1015.h"


void app_main(void)
{      
      motor_pid_control_init();
      createSystemTasks();
   
      vTaskDelay(portMAX_DELAY);
      //Never call vTaskStartScheduler() in ESP32 applications because the Espressif ESP-IDF startup process starts the FreeRTOS scheduler automatically
}