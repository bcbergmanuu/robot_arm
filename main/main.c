#include <stdio.h>
#include "esp_log.h"
#include "task_handles.h"

#include "freertos/FreeRTOS.h"

static const char *TAG = "main";

void app_main(void)
{      
      ESP_LOGI(TAG, "booting application...");
      vTaskDelay(pdMS_TO_TICKS(1000));
      createSystemTasks();
      
      vTaskDelay(portMAX_DELAY);
      //Never call vTaskStartScheduler() in ESP32 applications because the Espressif ESP-IDF startup process starts the FreeRTOS scheduler automatically
}