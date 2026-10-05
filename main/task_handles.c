
#include <stdio.h>
#include "motor_pid.h"
#include "esp_log.h"
#include "driver/gpio.h"   
#include "task_handles.h"
#include "adc_continuous_read.h"
#include "motor_pid.h"
#include "ads1015.h"
#include "motion_control.h"

static const char *TAG = "task_handles";

TaskHandle_t taskHandle_adc = NULL, taskHandle_pid = NULL, taskHandle_sigmaAdc = NULL;  

void createSystemTasks() {

      
      ESP_ERROR_CHECK(xTaskCreate(
            run_ads1015adc,
            "ads1015",
            4096,
            NULL,
            9,
            NULL            
      )!= pdPASS);
      vTaskDelay(pdMS_TO_TICKS(50));
     


      ESP_ERROR_CHECK(xTaskCreate(
            motor_pid_control,
            "motorPid",
            4096,
            NULL,
            11,
            &taskHandle_pid
      ) != pdPASS);


            ESP_ERROR_CHECK(xTaskCreate(
            continuous_adc_run,
            "sarAdcTask",
            4096,
            NULL,
            12,
            &taskHandle_adc
      ) !=pdPASS);


      // ESP_ERROR_CHECK(xTaskCreate(
      //       motor_position_loop,
      //       "motorPosition",
      //       4096,
      //       NULL,
      //       10,
      //       NULL
      // ) != pdPASS);
 
}