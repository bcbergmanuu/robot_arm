#include <stdio.h>
#include "motor_pid.h"
#include "esp_log.h"
#include "driver/gpio.h"   
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "adc_continuous_read.h"
#include "main.h"
#include "motion_control.h"
#include "ads1015.h"

TaskHandle_t taskHandle_adc = NULL, taskHandle_pid = NULL, taskHandle_sigmaAdc = NULL;  

void app_main(void)
{      
      // xTaskCreate(
      //       continuous_adc_run,
      //       "sarAdcTask",
      //       4096,
      //       NULL,
      //       12,
      //       &taskHandle_adc
      // );

      // xTaskCreate(
      //       motor_pid_control,
      //       "motorPid",
      //       4096,
      //       NULL,
      //       11,
      //       &taskHandle_pid
      // );      

      // xTaskCreate(
      //       motor_position_loop,
      //       "motorPosition",
      //       4096,
      //       NULL,
      //       10,
      //       NULL
      // );       

      xTaskCreate(
            run_sarADC,
            "sarADC",
            4096,
            NULL,
            9,
            NULL            
      );      
   
      vTaskDelay(portMAX_DELAY);
      //Never call vTaskStartScheduler() in ESP32 applications because the Espressif ESP-IDF startup process starts the FreeRTOS scheduler automatically
}