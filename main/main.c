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

TaskHandle_t taskHandle_adc = NULL, taskHandle_pid = NULL;  


SemaphoreHandle_t xSemaphore = NULL;

static void adcpin_isr(void *arg) {     
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;    
      xHigherPriorityTaskWoken = pdFALSE;    
            /* Unblock the task by releasing the semaphore. */
      xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken );            
}

void init_adc_rdy() {
      xSemaphore = xSemaphoreCreateBinary();

      gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_NEGEDGE, 
            .mode = GPIO_MODE_INPUT,        
            .pin_bit_mask = (1ULL << GPIO_NUM_3),
      };
      gpio_config(&io_conf);
      gpio_install_isr_service(0);
      gpio_isr_handler_add(GPIO_NUM_3, adcpin_isr, NULL);
      }

void app_main(void)
{      
      xTaskCreate(
            continuous_adc_run,
            "sarAdcTask",
            4096,
            NULL,
            10,
            &taskHandle_adc
      );

      xTaskCreate(
            init_motor,
            "motorTask",
            4096,
            NULL,
            10,
            &taskHandle_pid
      );      

      xTaskCreate(
            start_motor,
            "motorCtrl",
            4096,
            NULL,
            10,
            NULL
      );       
      
      init_adc_rdy();
      init_adc();      
      
      uint16_t readval = 0;
      uint32_t total = 0;
      int printcounter = 0;
      float average_adc = 0;

      while(true){
             if(xSemaphoreTake(xSemaphore, pdMS_TO_TICKS(1000)) == pdTRUE) {
                read_adc(&readval);
                printcounter ++;
                total += readval;
                if(printcounter > 3300) {      
                    float volt = 0, amp = 0, ampPower;
                    average_adc = total / printcounter;
                    volt = average_adc * 125e-7; //.256mv pp
                    amp = volt * 220; //amp = volt * 220r
                    ampPower = amp / 2.2; // tb9051
                    printf("adcval = %f, volt: %f, ampPower %f \n", average_adc, volt, ampPower);
                    average_adc = 0;
                    total = 0;
                    printcounter = 0;
                }
            }
      }
      vTaskDelay(portMAX_DELAY);
      //Never call vTaskStartScheduler() in ESP32 applications because the Espressif ESP-IDF startup process starts the FreeRTOS scheduler automatically
}