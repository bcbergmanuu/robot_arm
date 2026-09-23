#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "bdc_motor.h"
#include "pid_ctrl.h"


#include "motor_pid.h"

static const char *TAG = "motion_control";

controls motor_c;

void start_motor(void *arg) {

    while(1) {
        
        ESP_LOGI(TAG, "init move loop");
        for(int x = 0; x < 4; x++){
            motor_c.velocity = (x + 1) * 5000;
            motor_c.position = 0;        
            set_controls(motor_c);
            ESP_LOGI(TAG, "motor moved to position 0");
            vTaskDelay(4000/portTICK_PERIOD_MS);                
                            
            motor_c.position = 10000;                  
            set_controls(motor_c);
            vTaskDelay(4000/portTICK_PERIOD_MS);        
        }
    }
}