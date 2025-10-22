/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "ble_connect.h"
#include "bdc_motor.h"
#include "motor_pid.h"

static const char *tag = "main";





void app_main(void)
{    
    ESP_LOGI(tag, "main initiate");
    init_ble();       
    init_motor();

    

}
