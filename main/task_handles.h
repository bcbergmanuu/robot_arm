#ifndef _taskhandles_h__
#define _taskhandles_h__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


void createSystemTasks();

extern TaskHandle_t taskHandle_adc, taskHandle_pid;  

#endif

