idf_component_register(SRCS "task_handles.c" "ads1015.c" "motion_control.c" "main.c" "motor_pid.c" "storage.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_mcpwm bt nvs_flash esp_driver_pcnt esp_adc esp_driver_i2c esp_driver_gpio esp_timer esp_driver_twai
                    )