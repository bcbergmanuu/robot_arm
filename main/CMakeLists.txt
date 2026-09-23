idf_component_register(SRCS "motion_control.c" "adc_continuous_read.c" "main.c" "motor_pid.c" "storage.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_mcpwm bt nvs_flash esp_driver_pcnt esp_adc
                    )