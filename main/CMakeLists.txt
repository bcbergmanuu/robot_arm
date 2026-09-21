idf_component_register(SRCS "main.c" "motor_pid.c" "storage.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_mcpwm bt nvs_flash esp_driver_pcnt
                    INCLUDE_DIRS "." )