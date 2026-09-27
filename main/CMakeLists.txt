idf_component_register(SRCS "main.c" "axis_task.c" "hal_motor.c" "hal_encoder.c" "hal_current.c" "hal_can.c"
                    INCLUDE_DIRS "."
                    REQUIRES axis_core esp_driver_mcpwm esp_driver_pcnt esp_driver_gptimer esp_driver_twai esp_adc esp_timer
                    )
