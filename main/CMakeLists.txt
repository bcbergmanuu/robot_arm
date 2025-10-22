idf_component_register(SRCS "motor_pid.c" "ble_connect.c" "gatt_svr.c" "main.c" "storage.c" "gatt_svr.c"
                        PRIV_REQUIRES bt nvs_flash esp_driver_pcnt
                        INCLUDE_DIRS ".")
