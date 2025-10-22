#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOSConfig.h"
#include "storage.h"



static const char *tag = "storage";

const char *nvs_pid_keys[setpoints_num] = {"pid_cur", "pid_vel", "pid_pos"};

pid_ctrl_parameter_store pid_stored_values[setpoints_num];

/// @brief position + velocity + current
uint16_t setpoints[setpoints_num];


/// @brief Store values to nvs
/// @param values all values
/// @param keys all keys
/// @param amount amount of key-value pairs to store
/// @return 0 if no errors
int store_nvs(uint64_t *values, const char **keys, size_t amount) {
    nvs_handle_t nvs_handle;
    int err = 0;        
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < amount; i++)  {    
        
        int err = 0;
        ESP_LOGI(tag, "NVS store key:%s, value:%i", keys[i], values[i]);
        err = nvs_set_u64(nvs_handle, keys[i], values[i]);
        if (err != ESP_OK) {
            ESP_LOGE(tag, "nvs write failed");
            return err;
        }     
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Failed to commit NVS changes!");
        return err;
    }
    nvs_close(nvs_handle);
    return err;
}

int read_nvs(uint64_t **values, const char **keys, size_t amount) {
    
    nvs_handle_t nvs_handle_read;
    int err =0;
    
    err = nvs_open("storage", NVS_READONLY, &nvs_handle_read);
    if (err != ESP_OK) {
        ESP_LOGE(tag, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < amount; i++)  {    
        ESP_LOGI(tag, "Reading from NVS");
        err = nvs_get_u64(nvs_handle_read, keys[i], values[i]);
        switch (err) {
            case ESP_OK:
                ESP_LOGI(tag, "read back value" PRIu16, *values[i]);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                ESP_LOGW(tag, "The value is not initialized yet!");
                break;
            default:
                ESP_LOGE(tag, "Error (%s) reading!", esp_err_to_name(err));
                break;
        }    
    }
    nvs_close(nvs_handle_read);

    return err;
}