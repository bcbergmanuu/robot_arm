#ifndef storage_nvs
#define storage_nvs
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

//position + velocity + current
#define setpoints_num 3

extern const char *nvs_pid_keys[setpoints_num];

typedef union {
    struct {
        uint16_t kp;
        uint16_t ki;
        uint16_t kd;		
    };
    uint64_t bits;
} pid_ctrl_parameter_store;

extern pid_ctrl_parameter_store pid_stored_values[setpoints_num];

int store_nvs(uint64_t *values, const char **keys, size_t amount);
int read_nvs(uint64_t **values, const char **keys, size_t amount);

#ifdef __cplusplus
}
#endif

#endif