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
#include "driver/pulse_cnt.h"

#include "storage.h"
#include "motor_pid.h"
#include "storage.h"



#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              7
#define BDC_MCPWM_GPIO_B              44

#define BDC_ENCODER_GPIO_A            9
#define BDC_ENCODER_GPIO_B            8
//#define BDC_ENCODER_PCNT_HIGH_LIMIT   10000
//#define BDC_ENCODER_PCNT_LOW_LIMIT    -10000



static int position_setpoint = 0;
static const char *TAG = "motor_pid";

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_controls[pid_control_count];    
    int position;
    int velocity;
    float pid_out[pid_control_count];        
    
} motor_control_context_t;


static void encoder_update(motor_control_context_t *ctx) {    
    int cur_pulse_count = 0;
    pcnt_unit_get_count(ctx->pcnt_encoder, &cur_pulse_count);
    pcnt_unit_clear_count(ctx->pcnt_encoder);
    ctx->position += cur_pulse_count;
    ctx->velocity = cur_pulse_count;    
}

static void pid_position_cb(motor_control_context_t *ctx)
{             
    pid_compute(ctx->pid_controls[position], -ctx->position + position_setpoint, &ctx->pid_out[position]);
}

void print_task(motor_control_context_t *ctx) {

    ESP_LOGI(TAG, "pos_atc: %d, vel_act: %d, pid_pos: %f, pid_vel: %f", ctx->position, ctx->velocity, ctx->pid_out[position], ctx->pid_out[velocity]);    
}

//inner loop -- torque (I) control
static void pid_loop_cb(void *args)
{    
    motor_control_context_t *ctx = (motor_control_context_t *)args;     
    
    encoder_update(ctx);
    
    static int inner_loop_cnt = 0, display_cnt = 0;  
    
    if(inner_loop_cnt > 10) {
        inner_loop_cnt = 0;
        pid_position_cb(ctx);
    }
    if(display_cnt > 1000) {
        display_cnt = 0;
        print_task(ctx);
    }
             
    bdc_motor_handle_t motor = ctx->motor;
            
    pid_compute(ctx->pid_controls[velocity], (ctx->pid_out[position]), &ctx->pid_out[velocity]);         
        
    if(ctx->pid_out[velocity] > 0) {
        bdc_motor_forward(motor);                
    } else {
        bdc_motor_reverse(motor);
    }

    bdc_motor_set_speed(motor, (uint32_t)abs((int)ctx->pid_out[velocity]));

    inner_loop_cnt++; 
    display_cnt++;
}




// static void program_cb(void *args) {    
//     motor_control_context_t *ctx = (motor_control_context_t *)args;     
//     if (ctx->setpoint == 0) {
//         ctx->setpoint = 100000;
//     } else {
//         ctx->setpoint = 0;
//     }
// }



pid_ctrl_block_handle_t pid_ctrls[pid_control_count];

pid_ctrl_parameter_t pid_params[pid_control_count] = { 
    //velocity
    {
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = -BDC_MCPWM_DUTY_TICK_MAX,
        .max_integral = 1000,
        .min_integral = -1000,
    },
    //position
    {        
        .cal_type = PID_CAL_TYPE_INCREMENTAL,        
        .max_integral = 1000,
        .min_integral = -1000,
    }
};

int update_pid_params() {
    int err = 0;
    //nvs get initial pid_values    
    
    uint64_t readvalues[pid_control_count];
    err = read_nvs(readvalues, nvs_pid_keys, pid_control_count);    

    for(int x = 0; x< pid_control_count; x++) {
        pid_ctrl_parameter_store store;
        store.bits = readvalues[x];
        pid_params[x].kp = (float)store.kp/1000;
        pid_params[x].ki = (float)store.ki/1000;
        pid_params[x].kd = (float)store.kd/1000;
        ESP_LOGI(TAG, "wrote values to controller %s: P:%f I:%f D:%f", nvs_pid_keys[x], pid_params[x].kp, pid_params[x].ki, pid_params[x].kd);

        err |= pid_update_parameters(pid_ctrls[x], &pid_params[x]);
    }

    if(err != ESP_OK) {        
        return 0;
    }         

    return err; 
}


int set_velocity(int velocity) {
    pid_params[position].max_output = velocity;
    pid_params[position].min_output = -velocity;
    return 0;
}

int set_position(int new_position) {
    position_setpoint = new_position;
    return 0;
}



int init_motor() {
    
    ESP_ERROR_CHECK(set_velocity(50));
    ESP_ERROR_CHECK(set_position(0));

    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,        
    };

    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit = -10000,
        //.flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    // pcnt_glitch_filter_config_t filter_config = {
    //     .max_glitch_ns = 1000,
    // };
    // ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
    
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,
        .level_gpio_num = BDC_ENCODER_GPIO_B,
    };

    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_B,
        .level_gpio_num = BDC_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));


    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    //ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    //ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    motor_ctrl_ctx.pcnt_encoder = pcnt_unit;

    ESP_LOGI(TAG, "Create PID control blocks");


    pid_ctrl_config_t pid_configs[pid_control_count];
    for(int x= 0; x< pid_control_count; x++) {
        pid_configs[x].init_param = pid_params[x];    

        ESP_ERROR_CHECK(pid_new_control_block(&pid_configs[x], &pid_ctrls[x]));
        motor_ctrl_ctx.pid_controls[x] = pid_ctrls[x];
    }    

    update_pid_params();

    ESP_LOGI(TAG, "timer to do PID position");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_position_loop"
    };
    esp_timer_handle_t pid_loop_timer = NULL;
    
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));    

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));        

    ESP_LOGI(TAG, "Start pid control loops");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, 100));    
    
    return 0;
}