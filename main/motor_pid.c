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

#define ts_inner 0.001
#define ts_outer 0.005
#define loop_frequency ((int) (1) / (ts_inner))
#define innerouter_ratio ((int) (ts_outer)/(ts_outer))
#define MEASURE_STATE true

static const char *TAG = "motor_pid";

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_controls[pid_control_count];  
    
    int position_measured, velocity_measured;

    float position_target, velocity_target, pwm_speedvalue;            
    
} motor_control_context_t;


void printer(motor_control_context_t *ctx) {

    ESP_LOGI(TAG, "pos_meas: %d, vel_meas: %d, velocity_target: %f, position target: %f, pwm_speed: %f", ctx->position_measured, ctx->velocity_measured, ctx->velocity_target, ctx->position_target, ctx->pwm_speedvalue);
}

#ifdef MEASURE_STATE

#define storage_space 600
static int velocity_array[storage_space] = {0};
static int setpoint_speed_array[storage_space] = {0};

static int datapos = 0;

static void motor_measure(motor_control_context_t *ctx) {
   
    switch (datapos)
    {
        case 0:
            ctx->velocity_target = 0;
            break;
        case 200:
            ctx->velocity_target = 400;
            break;
        case 500:
            ctx->velocity_target = 0;
            break;
        case 550:
            ctx->velocity_target = 400;
            break;       
        case 600:
            ctx->velocity_target = 0;
            break;
        default:
            break;
    }
    

    bdc_motor_set_speed(ctx->motor, (uint32_t)abs((int)ctx->velocity_target));
     
    if(datapos < storage_space) {
        velocity_array[datapos] = -ctx->velocity_measured;
        setpoint_speed_array[datapos] = ctx->velocity_target;
    }

    datapos++;    
}
#endif

static void pid_loop_cb(void *args)
{        
    motor_control_context_t *ctx = (motor_control_context_t *)args;    
    
    static int outer_loop_cnt = 0, display_cnt = 0;

    

    //encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(ctx->pcnt_encoder, &cur_pulse_count);
    pcnt_unit_clear_count(ctx->pcnt_encoder);
    ctx->position_measured += cur_pulse_count;
    ctx->velocity_measured = cur_pulse_count*1000;    
    //encoder
   
             
    bdc_motor_handle_t motor = ctx->motor;

#ifdef MEASURE_STATE
    motor_measure(args);
    return; 
#endif
            
    pid_compute(ctx->pid_controls[pid_velocity], -((float)ctx->velocity_measured) + ctx->velocity_target, &ctx->pwm_speedvalue); 
        
    if(ctx->pwm_speedvalue < 0) {
        bdc_motor_forward(motor);                
    } else {
        bdc_motor_reverse(motor);
    }

    bdc_motor_set_speed(motor, (uint32_t)abs((int)ctx->pwm_speedvalue));

    if(outer_loop_cnt >= innerouter_ratio -1) {
        outer_loop_cnt = 0;
        pid_compute(ctx->pid_controls[pid_position], -((float)ctx->position_measured) + ctx->position_target, &ctx->velocity_target);
        
    }
    if(display_cnt > 500) {
        display_cnt = 0;
        printer(ctx);        
    }
    outer_loop_cnt++; 
    display_cnt++;
}

pid_ctrl_block_handle_t pid_ctrls[pid_control_count];

pid_ctrl_parameter_t pid_params[pid_control_count] = { 
    //velocity
    {
        .cal_type = PID_CAL_TYPE_POSITIONAL,        
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = -BDC_MCPWM_DUTY_TICK_MAX,
    },
    //position
    {        
        .cal_type = PID_CAL_TYPE_POSITIONAL,        
        .max_integral = 1000,
        .min_integral = -1000,
        .max_output = 20000,  //adjustable for speed
        .min_output = -20000,
    }
};

void inform_paramUpdate(enum pid_controls x) {
    ESP_LOGI(TAG, "wrote values to controller %s: P:%f I:%f D:%f", nvs_pid_keys[x], pid_params[x].kp, pid_params[x].ki, pid_params[x].kd);
}

int update_pid_params() {
    int err = 0;
    //nvs get initial pid_values    
    
    uint64_t readvalues[pid_control_count];
    err = read_nvs(readvalues, nvs_pid_keys, pid_control_count);    
    //pid_vel", "pid_pos"
            
    //for(int x = 0; x< pid_control_count; x++) {
        //pid_ctrl_parameter_store store;
        //store.bits = readvalues[x];
    
    pid_params[pid_velocity].kp = 0.0251991803981781; //(float)store.kp/1000;
    pid_params[pid_velocity].ki = 1.23197084778611*ts_inner; //(float)store.ki/1000;
    pid_params[pid_velocity].kd = 0; //(float)store.kd/1000;
    
    inform_paramUpdate(pid_velocity);
    err |= pid_update_parameters(pid_ctrls[pid_velocity], &pid_params[pid_velocity]);

    
    pid_params[pid_position].kp = 34.0751495621794;//(float)store.kp/1000;
    pid_params[pid_position].ki = 0.00614763518537804*ts_outer;//(float)store.ki/1000;
    pid_params[pid_position].kd = 0;//(float)store.kd/1000;

    inform_paramUpdate(pid_position);
    err |= pid_update_parameters(pid_ctrls[pid_position], &pid_params[pid_position]);
        
    //}

    if(err != ESP_OK) {        
        return 0;
    }         

    return err; 
}

void setMotorVelocity(int velocity) {
    pid_params[pid_position].max_output = velocity;
    pid_params[pid_position].min_output = -velocity;
}

int init_motor() {
    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,               
        .position_measured = 0,
        .position_target = 0,
        .velocity_measured = 0,
        .velocity_target = 0,
        .pwm_speedvalue = 0
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
    
#ifdef MEASURE_STATE  
    memset(velocity_array, 0, sizeof(velocity_array));
    datapos = 0;        
    
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, 1000));   //wait to execute 400 times 
#endif
    
    while(1) {
#ifdef MEASURE_STATE
        
        vTaskDelay(1000/portTICK_PERIOD_MS);
        esp_timer_stop(pid_loop_timer);
        vTaskDelay(100/portTICK_PERIOD_MS);        
        for(int x = 0; x< storage_space; x++) {
                printf("%d %d\n",setpoint_speed_array[x], velocity_array[x]);
        }     

        printf("\nprint finished, rows: %d\n", storage_space);
        vTaskDelay(portMAX_DELAY);        
#endif

        for(int x = 0; x < 4; x++){
            motor_ctrl_ctx.position_target = 0;                
            vTaskDelay(4000/portTICK_PERIOD_MS);                
            
                    
            
            
            motor_ctrl_ctx.position_target = 10000;                  
            //ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, 1000));   //wait to execute 400 times 
            
            vTaskDelay(4000/portTICK_PERIOD_MS);
             
            setMotorVelocity((x + 1) * 5000);
            update_pid_params();
        }

        
    }

    
    
    
    return 0;
}