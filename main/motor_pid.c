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
#define BDC_MCPWM_GPIO_A              44
#define BDC_MCPWM_GPIO_B              7

#define BDC_ENCODER_GPIO_A            8
#define BDC_ENCODER_GPIO_B            9
//#define BDC_ENCODER_PCNT_HIGH_LIMIT   10000
//#define BDC_ENCODER_PCNT_LOW_LIMIT    -10000

#define BDC_PID_LOOP_VELOCITY_PERIOD_MS       2   // calculate the motor speed every x ms
#define BDC_PID_EXPECT_SPEED          30 // expected motor speed, in the pulses counted by the rotary encoder
#define EPSILON_OFFSET                2000


static const char *TAG = "motor_pid";

typedef struct {
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_velocity;
    pid_ctrl_block_handle_t pid_position;    
    int real_pulses;
    int velocity;
    float new_torque;
    float velocity_setpoint;
    int setpoint;
    
} motor_control_context_t;

//inner loop
static void pid_velocity_loop_cb(void *args)
{
    static int last_pulse_count = 0;

    motor_control_context_t *ctx = (motor_control_context_t *)args;            
    bdc_motor_handle_t motor = ctx->motor;

    int cur_pulse_count = 0;
    pcnt_unit_get_count(ctx->pcnt_encoder, &cur_pulse_count);
    ctx->real_pulses = cur_pulse_count + last_pulse_count;
    ctx->velocity = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;
    
    ctx->new_torque = 0;
    
    pid_compute(ctx->pid_velocity, ctx->velocity + ctx->velocity_setpoint, &ctx->new_torque);         
        
    if(ctx->new_torque > 0) {
        bdc_motor_forward(motor);                
    } else {
        bdc_motor_reverse(motor);
    }

    bdc_motor_set_speed(motor, (uint32_t)abs((int)ctx->new_torque));
}

//outer loop
static void pid_positon_loop_cb(void *args)
{
    
    motor_control_context_t *ctx = (motor_control_context_t *)args;     

    // get the result from rotary encoder
  
 
    ctx->velocity_setpoint = 0;
    
     //pid torque(position)
    pid_compute(ctx->pid_position, ctx->real_pulses - ctx->setpoint, &ctx->velocity_setpoint);        
}

static void program_cb(void *args) {    
    motor_control_context_t *ctx = (motor_control_context_t *)args;     
    if (ctx->setpoint == 0) {
        ctx->setpoint = 100000;
    } else {
        ctx->setpoint = 0;
    }
}

enum pid_control_names {
    current = 0,
    velocity = 1,
    position = 2,
};

pid_ctrl_block_handle_t pid_ctrls[setpoints_num];

pid_ctrl_parameter_t pid_current_params[setpoints_num] = { 

    {
        //current
        .kp = 0.5,
        .ki = .2,
        .kd = .1,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = -BDC_MCPWM_DUTY_TICK_MAX,
        .max_integral = 1000,
        .min_integral = -1000,
    },
    {
        //velocity
        .kp = 0.5,
        .ki = .2,
        .kd = .1,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = -BDC_MCPWM_DUTY_TICK_MAX,
        .max_integral = 1000,
        .min_integral = -1000,
    },
    {    
        //position
        .kp = 0.5,
        .ki = .2,
        .kd = .1,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_output   = BDC_PID_EXPECT_SPEED - 1,
        .min_output   = -BDC_PID_EXPECT_SPEED,
        .max_integral = 100,
        .min_integral = -100,
    }
};

pid_ctrl_parameter_t list[4];

int update_pid_params() {
    int err = 0;
    //nvs get initial pid_values    
    uint64_t testarr[3];
    //(pid_ctrl_parameter_store*)pid_stored_values
    err = read_nvs((uint64_t *)testarr, nvs_pid_keys, setpoints_num);
    
    for(int num = 0; num < setpoints_num; num++) {
        pid_current_params[num].kp = pid_stored_values[num].kd;
        pid_current_params[num].ki = pid_stored_values[num].ki;
        pid_current_params[num].kd = pid_stored_values[num].kd;
    }

    for(int pidcontrol = 0; pidcontrol < setpoints_num; pidcontrol++) {
        err |= pid_update_parameters(pid_ctrls[pidcontrol], &pid_current_params[pidcontrol]);    
    }
    return err;
}

void print_task() {

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        // the following logging format is according to the requirement of serial-studio frame format
        // also see the dashboard config file `serial-studio-dashboard.json` for more information
        //#if SERIAL_STUDIO_DEBUG
        // printf("position_actual:\t%d\r\n", motor_ctrl_ctx.real_pulses);
        // printf("torque_setpoint:\t%f \r\n", motor_ctrl_ctx.new_torque);
        // printf("velocity_setpoint:\t%f \r\n", motor_ctrl_ctx.velocity_setpoint);
        // printf("velocity_actual:\t%d \r\n", motor_ctrl_ctx.velocity);
        
        //#endif
    }
}


int init_motor() {

    update_pid_params();

    static motor_control_context_t motor_ctrl_ctx = {
        .pcnt_encoder = NULL,
        .setpoint = 0,
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
        .high_limit = 100000,
        .low_limit = -100000,
        .flags.accum_count = true, // enable counter accumulation
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
        
    pid_ctrl_config_t pid_velocity_config = {
        .init_param = pid_current_params[velocity],
    };
    
    pid_ctrl_config_t pid_position_config = {
        .init_param = pid_current_params[position],
    };

    ESP_ERROR_CHECK(pid_new_control_block(&pid_velocity_config, &pid_ctrls[velocity]));
    motor_ctrl_ctx.pid_velocity = pid_ctrls[velocity];

    

    ESP_ERROR_CHECK(pid_new_control_block(&pid_position_config, &pid_ctrls[position]));
    motor_ctrl_ctx.pid_position = pid_ctrls[position];

    ESP_LOGI(TAG, "timer to do PID velocity");
    const esp_timer_create_args_t periodic_timer_velocity_args = {
        .callback = pid_velocity_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_velocity_loop"
    };

    ESP_LOGI(TAG, "timer to do PID position");
    const esp_timer_create_args_t periodic_timer_position_args = {
        .callback = pid_positon_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_position_loop"
    };
    esp_timer_handle_t pid_loop_velocity_timer = NULL;
    esp_timer_handle_t pid_loop_position_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_position_args, &pid_loop_position_timer));
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_velocity_args, &pid_loop_velocity_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));

    ESP_LOGI(TAG, "Start pid control loops");
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_velocity_timer, BDC_PID_LOOP_VELOCITY_PERIOD_MS * 1000));
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_position_timer, BDC_PID_LOOP_VELOCITY_PERIOD_MS * 10 * 1000));

    const esp_timer_create_args_t runprogram = {
        .callback = program_cb,
        .arg = &motor_ctrl_ctx,
        .name = "program"
    };
    esp_timer_handle_t program_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&runprogram, &program_timer));
    //ESP_ERROR_CHECK(esp_timer_start_periodic(program_timer, 1000 * 1000 * 60));
    
    return 0;
}