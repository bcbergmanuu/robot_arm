#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"


#include "motor_pid.h"

#include "esp_adc/adc_continuous.h"

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
#define BDC_MCPWM_GPIO_A              7
#define BDC_MCPWM_GPIO_B              8

#define BDC_ENCODER_GPIO_A            1
#define BDC_ENCODER_GPIO_B            9

#define ts_inner 0.001
#define ts_outer 0.005
#define loop_frequency ((int) (1) / (ts_inner))
#define innerouter_ratio ((int) (ts_outer)/(ts_inner))

static const char *TAG = "motor_pid";






// void read_current() {
// // Using the read and parse function
//     adc_continuous_data_t parsed_data[64];  // User specifies maximum number of samples
//     uint32_t num_samples = 0;

//     esp_err_t ret = adc_continuous_read_parse(handle, parsed_data, 64, &num_samples, 1000);
//     if (ret == ESP_OK) {
//         for (int i = 0; i < num_samples; i++) {
//             if (parsed_data[i].valid) {
//                 ESP_LOGI(TAG, "ADC%d, Channel: %d, Value: %"PRIu32,
//                         parsed_data[i].unit + 1,
//                         parsed_data[i].channel,
//                         parsed_data[i].raw_data);
//             }
//         }
//     }
// }



void printer(motor_control_context_t *ctx) {
    ESP_LOGI(TAG, "pos_meas: %-4d vel_meas: %-4d vel_tar: %-4.2f pos_tar: %-4.2f pwm_speed: %-4.2f, torque_tar: %-4.2f, torque_meas: %-8.2f", 
        ctx->position_measured, ctx->velocity_measured, ctx->velocity_target, ctx->position_target, ctx->pwm_speedvalue, ctx->target_torque, ctx->torque_measured);
}



pid_ctrl_block_handle_t pid_ctrls[pid_control_count];

pid_ctrl_parameter_t pid_params[pid_control_count] = { 
    
    //torque, this outputs into pwm
    {        
        .cal_type = PID_CAL_TYPE_POSITIONAL,        
        .max_integral = 1000,
        .min_integral = -1000,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1, 
        .min_output   = -BDC_MCPWM_DUTY_TICK_MAX, 
        
    },
    //velocity
      
    {
        .cal_type = PID_CAL_TYPE_POSITIONAL,        
        .max_output = 3000,  //this outputs into torque request
        .min_output = -3000, //this outputs into torque request
        
    },
    //position
    {  
        .cal_type = PID_CAL_TYPE_POSITIONAL,
        .max_integral = 1000,
        .min_integral = -1000,        
        .max_output = 2000, //this outputs into max velocity
        .min_output = -2000 //this outputs into max velocity
    },
};

// void inform_paramUpdate(enum pid_controls x) {
//     ESP_LOGI(TAG, "wrote values to controller %s: P:%f I:%f D:%f", nvs_pid_keys[x], pid_params[x].kp, pid_params[x].ki, pid_params[x].kd);
// }

int update_pid_params() {
    int err = 0;
    //nvs get initial pid_values    
    
    //uint64_t readvalues[pid_control_count];
    // err = read_nvs(readvalues, nvs_pid_keys, pid_control_count);    
    //pid_vel", "pid_pos"
            
    //for(int x = 0; x< pid_control_count; x++) {
        //pid_ctrl_parameter_store store;
        //store.bits = readvalues[x];
    
    pid_params[pid_velocity].kp = 0.0251991803981781; //(float)store.kp/1000;
    pid_params[pid_velocity].ki = 1.23197084778611*ts_inner; //(float)store.ki/1000;
    pid_params[pid_velocity].kd = 0; //(float)store.kd/1000;
    
    //inform_paramUpdate(pid_velocity);
    err |= pid_update_parameters(pid_ctrls[pid_velocity], &pid_params[pid_velocity]);

    
    pid_params[pid_position].kp = 34.0751495621794;//(float)store.kp/1000;
    pid_params[pid_position].ki = 0.00614763518537804*ts_outer;//(float)store.ki/1000;
    pid_params[pid_position].kd = 0;//(float)store.kd/1000;

    //inform_paramUpdate(pid_position);
    err |= pid_update_parameters(pid_ctrls[pid_position], &pid_params[pid_position]);        

        
    pid_params[pid_torque].kp = 100;//(float)store.kp/1000;
    pid_params[pid_torque].ki = 0*ts_outer;//(float)store.ki/1000;
    pid_params[pid_torque].kd = 0;//(float)store.kd/1000;

    //inform_paramUpdate(pid_position);
    err |= pid_update_parameters(pid_ctrls[pid_torque], &pid_params[pid_torque]);

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

static controls motion_paramters;

void set_controls(controls c) {
      motion_paramters.current = c.current;
      motion_paramters.position = c.position;
}

static float last_adc;
void updateAdcValue(float value) {
    last_adc = value;
}

#define storage_space 1000
static float torque_array[storage_space] = {0};
static int target_pwm_array[storage_space] = {0};
static int time_array[storage_space] = {0};
static int velocity_array[storage_space] = {0};
static int position_array[storage_space] = {0};

static int datapos = 0;

bool motor_measure(motor_control_context_t *ctx) {
   
    switch (datapos++)
    {
        case 0:
            ctx->pwm_speedvalue = 0;
            break;
        case 400:
            ctx->pwm_speedvalue = 400;
            break;
        case 800:
            ctx->pwm_speedvalue = 0;
            break;              
        default:
            break;
    }    
         
    if(datapos < storage_space) {
        time_array[datapos] = 200*datapos;
        torque_array[datapos] = ctx->torque_measured;
        target_pwm_array[datapos] = ctx->pwm_speedvalue;
        velocity_array[datapos] = -ctx->velocity_measured;
        position_array[datapos] = -ctx->position_measured;
        return true;
    }    
    return false;
}

void print_stepresponse() {
    for(int x =0; x< storage_space; x++) {
        printf("%d,%d,%d,%d,%f\n", time_array[x], position_array[x], velocity_array[x], target_pwm_array[x], torque_array[x]);
    }
}

void init_motor(void *arg) {
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
    

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "Forward motor");
    ESP_ERROR_CHECK(bdc_motor_forward(motor));        

    ESP_LOGI(TAG, "Start pid control loops");

           
    //while(1) {
// #ifdef MEASURE_STATE
        
//         vTaskDelay(1000/portTICK_PERIOD_MS);
//         esp_timer_stop(pid_loop_timer);
//         vTaskDelay(100/portTICK_PERIOD_MS);        
//         for(int x = 0; x< storage_space; x++) {
//                 printf("%d %d\n",setpoint_speed_array[x], velocity_array[x]);
//         }     

//         printf("\nprint finished, rows: %d\n", storage_space);
//         vTaskDelay(portMAX_DELAY);        
// #endif
        



        
        // ESP_LOGI(TAG, "init move loop");
        // for(int x = 0; x < 4; x++){
        //     motor_ctrl_ctx.position_target = 0;                
        //     vTaskDelay(4000/portTICK_PERIOD_MS);                
                            
        //     motor_ctrl_ctx.position_target = 10000;                  
            
        //     vTaskDelay(4000/portTICK_PERIOD_MS);
             
        //     setMotorVelocity((x + 1) * 5000);
        //     update_pid_params();   
        // }
    //}
    int cur_pulse_cntr = 0, torque_loop_cntr =0, outer_loop_cnt = 0, display_cnt = 0;                        

    while(1){
        ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY);   
        if(!motor_measure(&motor_ctrl_ctx)) break;
        motor_ctrl_ctx.position_target = motion_paramters.position;
        setMotorVelocity(motion_paramters.velocity);
        motor_ctrl_ctx.torque_measured = last_adc;
        
        //torque
        //pid_compute(motor_ctrl_ctx.pid_controls[pid_torque], motor_ctrl_ctx.target_torque-motor_ctrl_ctx.torque_measured, &motor_ctrl_ctx.pwm_speedvalue);         
        
        if(torque_loop_cntr++ >= 5) {
            //encoder            
            pcnt_unit_get_count(motor_ctrl_ctx.pcnt_encoder, &cur_pulse_cntr);
            pcnt_unit_clear_count(motor_ctrl_ctx.pcnt_encoder);
            motor_ctrl_ctx.position_measured += cur_pulse_cntr;
            motor_ctrl_ctx.velocity_measured = cur_pulse_cntr*1000;    //aanpassen!
            //encoder

            torque_loop_cntr = 0;
            //pid_compute(motor_ctrl_ctx.pid_controls[pid_velocity], -((float)motor_ctrl_ctx.velocity_measured) + motor_ctrl_ctx.velocity_target, &motor_ctrl_ctx.target_torque); 

            
            if(outer_loop_cnt++ >= innerouter_ratio) {
                outer_loop_cnt = 0;
               // pid_compute(motor_ctrl_ctx.pid_controls[pid_position], -((float)motor_ctrl_ctx.position_measured) + motor_ctrl_ctx.position_target, &motor_ctrl_ctx.velocity_target);                        
            }
        }

        if(motor_ctrl_ctx.target_torque > 0) {
            bdc_motor_forward(motor);                
        } else {
            bdc_motor_reverse(motor);
        }

        bdc_motor_set_speed(motor, (uint32_t)abs((int)motor_ctrl_ctx.pwm_speedvalue));                
            
        // if(display_cnt++ > 1000) {
        //     display_cnt = 0;
        //     printer(&motor_ctrl_ctx);        
        // }         
    }
    print_stepresponse();
    vTaskDelay(portMAX_DELAY);
    
}