#pragma once

/* board.h -- pinout of the bcbergmanuu/dc-motor-driver PCB (XIAO ESP32-S3, TB9051FTG H-bridge) */

#include "hal/adc_types.h"

#define BOARD_PWM_A_GPIO        7    /* MOT_PWM2 */
#define BOARD_PWM_B_GPIO        8    /* MOT_PWM1 */
#define BOARD_ENC_A_GPIO        1    /* ENCODER_B net */
#define BOARD_ENC_B_GPIO        9    /* ENCODER_A net */
#define BOARD_CURRENT_ADC_UNIT  ADC_UNIT_1
#define BOARD_CURRENT_ADC_CH    ADC_CHANNEL_3   /* GPIO4, MOT_OCM */
#define BOARD_CAN_TX_GPIO       43
#define BOARD_CAN_RX_GPIO       44
#define BOARD_OCC_GPIO          2    /* TB9051 OCC: left unconfigured (see docs/bringup.md) */
#define BOARD_PWM_FREQ_HZ       25000
#define BOARD_PWM_RES_HZ        10000000
#define BOARD_CURRENT_MV_PER_A  528.0f   /* mirrors current_sense.mv_per_a in config/arm.yaml */
