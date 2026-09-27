#include "hal_motor.h"

#include <math.h>

#include "board.h"
#include "driver/mcpwm_prelude.h"
#include "esp_check.h"

/*
 * MCPWM used directly (not espressif/bdc_motor): both TB9051 inputs are plain PWM generators that go
 * high at timer-zero and low at their own comparator. Direction is chosen by which comparator is
 * non-zero, so a duty update is two compare-register writes and never touches force levels.
 * A compare value of 0 keeps that output low (the compare event wins over TEZ; bdc_motor relies on the
 * same for speed 0), so duty 0 drives PWM1 = PWM2 = low: the TB9051 brake state.
 * Positive duty modulates BOARD_PWM_A_GPIO, as bdc_motor_forward() did in the old firmware.
 * Compare values are shadowed and latched at timer-zero, so both sides switch in the same PWM period.
 */

#define PERIOD_TICKS (BOARD_PWM_RES_HZ / BOARD_PWM_FREQ_HZ) /* 400 ticks at 25 kHz / 10 MHz */

static mcpwm_cmpr_handle_t s_cmp_a, s_cmp_b;

static void setup_generator(mcpwm_oper_handle_t oper, mcpwm_cmpr_handle_t cmp, int gpio)
{
    mcpwm_generator_config_t gen_cfg = {.gen_gpio_num = gpio};
    mcpwm_gen_handle_t gen = NULL;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_cfg, &gen));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, cmp, MCPWM_GEN_ACTION_LOW)));
}

void hal_motor_init(void)
{
    mcpwm_timer_config_t timer_cfg = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = BOARD_PWM_RES_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = PERIOD_TICKS,
    };
    mcpwm_timer_handle_t timer = NULL;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_cfg, &timer));

    mcpwm_operator_config_t oper_cfg = {.group_id = 0};
    mcpwm_oper_handle_t oper = NULL;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_comparator_config_t cmp_cfg = {.flags.update_cmp_on_tez = true};
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &s_cmp_a));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_cfg, &s_cmp_b));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp_a, 0));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmp_b, 0));

    setup_generator(oper, s_cmp_a, BOARD_PWM_A_GPIO);
    setup_generator(oper, s_cmp_b, BOARD_PWM_B_GPIO);

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void hal_motor_set_duty(float duty)
{
    if (!isfinite(duty)) {
        duty = 0.0f;
    }
    float mag = fminf(fabsf(duty), 1.0f);
    uint32_t ticks = (uint32_t)lroundf(mag * (float)PERIOD_TICKS);
    /* Both arguments are always valid (ticks <= period), so the return values carry no information. */
    mcpwm_comparator_set_compare_value(s_cmp_a, duty > 0.0f ? ticks : 0);
    mcpwm_comparator_set_compare_value(s_cmp_b, duty < 0.0f ? ticks : 0);
}
