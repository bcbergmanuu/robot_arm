#include "hal_encoder.h"

#include "board.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"

/*
 * The hardware counter is only 16 bits; with flags.accum_count and watch points at both limits the
 * driver folds every overflow into a software accumulator, so pcnt_unit_get_count() returns the full
 * accumulated count. Edge/level actions are those of the previous (working) firmware.
 */

#define PCNT_LIMIT 10000

static pcnt_unit_handle_t s_unit;

static void new_channel(int edge_gpio, int level_gpio, pcnt_channel_edge_action_t pos, pcnt_channel_edge_action_t neg)
{
    pcnt_chan_config_t chan_cfg = {.edge_gpio_num = edge_gpio, .level_gpio_num = level_gpio};
    pcnt_channel_handle_t chan = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &chan_cfg, &chan));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan, pos, neg));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
}

void hal_encoder_init(void)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -PCNT_LIMIT,
        .high_limit = PCNT_LIMIT,
        .flags.accum_count = 1,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_unit));

    pcnt_glitch_filter_config_t filter_cfg = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit, &filter_cfg));

    new_channel(BOARD_ENC_A_GPIO, BOARD_ENC_B_GPIO, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    new_channel(BOARD_ENC_B_GPIO, BOARD_ENC_A_GPIO, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, -PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, PCNT_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_unit));
}

int32_t hal_encoder_read(void)
{
    int count = 0;
    pcnt_unit_get_count(s_unit, &count); /* only fails on a NULL handle/pointer */
    return (int32_t)count;
}
