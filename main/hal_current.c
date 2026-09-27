#include "hal_current.h"

#include <stdint.h>

#include "board.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * ADC1 CH3 sampled at 20 kHz by DMA. One conversion frame = 20 samples = 1 ms, so the conversion-done
 * callback fires at 1 kHz and only notifies current_task. The task averages the raw samples of the
 * newest frame, converts that average to mV with the curve-fitting calibration (one conversion per
 * frame, not per sample) and to mA via BOARD_CURRENT_MV_PER_A, and publishes it in a 32-bit float
 * (aligned word stores/loads are atomic on Xtensa, so readers never see a torn value), then bumps a
 * sequence number so consumers can detect a stalled pipeline.
 */

#define SAMPLE_FREQ_HZ 20000
#define SAMPLES_PER_FRAME 20
#define FRAME_BYTES (SAMPLES_PER_FRAME * SOC_ADC_DIGI_RESULT_BYTES)
#define TASK_PRIO (configMAX_PRIORITIES - 3) /* below the axis task */
#define TASK_CORE 0                          /* keep it off the axis task's core */

static adc_continuous_handle_t s_adc;
static adc_cali_handle_t s_cali;
static TaskHandle_t s_task;
static volatile float s_current_ma;
static volatile uint32_t s_seq; /* bumped after every published value (single writer: current_task) */

static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    (void)handle;
    (void)edata;
    (void)user_data;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_task, &woken);
    return woken == pdTRUE;
}

/* Average of the valid samples in one frame, or -1 when there are none. */
static int frame_average_raw(const uint8_t *buf, uint32_t len)
{
    adc_continuous_data_t parsed[SAMPLES_PER_FRAME];
    uint32_t n = 0;
    if (adc_continuous_parse_data(s_adc, buf, len, parsed, &n) != ESP_OK) {
        return -1;
    }
    uint32_t sum = 0, count = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (parsed[i].valid) {
            sum += parsed[i].raw_data;
            count++;
        }
    }
    return count ? (int)((sum + count / 2) / count) : -1;
}

static void current_task(void *arg)
{
    (void)arg;
    static uint8_t buf[FRAME_BYTES];
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        /* Drain everything that is buffered and keep the newest frame's average. */
        int raw = -1;
        uint32_t len = 0;
        while (adc_continuous_read(s_adc, buf, FRAME_BYTES, &len, 0) == ESP_OK) {
            int avg = frame_average_raw(buf, len);
            if (avg >= 0) {
                raw = avg;
            }
        }
        int mv = 0;
        if (raw >= 0 && adc_cali_raw_to_voltage(s_cali, raw, &mv) == ESP_OK) {
            s_current_ma = (float)mv * (1000.0f / BOARD_CURRENT_MV_PER_A);
            s_seq = s_seq + 1;
        }
    }
}

void hal_current_init(void)
{
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BOARD_CURRENT_ADC_UNIT,
        .chan = BOARD_CURRENT_ADC_CH,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali));

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = 4 * FRAME_BYTES,
        .conv_frame_size = FRAME_BYTES,
        .flags.flush_pool = 1, /* if the task falls behind, drop old samples instead of stalling */
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&handle_cfg, &s_adc));

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = BOARD_CURRENT_ADC_CH,
        .unit = BOARD_CURRENT_ADC_UNIT,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_adc, &dig_cfg));

    BaseType_t ok = xTaskCreatePinnedToCore(current_task, "current", 3072, NULL, TASK_PRIO, &s_task, TASK_CORE);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    adc_continuous_evt_cbs_t cbs = {.on_conv_done = on_conv_done};
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_adc, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(s_adc));
}

float hal_current_ma(void)
{
    return s_current_ma;
}

uint32_t hal_current_seq(void)
{
    return s_seq;
}
