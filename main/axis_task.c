#include "axis_task.h"

#include "axis/axis.h"
#include "axis/config_table.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal_can.h"
#include "hal_current.h"
#include "hal_encoder.h"
#include "hal_motor.h"

/*
 * Control loop: a 1 MHz gptimer alarms every 1000 ticks (auto-reload); its ISR only notifies the
 * control task. Per tick the task drains CAN RX into the axis core, runs axis_tick on the latest
 * encoder/current readings, writes the duty and drains the core's TX queue to CAN -- all non-blocking.
 * Nothing in that path logs: once per second the control task copies a small snapshot that the
 * low-priority status task prints (and uses to kick CAN bus-off recovery).
 */

#define TIMER_RES_HZ 1000000
#define CONTROL_PRIO (configMAX_PRIORITIES - 2)
#define CONTROL_CORE 1
#define STATUS_PRIO 2
#define STATUS_CORE 0

static const char *TAG = "axis";

static axis_t s_axis;
static TaskHandle_t s_control_task;

typedef struct {
    int32_t pos;
    float duty, current_ma;
    uint8_t state, faults;
    bool homed;
    uint32_t overruns;   /* ticks the control task missed (timer fired again before it waited) */
    uint32_t max_body_us; /* longest loop body in the last second */
} snapshot_t;

static snapshot_t s_snap;
static portMUX_TYPE s_snap_mux = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &woken);
    return woken == pdTRUE;
}

static void control_task(void *arg)
{
    (void)arg;
    uint32_t overruns = 0, max_body_us = 0, ticks = 0;
    for (;;) {
        uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (pending > 1) {
            overruns += pending - 1;
        }
        int64_t t0 = esp_timer_get_time();

        can_frame_t f;
        while (hal_can_recv(&f)) {
            axis_on_frame(&s_axis, &f);
        }

        axis_inputs_t in = {.encoder_raw = hal_encoder_read(), .current_ma = hal_current_ma()};
        axis_outputs_t out;
        axis_tick(&s_axis, &in, &out);
        hal_motor_set_duty(out.duty);

        while (axis_pop_tx(&s_axis, &f)) {
            hal_can_send(&f); /* drops are counted inside hal_can */
        }

        uint32_t body_us = (uint32_t)(esp_timer_get_time() - t0);
        if (body_us > max_body_us) {
            max_body_us = body_us;
        }
        if (++ticks >= AXIS_TICK_HZ) {
            ticks = 0;
            portENTER_CRITICAL(&s_snap_mux);
            s_snap = (snapshot_t){
                .pos = s_axis.pos,
                .duty = s_axis.duty,
                .current_ma = s_axis.current_ma,
                .state = (uint8_t)s_axis.state,
                .faults = s_axis.faults,
                .homed = s_axis.homed,
                .overruns = overruns,
                .max_body_us = max_body_us,
            };
            portEXIT_CRITICAL(&s_snap_mux);
            max_body_us = 0;
        }
    }
}

static void status_task(void *arg)
{
    const uint8_t node = (uint8_t)(uintptr_t)arg;
    static const char *const state_names[] = {"DISABLED", "HOMING", "READY", "FAULT"};
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
        hal_can_service();

        snapshot_t s;
        portENTER_CRITICAL(&s_snap_mux);
        s = s_snap;
        portEXIT_CRITICAL(&s_snap_mux);
        hal_can_stats_t c;
        hal_can_get_stats(&c);

        ESP_LOGI(TAG,
                 "node %u %s%s faults=0x%02x pos=%ld duty=%.3f I=%.0fmA | loop max=%luus overruns=%lu | "
                 "can st=%u tec=%u rec=%u buserr=%lu rxdrop=%lu txdrop=%lu txfail=%lu",
                 node, s.state < 4 ? state_names[s.state] : "?", s.homed ? "(homed)" : "", s.faults,
                 (long)s.pos, (double)s.duty, (double)s.current_ma, (unsigned long)s.max_body_us,
                 (unsigned long)s.overruns, c.state, c.tec, c.rec, (unsigned long)c.bus_errors,
                 (unsigned long)c.rx_dropped, (unsigned long)c.tx_dropped, (unsigned long)c.tx_failed);
    }
}

void axis_task_start(uint8_t node_id)
{
    const axis_config_t *cfg = axis_config_for_node(node_id);
    ESP_ERROR_CHECK(cfg ? ESP_OK : ESP_ERR_INVALID_ARG);
    axis_init(&s_axis, cfg);

    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "axis_ctrl", 4096, NULL, CONTROL_PRIO, &s_control_task,
                                            CONTROL_CORE);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ok = xTaskCreatePinnedToCore(status_task, "axis_status", 3072, (void *)(uintptr_t)node_id, STATUS_PRIO, NULL,
                                 STATUS_CORE);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = TIMER_RES_HZ,
    };
    gptimer_handle_t timer = NULL;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &timer));
    gptimer_event_callbacks_t cbs = {.on_alarm = on_alarm};
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, NULL));
    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = TIMER_RES_HZ / AXIS_TICK_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_enable(timer));
    ESP_ERROR_CHECK(gptimer_start(timer));

    ESP_LOGI(TAG, "node %u (%s) control loop running at %d Hz on core %d", node_id, cfg->name, AXIS_TICK_HZ,
             CONTROL_CORE);
}
