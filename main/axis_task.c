#include "axis_task.h"

#include <stdatomic.h>

#include "axis/axis.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal_can.h"
#include "hal_current.h"
#include "hal_encoder.h"
#include "hal_motor.h"

/*
 * Control loop: a 1 MHz gptimer alarms every 1000 ticks (auto-reload); its ISR notifies the control
 * task. Per tick the task drains CAN RX into the axis core, runs axis_tick on the latest
 * encoder/current readings, writes the duty and queues the core's TX frames for CAN -- all
 * non-blocking. Nothing in that path logs: once per second the control task copies a small snapshot
 * that the low-priority status task prints (and uses to kick CAN bus-off recovery).
 *
 * Two independent layers stop the motor if the control task stalls (the axis core can only zero the
 * duty while it is running):
 *  - primary: the gptimer ISR counts ticks since the task last completed a loop body; beyond
 *    STALL_TICKS it trips hal_motor (both bridge inputs low, latched until reboot) and counts it. The
 *    control task notices the latch on its next iteration and injects a local ESTOP frame through
 *    axis_on_frame, once, so the axis reports FAULT/ESTOP on the bus instead of READY with a braked
 *    bridge (the loop body that trips will usually still be running -- the ISR only stalls the motor,
 *    not the task -- so this is normally the very next tick).
 *  - secondary: the control task is subscribed to the task watchdog (CONFIG_ESP_TASK_WDT_TIMEOUT_S=1,
 *    CONFIG_ESP_TASK_WDT_PANIC=y), so a stall that also stops the ISR path resets the chip.
 *
 * Stale current: hal_current publishes a sequence number with every value. If it has not advanced
 * for CURRENT_STALE_TICKS, the overcurrent protection is blind, so the task feeds CURRENT_STALE_MA
 * instead of the frozen reading: READY runs into the core's normal OVERCURRENT fault (visible on the
 * bus), the telemetry current reads 32767 mA (the int16 clamp), and the status line counts it. In
 * HOMING a huge current would count as stall evidence and could latch a false home, so homing is
 * aborted instead with a local DISABLE command through the core's normal command path.
 */

#define TIMER_RES_HZ 1000000
#define CONTROL_PRIO (configMAX_PRIORITIES - 2)
#define CONTROL_CORE 1
#define STATUS_PRIO 2
#define STATUS_CORE 0
#define STALL_TICKS 20          /* ms without a completed loop body before the ISR trips the motor */
#define CURRENT_STALE_TICKS 50  /* ms without a new current sample before it is treated as stale */
#define CURRENT_STALE_MA 1.0e6f /* above any max_current_ma: the overcurrent fault then fires */

static const char *TAG = "axis";

static axis_t s_axis;
static TaskHandle_t s_control_task;

static atomic_uint s_isr_tick;  /* gptimer alarms since start */
static atomic_uint s_done_tick; /* s_isr_tick value when the task last completed a loop body */
static atomic_uint s_stall_trips;
static bool s_estop_injected; /* control_task only: latches once axis_on_frame has been told about the trip */

typedef struct {
    int32_t pos;
    float duty, current_ma;
    uint8_t state, faults;
    bool homed;
    uint32_t overruns;     /* ticks the control task missed (timer fired again before it waited) */
    uint32_t max_body_us;  /* longest loop body in the last second */
    uint32_t stale_events; /* times the current reading went stale */
    bool current_stale;    /* stale right now */
} snapshot_t;

static snapshot_t s_snap;
static portMUX_TYPE s_snap_mux = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;
    unsigned now = atomic_fetch_add(&s_isr_tick, 1) + 1;
    if (now - atomic_load(&s_done_tick) > STALL_TICKS && !hal_motor_tripped()) {
        hal_motor_trip_from_isr();
        atomic_fetch_add(&s_stall_trips, 1);
    }
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_control_task, &woken);
    return woken == pdTRUE;
}

/* Current for this tick; handles a stalled current pipeline (see top of file). */
static float current_input(uint32_t *stale_ticks, uint32_t *last_seq, uint32_t *stale_events)
{
    uint32_t seq = hal_current_seq();
    if (seq != *last_seq) {
        *last_seq = seq;
        *stale_ticks = 0;
        return hal_current_ma();
    }
    if (*stale_ticks <= CURRENT_STALE_TICKS) { /* saturates at CURRENT_STALE_TICKS + 1 */
        (*stale_ticks)++;
        if (*stale_ticks <= CURRENT_STALE_TICKS) {
            return hal_current_ma(); /* a few missed samples are harmless */
        }
        (*stale_events)++; /* just went stale */
    }
    if (s_axis.state == AXIS_HOMING) {
        can_frame_t disable;
        proto_encode_command(&disable, s_axis.cfg->node_id, PROTO_CMD_DISABLE);
        axis_on_frame(&s_axis, &disable);
    }
    return CURRENT_STALE_MA;
}

static void control_task(void *arg)
{
    (void)arg;
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    uint32_t overruns = 0, max_body_us = 0, ticks = 0;
    uint32_t cur_stale_ticks = 0, cur_last_seq = hal_current_seq(), cur_stale_events = 0;
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

        /* hal_motor already forces the bridge to brake once tripped (see hal_motor.c); this makes the
         * axis core itself report FAULT/ESTOP on the bus instead of READY with a braked bridge, the
         * first tick after the ISR latches the trip. */
        if (hal_motor_tripped() && !s_estop_injected) {
            s_estop_injected = true;
            can_frame_t estop;
            proto_encode_estop(&estop);
            axis_on_frame(&s_axis, &estop);
        }

        axis_inputs_t in = {
            .encoder_raw = hal_encoder_read(),
            .current_ma = current_input(&cur_stale_ticks, &cur_last_seq, &cur_stale_events),
        };
        axis_outputs_t out;
        axis_tick(&s_axis, &in, &out);
        hal_motor_set_duty(out.duty);

        while (axis_pop_tx(&s_axis, &f)) {
            hal_can_send(&f); /* drops are counted inside hal_can */
        }

        esp_task_wdt_reset();
        atomic_store(&s_done_tick, atomic_load(&s_isr_tick));

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
                .stale_events = cur_stale_events,
                .current_stale = cur_stale_ticks > CURRENT_STALE_TICKS,
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
                 "node %u %s%s faults=0x%02x pos=%ld duty=%.3f I=%.0fmA%s | loop max=%luus overruns=%lu "
                 "stall_trips=%u%s cur_stale=%lu | "
                 "can st=%u tec=%u rec=%u buserr=%lu rxdrop=%lu txdrop=%lu txfail=%lu",
                 node, s.state < 4 ? state_names[s.state] : "?", s.homed ? "(homed)" : "", s.faults,
                 (long)s.pos, (double)s.duty, (double)s.current_ma, s.current_stale ? "(STALE)" : "",
                 (unsigned long)s.max_body_us, (unsigned long)s.overruns, atomic_load(&s_stall_trips),
                 hal_motor_tripped() ? "(MOTOR TRIPPED, reboot to clear)" : "", (unsigned long)s.stale_events,
                 c.state, c.tec, c.rec, (unsigned long)c.bus_errors, (unsigned long)c.rx_dropped,
                 (unsigned long)c.tx_dropped, (unsigned long)c.tx_failed);
    }
}

void axis_task_start(const axis_config_t *cfg)
{
    axis_init(&s_axis, cfg);

    BaseType_t ok = xTaskCreatePinnedToCore(control_task, "axis_ctrl", 4096, NULL, CONTROL_PRIO, &s_control_task,
                                            CONTROL_CORE);
    ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ok = xTaskCreatePinnedToCore(status_task, "axis_status", 3072, (void *)(uintptr_t)cfg->node_id, STATUS_PRIO,
                                 NULL, STATUS_CORE);
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

    ESP_LOGI(TAG, "node %u (%s) control loop running at %d Hz on core %d", cfg->node_id, cfg->name, AXIS_TICK_HZ,
             CONTROL_CORE);
}
