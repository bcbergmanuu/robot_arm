#include "hal_can.h"

#include <stddef.h>
#include <string.h>

#include "board.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/*
 * RX: the driver's on_rx_done callback (ISR) copies each standard data frame into a FreeRTOS queue
 * that the axis task drains without blocking.
 *
 * TX: twai_node_transmit() queues a *pointer* to the caller's twai_frame_t (and its data buffer)
 * until the frame is on the wire, so frames live in a static pool whose slots are released in
 * on_tx_done. The pool has exactly TX_DEPTH slots, so the driver queue can never be full when we
 * submit (no blocking, no driver error log). A full pool or bus-off drops the frame and counts it.
 */

#define BITRATE 1000000
#define TX_DEPTH 16
#define RX_DEPTH 32

static twai_node_handle_t s_node;
static QueueHandle_t s_rxq;

static twai_frame_t s_tx_frames[TX_DEPTH];
static uint8_t s_tx_data[TX_DEPTH][TWAI_FRAME_MAX_LEN];
static uint32_t s_tx_free = (1u << TX_DEPTH) - 1u; /* bit i set = slot i free */
static portMUX_TYPE s_tx_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint8_t s_state = TWAI_ERROR_ACTIVE;
static volatile bool s_recovering;
static volatile uint32_t s_rx_dropped, s_tx_dropped, s_tx_failed;

static bool IRAM_ATTR on_rx_done(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;
    (void)user_ctx;
    uint8_t data[TWAI_FRAME_MAX_LEN];
    twai_frame_t rx = {.buffer = data, .buffer_len = sizeof(data)};
    if (twai_node_receive_from_isr(node, &rx) != ESP_OK || rx.header.ide || rx.header.rtr) {
        return false;
    }
    can_frame_t f = {
        .id = (uint16_t)(rx.header.id & TWAI_STD_ID_MASK),
        .len = (uint8_t)(rx.header.dlc > TWAI_FRAME_MAX_LEN ? TWAI_FRAME_MAX_LEN : rx.header.dlc),
    };
    memcpy(f.data, data, f.len);
    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(s_rxq, &f, &woken) != pdTRUE) {
        s_rx_dropped++;
    }
    return woken == pdTRUE;
}

static void IRAM_ATTR release_slot(unsigned idx)
{
    portENTER_CRITICAL_SAFE(&s_tx_mux);
    s_tx_free |= 1u << idx;
    portEXIT_CRITICAL_SAFE(&s_tx_mux);
}

static bool IRAM_ATTR on_tx_done(twai_node_handle_t node, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    if (!edata->is_tx_success) {
        s_tx_failed++;
    }
    ptrdiff_t idx = edata->done_tx_frame - s_tx_frames;
    if (idx >= 0 && idx < TX_DEPTH) {
        release_slot((unsigned)idx);
    }
    return false;
}

static bool IRAM_ATTR on_state_change(twai_node_handle_t node, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    (void)node;
    (void)user_ctx;
    s_state = (uint8_t)edata->new_sta;
    if (edata->new_sta != TWAI_ERROR_BUS_OFF) {
        s_recovering = false;
    }
    return false;
}

void hal_can_init(void)
{
    s_rxq = xQueueCreate(RX_DEPTH, sizeof(can_frame_t));
    ESP_ERROR_CHECK(s_rxq ? ESP_OK : ESP_ERR_NO_MEM);

    twai_onchip_node_config_t cfg = {
        .io_cfg = {
            .tx = BOARD_CAN_TX_GPIO,
            .rx = BOARD_CAN_RX_GPIO,
            .quanta_clk_out = GPIO_NUM_NC,
            .bus_off_indicator = GPIO_NUM_NC,
        },
        .bit_timing.bitrate = BITRATE,
        .fail_retry_cnt = -1, /* normal CAN: retransmit until sent (single-shot would also abort on lost arbitration) */
        .tx_queue_depth = TX_DEPTH,
    };
    ESP_ERROR_CHECK(twai_new_node_onchip(&cfg, &s_node));

    /* Accept every standard-id frame; the axis core filters by node id itself. */
    twai_mask_filter_config_t filter = {.id = 0, .mask = 0, .is_ext = false};
    ESP_ERROR_CHECK(twai_node_config_mask_filter(s_node, 0, &filter));

    twai_event_callbacks_t cbs = {
        .on_rx_done = on_rx_done,
        .on_tx_done = on_tx_done,
        .on_state_change = on_state_change,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_node));
}

bool hal_can_recv(can_frame_t *f)
{
    return xQueueReceive(s_rxq, f, 0) == pdTRUE;
}

bool hal_can_send(const can_frame_t *f)
{
    if (s_state == TWAI_ERROR_BUS_OFF) { /* the driver would reject (and log) it */
        s_tx_dropped++;
        return false;
    }
    portENTER_CRITICAL(&s_tx_mux);
    uint32_t free_mask = s_tx_free;
    unsigned idx = free_mask ? (unsigned)__builtin_ctz(free_mask) : TX_DEPTH;
    if (idx < TX_DEPTH) {
        s_tx_free &= ~(1u << idx);
    }
    portEXIT_CRITICAL(&s_tx_mux);
    if (idx == TX_DEPTH) {
        s_tx_dropped++;
        return false;
    }

    uint8_t len = f->len > TWAI_FRAME_MAX_LEN ? TWAI_FRAME_MAX_LEN : f->len;
    memcpy(s_tx_data[idx], f->data, len);
    s_tx_frames[idx] = (twai_frame_t){
        .header = {.id = f->id & TWAI_STD_ID_MASK, .dlc = len},
        .buffer = s_tx_data[idx],
        .buffer_len = len,
    };
    if (twai_node_transmit(s_node, &s_tx_frames[idx], 0) != ESP_OK) {
        release_slot(idx);
        s_tx_dropped++;
        return false;
    }
    return true;
}

void hal_can_service(void)
{
    if (s_state == TWAI_ERROR_BUS_OFF && !s_recovering) {
        s_recovering = true;
        twai_node_recover(s_node);
    }
}

void hal_can_get_stats(hal_can_stats_t *out)
{
    twai_node_status_t status = {0};
    twai_node_record_t record = {0};
    twai_node_get_info(s_node, &status, &record);
    *out = (hal_can_stats_t){
        .state = (uint8_t)status.state,
        .tec = status.tx_error_count,
        .rec = status.rx_error_count,
        .bus_errors = record.bus_err_num,
        .rx_dropped = s_rx_dropped,
        .tx_dropped = s_tx_dropped,
        .tx_failed = s_tx_failed,
    };
}
