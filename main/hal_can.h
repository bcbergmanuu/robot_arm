#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "axis/protocol.h"

/* TWAI (CAN 2.0) at 1 Mbit/s, 11-bit ids, on BOARD_CAN_TX_GPIO/BOARD_CAN_RX_GPIO. */

typedef struct {
    uint8_t state;        /* twai_error_state_t: 0 active, 1 warning, 2 passive, 3 bus-off */
    uint16_t tec, rec;    /* transmit / receive error counters */
    uint32_t bus_errors;  /* driver bus-error count since enable (reset on bus-off recovery) */
    uint32_t rx_dropped;  /* RX queue full */
    uint32_t tx_dropped;  /* TX queue/pool full, bus-off, or rejected by the driver */
    uint32_t tx_failed;   /* completed without success */
} hal_can_stats_t;

void hal_can_init(void);
bool hal_can_recv(can_frame_t *f);        /* non-blocking; false when nothing is queued */
bool hal_can_send(const can_frame_t *f);  /* non-blocking enqueue for the tx task; false (and counted) when full */
void hal_can_service(void);               /* call periodically from a low-priority task: starts bus-off recovery */
void hal_can_get_stats(hal_can_stats_t *out);
