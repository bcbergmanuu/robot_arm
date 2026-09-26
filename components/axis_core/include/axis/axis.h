#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "axis/axis_config.h"
#include "axis/pid.h"
#include "axis/protocol.h"

typedef enum { AXIS_DISABLED = 0, AXIS_HOMING = 1, AXIS_READY = 2, AXIS_FAULT = 3 } axis_state_t;

enum {
    AXIS_FAULT_WATCHDOG = 1u << 0,
    AXIS_FAULT_OVERCURRENT = 1u << 1,
    AXIS_FAULT_FOLLOWING = 1u << 2,
    AXIS_FAULT_ESTOP = 1u << 3,
    AXIS_FAULT_HOMING = 1u << 4
};

enum { AXIS_FLAG_HOMED = 1u << 0 };

typedef struct {
    int32_t encoder_raw; /* raw = accumulated hw count */
    float current_ma;
} axis_inputs_t;

typedef struct {
    float duty; /* signed, already x motor_sign */
} axis_outputs_t;

#define AXIS_TX_QUEUE_LEN 8
#define AXIS_VEL_WINDOW 8
#define AXIS_STATUS_DIVIDER 10 /* 100 Hz status at 1 kHz tick */
#define AXIS_HOME_SETTLE_MS 300 /* ignore stall detection right after homing starts */
#define AXIS_HOME_STALL_MS 100  /* consecutive ms of stall evidence before declaring home found */

typedef struct {
    const axis_config_t *cfg;
    axis_state_t state;
    uint8_t faults;
    bool homed;
    bool primed;         /* false until the first axis_tick has read the encoder */
    int32_t zero_offset; /* raw (sign-corrected) count that corresponds to position 0 */
    int32_t pos;         /* counts relative to home zero */
    float vel;           /* counts/s, estimated */
    int32_t pos_hist[AXIS_VEL_WINDOW];
    uint8_t hist_idx, hist_fill;
    uint8_t sp_kind; /* PROTO_SP_* */
    float target;    /* counts | counts/s | duty, per sp_kind */
    float sp_pos, sp_vel; /* trajectory generator state */
    pidc_t pos_pid, vel_pid;
    uint32_t ms_since_rx, overcurrent_ms, home_ms, stall_ms, tick;
    float current_ma, duty;
    can_frame_t txq[AXIS_TX_QUEUE_LEN];
    uint8_t tx_head, tx_count;
} axis_t;

void axis_init(axis_t *a, const axis_config_t *cfg);
void axis_tick(axis_t *a, const axis_inputs_t *in, axis_outputs_t *out);
void axis_on_frame(axis_t *a, const can_frame_t *f);
bool axis_pop_tx(axis_t *a, can_frame_t *out);
void axis_set_home(axis_t *a, int32_t raw_at_zero); /* marks homed; raw is sign-corrected */
