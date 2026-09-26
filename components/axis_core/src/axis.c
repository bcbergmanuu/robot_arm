#include "axis/axis.h"

#include <math.h>
#include <string.h>

static float clampf(float v, float lo, float hi) { return fminf(fmaxf(v, lo), hi); }

static void enter_fault(axis_t *a, uint8_t bits) {
    a->faults |= bits;
    a->state = AXIS_FAULT;
}

static void update_velocity_estimate(axis_t *a) {
    int32_t oldest = a->pos_hist[a->hist_idx];
    a->pos_hist[a->hist_idx] = a->pos;
    a->hist_idx = (uint8_t)((a->hist_idx + 1) % AXIS_VEL_WINDOW);
    if (a->hist_fill < AXIS_VEL_WINDOW) a->hist_fill++;
    a->vel = (float)(a->pos - oldest) * (float)AXIS_TICK_HZ / (float)a->hist_fill;
}

static void check_safety(axis_t *a, const axis_inputs_t *in) {
    a->current_ma = in->current_ma;
    a->ms_since_rx++;

    if ((a->state == AXIS_READY || a->state == AXIS_HOMING) && a->ms_since_rx > a->cfg->watchdog_ms) {
        enter_fault(a, AXIS_FAULT_WATCHDOG);
    }

    if (a->state != AXIS_DISABLED && a->state != AXIS_FAULT) {
        if (in->current_ma > a->cfg->max_current_ma) {
            a->overcurrent_ms++;
            if (a->overcurrent_ms > a->cfg->overcurrent_ms) enter_fault(a, AXIS_FAULT_OVERCURRENT);
        } else {
            a->overcurrent_ms = 0;
        }
    } else {
        a->overcurrent_ms = 0;
    }
}

static void set_sp_kind_bumpless(axis_t *a, uint8_t kind) {
    a->sp_kind = kind;
    a->sp_pos = (float)a->pos;
    a->sp_vel = a->vel;
    pidc_reset(&a->pos_pid);
    pidc_reset(&a->vel_pid);
}

static void handle_command(axis_t *a, uint8_t cmd) {
    switch (cmd) {
        case PROTO_CMD_DISABLE:
            if (a->state != AXIS_FAULT) a->state = AXIS_DISABLED;
            break;
        case PROTO_CMD_ENABLE:
            if (a->state == AXIS_DISABLED && a->faults == 0) {
                a->state = AXIS_READY;
                set_sp_kind_bumpless(a, PROTO_SP_VELOCITY);
                a->target = 0.0f;
            }
            break;
        case PROTO_CMD_HOME:
            if (a->state == AXIS_DISABLED || a->state == AXIS_READY) {
                a->state = AXIS_HOMING;
                a->home_ms = 0;
                a->stall_ms = 0;
            }
            break;
        case PROTO_CMD_CLEAR_FAULT:
            if (a->state == AXIS_FAULT) {
                a->faults = 0;
                a->state = AXIS_DISABLED;
            }
            break;
        default:
            break;
    }
}

static void handle_setpoint(axis_t *a, uint8_t kind, int32_t value) {
    if (a->state != AXIS_READY) return;
    if (kind == PROTO_SP_POSITION && !a->homed) return;

    if (kind != a->sp_kind) set_sp_kind_bumpless(a, kind);

    switch (kind) {
        case PROTO_SP_DUTY:
            a->target = (float)value / (float)PROTO_DUTY_SCALE;
            break;
        case PROTO_SP_VELOCITY:
            a->target = (float)value;
            break;
        case PROTO_SP_POSITION:
            a->target = (float)value;
            break;
        default:
            break;
    }
}

static void queue_tx(axis_t *a, const can_frame_t *f) {
    if (a->tx_count == AXIS_TX_QUEUE_LEN) {
        /* drop the oldest frame to make room */
        a->tx_head = (uint8_t)((a->tx_head + 1) % AXIS_TX_QUEUE_LEN);
        a->tx_count--;
    }
    uint8_t tail = (uint8_t)((a->tx_head + a->tx_count) % AXIS_TX_QUEUE_LEN);
    a->txq[tail] = *f;
    a->tx_count++;
}

static void queue_status(axis_t *a) {
    can_frame_t f;
    uint8_t flags = a->homed ? (uint8_t)AXIS_FLAG_HOMED : 0u;
    proto_encode_status(&f, a->cfg->node_id, a->pos, (uint8_t)a->state, a->faults, flags);
    queue_tx(a, &f);

    int16_t cur = (int16_t)clampf(roundf(a->current_ma), -32768.0f, 32767.0f);
    proto_encode_telemetry(&f, a->cfg->node_id, (int32_t)roundf(a->vel), cur);
    queue_tx(a, &f);
}

static float signf(float v) { return (v > 0.0f) ? 1.0f : ((v < 0.0f) ? -1.0f : 0.0f); }

static void run_ready(axis_t *a) {
    const axis_config_t *cfg = a->cfg;
    float v_des = 0.0f;

    switch (a->sp_kind) {
        case PROTO_SP_DUTY:
            a->duty = clampf(a->target, -cfg->max_duty, cfg->max_duty);
            return;

        case PROTO_SP_POSITION: {
            float goal = clampf(a->target, (float)cfg->pos_min, (float)cfg->pos_max);
            float dist = goal - a->sp_pos;
            v_des = signf(dist) * fminf(cfg->max_vel, sqrtf(2.0f * cfg->max_acc * fabsf(dist)));
            if (fabsf(dist) < 0.5f && fabsf(a->sp_vel) < cfg->max_acc * AXIS_DT) {
                a->sp_pos = goal;
                a->sp_vel = 0.0f;
                v_des = 0.0f;
            }
            break;
        }

        case PROTO_SP_VELOCITY: {
            float vmax = a->homed ? cfg->max_vel : cfg->home_vel;
            v_des = clampf(a->target, -vmax, vmax);
            if (a->homed) {
                /* Brake using the position we are about to reach this tick (one-step
                 * look-ahead), not the stale sp_pos: braking on the stale position
                 * lags by half a tick's travel (v*dt/2) and lets sp_pos punch through
                 * the soft limit before the brake curve catches up. */
                float lookahead = a->sp_pos + a->sp_vel * AXIS_DT;
                v_des = fminf(v_des, sqrtf(2.0f * cfg->max_acc * fmaxf(0.0f, (float)cfg->pos_max - lookahead)));
                v_des = fmaxf(v_des, -sqrtf(2.0f * cfg->max_acc * fmaxf(0.0f, lookahead - (float)cfg->pos_min)));
            }
            break;
        }

        default:
            a->duty = 0.0f;
            return;
    }

    a->sp_vel += clampf(v_des - a->sp_vel, -cfg->max_acc * AXIS_DT, cfg->max_acc * AXIS_DT);
    a->sp_pos += a->sp_vel * AXIS_DT;

    float vel_cmd = a->sp_vel + pidc_update(&a->pos_pid, a->sp_pos - (float)a->pos, AXIS_DT);
    a->duty = cfg->vel_ff * vel_cmd + pidc_update(&a->vel_pid, vel_cmd - a->vel, AXIS_DT);

    if (fabsf(a->sp_pos - (float)a->pos) > (float)cfg->max_following_error) {
        enter_fault(a, AXIS_FAULT_FOLLOWING);
    }
}

/* Stub: homing sequence lands in Task 7. */
static void run_homing(axis_t *a) { a->duty = 0.0f; }

void axis_init(axis_t *a, const axis_config_t *cfg) {
    memset(a, 0, sizeof(*a));
    a->cfg = cfg;
    a->state = AXIS_DISABLED;
    a->sp_kind = PROTO_SP_DUTY;
    pidc_init(&a->pos_pid, &cfg->pos_pid);
    pidc_init(&a->vel_pid, &cfg->vel_pid);
}

void axis_tick(axis_t *a, const axis_inputs_t *in, axis_outputs_t *out) {
    a->tick++;

    int32_t raw = in->encoder_raw * a->cfg->encoder_sign;
    a->pos = raw - a->zero_offset;
    update_velocity_estimate(a);

    check_safety(a, in);

    switch (a->state) {
        case AXIS_DISABLED:
        case AXIS_FAULT:
            a->duty = 0.0f;
            a->sp_pos = (float)a->pos;
            a->sp_vel = 0.0f;
            pidc_reset(&a->pos_pid);
            pidc_reset(&a->vel_pid);
            break;
        case AXIS_READY:
            run_ready(a);
            break;
        case AXIS_HOMING:
            run_homing(a);
            break;
    }

    out->duty = a->duty * (float)a->cfg->motor_sign;

    bool status_due = (a->tick % AXIS_STATUS_DIVIDER) == 0;
    if (a->state == AXIS_READY && a->sp_kind == PROTO_SP_DUTY) status_due = true;
    if (status_due) queue_status(a);
}

void axis_on_frame(axis_t *a, const can_frame_t *f) {
    uint8_t node = proto_node(f->id);
    if (node != a->cfg->node_id && node != PROTO_NODE_BROADCAST) return;

    switch (proto_type(f->id)) {
        case PROTO_MSG_ESTOP:
            enter_fault(a, AXIS_FAULT_ESTOP);
            break;
        case PROTO_MSG_HEARTBEAT:
            a->ms_since_rx = 0;
            break;
        case PROTO_MSG_COMMAND: {
            uint8_t cmd;
            if (proto_decode_command(f, &cmd)) {
                a->ms_since_rx = 0;
                handle_command(a, cmd);
            }
            break;
        }
        case PROTO_MSG_SETPOINT: {
            uint8_t kind;
            int32_t value;
            if (proto_decode_setpoint(f, &kind, &value)) {
                a->ms_since_rx = 0;
                handle_setpoint(a, kind, value);
            }
            break;
        }
        default:
            break;
    }
}

bool axis_pop_tx(axis_t *a, can_frame_t *out) {
    if (a->tx_count == 0) return false;
    *out = a->txq[a->tx_head];
    a->tx_head = (uint8_t)((a->tx_head + 1) % AXIS_TX_QUEUE_LEN);
    a->tx_count--;
    return true;
}

void axis_set_home(axis_t *a, int32_t raw_at_zero) {
    a->zero_offset = raw_at_zero;
    a->homed = true;
}
