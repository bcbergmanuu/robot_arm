#include "axis/protocol.h"

static void put_i32(uint8_t *out, int32_t v) {
    uint32_t u = (uint32_t)v;
    out[0] = (uint8_t)(u & 0xFF);
    out[1] = (uint8_t)((u >> 8) & 0xFF);
    out[2] = (uint8_t)((u >> 16) & 0xFF);
    out[3] = (uint8_t)((u >> 24) & 0xFF);
}

static int32_t get_i32(const uint8_t *in) {
    uint32_t u = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    return (int32_t)u;
}

static void put_i16(uint8_t *out, int16_t v) {
    uint16_t u = (uint16_t)v;
    out[0] = (uint8_t)(u & 0xFF);
    out[1] = (uint8_t)((u >> 8) & 0xFF);
}

static int16_t get_i16(const uint8_t *in) {
    uint16_t u = (uint16_t)in[0] | ((uint16_t)in[1] << 8);
    return (int16_t)u;
}

void proto_encode_estop(can_frame_t *f) {
    f->id = PROTO_ID(PROTO_MSG_ESTOP, PROTO_NODE_BROADCAST);
    f->len = 0;
}

void proto_encode_heartbeat(can_frame_t *f, uint8_t seq) {
    f->id = PROTO_ID(PROTO_MSG_HEARTBEAT, PROTO_NODE_BROADCAST);
    f->len = 1;
    f->data[0] = seq;
}

void proto_encode_command(can_frame_t *f, uint8_t node, uint8_t cmd) {
    f->id = PROTO_ID(PROTO_MSG_COMMAND, node);
    f->len = 1;
    f->data[0] = cmd;
}

void proto_encode_setpoint(can_frame_t *f, uint8_t node, uint8_t kind, int32_t value) {
    f->id = PROTO_ID(PROTO_MSG_SETPOINT, node);
    f->len = 5;
    f->data[0] = kind;
    put_i32(&f->data[1], value);
}

void proto_encode_status(can_frame_t *f, uint8_t node, int32_t pos, uint8_t state, uint8_t faults, uint8_t flags) {
    f->id = PROTO_ID(PROTO_MSG_STATUS, node);
    f->len = 7;
    put_i32(&f->data[0], pos);
    f->data[4] = state;
    f->data[5] = faults;
    f->data[6] = flags;
}

void proto_encode_telemetry(can_frame_t *f, uint8_t node, int32_t vel, int16_t current_ma) {
    f->id = PROTO_ID(PROTO_MSG_TELEMETRY, node);
    f->len = 6;
    put_i32(&f->data[0], vel);
    put_i16(&f->data[4], current_ma);
}

bool proto_decode_command(const can_frame_t *f, uint8_t *cmd) {
    if (proto_type(f->id) != PROTO_MSG_COMMAND || f->len != 1) return false;
    *cmd = f->data[0];
    return true;
}

bool proto_decode_setpoint(const can_frame_t *f, uint8_t *kind, int32_t *value) {
    if (proto_type(f->id) != PROTO_MSG_SETPOINT || f->len != 5) return false;
    *kind = f->data[0];
    *value = get_i32(&f->data[1]);
    return true;
}

bool proto_decode_status(const can_frame_t *f, int32_t *pos, uint8_t *state, uint8_t *faults, uint8_t *flags) {
    if (proto_type(f->id) != PROTO_MSG_STATUS || f->len != 7) return false;
    *pos = get_i32(&f->data[0]);
    *state = f->data[4];
    *faults = f->data[5];
    *flags = f->data[6];
    return true;
}

bool proto_decode_telemetry(const can_frame_t *f, int32_t *vel, int16_t *current_ma) {
    if (proto_type(f->id) != PROTO_MSG_TELEMETRY || f->len != 6) return false;
    *vel = get_i32(&f->data[0]);
    *current_ma = get_i16(&f->data[4]);
    return true;
}
