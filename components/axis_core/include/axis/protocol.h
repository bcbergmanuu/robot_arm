#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * CAN protocol: 1 Mbit/s, 11-bit standard ids, little-endian payloads.
 *
 * id = (type << 3) | node, node 1-6 = axes, node 0 = broadcast.
 * Lower id value = higher bus priority.
 */

typedef struct {
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
} can_frame_t;

enum { PROTO_NODE_BROADCAST = 0 };

enum {
    PROTO_MSG_ESTOP = 0x00,
    PROTO_MSG_HEARTBEAT = 0x01,
    PROTO_MSG_COMMAND = 0x02,
    PROTO_MSG_SETPOINT = 0x03,
    PROTO_MSG_STATUS = 0x10,
    PROTO_MSG_TELEMETRY = 0x11
};

enum { PROTO_CMD_DISABLE = 0, PROTO_CMD_ENABLE = 1, PROTO_CMD_HOME = 2, PROTO_CMD_CLEAR_FAULT = 3 };

enum { PROTO_SP_POSITION = 0, PROTO_SP_VELOCITY = 1, PROTO_SP_DUTY = 2 };

#define PROTO_DUTY_SCALE 10000

#define PROTO_ID(type, node) ((uint16_t)(((type) << 3) | ((node) & 0x7)))

static inline uint8_t proto_type(uint16_t id) { return (uint8_t)(id >> 3); }
static inline uint8_t proto_node(uint16_t id) { return (uint8_t)(id & 0x7); }

void proto_encode_estop(can_frame_t *f);
void proto_encode_heartbeat(can_frame_t *f, uint8_t seq);
void proto_encode_command(can_frame_t *f, uint8_t node, uint8_t cmd);
void proto_encode_setpoint(can_frame_t *f, uint8_t node, uint8_t kind, int32_t value);
void proto_encode_status(can_frame_t *f, uint8_t node, int32_t pos, uint8_t state, uint8_t faults, uint8_t flags);
void proto_encode_telemetry(can_frame_t *f, uint8_t node, int32_t vel, int16_t current_ma);

bool proto_decode_command(const can_frame_t *f, uint8_t *cmd);
bool proto_decode_setpoint(const can_frame_t *f, uint8_t *kind, int32_t *value);
bool proto_decode_status(const can_frame_t *f, int32_t *pos, uint8_t *state, uint8_t *faults, uint8_t *flags);
bool proto_decode_telemetry(const can_frame_t *f, int32_t *vel, int16_t *current_ma);
