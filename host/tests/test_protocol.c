#include <string.h>
#include "axis/protocol.h"
#include "tinytest.h"

static int frame_eq(const can_frame_t *f, uint16_t id, uint8_t len, const uint8_t *data) {
    return f->id == id && f->len == len && memcmp(f->data, data, len) == 0;
}

static void test_golden_vectors(void) {
    can_frame_t f;
    proto_encode_estop(&f);
    TT_CHECK(f.id == 0x000 && f.len == 0);
    proto_encode_heartbeat(&f, 7);
    TT_CHECK(frame_eq(&f, 0x008, 1, (const uint8_t[]){0x07}));
    proto_encode_command(&f, 1, PROTO_CMD_HOME);
    TT_CHECK(frame_eq(&f, 0x011, 1, (const uint8_t[]){0x02}));
    proto_encode_setpoint(&f, 3, PROTO_SP_VELOCITY, -1000);
    TT_CHECK(frame_eq(&f, 0x01B, 5, (const uint8_t[]){0x01, 0x18, 0xFC, 0xFF, 0xFF}));
    proto_encode_status(&f, 2, 123456, 2, 0x05, 0x01);
    TT_CHECK(frame_eq(&f, 0x082, 7, (const uint8_t[]){0x40, 0xE2, 0x01, 0x00, 0x02, 0x05, 0x01}));
    proto_encode_telemetry(&f, 6, -250000, 1500);
    TT_CHECK(frame_eq(&f, 0x08E, 6, (const uint8_t[]){0x70, 0x2F, 0xFC, 0xFF, 0xDC, 0x05}));
}

static void test_roundtrip_and_rejects(void) {
    can_frame_t f; uint8_t kind; int32_t value;
    proto_encode_setpoint(&f, 4, PROTO_SP_POSITION, 2147483647);
    TT_CHECK(proto_decode_setpoint(&f, &kind, &value) && kind == PROTO_SP_POSITION && value == 2147483647);
    int32_t pos; uint8_t st, fl, fg;
    TT_CHECK(!proto_decode_status(&f, &pos, &st, &fl, &fg));        /* wrong type */
    f.len = 4;
    TT_CHECK(!proto_decode_setpoint(&f, &kind, &value));             /* wrong length */
    int32_t vel; int16_t cur;
    proto_encode_telemetry(&f, 1, 42, -3);
    TT_CHECK(proto_decode_telemetry(&f, &vel, &cur) && vel == 42 && cur == -3);
    TT_CHECK(proto_node(f.id) == 1 && proto_type(f.id) == PROTO_MSG_TELEMETRY);
}

int main(void) { TT_RUN(test_golden_vectors); TT_RUN(test_roundtrip_and_rejects); return TT_DONE(); }
