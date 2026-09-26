# CAN protocol

1 Mbit/s, 11-bit standard ids, little-endian payloads.

`id = (type << 3) | node`, node 1-6 = axes, node 0 = broadcast. Lower id value = higher bus priority.

## Message table

| type | name | dir | len | payload |
|---|---|---|---|---|
| 0x00 | ESTOP | master→all (node 0) | 0 | — |
| 0x01 | HEARTBEAT | master→all (node 0) | 1 | u8 seq |
| 0x02 | COMMAND | master→node/all | 1 | u8 cmd: 0 DISABLE, 1 ENABLE, 2 HOME, 3 CLEAR_FAULT |
| 0x03 | SETPOINT | master→node | 5 | u8 kind (0 POSITION counts, 1 VELOCITY counts/s, 2 DUTY ×10000), i32 value |
| 0x10 | STATUS | node→master | 7 | i32 position counts, u8 state, u8 faults, u8 flags (bit0 homed) |
| 0x11 | TELEMETRY | node→master | 6 | i32 velocity counts/s, i16 current mA |

## Axis state

| value | name |
|---|---|
| 0 | DISABLED |
| 1 | HOMING |
| 2 | READY |
| 3 | FAULT |

## Fault bitmask

| bit | value | name |
|---|---|---|
| 0 | 1 | WATCHDOG |
| 1 | 2 | OVERCURRENT |
| 2 | 4 | FOLLOWING |
| 3 | 8 | ESTOP |
| 4 | 16 | HOMING |

## Rates and watchdog

STATUS and TELEMETRY are sent at 100 Hz (every tick while in DUTY mode). Any HEARTBEAT, COMMAND, or
SETPOINT addressed to the node or to broadcast feeds that node's watchdog.

## Golden vectors

These vectors are asserted verbatim by both `host/tests/test_protocol.c` and `pc/tests/test_protocol.py`,
which keeps the C and Python implementations in agreement.

| call | id | data (hex) |
|---|---|---|
| `encode_estop()` | 0x000 | (empty) |
| `encode_heartbeat(7)` | 0x008 | `07` |
| `encode_command(1, Command.HOME)` | 0x011 | `02` |
| `encode_setpoint(3, SetpointKind.VELOCITY, -1000)` | 0x01B | `01 18 FC FF FF` |
| `encode_status(2, 123456, AxisState.READY, 0x05, 0x01)` | 0x082 | `40 E2 01 00 02 05 01` |
| `encode_telemetry(6, -250000, 1500)` | 0x08E | `70 2F FC FF DC 05` |
