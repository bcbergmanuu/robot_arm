"""CAN protocol: 1 Mbit/s, 11-bit standard ids, little-endian payloads.

id = (type << 3) | node, node 1-6 = axes, node 0 = broadcast.
Lower id value = higher bus priority.

The golden vector literals below are shared with
host/tests/test_protocol.c -- keep the two implementations in agreement.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag

import can

NODE_BROADCAST = 0

DUTY_SCALE = 10000


class MsgType(IntEnum):
    ESTOP = 0x00
    HEARTBEAT = 0x01
    COMMAND = 0x02
    SETPOINT = 0x03
    STATUS = 0x10
    TELEMETRY = 0x11


class Command(IntEnum):
    DISABLE = 0
    ENABLE = 1
    HOME = 2
    CLEAR_FAULT = 3


class SetpointKind(IntEnum):
    POSITION = 0
    VELOCITY = 1
    DUTY = 2


class AxisState(IntEnum):
    DISABLED = 0
    HOMING = 1
    READY = 2
    FAULT = 3


class Fault(IntFlag):
    WATCHDOG = 1
    OVERCURRENT = 2
    FOLLOWING = 4
    ESTOP = 8
    HOMING = 16


_SETPOINT_FMT = struct.Struct("<Bi")
_STATUS_FMT = struct.Struct("<iBBB")
_TELEMETRY_FMT = struct.Struct("<ih")


@dataclass(frozen=True)
class Estop:
    pass


@dataclass(frozen=True)
class Heartbeat:
    seq: int


@dataclass(frozen=True)
class CommandMsg:
    node: int
    command: Command


@dataclass(frozen=True)
class Setpoint:
    node: int
    kind: SetpointKind
    value: int


@dataclass(frozen=True)
class Status:
    node: int
    position: int
    state: AxisState
    faults: Fault
    homed: bool


@dataclass(frozen=True)
class Telemetry:
    node: int
    velocity: int
    current_ma: int


def make_id(type_: int, node: int) -> int:
    if not 0 <= node <= 7:
        raise ValueError(f"node out of range: {node}")
    return (int(type_) << 3) | node


def _node_of(arbitration_id: int) -> int:
    return arbitration_id & 0x7


def _type_of(arbitration_id: int) -> int:
    return arbitration_id >> 3


def encode_estop() -> can.Message:
    return can.Message(arbitration_id=make_id(MsgType.ESTOP, NODE_BROADCAST), data=b"", is_extended_id=False)


def encode_heartbeat(seq: int) -> can.Message:
    return can.Message(
        arbitration_id=make_id(MsgType.HEARTBEAT, NODE_BROADCAST),
        data=bytes([seq & 0xFF]),
        is_extended_id=False,
    )


def encode_command(node: int, cmd: int) -> can.Message:
    return can.Message(
        arbitration_id=make_id(MsgType.COMMAND, node),
        data=bytes([int(cmd) & 0xFF]),
        is_extended_id=False,
    )


def encode_setpoint(node: int, kind: int, value: int) -> can.Message:
    return can.Message(
        arbitration_id=make_id(MsgType.SETPOINT, node),
        data=_SETPOINT_FMT.pack(int(kind), value),
        is_extended_id=False,
    )


def encode_status(node: int, pos: int, state: int, faults: int, flags: int) -> can.Message:
    return can.Message(
        arbitration_id=make_id(MsgType.STATUS, node),
        data=_STATUS_FMT.pack(pos, int(state), int(faults), int(flags)),
        is_extended_id=False,
    )


def encode_telemetry(node: int, vel: int, current_ma: int) -> can.Message:
    return can.Message(
        arbitration_id=make_id(MsgType.TELEMETRY, node),
        data=_TELEMETRY_FMT.pack(vel, current_ma),
        is_extended_id=False,
    )


def decode(msg: can.Message) -> Estop | Heartbeat | CommandMsg | Setpoint | Status | Telemetry | None:
    type_ = _type_of(msg.arbitration_id)
    node = _node_of(msg.arbitration_id)
    data = bytes(msg.data)

    if type_ == MsgType.ESTOP and len(data) == 0:
        return Estop()
    if type_ == MsgType.HEARTBEAT and len(data) == 1:
        return Heartbeat(seq=data[0])
    if type_ == MsgType.COMMAND and len(data) == 1:
        return CommandMsg(node=node, command=Command(data[0]))
    if type_ == MsgType.SETPOINT and len(data) == _SETPOINT_FMT.size:
        kind, value = _SETPOINT_FMT.unpack(data)
        return Setpoint(node=node, kind=SetpointKind(kind), value=value)
    if type_ == MsgType.STATUS and len(data) == _STATUS_FMT.size:
        pos, state, faults, flags = _STATUS_FMT.unpack(data)
        return Status(node=node, position=pos, state=AxisState(state), faults=Fault(faults), homed=bool(flags & 0x01))
    if type_ == MsgType.TELEMETRY and len(data) == _TELEMETRY_FMT.size:
        vel, current_ma = _TELEMETRY_FMT.unpack(data)
        return Telemetry(node=node, velocity=vel, current_ma=current_ma)
    return None
