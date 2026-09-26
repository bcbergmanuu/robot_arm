import can
import pytest

from robotarm import protocol as p


def frame(msg: can.Message):
    return msg.arbitration_id, bytes(msg.data)


def test_golden_vectors():
    assert frame(p.encode_estop()) == (0x000, b"")
    assert frame(p.encode_heartbeat(7)) == (0x008, bytes([0x07]))
    assert frame(p.encode_command(1, p.Command.HOME)) == (0x011, bytes([0x02]))
    assert frame(p.encode_setpoint(3, p.SetpointKind.VELOCITY, -1000)) == (0x01B, bytes([0x01, 0x18, 0xFC, 0xFF, 0xFF]))
    assert frame(p.encode_status(2, 123456, p.AxisState.READY, 0x05, 0x01)) == (
        0x082, bytes([0x40, 0xE2, 0x01, 0x00, 0x02, 0x05, 0x01]))
    assert frame(p.encode_telemetry(6, -250000, 1500)) == (0x08E, bytes([0x70, 0x2F, 0xFC, 0xFF, 0xDC, 0x05]))


def test_decode_status():
    msg = p.decode(p.encode_status(2, 123456, p.AxisState.READY, 0x05, 0x01))
    assert msg == p.Status(node=2, position=123456, state=p.AxisState.READY,
                           faults=p.Fault.WATCHDOG | p.Fault.FOLLOWING, homed=True)


def test_decode_rejects_bad_length_and_unknown_type():
    bad = can.Message(arbitration_id=p.make_id(p.MsgType.STATUS, 2), data=b"\x00\x01", is_extended_id=False)
    assert p.decode(bad) is None
    assert p.decode(can.Message(arbitration_id=0x7F0, data=b"", is_extended_id=False)) is None


@pytest.mark.parametrize("value", [-(2**31), -1, 0, 1, 2**31 - 1])
def test_setpoint_roundtrip(value):
    assert p.decode(p.encode_setpoint(5, p.SetpointKind.POSITION, value)) == p.Setpoint(5, p.SetpointKind.POSITION, value)
