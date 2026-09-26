import pytest

from robotarm.config import load_arm_config
from robotarm.protocol import (
    AxisState,
    Command,
    Fault,
    SetpointKind,
    Status,
    decode,
    encode_command,
    encode_heartbeat,
    encode_setpoint,
)
from robotarm.sim.native import NativeAxis

NODE = 2  # shoulder


@pytest.fixture(scope="module")
def cfg():
    return load_arm_config()


def _statuses(axis: NativeAxis) -> list[Status]:
    return [s for s in (decode(m) for m in axis.recv_all()) if isinstance(s, Status)]


def test_duty_command_moves_joint_positive_within_a_second(cfg):
    axis_cfg = cfg.axis(NODE)
    j_joint = axis_cfg.motor.j_rotor * axis_cfg.gear_ratio**2 + 0.05  # pure inertia, referred to the joint

    axis = NativeAxis(NODE, cfg)
    axis.send(encode_command(NODE, Command.ENABLE))
    axis.send(encode_setpoint(NODE, SetpointKind.DUTY, 3000))  # 0.3

    q = qd = 0.0
    dt = 1e-3
    statuses: list[Status] = []
    for i in range(1000):
        if i % 100 == 0:
            axis.send(encode_heartbeat(i & 0xFF))
        torque = axis.step(1, q, qd)
        qd += (torque / j_joint) * dt
        q += qd * dt
        statuses += _statuses(axis)

    assert qd > 0.0
    assert q > 0.0
    assert statuses and all(s.node == NODE for s in statuses)

    dbg = axis.debug()
    assert dbg.state == AxisState.READY
    assert dbg.vel > 0.0


def test_watchdog_fault_after_250ms_without_heartbeats(cfg):
    axis = NativeAxis(NODE, cfg)
    axis.send(encode_command(NODE, Command.ENABLE))

    statuses: list[Status] = []
    for _ in range(250):
        axis.step(1, 0.0, 0.0)
        statuses += _statuses(axis)

    assert statuses, "expected periodic STATUS frames"
    assert statuses[-1].state == AxisState.FAULT
    assert Fault.WATCHDOG in statuses[-1].faults
