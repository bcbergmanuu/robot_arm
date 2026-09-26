import math

import pytest

from robotarm import protocol as p
from robotarm.config import load_arm_config
from robotarm.sim.harness import run_lockstep
from robotarm.sim.world import SimBus, SimWorld


@pytest.fixture
def cfg():
    return load_arm_config()


def heartbeat(bus):
    seq = [0]

    def on_ms(t):
        if round(t * 1000) % 50 == 0:
            bus.send(p.encode_heartbeat(seq[0] & 0xFF))
            seq[0] += 1
    return on_ms


def latest_statuses(bus) -> dict[int, p.Status]:
    """Drain the receive queue once; return the most recent STATUS per node."""
    statuses: dict[int, p.Status] = {}
    while (msg := bus.recv(timeout=0)) is not None:
        d = p.decode(msg)
        if isinstance(d, p.Status):
            statuses[d.node] = d
    return statuses


def latest_status(bus, node):
    return latest_statuses(bus).get(node)


def near_home_start(cfg):
    return {a.joint: a.home.position_rad - a.home.direction * math.radians(5) for a in cfg.axes}


def test_axes_report_disabled_after_power_up(cfg):
    world = SimWorld(cfg)
    bus = SimBus(world)
    run_lockstep(world, bus, 1.0)
    statuses = latest_statuses(bus)
    for node in range(1, 7):
        assert statuses[node].state == p.AxisState.DISABLED


def test_all_axes_home_and_back_off(cfg):
    world = SimWorld(cfg, initial_q=near_home_start(cfg))
    bus = SimBus(world)
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.HOME))
    run_lockstep(world, bus, 8.0, on_ms=heartbeat(bus))
    q = world.joint_positions()
    for i, a in enumerate(cfg.axes):
        edge = a.soft_limits_rad[0] if a.home.direction < 0 else a.soft_limits_rad[1]
        assert q[i] == pytest.approx(edge, abs=math.radians(1.0)), a.name


def test_position_move_on_shoulder(cfg):
    shoulder = cfg.axis_by_name("shoulder")
    world = SimWorld(cfg, initial_q=near_home_start(cfg))
    bus = SimBus(world)
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.HOME))
    run_lockstep(world, bus, 8.0, on_ms=heartbeat(bus))
    target = math.radians(30)
    bus.send(p.encode_setpoint(shoulder.node, p.SetpointKind.POSITION, shoulder.rad_to_counts(target)))
    run_lockstep(world, bus, 6.0, on_ms=heartbeat(bus))
    assert world.joint_positions()[1] == pytest.approx(target, abs=math.radians(0.5))
    st = latest_status(bus, shoulder.node)
    assert st.state == p.AxisState.READY and st.faults == 0


def test_axes_fault_when_heartbeats_stop(cfg):
    world = SimWorld(cfg)
    bus = SimBus(world)
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.ENABLE))
    run_lockstep(world, bus, 0.5)
    st = latest_status(bus, 3)
    assert st.state == p.AxisState.FAULT and p.Fault.WATCHDOG in st.faults


def test_enable_at_power_up_far_from_zero_stays_ready(cfg):
    # Encoders read ~30 deg (>> max_following_error) at the very first tick and ENABLE
    # arrives before it: the axes must resync instead of tripping FOLLOWING.
    world = SimWorld(cfg, initial_q={a.joint: math.radians(30) for a in cfg.axes})
    bus = SimBus(world)
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.ENABLE))
    run_lockstep(world, bus, 0.2, on_ms=heartbeat(bus))
    statuses = latest_statuses(bus)
    for node in range(1, 7):
        assert statuses[node].state == p.AxisState.READY and statuses[node].faults == 0, node


def test_outgoing_frames_carry_sim_time(cfg):
    world = SimWorld(cfg)
    bus = SimBus(world)
    run_lockstep(world, bus, 0.1)
    msgs = []
    while (msg := bus.recv(timeout=0)) is not None:
        msgs.append(msg)
    assert msgs
    assert all(0.0 < m.timestamp <= 0.1 + 1e-9 for m in msgs)
    assert world.time == pytest.approx(0.1)


def test_lockstep_is_deterministic(cfg):
    finals = []
    for _ in range(2):
        world = SimWorld(cfg, initial_q=near_home_start(cfg))
        bus = SimBus(world)
        bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.HOME))
        run_lockstep(world, bus, 1.5, on_ms=heartbeat(bus))
        finals.append(world.joint_positions())
    assert (finals[0] == finals[1]).all()


def test_tune_position_step_metrics(cfg):
    from robotarm.sim.tune import position_step

    m = position_step(cfg, "gripper")
    assert m.overshoot_pct < 5.0
    assert m.settle_time_s < 1.0
    assert m.steady_error < 0.1  # deg
    assert abs(m.home_offset_deg) < 1.0
    assert not m.faults
