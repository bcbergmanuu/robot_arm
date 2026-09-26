import math
import threading
import time

import pytest

from robotarm import protocol as p
from robotarm.config import load_arm_config
from robotarm.master.arm_client import ArmClient, JointState
from robotarm.sim.harness import run_lockstep
from robotarm.sim.server import SimServer, run_realtime
from robotarm.sim.world import SimBus, SimWorld
from robotarm.transport.tcp_bus import TcpBus


@pytest.fixture
def cfg():
    return load_arm_config()


@pytest.fixture
def sim(cfg):
    """Factory for (world, bus) pairs; closes every bus and world after the test."""
    created = []

    def make(initial_q=None):
        world = SimWorld(cfg, initial_q=initial_q)
        bus = SimBus(world)
        created.append((world, bus))
        return world, bus

    yield make
    for world, bus in created:
        bus.shutdown()
        world.close()


def near_home_start(cfg):
    return {a.joint: a.home.position_rad - a.home.direction * math.radians(5) for a in cfg.axes}


def test_heartbeats_keep_axes_ready(cfg, sim):
    world, bus = sim()
    client = ArmClient(bus, cfg)
    client.enable()
    run_lockstep(world, bus, 3.0, on_ms=client.poll)
    assert client.all_ready()
    assert not client.any_fault()


def test_home_reaches_all_homed(cfg, sim):
    world, bus = sim(near_home_start(cfg))
    client = ArmClient(bus, cfg)
    client.home()
    run_lockstep(world, bus, 8.0, on_ms=client.poll)
    assert client.all_homed()


def test_set_position_reaches_target(cfg, sim):
    world, bus = sim(near_home_start(cfg))
    client = ArmClient(bus, cfg)
    client.home()
    run_lockstep(world, bus, 8.0, on_ms=client.poll)

    shoulder = cfg.axis_by_name("shoulder")
    client.set_position(shoulder.node, 0.5)
    run_lockstep(world, bus, 6.0, on_ms=client.poll)

    assert client.joints[shoulder.node].position_rad == pytest.approx(0.5, abs=0.01)


def test_set_position_clamps_to_soft_limit(cfg, sim):
    world, bus = sim(near_home_start(cfg))
    client = ArmClient(bus, cfg)
    client.home()
    run_lockstep(world, bus, 8.0, on_ms=client.poll)

    shoulder = cfg.axis_by_name("shoulder")
    client.set_position(shoulder.node, 10.0)
    run_lockstep(world, bus, 6.0, on_ms=client.poll)

    assert client.joints[shoulder.node].position_rad == pytest.approx(
        shoulder.soft_limits_rad[1], abs=math.radians(1.0)
    )
    assert not client.any_fault()


def test_estop_then_clear_and_enable(cfg, sim):
    world, bus = sim()
    client = ArmClient(bus, cfg)
    client.enable()
    run_lockstep(world, bus, 0.5, on_ms=client.poll)

    client.estop()
    run_lockstep(world, bus, 0.5, on_ms=client.poll)
    assert all(js.state == p.AxisState.FAULT for js in client.joints.values())
    assert all(js.faults & p.Fault.ESTOP for js in client.joints.values())

    client.clear_faults()
    client.enable()
    run_lockstep(world, bus, 0.5, on_ms=client.poll)
    assert client.all_ready()


def test_connected_false_before_status_true_after(cfg, sim):
    world, bus = sim()
    client = ArmClient(bus, cfg)
    assert not client.connected(0.0)
    run_lockstep(world, bus, 0.05, on_ms=client.poll)
    assert client.connected(world.time)


def test_process_ignores_garbage_frame():
    import can

    cfg = load_arm_config()
    world = SimWorld(cfg)
    bus = SimBus(world)
    try:
        client = ArmClient(bus, cfg)
        garbage = can.Message(arbitration_id=0x7F0, data=b"\xff\xff\xff", is_extended_id=False)
        client.process(garbage)  # must not raise
        bad_command = can.Message(
            arbitration_id=p.make_id(p.MsgType.COMMAND, 1), data=bytes([0xFF]), is_extended_id=False
        )
        client.process(bad_command)  # must not raise
    finally:
        bus.shutdown()
        world.close()


def test_joint_state_defaults():
    js = JointState(node=1, name="hip")
    assert js.state == p.AxisState.DISABLED
    assert js.faults == p.Fault(0)
    assert js.homed is False
    assert js.last_seen is None


def test_close_sends_disable_broadcast(cfg, sim):
    world, bus = sim()
    client = ArmClient(bus, cfg)
    client.enable()
    run_lockstep(world, bus, 0.5, on_ms=client.poll)
    assert client.all_ready()

    client.close()
    run_lockstep(world, bus, 0.5, on_ms=client.poll)
    assert all(js.state == p.AxisState.DISABLED for js in client.joints.values())


def test_start_and_close_stop_the_background_thread(cfg, sim):
    world, bus = sim()
    client = ArmClient(bus, cfg)
    client.start()
    time.sleep(0.05)
    client.close()
    time.sleep(0.05)
    # Black-box: no thread named "ArmClient" should still be running.
    assert not any(t.name == "ArmClient" for t in threading.enumerate())


# --- Fix round 1 additions (controller findings, R18) ---


@pytest.mark.parametrize("bogus_timestamp", [0.0, time.time()])
def test_connected_ignores_msg_timestamp_uses_poll_now(cfg, sim, bogus_timestamp):
    """R18: last_seen must come from poll()'s `now`, never from msg.timestamp --
    a frame stamped 0.0 (TcpBus) or epoch time.time() (a real python-can backend)
    must still make connected() true once poll() has run with a monotonic-style now."""
    world, bus = sim()
    client = ArmClient(bus, cfg)
    now = time.monotonic()
    client.poll(now)  # establishes self._now on this clock; also drains any pending frames

    for axis in cfg.axes:
        msg = p.encode_status(axis.node, 0, p.AxisState.DISABLED, 0, 0)
        msg.timestamp = bogus_timestamp
        client.process(msg)

    assert client.connected(now)


def test_connected_over_tcp_then_stale_after_server_stops(cfg):
    """End-to-end over TcpBus (real-time SimServer), not the in-process SimBus:
    connected() must go true using the background thread's own wall clock, and
    must go false again once the sim stops feeding new STATUS frames.

    Stops the realtime driving loop (sim stop) rather than closing the TCP
    socket, so the client's still-running background thread keeps sending
    heartbeats successfully (just into a world that no longer advances or
    replies) instead of hitting a broken pipe -- that socket-error/reconnect
    path is out of scope for this fix (R18 is about the liveness clock)."""
    world = SimWorld(cfg)
    server = SimServer(world, port=0)
    stop = threading.Event()
    thread = threading.Thread(target=run_realtime, args=(world, server, False, stop), daemon=True)
    thread.start()

    bus = TcpBus(f"127.0.0.1:{server.port}")
    client = ArmClient(bus, cfg)
    client.start()
    try:
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not client.connected(time.monotonic()):
            time.sleep(0.02)
        assert client.connected(time.monotonic())
        assert all(js.state == p.AxisState.DISABLED for js in client.joints.values())

        stop.set()  # sim stop: run_realtime stops stepping/broadcasting; socket stays open
        thread.join(2)

        stale_s = 0.5
        deadline = time.monotonic() + stale_s + 1.0  # margin, bounded
        went_stale = False
        while time.monotonic() < deadline:
            if not client.connected(time.monotonic(), stale_s=stale_s):
                went_stale = True
                break
            time.sleep(0.02)
        assert went_stale
    finally:
        client.close()
        bus.shutdown()
        server.close()
        world.close()
