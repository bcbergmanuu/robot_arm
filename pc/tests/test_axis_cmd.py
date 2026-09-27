"""`robotarm axis {home,enable,disable,clear}` against a real-time TCP simulator."""

import math
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from robotarm import protocol as p
from robotarm.config import load_arm_config
from robotarm.master.axis_cmd import run_axis_command
from robotarm.sim.server import SimServer, run_realtime
from robotarm.sim.world import SimWorld
from robotarm.transport.tcp_bus import TcpBus

REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture
def cfg():
    return load_arm_config()


@pytest.fixture
def tcp_sim(cfg):
    """Real-time sim, every joint 3 deg from its home stop so homing takes well under a second."""
    q0 = {a.joint: a.home.position_rad - a.home.direction * math.radians(3) for a in cfg.axes}
    world = SimWorld(cfg, initial_q=q0)
    server = SimServer(world, port=0)
    stop = threading.Event()
    thread = threading.Thread(target=run_realtime, args=(world, server, False, stop), daemon=True)
    thread.start()
    yield server
    stop.set()
    thread.join(2)
    server.close()
    world.close()


def _url(server) -> str:
    return f"tcp://127.0.0.1:{server.port}"


def _status(server, node: int, timeout: float = 1.0) -> p.Status:
    bus = TcpBus(f"127.0.0.1:{server.port}")
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.05)
            decoded = p.decode(msg) if msg is not None else None
            if isinstance(decoded, p.Status) and decoded.node == node:
                return decoded
    finally:
        bus.shutdown()
    raise AssertionError(f"no STATUS from node {node}")


def _estop(server) -> None:
    bus = TcpBus(f"127.0.0.1:{server.port}")
    try:
        bus.send(p.encode_estop())
    finally:
        bus.shutdown()


def test_home_one_axis_leaves_it_homed_and_disabled(tcp_sim, cfg, capsys):
    node = cfg.axis_by_name("elbow").node
    assert run_axis_command(_url(tcp_sim), "home", node) == 0
    assert "homed" in capsys.readouterr().out
    st = _status(tcp_sim, node)
    assert st.homed and st.state == p.AxisState.DISABLED and not st.faults
    other = _status(tcp_sim, cfg.axis_by_name("shoulder").node)
    assert not other.homed  # only the requested axis


def test_enable_then_disable(tcp_sim, cfg, capsys):
    node = cfg.axis_by_name("gripper").node
    assert run_axis_command(_url(tcp_sim), "enable", node, hold_s=0.3) == 0
    assert "READY" in capsys.readouterr().out
    assert run_axis_command(_url(tcp_sim), "disable", node) == 0
    assert _status(tcp_sim, node).state == p.AxisState.DISABLED


def test_enable_refused_while_faulted_then_clear(tcp_sim, cfg, capsys):
    node = cfg.axis_by_name("gripper").node
    _estop(tcp_sim)
    assert run_axis_command(_url(tcp_sim), "enable", node) == 2
    err = capsys.readouterr().err.strip().splitlines()
    assert len(err) == 1 and err[0].startswith("error:") and "FAULT" in err[0]
    assert run_axis_command(_url(tcp_sim), "clear", node) == 0
    st = _status(tcp_sim, node)
    assert st.state == p.AxisState.DISABLED and not st.faults


def test_unknown_node_is_one_line_error(tcp_sim, capsys):
    assert run_axis_command(_url(tcp_sim), "home", 9) == 2
    err = capsys.readouterr().err.strip().splitlines()
    assert len(err) == 1 and err[0].startswith("error:")


def test_no_status_times_out_with_one_line_error(capsys):
    """A bus that carries no traffic (nothing answers): bounded wait, then `error:`."""
    world_cfg = load_arm_config()
    world = SimWorld(world_cfg)
    server = SimServer(world, port=0)  # accepts connections, but nobody steps the world
    try:
        t0 = time.monotonic()
        assert run_axis_command(_url(server), "enable", 1) == 2
        assert time.monotonic() - t0 < 5.0
        err = capsys.readouterr().err
        assert err.startswith("error:") and "timed out" in err
    finally:
        server.close()
        world.close()


def test_cli_entry_point(tcp_sim, cfg):
    node = cfg.axis_by_name("gripper").node
    r = subprocess.run([sys.executable, "-m", "robotarm", "axis", "enable", "--node", str(node),
                        "--bus", _url(tcp_sim)], capture_output=True, text=True, timeout=30, cwd=REPO_ROOT)
    assert r.returncode == 0, r.stderr
    assert "READY" in r.stdout
