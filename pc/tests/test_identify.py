import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from robotarm import protocol as p
from robotarm.analysis.steptest import load_step_csv
from robotarm.config import load_arm_config
from robotarm.master.arm_client import ArmClient
from robotarm.master.identify import IdentifyRun, _duty_refusal, _state_refusal, run_identify
from robotarm.sim.harness import run_lockstep
from robotarm.sim.server import SimServer, run_realtime
from robotarm.sim.world import SimBus, SimWorld
from robotarm.transport.tcp_bus import TcpBus

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


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


def test_identify_records_step_csv(cfg, sim, tmp_path):
    """Node 2 (shoulder) starting mid-range: run the step experiment in lockstep,
    check the recorded samples and the written CSV round-trips through stepfit's loader."""
    axis = cfg.axis_by_name("shoulder")
    # 0 rad is the arm's low-inertia, near-balanced "all-zero" pose (docs/simulator.md); well
    # inside the shoulder's soft limits ([-25, 100] deg), so this is a mid-range start, not one
    # pressed against a stop -- but far enough from either stop that a 20 deg on-phase can't hit one.
    world, bus = sim({axis.joint: 0.0})

    client = ArmClient(bus, cfg)
    run = IdentifyRun(client, axis.node, duty=axis.max_duty, pre_s=0.08, on_s=0.08, post_s=0.04)

    def on_ms(t: float) -> None:
        client.poll(t)
        run.update(t)

    run_lockstep(world, bus, 0.3, on_ms=on_ms)

    assert len(run.samples) >= 180

    pre_end = round(0.08 / 0.001)
    on_end = pre_end + round(0.08 / 0.001)
    positions = [s[1] for s in run.samples]
    pre_positions = positions[:pre_end]
    assert max(pre_positions) - min(pre_positions) <= 2  # flat (duty 0) during the pre-phase
    assert positions[on_end - 1] - positions[pre_end] > 5  # positive duty increases position

    csv_path = tmp_path / "output.csv"
    run.write_csv(csv_path)
    data = load_step_csv(csv_path)
    assert len(data.pos) == len(run.samples)
    assert data.duty.max() == pytest.approx(axis.max_duty, abs=1e-6)
    assert list(data.pos) == positions


def test_identify_run_disables_axis_when_finished(cfg, sim):
    axis = cfg.axis_by_name("shoulder")
    world, bus = sim()
    client = ArmClient(bus, cfg)
    run = IdentifyRun(client, axis.node, duty=axis.max_duty, pre_s=0.02, on_s=0.02, post_s=0.01)

    def on_ms(t: float) -> None:
        client.poll(t)
        run.update(t)

    run_lockstep(world, bus, 0.2, on_ms=on_ms)

    assert client.joints[axis.node].state == p.AxisState.DISABLED


def test_duty_refusal_exceeds_max_duty(cfg):
    axis = cfg.axis_by_name("shoulder")
    assert _duty_refusal(axis, axis.max_duty + 0.01) is not None
    assert _duty_refusal(axis, axis.max_duty) is None
    assert _duty_refusal(axis, -axis.max_duty) is None


def test_state_refusal_only_disabled_or_ready_allowed(cfg):
    axis = cfg.axis_by_name("shoulder")
    assert _state_refusal(axis, p.AxisState.DISABLED) is None
    assert _state_refusal(axis, p.AxisState.READY) is None
    assert _state_refusal(axis, p.AxisState.FAULT) is not None
    assert _state_refusal(axis, p.AxisState.HOMING) is not None


# --- Fix round 1: run_identify (CLI body) end-to-end over a real-time TCP sim -------------


@pytest.fixture
def tcp_sim(cfg):
    """A real-time SimServer + run_realtime thread, like robotarm.bus.open_bus's TcpBus target."""
    world = SimWorld(cfg)
    server = SimServer(world, port=0)
    stop = threading.Event()
    thread = threading.Thread(target=run_realtime, args=(world, server, False, stop), daemon=True)
    thread.start()
    yield server
    stop.set()
    thread.join(2)
    server.close()
    world.close()


def test_run_identify_over_tcp_writes_readable_csv(tcp_sim, cfg, tmp_path):
    axis = cfg.axis_by_name("shoulder")
    out = tmp_path / "out.csv"
    rc = run_identify(f"tcp://127.0.0.1:{tcp_sim.port}", axis.node, duty=axis.max_duty, out=out,
                       pre_s=0.02, on_s=0.02, post_s=0.01)
    assert rc == 0
    data = load_step_csv(out)
    assert len(data.pos) > 0
    assert data.duty.max() == pytest.approx(axis.max_duty, abs=1e-6)


def test_run_identify_exits_2_when_sim_goes_away(tcp_sim, cfg, tmp_path, capsys):
    """The sim (server + realtime loop) disappears mid-run: the ArmClient runner dies on its
    next heartbeat send, run_identify must report it as `error: bus lost: ...` and exit 2 --
    not hang, and not leave a traceback on stderr."""
    axis = cfg.axis_by_name("shoulder")
    out = tmp_path / "out.csv"
    result: dict[str, int] = {}

    def target() -> None:
        result["rc"] = run_identify(f"tcp://127.0.0.1:{tcp_sim.port}", axis.node, duty=axis.max_duty,
                                    out=out, pre_s=0.05, on_s=2.0, post_s=0.05)

    thread = threading.Thread(target=target)
    thread.start()
    time.sleep(0.2)  # well past the state check, safely inside the 2 s on-phase
    t0 = time.monotonic()
    tcp_sim.close()
    thread.join(timeout=5)
    assert not thread.is_alive()
    assert time.monotonic() - t0 < 1.0

    assert result["rc"] == 2
    err = capsys.readouterr().err
    assert "error: bus lost:" in err and "Traceback" not in err
    assert "partial recording" in err  # the samples recorded before the bus died were still written
    assert load_step_csv(out).pos.size > 0


def _start_identify_cli(port: int, out: Path, node: int, duty: float, on_s: float) -> subprocess.Popen:
    return subprocess.Popen(
        [sys.executable, "-m", "robotarm", "identify", "--bus", f"tcp://127.0.0.1:{port}",
         "--node", str(node), "--duty", f"{duty:g}", "--out", str(out),
         "--pre", "0.05", "--on", str(on_s), "--post", "0.05"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=REPO_ROOT,
    )


def _recv_status(bus: TcpBus, node: int, timeout: float) -> p.Status | None:
    """The last STATUS seen from `node` within `timeout`, returning early once it's DISABLED."""
    deadline = time.monotonic() + timeout
    last: p.Status | None = None
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        decoded = p.decode(msg) if msg is not None else None
        if isinstance(decoded, p.Status) and decoded.node == node:
            last = decoded
            if decoded.state == p.AxisState.DISABLED:
                return decoded
    return last


def test_identify_cli_ctrl_c_exits_cleanly_with_partial_csv_and_disables(tcp_sim, cfg, tmp_path):
    axis = cfg.axis_by_name("shoulder")
    out = tmp_path / "partial.csv"
    proc = _start_identify_cli(tcp_sim.port, out, axis.node, axis.max_duty, on_s=4.0)
    try:
        time.sleep(0.5)  # well past start-up/bus-open/state-check, safely inside the on-phase
        proc.send_signal(signal.SIGINT)
        _out, err = proc.communicate(timeout=5)
        err = err.decode()
        assert proc.returncode == 0, err
        assert "Traceback" not in err
        assert "partial recording" in err

        assert out.exists()
        data = load_step_csv(out)
        assert len(data.pos) > 0

        bus2 = TcpBus(f"127.0.0.1:{tcp_sim.port}")
        try:
            status = _recv_status(bus2, axis.node, timeout=2.0)
            assert status is not None and status.state == p.AxisState.DISABLED
        finally:
            bus2.shutdown()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()


def test_identify_default_out_is_a_scratch_name_not_output_txt(tcp_sim, cfg, tmp_path, monkeypatch):
    """The default must not overwrite the committed bench recording output.txt."""
    from robotarm.cli import build_parser

    args = build_parser().parse_args(["identify", "--bus", "sim", "--node", "2"])
    assert args.out is None
    monkeypatch.chdir(tmp_path)
    axis = cfg.axis_by_name("shoulder")
    rc = run_identify(f"tcp://127.0.0.1:{tcp_sim.port}", axis.node, duty=axis.max_duty,
                      pre_s=0.02, on_s=0.02, post_s=0.01)
    assert rc == 0
    assert (tmp_path / f"identify_node{axis.node}.csv").exists()
    assert not (tmp_path / "output.txt").exists()


def test_identify_heartbeats_are_keepalive_gated(tcp_sim, cfg, tmp_path, monkeypatch):
    """Like teleop (R23): the runner's HEARTBEATs must stop if the identify loop stalls, so the
    client is built with a keepalive timeout and the loop checks in on every iteration."""
    import robotarm.master.identify as identify_mod

    created = []

    class RecordingClient(identify_mod.ArmClient):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.keepalives = 0
            created.append(self)

        def keepalive(self, now):
            self.keepalives += 1
            super().keepalive(now)

    monkeypatch.setattr(identify_mod, "ArmClient", RecordingClient)
    axis = cfg.axis_by_name("shoulder")
    rc = run_identify(f"tcp://127.0.0.1:{tcp_sim.port}", axis.node, duty=axis.max_duty, out=tmp_path / "o.csv",
                      pre_s=0.02, on_s=0.02, post_s=0.01)
    assert rc == 0
    (client,) = created
    assert client._keepalive_timeout is not None and client._keepalive_timeout <= 0.2
    assert client.keepalives >= 10


def test_identify_internal_error_is_one_line(tcp_sim, cfg, tmp_path, monkeypatch, capsys):
    import robotarm.master.identify as identify_mod

    def boom(self, now):
        raise RuntimeError("kaboom")

    monkeypatch.setattr(identify_mod.IdentifyRun, "update", boom)
    axis = cfg.axis_by_name("shoulder")
    rc = run_identify(f"tcp://127.0.0.1:{tcp_sim.port}", axis.node, duty=axis.max_duty, out=tmp_path / "o.csv")
    assert rc == 2
    err = capsys.readouterr().err.strip().splitlines()
    assert err[-1].startswith("error: internal error:") and "kaboom" in err[-1]
    assert not any("Traceback" in line for line in err)
