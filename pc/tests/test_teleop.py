import math
import os
import select
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass

import pytest

from robotarm import protocol as p
from robotarm.config import load_arm_config
from robotarm.master.arm_client import ArmClient
from robotarm.master.gamepad import FakeGamepad
from robotarm.master.teleop import Mode, Teleop, load_teleop_config
from robotarm.sim.harness import run_lockstep
from robotarm.sim.server import SimServer, run_realtime
from robotarm.sim.world import SimBus, SimWorld

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
UPDATE_PERIOD_MS = 20  # teleop loop_hz = 50


@pytest.fixture
def cfg():
    return load_arm_config()


@pytest.fixture
def tcfg():
    return load_teleop_config()


def near_home_start(cfg):
    return {a.joint: a.home.position_rad - a.home.direction * math.radians(5) for a in cfg.axes}


@dataclass
class Rig:
    world: SimWorld
    bus: SimBus
    client: ArmClient
    teleop: Teleop
    pad: FakeGamepad
    ms: int = 0

    def drive(self, seconds: float) -> None:
        """Lockstep: client.poll(t) every ms, teleop.update(pad.poll(), t) every 20 ms."""

        def on_ms(t: float) -> None:
            self.client.poll(t)
            self.ms += 1
            if self.ms % UPDATE_PERIOD_MS == 0:
                self.teleop.update(self.pad.poll(), t)

        run_lockstep(self.world, self.bus, seconds, on_ms=on_ms)

    def joint(self, name: str):
        return self.client.joints[self.client.cfg.axis_by_name(name).node]


def _mid_soft_limits(cfg):
    """Park position well inside every soft range: 0 rad, except the gripper (0..55 deg) at mid-range."""
    out = {}
    for a in cfg.axes:
        lo, hi = a.soft_limits_rad
        out[a.node] = 0.0 if lo < 0.0 < hi else (lo + hi) / 2
    return out


@pytest.fixture
def rig(cfg, tcfg):
    """Homed, READY arm parked mid-range, driven by a Teleop with a FakeGamepad (nothing pressed)."""
    world = SimWorld(cfg, initial_q=near_home_start(cfg))
    bus = SimBus(world)
    try:
        client = ArmClient(bus, cfg)
        client.home()
        run_lockstep(world, bus, 8.0, on_ms=client.poll)
        assert client.all_homed() and client.all_ready()
        for node, q in _mid_soft_limits(cfg).items():
            client.set_position(node, q)
        run_lockstep(world, bus, 5.0, on_ms=client.poll)
        pad = FakeGamepad()
        r = Rig(world, bus, client, Teleop(client, cfg, tcfg), pad)
        r.drive(0.1)
        yield r
    finally:
        bus.shutdown()
        world.close()


def press(pad: FakeGamepad, *names: str) -> None:
    pad.set(buttons=pad.state.buttons | frozenset(names))


def release(pad: FakeGamepad, *names: str) -> None:
    pad.set(buttons=pad.state.buttons - frozenset(names))


# --- config -------------------------------------------------------------------


def test_load_teleop_config(tcfg):
    assert tcfg.loop_hz == 50
    assert tcfg.deadzone == pytest.approx(0.12)
    assert tcfg.speed_scale == {"normal": 0.5, "slow": 0.15}
    assert tcfg.buttons["deadman"] == "l1"
    assert tcfg.buttons["estop"] == "circle"
    bindings = {b.joint: b for b in tcfg.joint_mode}
    assert bindings["j2"].axis == "ly" and bindings["j2"].sign == 1
    assert bindings["j4"].buttons == ("dpad_up", "dpad_down")
    assert bindings["j6"].triggers == ("l2", "r2")
    assert tcfg.raw_joystick_fallback["buttons"]["l1"] == 9


@pytest.mark.parametrize("binding", ["{axis: lx, triggers: [l2, r2]}", "{axis: lz}", "{triggers: [l2]}"])
def test_load_teleop_config_rejects_bad_binding(tmp_path, binding):
    bad = tmp_path / "teleop.yaml"
    bad.write_text(
        "deadzone: 0.1\nloop_hz: 50\nspeed_scale: {normal: 0.5, slow: 0.1}\n"
        f"buttons: {{deadman: l1}}\njoint_mode:\n  j1: {binding}\n"
    )
    with pytest.raises(ValueError):
        load_teleop_config(bad)


# --- brief-required behaviour ---------------------------------------------------


def test_no_motion_without_deadman(rig):
    q0 = rig.joint("shoulder").position_rad
    rig.pad.set(ly=1.0)
    rig.drive(1.0)
    assert not rig.teleop.armed
    assert abs(math.degrees(rig.joint("shoulder").position_rad - q0)) < 0.2


def test_deadman_jogs_shoulder_up(rig):
    q0 = rig.joint("shoulder").position_rad
    press(rig.pad, "l1")
    rig.pad.set(ly=1.0)
    rig.drive(1.0)
    assert rig.teleop.armed
    assert math.degrees(rig.joint("shoulder").position_rad - q0) > 10.0
    assert not rig.client.any_fault()


def test_releasing_deadman_stops(rig):
    press(rig.pad, "l1")
    rig.pad.set(ly=1.0)
    rig.drive(1.0)
    assert abs(math.degrees(rig.joint("shoulder").velocity_rad_s)) > 10.0

    release(rig.pad, "l1")  # stick still deflected
    rig.drive(0.5)
    assert not rig.teleop.armed
    assert abs(math.degrees(rig.joint("shoulder").velocity_rad_s)) < 1.0


def test_gamepad_disconnect_stops_and_requires_rearm(rig):
    press(rig.pad, "l1")
    rig.pad.set(ly=1.0)
    rig.drive(0.5)
    assert abs(math.degrees(rig.joint("shoulder").velocity_rad_s)) > 10.0

    rig.pad.set(connected=False)
    rig.drive(0.5)
    assert not rig.teleop.armed
    assert abs(math.degrees(rig.joint("shoulder").velocity_rad_s)) < 1.0

    rig.pad.set(connected=True)  # l1 still held, stick still deflected
    q_back = rig.joint("shoulder").position_rad
    rig.drive(1.0)
    assert not rig.teleop.armed
    assert abs(math.degrees(rig.joint("shoulder").position_rad - q_back)) < 0.2

    release(rig.pad, "l1")
    rig.drive(0.1)
    press(rig.pad, "l1")
    q_rearm = rig.joint("shoulder").position_rad
    rig.drive(1.0)
    assert rig.teleop.armed
    assert math.degrees(rig.joint("shoulder").position_rad - q_rearm) > 10.0


def test_circle_is_estop_even_when_not_armed(rig):
    press(rig.pad, "circle")
    rig.drive(0.2)
    assert not rig.teleop.armed
    assert all(js.state == p.AxisState.FAULT for js in rig.client.joints.values())
    assert all(js.faults & p.Fault.ESTOP for js in rig.client.joints.values())


def test_triggers_drive_gripper(rig):
    """R5: velocity = (l2 - r2) * max, so r2 closes: j6 decreases and stops at its lower soft limit (0 = closed)."""
    gripper = rig.client.cfg.axis_by_name("gripper")
    lo, _hi = gripper.soft_limits_rad
    q0 = rig.joint("gripper").position_rad
    press(rig.pad, "l1")
    rig.pad.set(r2=1.0)
    rig.drive(0.2)
    assert rig.joint("gripper").position_rad < q0 - math.radians(2)

    rig.drive(1.8)
    assert rig.joint("gripper").position_rad == pytest.approx(lo, abs=math.radians(1.0))
    assert rig.joint("gripper").position_rad >= lo - math.radians(0.5)
    assert abs(math.degrees(rig.joint("gripper").velocity_rad_s)) < 1.0
    assert not rig.client.any_fault()


def test_teleop_cli_without_sim_exits_2():
    r = subprocess.run([sys.executable, "-m", "robotarm", "teleop", "--bus", "tcp://127.0.0.1:1", "--fake-gamepad"],
                       capture_output=True, text=True, timeout=20)
    assert r.returncode == 2
    assert r.stderr.startswith("error:") and "Traceback" not in r.stderr


# --- further rules ----------------------------------------------------------------


def test_deadman_held_at_startup_does_not_arm(cfg, tcfg):
    world = SimWorld(cfg)
    bus = SimBus(world)
    try:
        client = ArmClient(bus, cfg)
        pad = FakeGamepad()
        press(pad, "l1")
        teleop = Teleop(client, cfg, tcfg)
        teleop.update(pad.poll(), 0.0)
        assert not teleop.armed
        release(pad, "l1")
        teleop.update(pad.poll(), 0.02)
        press(pad, "l1")
        teleop.update(pad.poll(), 0.04)
        assert teleop.armed
    finally:
        bus.shutdown()
        world.close()


def test_toggle_mode_only_when_not_armed(rig):
    assert rig.teleop.mode is Mode.JOINT
    press(rig.pad, "square")
    rig.drive(0.04)
    assert rig.teleop.mode is Mode.CARTESIAN
    release(rig.pad, "square")
    rig.drive(0.04)

    press(rig.pad, "l1")
    rig.drive(0.04)
    assert rig.teleop.armed
    press(rig.pad, "square")
    rig.drive(0.04)
    assert rig.teleop.mode is Mode.CARTESIAN  # ignored while armed


def test_cartesian_mode_does_not_move_yet(rig):
    press(rig.pad, "square")
    rig.drive(0.04)
    release(rig.pad, "square")
    rig.drive(0.04)
    assert rig.teleop.mode is Mode.CARTESIAN
    q0 = rig.joint("shoulder").position_rad
    press(rig.pad, "l1")
    rig.pad.set(ly=1.0)
    rig.drive(1.0)
    assert abs(math.degrees(rig.joint("shoulder").position_rad - q0)) < 0.2


def test_estop_then_clear_and_enable_buttons(rig):
    press(rig.pad, "circle")
    rig.drive(0.1)
    release(rig.pad, "circle")
    rig.drive(0.1)
    assert rig.client.any_fault()

    press(rig.pad, "options")  # clear_faults
    rig.drive(0.1)
    release(rig.pad, "options")
    press(rig.pad, "cross")  # enable
    rig.drive(0.2)
    assert rig.client.all_ready()
    assert not rig.client.any_fault()


def test_home_button_homes_and_backs_off_inside_soft_limits(cfg, tcfg):
    """The not-armed zero-velocity stream must not cut short the firmware's post-home back-off."""
    world = SimWorld(cfg, initial_q=near_home_start(cfg))
    bus = SimBus(world)
    try:
        client = ArmClient(bus, cfg)
        pad = FakeGamepad()
        r = Rig(world, bus, client, Teleop(client, cfg, tcfg), pad)
        # Home promptly: within ~0.15 s wrist_bend sags onto its end stop, and homing an
        # axis that already rests on its stop trips OVERCURRENT (a firmware homing issue
        # outside teleop: AXIS_HOME_SETTLE_MS + AXIS_HOME_STALL_MS > overcurrent_ms).
        r.drive(0.02)
        press(pad, "triangle")
        r.drive(0.1)
        release(pad, "triangle")
        r.drive(9.0)
        assert client.all_homed() and client.all_ready()
        for a in cfg.axes:
            lo, hi = a.soft_limits_rad
            q = client.joints[a.node].position_rad
            assert lo - math.radians(0.5) <= q <= hi + math.radians(0.5), a.name
    finally:
        bus.shutdown()
        world.close()


def test_status_line_is_one_line(rig):
    press(rig.pad, "l1")
    rig.drive(0.1)
    line = rig.teleop.status_line()
    assert "\n" not in line
    assert "JOINT" in line and "ARMED" in line


# --- ArmClient runner errors (R19) ------------------------------------------------


class _DyingBus(SimBus):
    """A SimBus whose send() starts raising after a few frames, like a broken TCP pipe."""

    def __init__(self, world, ok_sends: int):
        super().__init__(world)
        self._ok_sends = ok_sends

    def send(self, msg, timeout=None):
        if self._ok_sends <= 0:
            raise BrokenPipeError(32, "Broken pipe")
        self._ok_sends -= 1
        super().send(msg, timeout)


def test_arm_client_runner_records_bus_error_and_stops(cfg):
    world = SimWorld(cfg)
    bus = _DyingBus(world, ok_sends=3)
    try:
        client = ArmClient(bus, cfg)
        assert not client.alive()
        client.start()
        deadline = time.monotonic() + 1.0
        while client.alive() and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not client.alive()
        assert isinstance(client.error, BrokenPipeError)
        client.close()  # the DISABLE broadcast fails too; close() must not raise
    finally:
        bus.shutdown()
        world.close()


# --- CLI end-to-end over TCP ---------------------------------------------------------


@pytest.fixture
def tcp_sim(cfg):
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


def _start_teleop_cli(port: int) -> subprocess.Popen:
    """Launch the CLI (binary pipes: text mode would translate its "\\r") and wait,
    bounded, for the first "\\r" -- the in-place status line, i.e. the loop is
    running. Anything before it is start-up chatter (e.g. pygame's banner)."""
    proc = subprocess.Popen(
        [sys.executable, "-m", "robotarm", "teleop", "--bus", f"tcp://127.0.0.1:{port}", "--fake-gamepad"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=REPO_ROOT,
    )
    fd = proc.stdout.fileno()
    deadline = time.monotonic() + 20.0
    while True:
        ready, _, _ = select.select([fd], [], [], max(0.0, deadline - time.monotonic()))
        if not ready:
            proc.kill()
            proc.communicate()
            pytest.fail("teleop CLI printed no status line within 20 s")
        ch = os.read(fd, 1)
        if not ch:
            _out, err = proc.communicate()
            pytest.fail(f"teleop CLI exited early ({proc.returncode}): {err.decode()}")
        if ch == b"\r":
            return proc


def test_teleop_cli_ctrl_c_exits_0(tcp_sim):
    proc = _start_teleop_cli(tcp_sim.port)
    try:
        time.sleep(0.2)
        proc.send_signal(signal.SIGINT)
        _out, err = proc.communicate(timeout=5)
        err = err.decode()
        assert proc.returncode == 0, err
        assert "Traceback" not in err
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()


def test_teleop_cli_exits_2_when_sim_goes_away(tcp_sim):
    proc = _start_teleop_cli(tcp_sim.port)
    try:
        time.sleep(0.2)
        tcp_sim.close()
        t0 = time.monotonic()
        _out, err = proc.communicate(timeout=5)
        err = err.decode()
        assert time.monotonic() - t0 < 1.0
        assert proc.returncode == 2
        assert err.startswith("error: bus lost:") and "Traceback" not in err
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
