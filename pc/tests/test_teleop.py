import dataclasses
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
from robotarm.master.kinematics import Kinematics, Pose
from robotarm.master import teleop as teleop_mod
from robotarm.master.teleop import JointBinding, Mode, Teleop, load_teleop_config
from robotarm.sim.harness import run_lockstep
from robotarm.sim.server import SimServer, run_realtime
from robotarm.sim.world import SimBus, SimWorld

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
UPDATE_PERIOD_MS = 20  # teleop loop_hz = 50
KEEPALIVE_TIMEOUT_S = 0.1  # as the CLI uses (R23)


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
    stalled: bool = False  # simulate a hung teleop loop: client.poll continues, keepalive/update don't

    def drive(self, seconds: float) -> None:
        """Lockstep: client.poll(t) every ms; like the CLI loop, client.keepalive(t) then
        teleop.update(pad.poll(), t) every 20 ms."""

        def on_ms(t: float) -> None:
            self.client.poll(t)
            self.ms += 1
            if self.ms % UPDATE_PERIOD_MS == 0 and not self.stalled:
                self.client.keepalive(t)
                self.teleop.update(self.pad.poll(), t)

        run_lockstep(self.world, self.bus, seconds, on_ms=on_ms)

    def joint(self, name: str):
        return self.client.joints[self.client.cfg.axis_by_name(name).node]


def make_client(bus, cfg):
    """Built the way the CLI builds it (R23): HEARTBEATs only while keepalive() is fresh."""
    return ArmClient(bus, cfg, keepalive_timeout=KEEPALIVE_TIMEOUT_S)


def alive_poll(client):
    """on_ms for plain lockstep runs with a keepalive client: keepalive + poll every ms."""

    def on_ms(t: float) -> None:
        client.keepalive(t)
        client.poll(t)

    return on_ms


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
        client = make_client(bus, cfg)
        client.home()
        run_lockstep(world, bus, 8.0, on_ms=alive_poll(client))
        assert client.all_homed() and client.all_ready()
        for node, q in _mid_soft_limits(cfg).items():
            client.set_position(node, q)
        run_lockstep(world, bus, 5.0, on_ms=alive_poll(client))
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
        client = make_client(bus, cfg)
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
    """Homing via Triangle: teleop must not cut short the firmware's post-home back-off (R21: no setpoints while idle)."""
    world = SimWorld(cfg, initial_q=near_home_start(cfg))
    bus = SimBus(world)
    try:
        client = make_client(bus, cfg)
        pad = FakeGamepad()
        r = Rig(world, bus, client, Teleop(client, cfg, tcfg), pad)
        r.drive(3.0)  # let wrist_bend sag onto its stop first: homing must cope (R20)
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


def _record_velocity_setpoints(client):
    """Wrap client.set_velocity so the test can see every (node, rad_s) teleop sends."""
    sent = []
    real = client.set_velocity

    def spy(node, rad_s):
        sent.append((node, rad_s))
        real(node, rad_s)

    client.set_velocity = spy
    return sent


def test_idle_sends_no_setpoints_only_zeros_after_disarm_or_loss(rig):
    """R21: while not jogging, teleop sends nothing -- except zero velocity to every jog
    axis for 3 updates after jogging stops and after the pad is lost."""
    n_jog = len(rig.teleop.cfg.joint_mode)
    sent = _record_velocity_setpoints(rig.client)

    rig.drive(0.2)  # 10 idle updates
    assert sent == []

    press(rig.pad, "l1")
    rig.pad.set(ly=1.0)
    rig.drive(0.2)
    assert len(sent) == 10 * n_jog and any(v != 0.0 for _n, v in sent)

    sent.clear()
    release(rig.pad, "l1")  # disarm edge
    rig.drive(0.2)
    assert len(sent) == 3 * n_jog and all(v == 0.0 for _n, v in sent)

    sent.clear()
    rig.pad.set(connected=False)  # pad loss while idle
    rig.drive(0.2)
    assert len(sent) == 3 * n_jog and all(v == 0.0 for _n, v in sent)


@pytest.mark.parametrize("inputs", [{"buttons": frozenset({"l1", "dpad_up"})},
                                    {"buttons": frozenset({"l1"}), "l2": 1.0}])
def test_slow_button_scales_dpad_and_triggers(rig, inputs):
    """R22: R1 scales every jog input by slow/normal, not only the sticks."""
    sent = _record_velocity_setpoints(rig.client)
    rig.drive(0.02)
    rig.pad.set(**inputs)
    rig.drive(0.04)
    fast = [v for _n, v in sent if v != 0.0]
    sent.clear()
    press(rig.pad, "r1")
    rig.drive(0.04)
    slow = [v for _n, v in sent if v != 0.0]
    ratio = rig.teleop.cfg.speed_scale["slow"] / rig.teleop.cfg.speed_scale["normal"]
    assert fast and len(slow) == len(fast)
    assert slow == pytest.approx([v * ratio for v in fast])


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


# --- Fix round 1 -------------------------------------------------------------------


def _all_estopped(client):
    return all(js.state == p.AxisState.FAULT and js.faults & p.Fault.ESTOP for js in client.joints.values())


def test_estop_held_through_reconnect_still_estops(rig):
    """I1: E-STOP is level-triggered and exempt from the reconnect latch."""
    press(rig.pad, "circle")
    rig.pad.set(connected=False)
    rig.drive(0.1)
    assert not rig.client.any_fault()  # nothing sent while the pad is gone
    rig.pad.set(connected=True)  # Circle still held
    rig.drive(0.1)
    assert _all_estopped(rig.client)


def test_estop_held_at_startup_estops_on_first_update(cfg, tcfg):
    """I1: Circle held when teleop starts is not swallowed by the start-up latch."""
    world = SimWorld(cfg)
    bus = SimBus(world)
    try:
        client = make_client(bus, cfg)
        pad = FakeGamepad()
        press(pad, "circle")
        sent = []
        real_estop = client.estop
        client.estop = lambda: (sent.append("estop"), real_estop())
        r = Rig(world, bus, client, Teleop(client, cfg, tcfg), pad)
        r.drive(0.02)  # exactly one update
        assert sent == ["estop"]
        r.drive(0.1)
        assert _all_estopped(client)
    finally:
        bus.shutdown()
        world.close()


def test_estop_blocks_arming(rig):
    press(rig.pad, "circle", "l1")
    rig.drive(0.1)
    assert not rig.teleop.armed


class _RecordingBus:
    """Minimal bus: records sent frames, never receives."""

    def __init__(self):
        self.sent = []

    def send(self, msg, timeout=None):
        self.sent.append(msg)

    def recv(self, timeout=None):
        return None

    def heartbeats(self):
        return sum(isinstance(p.decode(m), p.Heartbeat) for m in self.sent)


def test_keepalive_gates_heartbeats(cfg):
    """R23 (a): with keepalive_timeout set, poll() sends HEARTBEATs only while keepalive() is fresh."""
    bus = _RecordingBus()
    client = ArmClient(bus, cfg, heartbeat_hz=20.0, keepalive_timeout=0.1)
    t = 0.0
    for _ in range(40):  # 0.4 s, never kept alive
        client.poll(t)
        t += 0.01
    assert bus.heartbeats() == 0

    for _ in range(40):  # 0.4 s kept alive every 10 ms -> 20 Hz heartbeats
        client.keepalive(t)
        client.poll(t)
        t += 0.01
    fresh = bus.heartbeats()
    assert 7 <= fresh <= 9

    for _ in range(40):  # stale again: at most one more heartbeat within the 0.1 s timeout
        client.poll(t)
        t += 0.01
    assert bus.heartbeats() - fresh <= 2

    client.keepalive(t)
    client.poll(t)  # resumes as soon as it is fresh
    assert bus.heartbeats() > fresh


def test_keepalive_none_always_heartbeats(cfg):
    bus = _RecordingBus()
    client = ArmClient(bus, cfg)
    for i in range(40):
        client.poll(i * 0.01)
    assert bus.heartbeats() >= 7


def test_stalled_teleop_loop_watchdog_faults_and_joint_stops(rig):
    """R23 (b): the teleop loop hangs mid-jog while ArmClient.poll keeps running ->
    no more HEARTBEATs -> every axis WATCHDOG-faults within 0.4 s and the jogged joint stops.

    Jogs the hip (vertical axis, no gravity load): a FAULTed axis is unpowered, so a
    gravity-loaded joint like the shoulder stops being driven but then creeps (~1-2 deg/s
    in the sim) -- that is fault behaviour in general, not specific to this path."""
    press(rig.pad, "l1")
    rig.pad.set(lx=1.0)
    rig.drive(0.5)
    assert abs(math.degrees(rig.joint("hip").velocity_rad_s)) > 10.0

    rig.stalled = True
    rig.drive(0.4)
    assert all(js.state == p.AxisState.FAULT and js.faults & p.Fault.WATCHDOG for js in rig.client.joints.values())
    rig.drive(0.3)
    assert abs(math.degrees(rig.joint("hip").velocity_rad_s)) < 1.0


def _spy(client, name):
    calls = []
    real = getattr(client, name)

    def spy(*args, **kwargs):
        calls.append(args)
        return real(*args, **kwargs)

    setattr(client, name, spy)
    return calls


def test_home_and_enable_ignored_while_armed(rig):
    press(rig.pad, "l1")
    rig.drive(0.04)
    assert rig.teleop.armed
    homes, enables = _spy(rig.client, "home"), _spy(rig.client, "enable")
    press(rig.pad, "triangle", "cross")
    rig.drive(0.1)
    assert homes == [] and enables == []
    assert all(js.state == p.AxisState.READY for js in rig.client.joints.values())


@pytest.mark.parametrize("speed_scale", ["{normal: 0.5}", "{normal: 0.5, slow: 0}", "{normal: -1, slow: 0.1}", "0.5"])
def test_load_teleop_config_rejects_bad_speed_scale(tmp_path, speed_scale):
    bad = tmp_path / "teleop.yaml"
    bad.write_text(f"deadzone: 0.1\nloop_hz: 50\nspeed_scale: {speed_scale}\nbuttons: {{deadman: l1}}\n")
    with pytest.raises(ValueError, match="speed_scale"):
        load_teleop_config(bad)


def test_load_teleop_config_missing_key_is_value_error(tmp_path):
    bad = tmp_path / "teleop.yaml"
    bad.write_text("deadzone: 0.1\nloop_hz: 50\n")
    with pytest.raises(ValueError, match="missing key"):
        load_teleop_config(bad)


# --- run_teleop failure paths (in-process, in-process sim bus) ---------------------


def _one_error_line(capsys, prefix):
    err = capsys.readouterr().err
    assert err.startswith(prefix), err
    assert err.count("\n") == 1 and "Traceback" not in err, err


def test_run_teleop_config_error_exits_2(monkeypatch, capsys):
    def boom(path=None):
        raise ValueError("speed_scale needs 'normal' and 'slow' entries")

    monkeypatch.setattr(teleop_mod, "load_teleop_config", boom)
    assert teleop_mod.run_teleop("sim", "joint", gamepad=FakeGamepad()) == 2
    _one_error_line(capsys, "error: config: speed_scale")


def test_run_teleop_unknown_joint_exits_2(monkeypatch, capsys, tcfg):
    bad = dataclasses.replace(tcfg, joint_mode=tcfg.joint_mode + (JointBinding(joint="j9", axis="lx"),))
    monkeypatch.setattr(teleop_mod, "load_teleop_config", lambda path=None: bad)
    assert teleop_mod.run_teleop("sim", "joint", gamepad=FakeGamepad()) == 2
    _one_error_line(capsys, "error: config: teleop joint_mode names unknown joints")


def test_run_teleop_gamepad_init_failure_exits_2(monkeypatch, capsys):
    def boom(**_kwargs):
        raise RuntimeError("SDL exploded")

    monkeypatch.setattr(teleop_mod, "Gamepad", boom)
    assert teleop_mod.run_teleop("sim", "joint") == 2
    _one_error_line(capsys, "error: gamepad: SDL exploded")


def test_run_teleop_gamepad_poll_failure_exits_2_and_disables(monkeypatch, capsys):
    class BadPad:
        def poll(self):
            raise RuntimeError("hid read failed")

    closes = []
    real_close = ArmClient.close
    monkeypatch.setattr(ArmClient, "close", lambda self: (closes.append(1), real_close(self)))
    assert teleop_mod.run_teleop("sim", "joint", gamepad=BadPad()) == 2
    assert closes == [1]  # DISABLE still sent
    err = capsys.readouterr().err
    assert err.startswith("error: gamepad: hid read failed") and "Traceback" not in err


def test_run_teleop_ctrl_c_during_open_bus_exits_promptly(monkeypatch, capsys):
    def slow_open(url):
        raise KeyboardInterrupt

    monkeypatch.setattr(teleop_mod, "open_bus", slow_open)
    assert teleop_mod.run_teleop("tcp://10.255.255.1:1", "joint", gamepad=FakeGamepad()) == 130


def test_runner_failure_messages():
    assert teleop_mod._runner_failure(None) == "bus lost: runner stopped"
    assert teleop_mod._runner_failure(BrokenPipeError(32, "Broken pipe")).startswith("bus lost: ")
    assert teleop_mod._runner_failure(ZeroDivisionError("x")).startswith("internal error: ")


def test_arm_client_runner_records_internal_error(cfg):
    """A non-bus exception in the runner is still recorded and stops it."""

    class _BuggyBus(_RecordingBus):
        def recv(self, timeout=None):
            raise ZeroDivisionError("bug")

    client = ArmClient(_BuggyBus(), cfg)
    client.start()
    deadline = time.monotonic() + 1.0
    while client.alive() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert isinstance(client.error, ZeroDivisionError)
    client.close()


# --- Task 17: Cartesian jog mode -----------------------------------------------------

# A bent pose well inside every soft limit: gripper tilted forward and down (pitch 150 deg).
BENT_Q = {"j1": math.radians(20), "j2": math.radians(20), "j3": math.radians(70), "j4": math.radians(60),
          "j5": 0.0}


@pytest.fixture
def cart_rig(rig):
    """The homed rig moved to BENT_Q, switched to CARTESIAN (not armed)."""
    for joint, q in BENT_Q.items():
        rig.client.set_position(rig.client.cfg.axis_by_name(_name(rig, joint)).node, q)
    rig.drive(3.0)
    press(rig.pad, "square")
    rig.drive(0.04)
    release(rig.pad, "square")
    rig.drive(0.04)
    assert rig.teleop.mode is Mode.CARTESIAN and not rig.teleop.armed
    return rig


def _name(rig, joint):
    return next(a.name for a in rig.client.cfg.axes if a.joint == joint)


def _tcp(rig):
    return rig.world.data.site("tcp").xpos.copy()


def _measured_q(rig):
    return [rig.client.joints[a.node].position_rad for a in rig.client.cfg.axes[:5]]


def _record(client, name):
    """Wrap client.<name> so the test sees every call's args (the real method still runs)."""
    sent = []
    real = getattr(client, name)

    def spy(*args):
        sent.append(args)
        real(*args)

    setattr(client, name, spy)
    return sent


def test_cartesian_ry_raises_tcp_z_straight_up(cart_rig):
    rig = cart_rig
    p0 = _tcp(rig)
    press(rig.pad, "l1")
    rig.pad.set(ry=1.0)
    rig.drive(1.0)
    assert rig.teleop.armed
    p1 = _tcp(rig)
    assert p1[2] - p0[2] > 0.02
    assert math.hypot(p1[0] - p0[0], p1[1] - p0[1]) < 0.005
    assert not rig.client.any_fault()


def test_cartesian_arming_holds_current_pose(cart_rig):
    """Arming with the sticks centred initialises the target from FK of the measured joints: nothing moves."""
    rig = cart_rig
    p0 = _tcp(rig)
    sent = _record(rig.client, "set_position")
    press(rig.pad, "l1")
    rig.drive(0.5)
    assert len(sent) == 25 * 5  # j1..j5 every update
    assert float(sum((_tcp(rig) - p0) ** 2)) ** 0.5 < 0.001


def test_cartesian_radial_outward_stops_at_workspace_edge(cart_rig):
    rig = cart_rig
    kin = Kinematics(rig.client.cfg)
    start = kin.forward(_measured_q(rig))
    sent = _record(rig.client, "set_position")
    press(rig.pad, "l1")
    rig.pad.set(ly=1.0)
    rig.drive(10.0)
    assert rig.teleop.armed
    assert not rig.client.any_fault()
    assert all(abs(math.degrees(js.velocity_rad_s)) < 1.0 for js in rig.client.joints.values())

    q = _measured_q(rig)
    end = kin.forward(q)
    r0, r1 = math.hypot(start.x, start.y), math.hypot(end.x, end.y)
    assert r1 - r0 > 0.05  # it really moved outward...
    assert math.atan2(end.y, end.x) == pytest.approx(BENT_Q["j1"], abs=math.radians(1))  # ...along q1
    assert end.z == pytest.approx(start.z, abs=0.005) and end.pitch == pytest.approx(start.pitch, abs=0.02)
    # ...and stopped at the edge: 1 cm further out is not reachable.
    further = Pose(end.x * (r1 + 0.01) / r1, end.y * (r1 + 0.01) / r1, end.z, end.pitch, end.roll)
    assert kin.inverse(further, q) is None
    # Out of reach, the last reachable target is re-sent unchanged.
    last = sent[-10:]
    assert len({args for args in last}) == 5


def test_cartesian_rotates_about_z_and_rolls(cart_rig):
    rig = cart_rig
    kin = Kinematics(rig.client.cfg)
    start = kin.forward(_measured_q(rig))
    press(rig.pad, "l1")
    rig.pad.set(lx=-1.0, rx=1.0)  # stick left: counter-clockwise from above (+q1, as in joint mode)
    rig.drive(1.0)
    end = kin.forward(_measured_q(rig))
    assert math.atan2(end.y, end.x) - math.atan2(start.y, start.x) > math.radians(10)
    assert math.hypot(end.x, end.y) == pytest.approx(math.hypot(start.x, start.y), abs=0.005)
    assert end.z == pytest.approx(start.z, abs=0.005)
    assert end.roll - start.roll > math.radians(10)


def test_cartesian_dpad_pitches_gripper(cart_rig):
    rig = cart_rig
    kin = Kinematics(rig.client.cfg)
    start = kin.forward(_measured_q(rig))
    press(rig.pad, "l1")
    rig.drive(0.04)
    press(rig.pad, "dpad_down")
    rig.drive(1.0)
    release(rig.pad, "dpad_down")
    rig.drive(0.5)  # the joints lag a little mid-pitch; the settled TCP is where it started
    end = kin.forward(_measured_q(rig))
    assert start.pitch - end.pitch > math.radians(10)
    assert (end.x, end.y, end.z) == pytest.approx((start.x, start.y, start.z), abs=0.005)


def test_cartesian_slow_scales_speed(cart_rig):
    rig = cart_rig
    kin = Kinematics(rig.client.cfg)
    press(rig.pad, "l1", "r1")
    rig.pad.set(ry=1.0)
    z0 = kin.forward(_measured_q(rig)).z
    rig.drive(1.0)
    slow_dz = kin.forward(_measured_q(rig)).z - z0
    ratio = rig.teleop.cfg.speed_scale["slow"] / rig.teleop.cfg.speed_scale["normal"]
    expected = rig.teleop.cfg.cartesian_mode["linear_speed_m_s"] * ratio
    assert slow_dz == pytest.approx(expected, rel=0.3)


def test_cartesian_triggers_drive_gripper(cart_rig):
    rig = cart_rig
    q0 = rig.joint("gripper").position_rad
    press(rig.pad, "l1")
    rig.pad.set(r2=1.0)
    rig.drive(0.3)
    assert rig.joint("gripper").position_rad < q0 - math.radians(2)


def test_cartesian_disarm_sends_zeros_then_silence(cart_rig):
    """R21 holds in CARTESIAN too: zero velocity to every jog axis for 3 updates, then nothing."""
    rig = cart_rig
    press(rig.pad, "l1")
    rig.pad.set(ry=1.0)
    rig.drive(0.3)
    positions = _record(rig.client, "set_position")
    velocities = _record_velocity_setpoints(rig.client)
    release(rig.pad, "l1")
    rig.drive(0.3)
    assert positions == []
    n_jog = len(rig.teleop.cfg.joint_mode)
    assert len(velocities) == 3 * n_jog and all(v == 0.0 for _n, v in velocities)
    rig.drive(0.5)
    assert all(abs(math.degrees(js.velocity_rad_s)) < 1.0 for js in rig.client.joints.values())


def test_cartesian_holds_while_unhomed(cfg, tcfg):
    world = SimWorld(cfg, initial_q=near_home_start(cfg))
    bus = SimBus(world)
    try:
        client = make_client(bus, cfg)
        pad = FakeGamepad()
        teleop = Teleop(client, cfg, tcfg)
        teleop.mode = Mode.CARTESIAN
        r = Rig(world, bus, client, teleop, pad)
        press(pad, "cross")  # enable (not homed)
        r.drive(0.1)
        release(pad, "cross")
        r.drive(0.1)
        positions = _record(client, "set_position")
        press(pad, "l1")
        pad.set(ry=1.0)
        r.drive(0.5)
        assert teleop.armed
        assert positions == []
        assert "HOLD" in teleop.status_line() and "\n" not in teleop.status_line()
    finally:
        bus.shutdown()
        world.close()


# --- Task 16 review carry-overs ----------------------------------------------------------


def test_estop_held_blocks_clear_enable_home_in_same_update(rig):
    calls = {name: _spy(rig.client, name) for name in ("clear_faults", "enable", "home", "estop")}
    press(rig.pad, "circle", "options", "cross", "triangle")
    rig.drive(0.02)  # one update
    assert calls["estop"] and not calls["clear_faults"] and not calls["enable"] and not calls["home"]
    release(rig.pad, "circle")  # the other buttons stay held: no new edge, still nothing
    rig.drive(0.1)
    assert not calls["clear_faults"] and not calls["enable"] and not calls["home"]


@pytest.mark.parametrize(("loop_hz", "expected"), [(50, 0.1), (100, 0.1), (10, 0.2), (4, 0.5)])
def test_keepalive_timeout_scales_with_loop_rate(loop_hz, expected):
    assert teleop_mod.keepalive_timeout(loop_hz) == pytest.approx(expected)


def test_run_teleop_builds_client_with_loop_rate_keepalive(monkeypatch, tcfg):
    seen = []
    real_init = ArmClient.__init__

    def init(self, bus, cfg, *args, **kwargs):
        seen.append(kwargs.get("keepalive_timeout"))
        real_init(self, bus, cfg, *args, **kwargs)

    class StopPad:
        def poll(self):
            raise RuntimeError("done")

    monkeypatch.setattr(ArmClient, "__init__", init)
    monkeypatch.setattr(teleop_mod, "load_teleop_config", lambda path=None: dataclasses.replace(tcfg, loop_hz=10.0))
    assert teleop_mod.run_teleop("sim", "joint", gamepad=StopPad()) == 2
    assert seen == [pytest.approx(0.2)]


@pytest.mark.parametrize("body", ["buttons: l1\n", "buttons: {deadman: l1}\njoint_mode: [j1, j2]\n"])
def test_run_teleop_malformed_yaml_is_config_error(monkeypatch, capsys, tmp_path, body):
    """A structurally malformed file (a string/list where a mapping belongs) is a config error, not a crash."""
    bad = tmp_path / "teleop.yaml"
    bad.write_text("deadzone: 0.1\nloop_hz: 50\nspeed_scale: {normal: 0.5, slow: 0.1}\n" + body)
    real = teleop_mod.load_teleop_config
    monkeypatch.setattr(teleop_mod, "load_teleop_config", lambda path=None: real(bad))
    assert teleop_mod.run_teleop("sim", "joint", gamepad=FakeGamepad()) == 2
    _one_error_line(capsys, "error: config: ")


@pytest.mark.parametrize("cart", ["", "cartesian_mode: {linear_speed_m_s: 0.05}\n",
                                  "cartesian_mode: {linear_speed_m_s: 0.05, pitch_speed_rad_s: 0}\n",
                                  "cartesian_mode: [1, 2]\n"])
def test_load_teleop_config_rejects_bad_cartesian_mode(tmp_path, cart):
    bad = tmp_path / "teleop.yaml"
    bad.write_text("deadzone: 0.1\nloop_hz: 50\nspeed_scale: {normal: 0.5, slow: 0.1}\nbuttons: {deadman: l1}\n" + cart)
    with pytest.raises(ValueError, match="cartesian_mode"):
        load_teleop_config(bad)


# --- R33: stale_input_s (silent-pad detection in the real Gamepad) ------------------------


_VALID_TELEOP_BASE = ("deadzone: 0.1\nloop_hz: 50\nspeed_scale: {normal: 0.5, slow: 0.1}\nbuttons: {deadman: l1}\n"
                      "cartesian_mode: {linear_speed_m_s: 0.05, pitch_speed_rad_s: 0.5}\n")


def test_stale_input_s_loaded_with_default(tcfg, tmp_path):
    assert tcfg.stale_input_s == pytest.approx(0.5)
    cfg_file = tmp_path / "teleop.yaml"
    cfg_file.write_text(_VALID_TELEOP_BASE)
    assert load_teleop_config(cfg_file).stale_input_s == pytest.approx(0.5)
    cfg_file.write_text(_VALID_TELEOP_BASE + "stale_input_s: 0\n")
    assert load_teleop_config(cfg_file).stale_input_s == 0.0


@pytest.mark.parametrize("value", ["-0.1", ".nan", ".inf", "abc"])
def test_stale_input_s_rejects_bad_values(tmp_path, value):
    bad = tmp_path / "teleop.yaml"
    bad.write_text(_VALID_TELEOP_BASE + f"stale_input_s: {value}\n")
    with pytest.raises(ValueError, match="stale_input_s"):
        load_teleop_config(bad)
