"""PlayStation teleop: joint jog with a deadman, E-STOP and gamepad-loss handling.

`Teleop.update(pad, now)` is sans-IO (called at `loop_hz` by `run_teleop`, or
by tests in lockstep with the simulator); `run_teleop` is the body of the
`robotarm teleop` CLI. Safety rules (see docs/teleop.md):

- `estop` is level-triggered and never latched: while it is held (pad
  connected) E-STOP is sent every update, armed or not, and it disarms.
- The other buttons act on *edges*: `clear_faults`; `enable`, `home` and
  `toggle_mode` only when not armed. A button held at start-up or through a
  gamepad reconnect is not an edge until it has been released.
- `armed` goes true only on a rising edge of the deadman while the pad is
  connected; releasing the deadman or losing the pad disarms, and after a
  loss the deadman must be released and pressed again.
- Armed + JOINT ("jogging"): every mapped joint gets a velocity setpoint every
  update; the slow button scales every jog input (R22).
- Armed + CARTESIAN: not implemented yet (Task 17) -- behaves as not armed.
- The CLI loop calls `ArmClient.keepalive()` every iteration and builds its
  client with `keepalive_timeout` (R23): if the loop stalls, HEARTBEATs stop
  and the axes watchdog-fault instead of running on the last jog velocity.
- When jogging stops (deadman released, pad lost, E-STOP) and when the pad is
  lost, zero velocity goes to every jog axis for the next
  _STOP_REPEAT_UPDATES updates (repeated for robustness); otherwise, while not
  jogging, teleop sends no setpoints at all -- so it never overrides HOME's
  back-off move or anything else -- and ArmClient's heartbeats keep the axis
  watchdogs fed (R21).
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

import can
import yaml

from robotarm.bus import open_bus
from robotarm.config import ArmConfig, load_arm_config
from robotarm.master.arm_client import ArmClient
from robotarm.master.gamepad import FakeGamepad, Gamepad, GamepadState
from robotarm.protocol import AxisState, Fault
from robotarm.transport.tcp_bus import SimNotRunningError

DEFAULT_TELEOP_CONFIG_RELATIVE_PATH = "config/teleop.yaml"

_DPAD_FRACTION = 0.5  # a d-pad pair jogs at +/- this fraction of the joint's max velocity
_ANALOG_INPUTS = frozenset({"lx", "ly", "rx", "ry", "l2", "r2"})  # float fields of GamepadState
_STATUS_PERIOD_S = 0.1  # CLI status line refresh (10 Hz)
_KEEPALIVE_TIMEOUT_S = 0.1  # R23: HEARTBEATs stop this long after the loop's last iteration
_STOP_REPEAT_UPDATES = 3  # zero-velocity updates sent after jogging stops or the pad is lost (R21)


class Mode(Enum):
    JOINT = "joint"
    CARTESIAN = "cartesian"


@dataclass(frozen=True)
class JointBinding:
    """One `joint_mode` entry: exactly one of axis / buttons / triggers is set.

    axis:     velocity = stick value * sign * max * speed scale
    buttons:  (first, second) -> +/- _DPAD_FRACTION * max * sign
    triggers: (first, second) -> (first - second) * sign * max

    The d-pad and trigger speeds above are the normal-speed values; holding
    the slow button scales them by speed_scale.slow / speed_scale.normal,
    just as it scales the sticks (R22).
    """

    joint: str
    axis: str | None = None
    buttons: tuple[str, str] | None = None
    triggers: tuple[str, str] | None = None
    sign: int = 1


@dataclass(frozen=True)
class TeleopConfig:
    deadzone: float
    loop_hz: float
    speed_scale: dict[str, float]  # {"normal": f, "slow": f}: fraction of each joint's max velocity
    buttons: dict[str, str]  # role (deadman, slow, enable, estop, home, toggle_mode, clear_faults) -> button
    joint_mode: tuple[JointBinding, ...]
    cartesian_mode: dict[str, float]
    raw_joystick_fallback: dict[str, Any]


def _default_teleop_config_path() -> Path:
    return Path(__file__).resolve().parents[3] / DEFAULT_TELEOP_CONFIG_RELATIVE_PATH


def _parse_binding(joint: str, raw: dict[str, Any]) -> JointBinding:
    kinds = [k for k in ("axis", "buttons", "triggers") if k in raw]
    if len(kinds) != 1:
        raise ValueError(f"joint_mode.{joint}: needs exactly one of axis/buttons/triggers, got {kinds}")
    sign = int(raw.get("sign", 1))
    if sign not in (1, -1):
        raise ValueError(f"joint_mode.{joint}: sign must be 1 or -1, got {sign}")
    kind = kinds[0]
    names = (str(raw[kind]),) if kind == "axis" else tuple(str(x) for x in raw[kind])
    if kind != "axis" and len(names) != 2:
        raise ValueError(f"joint_mode.{joint}.{kind}: needs exactly two names, got {list(names)}")
    if kind != "buttons" and not set(names) <= _ANALOG_INPUTS:
        raise ValueError(f"joint_mode.{joint}.{kind}: {list(names)} must be among {sorted(_ANALOG_INPUTS)}")
    if kind == "axis":
        return JointBinding(joint=joint, axis=names[0], sign=sign)
    return JointBinding(joint=joint, sign=sign, **{kind: names})


def _parse_speed_scale(raw: Any) -> dict[str, float]:
    if not isinstance(raw, dict) or not {"normal", "slow"} <= raw.keys():
        raise ValueError(f"speed_scale needs 'normal' and 'slow' entries, got {raw!r}")
    scale = {k: float(v) for k, v in raw.items()}
    bad = {k: v for k, v in scale.items() if not v > 0.0}
    if bad:
        raise ValueError(f"speed_scale values must be > 0, got {bad}")
    return scale


def load_teleop_config(path: str | Path | None = None) -> TeleopConfig:
    """Load config/teleop.yaml (or `path`). Raises ValueError on missing keys or bad values."""
    path = Path(path) if path is not None else _default_teleop_config_path()
    with path.open() as f:
        raw = yaml.safe_load(f) or {}
    try:
        return _build_teleop_config(raw)
    except KeyError as exc:
        raise ValueError(f"{path}: missing key {exc}") from exc


def _build_teleop_config(raw: dict[str, Any]) -> TeleopConfig:
    loop_hz = float(raw["loop_hz"])
    if not loop_hz > 0.0:
        raise ValueError(f"loop_hz must be > 0, got {loop_hz}")
    return TeleopConfig(
        deadzone=float(raw["deadzone"]),
        loop_hz=loop_hz,
        speed_scale=_parse_speed_scale(raw["speed_scale"]),
        buttons={k: str(v) for k, v in raw["buttons"].items()},
        joint_mode=tuple(_parse_binding(j, b) for j, b in (raw.get("joint_mode") or {}).items()),
        cartesian_mode={k: float(v) for k, v in (raw.get("cartesian_mode") or {}).items()},
        raw_joystick_fallback=raw.get("raw_joystick_fallback") or {},
    )


class Teleop:
    def __init__(self, client: ArmClient, arm_cfg: ArmConfig, cfg: TeleopConfig) -> None:
        self.client = client
        self.arm_cfg = arm_cfg
        self.cfg = cfg
        self.mode = Mode.JOINT
        self.armed = False
        self._pad = GamepadState()
        self._now: float | None = None
        self._jogging = False
        self._stop_updates_left = 0

        by_joint = {a.joint: a for a in arm_cfg.axes}
        unknown = [b.joint for b in cfg.joint_mode if b.joint not in by_joint]
        if unknown:
            raise ValueError(f"teleop joint_mode names unknown joints: {unknown}")
        self._bindings = [(b, by_joint[b.joint]) for b in cfg.joint_mode]
        self._jog_axes = [axis for _b, axis in self._bindings]
        # Buttons treated as "held" in the previous update. Starting from every
        # configured button means nothing held at start-up counts as an edge.
        self._latched_all = frozenset(cfg.buttons.values())
        self._prev_buttons = self._latched_all

    # -- update ------------------------------------------------------------------

    def update(self, pad: GamepadState, now: float) -> None:
        self._now = now
        pad_lost = self._pad.connected and not pad.connected
        self._pad = pad
        buttons = pad.buttons if pad.connected else frozenset()
        edges = buttons - self._prev_buttons
        # A lost pad latches every button: after reconnecting, a held button
        # (including the deadman) must be released before it acts again.
        self._prev_buttons = buttons if pad.connected else self._latched_all

        roles = self.cfg.buttons
        estop_held = roles.get("estop") in buttons  # level, not edge: no latch can swallow it
        if estop_held:
            self.client.estop()

        deadman = roles.get("deadman")
        if not pad.connected or deadman not in buttons or estop_held:
            self.armed = False
        elif deadman in edges:
            self.armed = True

        if roles.get("clear_faults") in edges:
            self.client.clear_faults()
        if not self.armed:
            if roles.get("enable") in edges:
                self.client.enable()
            if roles.get("home") in edges:
                self.client.home()
            if roles.get("toggle_mode") in edges:
                self.mode = Mode.CARTESIAN if self.mode is Mode.JOINT else Mode.JOINT

        jogging = self.armed and self.mode is Mode.JOINT
        if (self._jogging and not jogging) or pad_lost:
            self._stop_updates_left = _STOP_REPEAT_UPDATES
        self._jogging = jogging

        if jogging:
            self._jog_joints(pad)
        elif self._stop_updates_left > 0:
            self._stop_updates_left -= 1
            for axis in self._jog_axes:
                self.client.set_velocity(axis.node, 0.0)

    def _jog_joints(self, pad: GamepadState) -> None:
        slow = self.cfg.buttons.get("slow") in pad.buttons
        scale = self.cfg.speed_scale["slow" if slow else "normal"]
        normal = self.cfg.speed_scale["normal"]  # d-pad/trigger speeds are specified at normal speed (R22)
        for binding, axis in self._bindings:
            vmax = axis.max_velocity_rad_s
            if binding.axis is not None:
                v = getattr(pad, binding.axis) * vmax * scale
            elif binding.buttons is not None:
                plus, minus = binding.buttons
                v = _DPAD_FRACTION * vmax * ((plus in pad.buttons) - (minus in pad.buttons))
                v *= scale / normal
            else:
                first, second = binding.triggers
                v = (getattr(pad, first) - getattr(pad, second)) * vmax * scale / normal
            v = max(-vmax, min(vmax, v * binding.sign))
            self.client.set_velocity(axis.node, v)

    # -- status --------------------------------------------------------------------

    def status_line(self) -> str:
        """One line: mode, armed, pad/arm link, axis states, joint angles, faults."""
        joints = [self.client.joints[a.node] for a in self.arm_cfg.axes]
        link_ok = self._now is not None and self.client.connected(self._now)
        states = "/".join(sorted({js.state.name for js in joints}))
        homed = sum(js.homed for js in joints)
        angles = " ".join(f"{a.joint} {math.degrees(js.position_rad):+.0f}" for a, js in zip(self.arm_cfg.axes, joints, strict=True))
        mode = "JOINT" if self.mode is Mode.JOINT else "CART "
        # Kept short (~80-95 columns) so the CLI's in-place "\r" refresh rarely wraps.
        line = (f"{mode} {'ARMED' if self.armed else 'safe '} pad:{'ok' if self._pad.connected else 'LOST'} "
                f"arm:{'ok' if link_ok else 'LOST'} {states} homed:{homed}/{len(joints)} | {angles}")
        faults = [f"{a.joint}:{_fault_names(js.faults)}" for a, js in zip(self.arm_cfg.axes, joints, strict=True)
                  if js.state == AxisState.FAULT or js.faults]
        if faults:
            line += " | FAULT " + " ".join(faults)
        return line


def _fault_names(faults: Fault) -> str:
    return "|".join(f.name for f in Fault if f in faults and f.name) or "?"


# -- CLI -------------------------------------------------------------------------------


def run_teleop(bus_url: str, mode: str, gamepad=None) -> int:
    """Body of `robotarm teleop`. Returns the process exit code.

    0 after Ctrl-C (or SIGTERM) in the loop; 130 after Ctrl-C while the bus is
    still opening; 2 on any failure (config, bus open, gamepad, bus lost,
    internal error), reported as a single `error: ...` line on stderr. Once the
    loop has started the client is always closed, which sends a DISABLE
    broadcast when the bus still works (otherwise the axes watchdog-fault on
    their own).
    """
    try:
        arm_cfg = load_arm_config()
        cfg = load_teleop_config()
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        return _fail(f"config: {exc}")

    try:
        bus = open_bus(bus_url)
    except (SimNotRunningError, can.CanError, OSError, ValueError) as exc:
        return _fail(str(exc))
    except KeyboardInterrupt:  # Ctrl-C during a slow open (our handler isn't installed yet)
        return 130
    try:
        stop = threading.Event()
        old_handlers = _install_stop_handlers(stop)  # before Gamepad(): it preserves the caller's SIGINT handler
        try:
            return _teleop_loop(bus, arm_cfg, cfg, Mode(mode), gamepad, stop)
        finally:
            _restore_handlers(old_handlers)
    finally:
        bus.shutdown()


def _fail(message: str) -> int:
    print(f"error: {message}", file=sys.stderr)
    return 2


def _runner_failure(error: BaseException | None) -> str:
    if error is None:
        return "bus lost: runner stopped"
    if isinstance(error, (can.CanError, OSError)):
        return f"bus lost: {error}"
    return f"internal error: {error!r}"


def _teleop_loop(bus: can.BusABC, arm_cfg: ArmConfig, cfg: TeleopConfig, mode: Mode, gamepad,
                 stop: threading.Event) -> int:
    client = ArmClient(bus, arm_cfg, keepalive_timeout=_KEEPALIVE_TIMEOUT_S)
    try:
        teleop = Teleop(client, arm_cfg, cfg)
    except ValueError as exc:
        return _fail(f"config: {exc}")
    teleop.mode = mode
    if gamepad is None:
        try:
            gamepad = Gamepad(deadzone=cfg.deadzone, raw_fallback=cfg.raw_joystick_fallback)
        except Exception as exc:  # noqa: BLE001 -- any SDL/pygame start-up failure
            return _fail(f"gamepad: {exc}")

    period_s = 1.0 / cfg.loop_hz
    failure: str | None = None
    client.start()
    try:
        next_tick = time.monotonic()
        next_status = next_tick
        while not stop.is_set():
            if not client.alive():
                failure = _runner_failure(client.error)
                break
            now = time.monotonic()
            # First thing each iteration (R23): anything below that stalls -- the
            # gamepad, the bus, or a blocked stdout -- stops the HEARTBEATs within
            # _KEEPALIVE_TIMEOUT_S, and the axes then watchdog-fault.
            client.keepalive(now)
            try:
                pad = gamepad.poll()
            except Exception as exc:  # noqa: BLE001
                failure = f"gamepad: {exc}"
                break
            try:
                teleop.update(pad, now)
            except (can.CanError, OSError) as exc:
                failure = f"bus lost: {exc}"
                break
            except Exception as exc:  # noqa: BLE001
                failure = f"internal error: {exc!r}"
                break
            if now >= next_status:
                sys.stdout.write("\r" + teleop.status_line() + "\x1b[K")
                sys.stdout.flush()
                next_status = now + _STATUS_PERIOD_S
            next_tick += period_s
            stop.wait(max(0.0, next_tick - time.monotonic()))
    finally:
        client.close()
        sys.stdout.write("\n")  # end the in-place status line
        sys.stdout.flush()
    if failure is not None:
        return _fail(failure)
    return 0


def _install_stop_handlers(stop: threading.Event) -> dict[int, Any]:
    if threading.current_thread() is not threading.main_thread():
        return {}

    def _on_signal(_signum, _frame) -> None:
        stop.set()

    return {sig: signal.signal(sig, _on_signal) for sig in (signal.SIGINT, signal.SIGTERM)}


def _restore_handlers(old: dict[int, Any]) -> None:
    for sig, handler in old.items():
        signal.signal(sig, handler)


def _run(args: argparse.Namespace) -> int:
    return run_teleop(args.bus, args.mode, gamepad=FakeGamepad() if args.fake_gamepad else None)


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("teleop", help="jog the arm with a PlayStation controller (L1 = deadman)")
    parser.add_argument("--bus", required=True,
                        help="bus URL: tcp://host:port, sim, slcan:/dev/tty..., gs_usb:0, socketcan:can0, ...")
    parser.add_argument("--mode", choices=[m.value for m in Mode], default=Mode.JOINT.value,
                        help="start mode (default: joint; cartesian arrives with Task 17)")
    parser.add_argument("--fake-gamepad", action="store_true",
                        help="use a fake, always-connected gamepad with nothing pressed (headless runs)")
    parser.set_defaults(func=_run)
