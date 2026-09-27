"""PlayStation-style gamepad input via SDL (pygame), with a radial deadzone.

`Gamepad` talks to a real pad through `pygame._sdl2.controller` (SDL's
GameController API, which already speaks PS-agnostic names: `a`, `b`,
`leftshoulder`, ...). If that extension module isn't available in this
pygame build, it falls back to raw `pygame.joystick` plus the index map in
`config/teleop.yaml`'s `raw_joystick_fallback` section (handed to the
constructor by the caller -- this module never reads config files itself).

pygame ties its event queue to the SDL video subsystem, so `pygame.event`
needs `pygame.display.init()` to have run even though we never open a
window. `Gamepad` sets SDL to its headless "dummy" video driver (via
`os.environ.setdefault`, so a caller's own `SDL_VIDEODRIVER` still wins) the
first time it initialises SDL: no OS window ever appears, but hot-plug
events (CONTROLLERDEVICEADDED/REMOVED, JOYDEVICEADDED/REMOVED) still flow.
`SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS` keeps the pad live even if some other
window has focus.

`Gamepad().poll()` never raises: with no pad attached (CI, a laptop with
nothing plugged in) it returns a disconnected `GamepadState`.

Silent-pad detection (controller ruling R33): a Bluetooth pad that goes out of
range or flat is not always reported as removed by SDL -- it can keep returning
its last report forever, including a stick held forward. So `Gamepad` (not
`FakeGamepad`) also reports `connected=False` when a stick is deflected past the
deadzone and ALL raw inputs (every axis, trigger, button, hat) have been
bit-identical for `stale_input_s` seconds (0 disables the check). Real sticks
held by a hand jitter by at least one LSB; a stick resting at centre is not
checked (a still pad lying on the table is fine). A USB cable avoids the whole
issue and is recommended for first hardware sessions.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import threading
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Callable

import pygame
import yaml

try:
    import pygame._sdl2.controller as _sdl_controller
except ImportError:  # pragma: no cover - depends on the pygame build
    _sdl_controller = None

# SDL GameController button index -> PlayStation name.
_CONTROLLER_BUTTON_TO_NAME = {
    pygame.CONTROLLER_BUTTON_A: "cross",
    pygame.CONTROLLER_BUTTON_B: "circle",
    pygame.CONTROLLER_BUTTON_X: "square",
    pygame.CONTROLLER_BUTTON_Y: "triangle",
    pygame.CONTROLLER_BUTTON_LEFTSHOULDER: "l1",
    pygame.CONTROLLER_BUTTON_RIGHTSHOULDER: "r1",
    pygame.CONTROLLER_BUTTON_BACK: "share",
    pygame.CONTROLLER_BUTTON_START: "options",
    pygame.CONTROLLER_BUTTON_GUIDE: "ps",
    pygame.CONTROLLER_BUTTON_LEFTSTICK: "l3",
    pygame.CONTROLLER_BUTTON_RIGHTSTICK: "r3",
    pygame.CONTROLLER_BUTTON_DPAD_UP: "dpad_up",
    pygame.CONTROLLER_BUTTON_DPAD_DOWN: "dpad_down",
    pygame.CONTROLLER_BUTTON_DPAD_LEFT: "dpad_left",
    pygame.CONTROLLER_BUTTON_DPAD_RIGHT: "dpad_right",
}

# Every SDL GameController axis, in the order the raw snapshot stores them (see Gamepad._read_raw).
_CONTROLLER_AXES = (
    pygame.CONTROLLER_AXIS_LEFTX,
    pygame.CONTROLLER_AXIS_LEFTY,
    pygame.CONTROLLER_AXIS_RIGHTX,
    pygame.CONTROLLER_AXIS_RIGHTY,
    pygame.CONTROLLER_AXIS_TRIGGERLEFT,
    pygame.CONTROLLER_AXIS_TRIGGERRIGHT,
)

_INT16_MAX = 32767  # SDL axis range is -32768..32767; symmetric normalisation


def _norm_axis(raw: int) -> float:
    """SDL stick axis (-32768..32767) -> [-1, 1]."""
    return max(-1.0, min(1.0, raw / _INT16_MAX))


def _norm_trigger(raw: int) -> float:
    """SDL trigger axis (0..32767) -> [0, 1]."""
    return max(0.0, min(1.0, raw / _INT16_MAX))


@dataclass(frozen=True)
class GamepadState:
    connected: bool = False
    lx: float = 0.0
    ly: float = 0.0
    rx: float = 0.0
    ry: float = 0.0  # [-1, 1], +y = stick pushed UP
    l2: float = 0.0
    r2: float = 0.0  # [0, 1]
    buttons: frozenset[str] = frozenset()
    # names: cross circle square triangle l1 r1 share options ps
    #        dpad_up dpad_down dpad_left dpad_right l3 r3


def apply_deadzone(x: float, y: float, deadzone: float) -> tuple[float, float]:
    """Radial deadzone over (x, y): below `deadzone` magnitude -> (0, 0);
    above it, rescaled so the deadzone edge maps to 0 and full deflection
    (magnitude 1.0) still maps to 1.0. Radial (not per-axis) so a stick held
    at 45 degrees doesn't need to travel further than one held straight.
    """
    magnitude = math.hypot(x, y)
    if magnitude <= deadzone:
        return 0.0, 0.0
    scale = min(1.0, (magnitude - deadzone) / (1.0 - deadzone)) / magnitude
    return x * scale, y * scale


class FakeGamepad:
    """In-memory stand-in for tests and headless runs: no SDL, no hardware."""

    def __init__(self) -> None:
        self.state = GamepadState(connected=True)

    def set(self, **changes: Any) -> None:
        self.state = replace(self.state, **changes)

    def poll(self) -> GamepadState:
        return self.state


class Gamepad:
    """Real PlayStation-style pad via SDL (pygame).

    Uses `pygame._sdl2.controller` (SDL GameController API) when available;
    otherwise falls back to raw `pygame.joystick` plus `raw_fallback` (the
    `raw_joystick_fallback` section of config/teleop.yaml, as a dict with
    `buttons: {name: index}` and `axes: {name: index}` -- the caller loads
    and passes it; this class never touches config files).
    """

    def __init__(self, deadzone: float = 0.12, raw_fallback: dict[str, Any] | None = None,
                 stale_input_s: float = 0.5, clock: Callable[[], float] = time.monotonic) -> None:
        self._deadzone = deadzone
        self._raw_fallback = raw_fallback or {}
        self._stale_input_s = stale_input_s
        self._clock = clock
        self._last_raw: tuple | None = None
        self._last_raw_change = 0.0
        self._use_controller_api = _sdl_controller is not None
        self._device: Any = None  # pygame._sdl2.controller.Controller or pygame.joystick.Joystick
        self._caller_sigint_handler: Any = None
        # Computed once: whether we're allowed to touch SIGINT at all (signal.signal()
        # raises off the main thread; getsignal() is safe anywhere, but there's nothing
        # useful to do with a handler we could never restore).
        self._can_restore_sigint = threading.current_thread() is threading.main_thread()
        self._ready = self._init_sdl()

    def _init_sdl(self) -> bool:
        # Capture whatever SIGINT handler the caller already had installed --
        # sim/server.py and the Task 16 teleop loop install their own for graceful
        # shutdown, and pygame.display.init() below must not clobber it.
        if self._can_restore_sigint:
            self._caller_sigint_handler = signal.getsignal(signal.SIGINT)
        os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")  # headless: no window (see module docstring)
        try:
            pygame.display.init()  # enables pygame.event; SDL grabs SIGINT for itself here
            self._restore_sigint()
            pygame.joystick.init()
            if self._use_controller_api:
                _sdl_controller.init()
            return True
        except pygame.error:
            return False

    def _restore_sigint(self) -> None:
        """Put back the SIGINT handler captured in _init_sdl.

        SDL's video subsystem installs its own native SIGINT handler once
        `pygame.display.init()` runs (even with the dummy driver), silently
        swallowing Ctrl-C (or whatever the caller's handler was supposed to
        do) instead -- confirmed by direct experimentation (SIGTERM is
        swallowed the same way; only SIGKILL gets through). A single restore
        right after `pygame.display.init()` is enough in practice, but this
        also runs on every poll() as cheap insurance (one syscall) against
        anything -- SDL or otherwise -- touching SIGINT again later.
        """
        if not self._can_restore_sigint:
            return
        try:
            signal.signal(signal.SIGINT, self._caller_sigint_handler)
        except ValueError:
            pass

    def poll(self) -> GamepadState:
        if not self._ready:
            return GamepadState(connected=False)
        try:
            return self._poll_inner()
        except pygame.error:
            self._device = None
            return GamepadState(connected=False)

    def _poll_inner(self) -> GamepadState:
        self._pump_events()
        if self._device is None:
            self._last_raw = None
            return GamepadState(connected=False)
        raw = self._read_raw()
        if raw is None:  # detached
            self._device = None
            self._last_raw = None
            return GamepadState(connected=False)
        state = self._decode_controller(raw) if self._use_controller_api else self._decode_raw_joystick(raw)
        if self._input_frozen(raw, state):
            return GamepadState(connected=False)
        return state

    def _pump_events(self) -> None:
        self._restore_sigint()  # see _restore_sigint's docstring: SDL can re-grab it
        pygame.event.pump()
        for event in pygame.event.get():
            self._handle_hotplug(event)

    def _input_frozen(self, raw: tuple, state: GamepadState) -> bool:
        """R33: a stick deflected past the deadzone while every raw input has stayed bit-identical
        for stale_input_s -> treat the pad as lost (see the module docstring)."""
        now = self._clock()
        if raw != self._last_raw:
            self._last_raw = raw
            self._last_raw_change = now
            return False
        if self._stale_input_s <= 0:
            return False
        deflected = any((state.lx, state.ly, state.rx, state.ry))
        return deflected and now - self._last_raw_change >= self._stale_input_s

    def _read_raw(self) -> tuple | None:
        """Every raw input of the open device as a comparable snapshot, or None if it detached."""
        if self._use_controller_api:
            c = self._device
            if not c.attached():
                return None
            return (tuple(c.get_axis(i) for i in _CONTROLLER_AXES),
                    tuple(bool(c.get_button(i)) for i in _CONTROLLER_BUTTON_TO_NAME))
        j = self._device
        if not j.get_init():
            return None
        return (tuple(j.get_axis(i) for i in range(j.get_numaxes())),
                tuple(bool(j.get_button(i)) for i in range(j.get_numbuttons())),
                tuple(j.get_hat(i) for i in range(j.get_numhats())))

    def _handle_hotplug(self, event: pygame.event.Event) -> None:
        if self._use_controller_api:
            if event.type == pygame.CONTROLLERDEVICEADDED and self._device is None:
                self._open_controller(event.device_index)
            elif event.type == pygame.CONTROLLERDEVICEREMOVED and self._device is not None:
                if self._device.id == event.instance_id:
                    self._device = None
        else:
            if event.type == pygame.JOYDEVICEADDED and self._device is None:
                self._open_joystick(event.device_index)
            elif event.type == pygame.JOYDEVICEREMOVED and self._device is not None:
                if self._device.get_instance_id() == event.instance_id:
                    self._device = None

    def _open_controller(self, device_index: int) -> None:
        try:
            if _sdl_controller.is_controller(device_index):
                self._device = _sdl_controller.Controller(device_index)
        except pygame.error:
            self._device = None

    def _open_joystick(self, device_index: int) -> None:
        try:
            self._device = pygame.joystick.Joystick(device_index)
        except pygame.error:
            self._device = None

    def _decode_controller(self, raw: tuple) -> GamepadState:
        axis_values, button_values = raw
        a = dict(zip(_CONTROLLER_AXES, axis_values))
        lx = _norm_axis(a[pygame.CONTROLLER_AXIS_LEFTX])
        ly = -_norm_axis(a[pygame.CONTROLLER_AXIS_LEFTY])  # SDL +y is down
        rx = _norm_axis(a[pygame.CONTROLLER_AXIS_RIGHTX])
        ry = -_norm_axis(a[pygame.CONTROLLER_AXIS_RIGHTY])
        lx, ly = apply_deadzone(lx, ly, self._deadzone)
        rx, ry = apply_deadzone(rx, ry, self._deadzone)
        l2 = _norm_trigger(a[pygame.CONTROLLER_AXIS_TRIGGERLEFT])
        r2 = _norm_trigger(a[pygame.CONTROLLER_AXIS_TRIGGERRIGHT])
        buttons = frozenset(name for name, pressed in zip(_CONTROLLER_BUTTON_TO_NAME.values(), button_values)
                            if pressed)
        return GamepadState(connected=True, lx=lx, ly=ly, rx=rx, ry=ry, l2=l2, r2=r2, buttons=buttons)

    def _decode_raw_joystick(self, raw: tuple) -> GamepadState:
        axis_values, button_values, _hats = raw
        axes = self._raw_fallback.get("axes", {})
        buttons = self._raw_fallback.get("buttons", {})

        def axis(name: str) -> float:
            index = axes.get(name)
            return axis_values[index] if index is not None and index < len(axis_values) else 0.0

        lx, ly = axis("lx"), -axis("ly")
        rx, ry = axis("rx"), -axis("ry")
        lx, ly = apply_deadzone(lx, ly, self._deadzone)
        rx, ry = apply_deadzone(rx, ry, self._deadzone)
        # Raw HID trigger axes on macOS typically rest at -1 (released) and
        # reach +1 (fully pressed) rather than the GameController API's
        # native 0..1 -- rescale. (assumed: unverified against real hardware)
        l2 = max(0.0, min(1.0, (axis("l2") + 1.0) / 2.0))
        r2 = max(0.0, min(1.0, (axis("r2") + 1.0) / 2.0))
        pressed = frozenset(name for name, index in buttons.items()
                            if index < len(button_values) and button_values[index])
        return GamepadState(connected=True, lx=lx, ly=ly, rx=rx, ry=ry, l2=l2, r2=r2, buttons=pressed)


def _format_state(state: GamepadState) -> str:
    if not state.connected:
        return "disconnected"
    buttons = ",".join(sorted(state.buttons)) or "-"
    return (f"lx={state.lx:+.2f} ly={state.ly:+.2f} rx={state.rx:+.2f} ry={state.ry:+.2f} "
            f"l2={state.l2:.2f} r2={state.r2:.2f} buttons=[{buttons}]")


_TELEOP_CONFIG_RELATIVE_PATH = "config/teleop.yaml"


def _load_raw_fallback() -> dict[str, Any]:
    """Best-effort read of config/teleop.yaml's raw_joystick_fallback section, for the
    `gamepad-test` CLI demo only -- Gamepad itself never touches config files (see
    the class docstring); Task 16's teleop loop does its own config loading.
    """
    repo_root = Path(__file__).resolve().parents[3]
    path = repo_root / _TELEOP_CONFIG_RELATIVE_PATH
    try:
        with path.open() as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        return {}
    return cfg.get("raw_joystick_fallback", {})


def _run_gamepad_test(args: argparse.Namespace) -> int:
    pad = Gamepad(deadzone=args.deadzone, raw_fallback=_load_raw_fallback())
    period_s = 0.1  # 10 Hz
    try:
        while True:
            print(_format_state(pad.poll()))
            time.sleep(period_s)
    except KeyboardInterrupt:
        return 0


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("gamepad-test", help="print live gamepad state 10x/s until Ctrl-C")
    parser.add_argument("--deadzone", type=float, default=0.12, help="radial stick deadzone (default: 0.12)")
    parser.set_defaults(func=_run_gamepad_test)
