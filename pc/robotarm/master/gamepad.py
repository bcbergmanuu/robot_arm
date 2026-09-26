"""PlayStation-style gamepad input via SDL (pygame), with a radial deadzone.

`Gamepad` talks to a real pad through `pygame._sdl2.controller` (SDL's
GameController API, which already speaks PS-agnostic names: `a`, `b`,
`leftshoulder`, ...). If that extension module isn't available in this
pygame build, it falls back to raw `pygame.joystick` plus the index map in
`config/teleop.yaml`'s `raw_joystick_fallback` section (handed to the
constructor by the caller -- this module never reads config files itself).

pygame ties its event queue to the SDL video subsystem, so `pygame.event`
needs `pygame.display.init()` to have run even though we never open a
window. We use SDL's headless "dummy" video driver for that: no OS window
ever appears, but hot-plug events (CONTROLLERDEVICEADDED/REMOVED,
JOYDEVICEADDED/REMOVED) still flow. `SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS`
keeps the pad live even if some other window has focus.

`Gamepad().poll()` never raises: with no pad attached (CI, a laptop with
nothing plugged in) it returns a disconnected `GamepadState`.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import time
from dataclasses import dataclass, replace
from typing import Any

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame  # noqa: E402  (import after the env vars above, which SDL reads at init)

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

_INT16_MAX = 32767  # SDL axis range is -32768..32767; symmetric normalisation


def _restore_sigint() -> None:
    """SDL's video subsystem installs its own native SIGINT handler; put Python's
    back so Ctrl-C raises KeyboardInterrupt instead of vanishing. Never raises
    (e.g. when called from a non-main thread, where signal.signal is unavailable).
    """
    try:
        signal.signal(signal.SIGINT, signal.default_int_handler)
    except ValueError:
        pass


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

    def __init__(self, deadzone: float = 0.12, raw_fallback: dict[str, Any] | None = None) -> None:
        self._deadzone = deadzone
        self._raw_fallback = raw_fallback or {}
        self._use_controller_api = _sdl_controller is not None
        self._device: Any = None  # pygame._sdl2.controller.Controller or pygame.joystick.Joystick
        self._ready = self._init_sdl()

    def _init_sdl(self) -> bool:
        try:
            pygame.display.init()  # dummy driver (see module docstring): no window, but enables events
            _restore_sigint()
            pygame.joystick.init()
            if self._use_controller_api:
                _sdl_controller.init()
            return True
        except pygame.error:
            return False

    def poll(self) -> GamepadState:
        if not self._ready:
            return GamepadState(connected=False)
        try:
            return self._poll_inner()
        except pygame.error:
            self._device = None
            return GamepadState(connected=False)

    def _poll_inner(self) -> GamepadState:
        # SDL's video/event subsystem races our one-time restore in _init_sdl and can
        # re-grab SIGINT for itself shortly after init (observed empirically -- a
        # handful of extra instructions between display.init() and the restore call
        # is enough to lose the race). Re-asserting it every poll costs one syscall
        # and reliably keeps Ctrl-C working as KeyboardInterrupt.
        _restore_sigint()
        pygame.event.pump()
        for event in pygame.event.get():
            self._handle_hotplug(event)
        if self._device is None:
            return GamepadState(connected=False)
        if self._use_controller_api:
            return self._read_controller()
        return self._read_raw_joystick()

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

    def _read_controller(self) -> GamepadState:
        c = self._device
        if not c.attached():
            self._device = None
            return GamepadState(connected=False)
        lx = _norm_axis(c.get_axis(pygame.CONTROLLER_AXIS_LEFTX))
        ly = -_norm_axis(c.get_axis(pygame.CONTROLLER_AXIS_LEFTY))  # SDL +y is down
        rx = _norm_axis(c.get_axis(pygame.CONTROLLER_AXIS_RIGHTX))
        ry = -_norm_axis(c.get_axis(pygame.CONTROLLER_AXIS_RIGHTY))
        lx, ly = apply_deadzone(lx, ly, self._deadzone)
        rx, ry = apply_deadzone(rx, ry, self._deadzone)
        l2 = _norm_trigger(c.get_axis(pygame.CONTROLLER_AXIS_TRIGGERLEFT))
        r2 = _norm_trigger(c.get_axis(pygame.CONTROLLER_AXIS_TRIGGERRIGHT))
        buttons = frozenset(name for index, name in _CONTROLLER_BUTTON_TO_NAME.items() if c.get_button(index))
        return GamepadState(connected=True, lx=lx, ly=ly, rx=rx, ry=ry, l2=l2, r2=r2, buttons=buttons)

    def _read_raw_joystick(self) -> GamepadState:
        j = self._device
        if not j.get_init():
            self._device = None
            return GamepadState(connected=False)
        axes = self._raw_fallback.get("axes", {})
        buttons = self._raw_fallback.get("buttons", {})

        def axis(name: str) -> float:
            index = axes.get(name)
            return j.get_axis(index) if index is not None else 0.0

        lx, ly = axis("lx"), -axis("ly")
        rx, ry = axis("rx"), -axis("ry")
        lx, ly = apply_deadzone(lx, ly, self._deadzone)
        rx, ry = apply_deadzone(rx, ry, self._deadzone)
        # Raw HID trigger axes on macOS typically rest at -1 (released) and
        # reach +1 (fully pressed) rather than the GameController API's
        # native 0..1 -- rescale. (assumed: unverified against real hardware)
        l2 = max(0.0, min(1.0, (axis("l2") + 1.0) / 2.0))
        r2 = max(0.0, min(1.0, (axis("r2") + 1.0) / 2.0))
        pressed = frozenset(name for name, index in buttons.items() if j.get_button(index))
        return GamepadState(connected=True, lx=lx, ly=ly, rx=rx, ry=ry, l2=l2, r2=r2, buttons=pressed)


def _format_state(state: GamepadState) -> str:
    if not state.connected:
        return "disconnected"
    buttons = ",".join(sorted(state.buttons)) or "-"
    return (f"lx={state.lx:+.2f} ly={state.ly:+.2f} rx={state.rx:+.2f} ry={state.ry:+.2f} "
            f"l2={state.l2:.2f} r2={state.r2:.2f} buttons=[{buttons}]")


def _run_gamepad_test(args: argparse.Namespace) -> int:
    pad = Gamepad(deadzone=args.deadzone)
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
