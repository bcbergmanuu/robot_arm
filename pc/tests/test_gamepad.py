import pytest

from robotarm.master.gamepad import FakeGamepad, GamepadState, apply_deadzone


def test_small_stick_noise_is_zero():
    assert apply_deadzone(0.05, -0.08, 0.12) == (0.0, 0.0)


def test_deadzone_rescales_to_full_range():
    x, y = apply_deadzone(1.0, 0.0, 0.12)
    assert x == pytest.approx(1.0) and y == 0.0
    x, _ = apply_deadzone(0.56, 0.0, 0.12)
    assert x == pytest.approx(0.5, abs=1e-6)


def test_diagonal_is_radial_not_per_axis():
    x, y = apply_deadzone(0.1, 0.1, 0.12)       # |v| = 0.141 > 0.12 -> not zero
    assert x > 0 and y > 0


def test_fake_gamepad_set():
    pad = FakeGamepad()
    pad.set(lx=0.5, buttons=frozenset({"l1"}))
    s = pad.poll()
    assert s.connected and s.lx == 0.5 and "l1" in s.buttons


def test_real_gamepad_without_device_reports_disconnected():
    from robotarm.master.gamepad import Gamepad
    state = Gamepad().poll()       # CI/laptop without a pad: must not raise
    assert isinstance(state, GamepadState)


def test_poll_preserves_callers_sigint_handler():
    """sim/server.py and the Task 16 teleop loop install their own SIGINT handler for
    graceful shutdown; Gamepad must not clobber it with pygame's/its own default.
    """
    import signal

    from robotarm.master.gamepad import Gamepad

    def custom_handler(signum, frame):
        pass

    original = signal.getsignal(signal.SIGINT)
    signal.signal(signal.SIGINT, custom_handler)
    try:
        pad = Gamepad()
        pad.poll()
        pad.poll()
        assert signal.getsignal(signal.SIGINT) is custom_handler
    finally:
        signal.signal(signal.SIGINT, original)


# --- R33: a silent (e.g. Bluetooth) pad freezes its last report instead of disconnecting ---------


class _Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


_CENTERED = ((0, 0, 0, 0, 0, 0), (False,) * 15)
_LX_FULL = ((32767, 0, 0, 0, 0, 0), (False,) * 15)


def _frozen_pad(monkeypatch, raw, stale_input_s=0.5):
    """A real Gamepad whose device reads are replaced by a settable raw snapshot (no SDL device)."""
    from robotarm.master.gamepad import Gamepad

    clock = _Clock()
    pad = Gamepad(stale_input_s=stale_input_s, clock=clock)
    holder = {"raw": raw}
    monkeypatch.setattr(pad, "_ready", True)
    monkeypatch.setattr(pad, "_device", object())
    monkeypatch.setattr(pad, "_pump_events", lambda: None)
    monkeypatch.setattr(pad, "_read_raw", lambda: holder["raw"])
    monkeypatch.setattr(pad, "_use_controller_api", True)
    return pad, clock, holder


def test_frozen_input_with_deflected_stick_reports_disconnected(monkeypatch):
    pad, clock, holder = _frozen_pad(monkeypatch, _LX_FULL)
    s = pad.poll()
    assert s.connected and s.lx == pytest.approx(1.0)
    clock.t = 0.45
    assert pad.poll().connected
    clock.t = 0.55
    assert not pad.poll().connected          # bit-identical for > 0.5 s while deflected
    holder["raw"] = ((32700, 0, 0, 0, 0, 0), (False,) * 15)
    clock.t = 0.6
    assert pad.poll().connected              # any change: live again


def test_frozen_input_with_centered_sticks_stays_connected(monkeypatch):
    pad, clock, _ = _frozen_pad(monkeypatch, _CENTERED)
    for t in (0.0, 1.0, 5.0):
        clock.t = t
        assert pad.poll().connected


def test_frozen_deflection_below_deadzone_stays_connected(monkeypatch):
    pad, clock, _ = _frozen_pad(monkeypatch, ((1000, 0, 0, 0, 0, 0), (False,) * 15))
    for t in (0.0, 1.0):
        clock.t = t
        assert pad.poll().connected


def test_stale_input_check_disabled_with_zero(monkeypatch):
    pad, clock, _ = _frozen_pad(monkeypatch, _LX_FULL, stale_input_s=0.0)
    for t in (0.0, 1.0, 5.0):
        clock.t = t
        assert pad.poll().connected


def test_fake_gamepad_never_goes_stale():
    pad = FakeGamepad()
    pad.set(lx=1.0)
    for _ in range(3):
        assert pad.poll().connected
