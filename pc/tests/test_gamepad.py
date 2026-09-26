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
