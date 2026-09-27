import subprocess
import sys

import pytest

from robotarm.bus import open_bus


def test_unknown_scheme():
    with pytest.raises(ValueError):
        open_bus("carrier-pigeon://x")


def test_monitor_without_sim_exits_2_with_one_line_error():
    r = subprocess.run([sys.executable, "-m", "robotarm", "monitor", "--bus", "tcp://127.0.0.1:1"],
                       capture_output=True, text=True, timeout=20)
    assert r.returncode == 2
    assert r.stderr.strip().startswith("error:") and "Traceback" not in r.stderr


_CLI_BUS_COMMANDS = [
    ["monitor"],
    ["teleop", "--fake-gamepad"],
    ["identify", "--node", "2"],
]


@pytest.mark.parametrize("url", ["slcan:/dev/nonexistent", "gs_usb:0"])
@pytest.mark.parametrize("command", _CLI_BUS_COMMANDS, ids=lambda c: c[0])
def test_real_bus_without_device_is_one_line_error(command, url):
    """No adapter attached (or its python-can backend missing): a single `error:` line, exit 2."""
    r = subprocess.run([sys.executable, "-m", "robotarm", *command, "--bus", url],
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 2, r.stderr
    lines = r.stderr.strip().splitlines()
    assert len(lines) == 1 and lines[0].startswith("error:"), r.stderr
    assert "Traceback" not in r.stderr


def test_gs_usb_channel_is_passed_as_integer_index(monkeypatch):
    import can

    seen = {}

    def fake_bus(**kwargs):
        seen.update(kwargs)
        return object()

    monkeypatch.setattr(can, "Bus", fake_bus)
    open_bus("gs_usb:1")
    assert seen["interface"] == "gs_usb" and seen["index"] == 1 and seen["bitrate"] == 1_000_000
    with pytest.raises(ValueError, match="gs_usb"):
        open_bus("gs_usb:first")
