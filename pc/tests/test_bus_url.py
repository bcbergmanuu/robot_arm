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
