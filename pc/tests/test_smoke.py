import subprocess
import sys


def test_cli_help_runs():
    out = subprocess.run([sys.executable, "-m", "robotarm", "--help"], capture_output=True, text=True)
    assert out.returncode == 0
    assert "robotarm" in out.stdout
