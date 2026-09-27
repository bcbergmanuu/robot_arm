import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session", autouse=True)
def host_build():
    """Build the C host targets (libsimaxis, tests) once per test session."""
    subprocess.run(["make", "-s", "host"], cwd=REPO_ROOT, check=True)
    return REPO_ROOT / "build" / "host"
