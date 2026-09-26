"""ctypes bindings for build/host/libsimaxis (host/sim/simlib_api.c).

The Structure classes mirror the C structs in host/sim/motor_model.h and
host/sim/bench.h field for field; keep them in sync.
"""

from __future__ import annotations

import ctypes
import os
import sys
from dataclasses import dataclass
from functools import cache
from pathlib import Path

import numpy as np

LIB_ENV_VAR = "ROBOTARM_SIMAXIS_LIB"


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


class MotorParams(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in (
        "R", "L", "kt", "supply_v", "gear_ratio", "gear_efficiency",
        "counts_per_motor_rev", "sense_mv_per_a", "adc_max_mv",
    )]


class BenchParams(ctypes.Structure):
    _fields_ = [
        ("motor", MotorParams),
        ("j_total", ctypes.c_float),
        ("b_viscous", ctypes.c_float),
        ("tau_coulomb", ctypes.c_float),
    ]


@dataclass(frozen=True)
class BenchResult:
    pos: np.ndarray         # int32 encoder counts at each sample
    omega: np.ndarray       # rad/s at the motor shaft
    current_ma: np.ndarray  # as seen by the current-sense ADC


def _library_path() -> Path:
    override = os.environ.get(LIB_ENV_VAR)
    if override:
        return Path(override)
    suffix = "dylib" if sys.platform == "darwin" else "so"
    return _repo_root() / "build" / "host" / f"libsimaxis.{suffix}"


def load_library() -> ctypes.CDLL:
    """Load libsimaxis ($ROBOTARM_SIMAXIS_LIB if set, else <repo>/build/host)."""
    path = _library_path()
    if not path.is_file():
        raise FileNotFoundError(f"libsimaxis not built -- run `make host` (looked for {path})")
    return _load(str(path))


@cache
def _load(path: str) -> ctypes.CDLL:
    lib = ctypes.CDLL(path)
    f32 = np.ctypeslib.ndpointer(np.float32, flags="C_CONTIGUOUS")
    i32 = np.ctypeslib.ndpointer(np.int32, flags="C_CONTIGUOUS")
    lib.simaxis_bench_run.argtypes = [ctypes.POINTER(BenchParams), f32, ctypes.c_int, ctypes.c_float, i32, f32, f32]
    lib.simaxis_bench_run.restype = ctypes.c_int
    return lib


def run_bench(params: BenchParams, duty: np.ndarray, dt: float) -> BenchResult:
    """Run the bench model over a duty profile sampled every dt seconds."""
    duty = np.ascontiguousarray(duty, dtype=np.float32)
    n = len(duty)
    pos = np.zeros(n, dtype=np.int32)
    omega = np.zeros(n, dtype=np.float32)
    current = np.zeros(n, dtype=np.float32)
    rc = load_library().simaxis_bench_run(ctypes.byref(params), duty, n, dt, pos, omega, current)
    if rc != 0:
        raise ValueError("simaxis_bench_run rejected its arguments (j_total must be > 0, dt > 0)")
    return BenchResult(pos=pos, omega=omega, current_ma=current)
