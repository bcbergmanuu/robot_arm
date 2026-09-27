"""ctypes bindings for build/host/libsimaxis (host/sim/simlib_api.c).

The Structure classes mirror the C structs in host/sim/motor_model.h,
host/sim/bench.h and host/sim/simaxis.h field for field; keep them in sync.
"""

from __future__ import annotations

import ctypes
import os
import sys
from dataclasses import dataclass
from functools import cache
from pathlib import Path

import can
import numpy as np

from robotarm.config import ArmConfig
from robotarm.protocol import AxisState, Fault

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


class SimAxisDebugStruct(ctypes.Structure):
    _fields_ = [
        ("duty", ctypes.c_float),
        ("current_ma", ctypes.c_float),
        ("motor_torque", ctypes.c_float),
        ("pos", ctypes.c_int32),
        ("vel", ctypes.c_float),
        ("state", ctypes.c_uint8),
        ("faults", ctypes.c_uint8),
        ("homed", ctypes.c_uint8),
    ]


@dataclass(frozen=True)
class SimAxisDebug:
    duty: float
    current_ma: float
    motor_torque: float
    pos: int
    vel: float
    state: AxisState
    faults: Fault
    homed: bool


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

    lib.simaxis_create.argtypes = [ctypes.c_uint8, ctypes.POINTER(MotorParams)]
    lib.simaxis_create.restype = ctypes.c_void_p
    lib.simaxis_destroy.argtypes = [ctypes.c_void_p]
    lib.simaxis_destroy.restype = None
    lib.simaxis_step.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_double, ctypes.c_double]
    lib.simaxis_step.restype = ctypes.c_double
    lib.simaxis_rx.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint8)]
    lib.simaxis_rx.restype = None
    lib.simaxis_tx.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_uint16), ctypes.POINTER(ctypes.c_uint8),
                               ctypes.POINTER(ctypes.c_uint8)]
    lib.simaxis_tx.restype = ctypes.c_int
    lib.simaxis_get_debug.argtypes = [ctypes.c_void_p, ctypes.POINTER(SimAxisDebugStruct)]
    lib.simaxis_get_debug.restype = None
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


def motor_params_for_axis(node: int, cfg: ArmConfig) -> MotorParams:
    """Build the simaxis motor_params_t for one axis from the shared arm config."""
    axis = cfg.axis(node)
    return MotorParams(
        R=axis.motor.R,
        L=axis.motor.L,
        kt=axis.motor.kt,
        supply_v=cfg.supply_voltage,
        gear_ratio=axis.gear_ratio,
        gear_efficiency=axis.gear_efficiency,
        counts_per_motor_rev=4 * axis.encoder_cpr,
        sense_mv_per_a=axis.motor.sense_mv_per_a,
        adc_max_mv=cfg.adc_max_mv,
    )


class NativeAxis:
    """One simulated CAN node: an axis_core axis_t plus the DC motor model (host/sim/simaxis.c),
    driven from Python -- the MuJoCo arm sim (Task 12) runs six of these, one per joint."""

    def __init__(self, node: int, cfg: ArmConfig) -> None:
        self._lib = load_library()
        self._motor = motor_params_for_axis(node, cfg)
        handle = self._lib.simaxis_create(node, ctypes.byref(self._motor))
        if not handle:
            raise ValueError(f"no axis config for node {node} (see components/axis_core/src/config_table.c)")
        self._handle = handle
        self.node = node

    def step(self, n_ticks: int, q: float, qd: float) -> float:
        """Advance n_ticks 1 kHz ticks with the joint held at (q, qd); returns the mean joint torque (Nm)."""
        return self._lib.simaxis_step(self._handle, n_ticks, q, qd)

    def send(self, msg: can.Message) -> None:
        data = bytes(msg.data)
        buf = (ctypes.c_uint8 * len(data))(*data)
        self._lib.simaxis_rx(self._handle, msg.arbitration_id, len(data), buf)

    def recv_all(self) -> list[can.Message]:
        """Drain every CAN frame this node has queued for transmission."""
        out: list[can.Message] = []
        frame_id = ctypes.c_uint16()
        length = ctypes.c_uint8()
        data = (ctypes.c_uint8 * 8)()
        while self._lib.simaxis_tx(self._handle, ctypes.byref(frame_id), ctypes.byref(length), data):
            out.append(can.Message(arbitration_id=frame_id.value, data=bytes(data[:length.value]),
                                   is_extended_id=False))
        return out

    def debug(self) -> SimAxisDebug:
        raw = SimAxisDebugStruct()
        self._lib.simaxis_get_debug(self._handle, ctypes.byref(raw))
        return SimAxisDebug(
            duty=raw.duty,
            current_ma=raw.current_ma,
            motor_torque=raw.motor_torque,
            pos=raw.pos,
            vel=raw.vel,
            state=AxisState(raw.state),
            faults=Fault(raw.faults),
            homed=bool(raw.homed),
        )

    def close(self) -> None:
        if self._handle is not None:
            self._lib.simaxis_destroy(self._handle)
            self._handle = None

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:
            pass
