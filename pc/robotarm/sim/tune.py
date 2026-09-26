"""`robotarm tune --axis NAME`: step-response metrics for one axis in the lockstep simulator.

Homes every axis from 5 deg before its stop, then applies a 20 deg position step
(away from the stop) to the chosen axis and prints overshoot, settle time and
steady-state error. `--velocity` instead applies a velocity step (half of
max_velocity) and reports the same metrics for the joint velocity.

Gains live in config/arm.yaml and are compiled into libsimaxis, so the tuning
loop is: edit gains -> `uv run robotarm gen-config` -> `make host` -> rerun.
Procedure per axis: raise vel_kp until the velocity step overshoots > 10 %, then
take 50 %; set vel_ki so vel_kp/vel_ki is 20-50 ms; set pos_kp for 1-2 % overshoot.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np

from robotarm import protocol as p
from robotarm.config import ArmConfig, AxisConfig, load_arm_config
from robotarm.sim.harness import run_lockstep
from robotarm.sim.model import JOINT_NAMES
from robotarm.sim.world import SimBus, SimWorld

HOME_SECONDS = 8.0
HEARTBEAT_MS = 50
SETTLE_BAND = 0.02  # fraction of the step size


@dataclass(frozen=True)
class StepMetrics:
    overshoot_pct: float
    settle_time_s: float    # last exit from the +-2 % band, from the moment the step is commanded
    steady_error: float     # mean |error| over the last 100 ms, in the step's unit
    unit: str
    home_offset_deg: float  # joint angle at the backed-off pose minus the soft limit (homing accuracy)
    faults: p.Fault         # the tuned axis's faults at the end of the step (should be none)


class _Session:
    """A homed SimWorld + SimBus with heartbeats, recording one axis each ms."""

    def __init__(self, cfg: ArmConfig, axis: AxisConfig) -> None:
        start = {a.joint: a.home.position_rad - a.home.direction * math.radians(5) for a in cfg.axes}
        self.world = SimWorld(cfg, initial_q=start)
        self.bus = SimBus(self.world)
        self.index = JOINT_NAMES.index(axis.joint)
        joint = self.world.model.joint(axis.joint)
        self._qpos, self._dof = joint.qposadr[0], joint.dofadr[0]
        self._seq = 0
        self.samples: list[tuple[float, float]] = []  # (q, qd) of the tuned joint
        self.bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.HOME))
        run_lockstep(self.world, self.bus, HOME_SECONDS, on_ms=self._on_ms)
        if not all(ax.debug().homed for ax in self.world.axes):
            raise RuntimeError(f"not every axis homed within {HOME_SECONDS:.0f} s")
        edge = axis.soft_limits_rad[0] if axis.home.direction < 0 else axis.soft_limits_rad[1]
        self.home_offset_deg = math.degrees(self.world.joint_positions()[self.index] - edge)

    def _on_ms(self, t: float) -> None:
        if round(t * 1000) % HEARTBEAT_MS == 0:
            self.bus.send(p.encode_heartbeat(self._seq & 0xFF))
            self._seq += 1
        while self.bus.recv(timeout=0) is not None:
            pass

    def _record(self, t: float) -> None:
        self._on_ms(t)
        self.samples.append((float(self.world.data.qpos[self._qpos]), float(self.world.data.qvel[self._dof])))

    def run(self, seconds: float) -> np.ndarray:
        self.samples = []
        run_lockstep(self.world, self.bus, seconds, on_ms=self._record)
        return np.array(self.samples)

    def close(self) -> None:
        self.bus.shutdown()
        self.world.close()


def _metrics(signal: np.ndarray, target: float, unit: str, session: _Session) -> StepMetrics:
    """Metrics for a step from 0 to `target` (rad or rad/s); errors are reported in degrees."""
    direction = math.copysign(1.0, target)
    overshoot = max(0.0, float(np.max((signal - target) * direction))) / abs(target) * 100.0
    outside = np.nonzero(np.abs(signal - target) > SETTLE_BAND * abs(target))[0]
    settle = (outside[-1] + 1) / 1000.0 if len(outside) else 0.0
    steady = float(np.mean(np.abs(signal[-100:] - target)))
    faults = session.world.axes[session.index].debug().faults
    return StepMetrics(overshoot, settle, math.degrees(steady), unit, session.home_offset_deg, faults)


def position_step(cfg: ArmConfig, axis_name: str, step_deg: float = 20.0, seconds: float = 3.0) -> StepMetrics:
    """20 deg position step away from the backed-off home pose."""
    axis = cfg.axis_by_name(axis_name)
    session = _Session(cfg, axis)
    try:
        edge = axis.soft_limits_rad[0] if axis.home.direction < 0 else axis.soft_limits_rad[1]
        step = -axis.home.direction * math.radians(step_deg)
        # Measured as displacement from the actual start pose, so the (separate) homing
        # offset between MuJoCo's q and the axis's own zero does not count as tracking error.
        q0 = float(session.world.joint_positions()[session.index])
        session.bus.send(p.encode_setpoint(axis.node, p.SetpointKind.POSITION, axis.rad_to_counts(edge + step)))
        q = session.run(seconds)[:, 0]
        return _metrics(q - q0, step, "deg", session)
    finally:
        session.close()


def velocity_step(cfg: ArmConfig, axis_name: str, fraction: float = 0.5, seconds: float = 0.6) -> StepMetrics:
    """Velocity step (fraction x max_velocity) away from the backed-off home pose."""
    axis = cfg.axis_by_name(axis_name)
    session = _Session(cfg, axis)
    try:
        target = -axis.home.direction * fraction * axis.max_velocity_rad_s
        counts_per_s = round(target * axis.counts_per_rad)
        session.bus.send(p.encode_setpoint(axis.node, p.SetpointKind.VELOCITY, counts_per_s))
        qd = session.run(seconds)[:, 1]
        return _metrics(qd, target, "deg/s", session)
    finally:
        session.close()


def _run(args: argparse.Namespace) -> int:
    cfg = load_arm_config()
    axis = cfg.axis_by_name(args.axis)
    g = axis.gains
    print(f"{axis.name}: pos_kp={g.pos_kp} pos_ki={g.pos_ki} vel_kp={g.vel_kp} vel_ki={g.vel_ki} "
          f"vel_i_limit={g.vel_i_limit}")
    if args.velocity:
        m = velocity_step(cfg, axis.name)
        label = f"velocity step {math.degrees(0.5 * axis.max_velocity_rad_s):.1f} deg/s"
    else:
        m = position_step(cfg, axis.name)
        label = "position step 20 deg"
    print(f"{label}: overshoot {m.overshoot_pct:.2f} %  settle(2 %) {m.settle_time_s:.3f} s  "
          f"steady error {m.steady_error:.4f} {m.unit}  (homing offset {m.home_offset_deg:+.2f} deg)")
    if m.faults:
        print(f"axis faulted during the step: {m.faults!r}")
        return 1
    return 0


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("tune", help="step-response metrics for one axis in the simulator")
    parser.add_argument("--axis", required=True, help="axis name from config/arm.yaml (e.g. shoulder)")
    parser.add_argument("--velocity", action="store_true", help="velocity step instead of a 20 deg position step")
    parser.set_defaults(func=_run)
