"""`robotarm stepfit`: fit the bench motor model to a recorded open-loop step response.

The recording (e.g. output.txt) has columns `time_us,position,velocity,pwm_ticks,current`.
The model (host/sim/bench.c) is the DC-motor electrical model driving an inertia with
viscous and Coulomb friction (with stiction), duty 0 = shorted winding (brake). The motor
electrical constants come from config/arm.yaml; J, b and Tc are fitted to the position trace.
"""

from __future__ import annotations

import argparse
import itertools
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares

from robotarm.config import MotorConfig, load_arm_config
from robotarm.sim.native import BenchParams, MotorParams, run_bench

PWM_TICK_MAX = 400      # BDC_MCPWM_DUTY_TICK_MAX in the firmware that made the recording
DEFAULT_MOTOR = "faulhaber_2657cr_12v"

_FIT_KEYS = ("j_total", "b_viscous", "tau_coulomb")
_MOTOR_KEYS = ("R", "L", "kt", "supply_v", "gear_ratio", "gear_efficiency",
               "counts_per_motor_rev", "sense_mv_per_a", "adc_max_mv")


@dataclass(frozen=True)
class StepData:
    t_s: np.ndarray
    pos: np.ndarray          # encoder counts as recorded (response to +duty)
    duty: np.ndarray         # 0..1
    current_raw: np.ndarray  # recorded current column (mA, only valid while driven)


@dataclass(frozen=True)
class FitReport:
    rmse_counts: float
    final_error_pct: float


def load_step_csv(path: Path) -> StepData:
    """Load a step recording. Positions are kept as recorded: the pwm was applied in
    reverse on the bench, so the (negated) recorded position is the response to +duty."""
    raw = np.loadtxt(path, delimiter=",", ndmin=2)
    return StepData(
        t_s=raw[:, 0] * 1e-6,
        pos=raw[:, 1].astype(np.int64),
        duty=raw[:, 3] / PWM_TICK_MAX,
        current_raw=raw[:, 4],
    )


def _sample_dt(data: StepData) -> float:
    return float(np.median(np.diff(data.t_s)))


def simulate(params: BenchParams, data: StepData) -> np.ndarray:
    """Simulated encoder positions at the data sample times."""
    return run_bench(params, data.duty, _sample_dt(data)).pos.astype(np.int64)


def motor_params(motor: MotorConfig, supply_v: float, counts_per_rev: float, adc_max_mv: float) -> MotorParams:
    """Motor alone on the bench: no gearbox, encoder counts per motor revolution."""
    return MotorParams(R=motor.R, L=motor.L, kt=motor.kt, supply_v=supply_v, gear_ratio=1.0,
                       gear_efficiency=1.0, counts_per_motor_rev=counts_per_rev,
                       sense_mv_per_a=motor.sense_mv_per_a, adc_max_mv=adc_max_mv)


def _bench(mp: MotorParams, x: np.ndarray) -> BenchParams:
    j, b, tc = np.exp(x)
    return BenchParams(motor=mp, j_total=j, b_viscous=b, tau_coulomb=tc)


def _report(params: BenchParams, data: StepData) -> FitReport:
    err = simulate(params, data) - data.pos
    final = float(data.pos[-1])
    return FitReport(rmse_counts=float(np.sqrt(np.mean(err**2))),
                     final_error_pct=100.0 * abs(float(err[-1])) / abs(final))


def fit_bench(data: StepData, motor: MotorConfig, supply_v: float,
              counts_per_rev: float, adc_max_mv: float) -> tuple[BenchParams, FitReport]:
    """Least-squares fit of (log J, log b, log Tc) to the recorded position trace.

    The encoder quantisation makes the cost piecewise constant, hence the coarse
    finite-difference step and a small grid of starts scaled by the motor's
    electrical time constant and stall torque."""
    mp = motor_params(motor, supply_v, counts_per_rev, adc_max_mv)
    electrical_damping = motor.kt**2 / motor.R          # Nm s/rad of the (shorted) winding
    stall_torque = motor.kt * supply_v / motor.R

    def residual(x: np.ndarray) -> np.ndarray:
        return (simulate(_bench(mp, x), data) - data.pos).astype(float)

    lo = np.log([1e-9, 1e-9, 1e-6])
    hi = np.log([1e-1, 1e-1, 10.0])
    best = None
    for tau_mech, tc_frac in itertools.product((0.01, 0.05, 0.2), (0.05, 0.3, 0.6)):
        x0 = np.log([electrical_damping * tau_mech, 0.1 * electrical_damping, tc_frac * stall_torque])
        res = least_squares(residual, x0, bounds=(lo, hi), diff_step=1e-2)
        if best is None or res.cost < best.cost:
            best = res
    params = _bench(mp, best.x)
    return params, _report(params, data)


def _f32(x: float) -> float:
    """Shortest decimal that round-trips the C float (keeps the YAML readable)."""
    return float(f"{x:.7g}")


def save_identified(path: Path, params: BenchParams, report: FitReport, meta: dict) -> None:
    doc = {
        **meta,
        "motor": {k: _f32(getattr(params.motor, k)) for k in _MOTOR_KEYS},
        **{k: _f32(getattr(params, k)) for k in _FIT_KEYS},
        "fit": {"rmse_counts": round(report.rmse_counts, 3),
                "final_error_pct": round(report.final_error_pct, 3)},
    }
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = "# GENERATED by `uv run robotarm stepfit` -- bench model identified from a step recording.\n"
    path.write_text(header + yaml.safe_dump(doc, sort_keys=False))


def load_identified(path: Path) -> BenchParams:
    doc = yaml.safe_load(Path(path).read_text())
    motor = MotorParams(**{k: float(doc["motor"][k]) for k in _MOTOR_KEYS})
    return BenchParams(motor=motor, **{k: float(doc[k]) for k in _FIT_KEYS})


def plot_fit(path: Path, data: StepData, params: BenchParams, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    res = run_bench(params, data.duty, _sample_dt(data))
    t_ms = data.t_s * 1e3
    fig, (ax_pos, ax_err, ax_cur) = plt.subplots(3, 1, sharex=True, figsize=(8, 8),
                                                 gridspec_kw={"height_ratios": [3, 1, 1.5]})
    ax_pos.plot(t_ms, data.pos, label="recorded (output.txt)", color="#1f77b4", lw=2)
    ax_pos.plot(t_ms, res.pos, label="model", color="#d62728", lw=1.2, ls="--")
    ax_duty = ax_pos.twinx()
    ax_duty.fill_between(t_ms, data.duty, step="post", color="0.85", zorder=0)
    ax_duty.set_ylim(0, 4)
    ax_duty.set_yticks([])
    ax_pos.set_zorder(ax_duty.get_zorder() + 1)
    ax_pos.patch.set_visible(False)
    ax_pos.set_ylabel("position [counts]")
    ax_pos.set_title(title)
    ax_pos.legend(loc="upper left")
    ax_err.plot(t_ms, res.pos - data.pos, color="#d62728")
    ax_err.axhline(0, color="0.5", lw=0.8)
    ax_err.set_ylabel("model - rec.\n[counts]")
    driven = data.duty > 0
    ax_cur.plot(t_ms, np.where(driven, data.current_raw, np.nan), label="recorded", color="#1f77b4")
    ax_cur.plot(t_ms, np.where(driven, res.current_ma, np.nan), label="model", color="#d62728", ls="--")
    ax_cur.set_ylabel("current [mA]")
    ax_cur.set_xlabel("time [ms]  (grey: duty 100 %)")
    ax_cur.legend(loc="upper right")
    for ax in (ax_pos, ax_err, ax_cur):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def _cmd_stepfit(args: argparse.Namespace) -> int:
    data = load_step_csv(args.csv)
    arm_cfg = load_arm_config()
    motors = arm_cfg.motors
    if args.motor not in motors:
        raise SystemExit(f"unknown motor {args.motor!r}; known: {', '.join(motors)}")
    params, report = fit_bench(data, motors[args.motor], args.supply, args.cpr, arm_cfg.adc_max_mv)
    final = float(data.pos[-1])
    print(f"motor {args.motor} @ {args.supply:g} V, {args.cpr:g} counts/rev")
    print(f"  j_total     = {params.j_total:.4g} kg m^2")
    print(f"  b_viscous   = {params.b_viscous:.4g} Nm s/rad")
    print(f"  tau_coulomb = {params.tau_coulomb:.4g} Nm")
    print(f"  rmse {report.rmse_counts:.2f} counts ({100 * report.rmse_counts / abs(final):.2f} % of final), "
          f"final error {report.final_error_pct:.2f} %")
    out, plot = output_paths(args)
    meta = {"source": Path(args.csv).name, "motor_name": args.motor}
    save_identified(out, params, report, meta)
    print(f"wrote {out}")
    plot_fit(plot, data, params, f"Bench step response: {args.motor} @ {args.supply:g} V, {args.cpr:g} counts/rev")
    print(f"wrote {plot}")
    return 0


def output_paths(args: argparse.Namespace) -> tuple[Path, Path]:
    """--out/--plot, defaulting to scratch files stepfit_<csv stem>.yaml/.png in the current
    directory -- never the committed reference fit (config/bench_identified.yaml,
    docs/img/stepfit.png), which only changes when passed explicitly."""
    stem = Path(args.csv).stem
    out = args.out if args.out is not None else Path(f"stepfit_{stem}.yaml")
    plot = args.plot if args.plot is not None else Path(f"stepfit_{stem}.png")
    return out, plot


def register(subparsers: argparse._SubParsersAction) -> None:
    p = subparsers.add_parser("stepfit", help="fit the bench motor model to a recorded step response")
    p.add_argument("csv", nargs="?", default="output.txt", type=Path)
    p.add_argument("--cpr", type=float, default=256.0, help="encoder counts per motor revolution (x4 lines)")
    p.add_argument("--motor", default=DEFAULT_MOTOR, help="motor name from config/arm.yaml")
    p.add_argument("--supply", type=float, default=12.0, help="bridge supply voltage during the recording")
    p.add_argument("--plot", type=Path, default=None,
                   help="fit plot (default: stepfit_<csv name>.png in the current directory)")
    p.add_argument("--out", type=Path, default=None,
                   help="identified parameters YAML (default: stepfit_<csv name>.yaml in the current directory; "
                        "the committed reference fit is config/bench_identified.yaml)")
    p.set_defaults(func=_cmd_stepfit)
