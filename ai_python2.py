#!/usr/bin/env python3
"""Fit a first-order-plus-dead-time PWM-to-current model and plot the result.

CSV format: six comma-separated columns, no header required:
    1 index, 2 time_us, 3 encoder position, 4 ignored, 5 PWM duty command,
    6 measured current.

Install dependencies:
    python -m pip install numpy scipy matplotlib

Run:
    python fit_pwm_current.py motor_log.csv

Optional: fit only a selected interval, in seconds relative to the first log
sample, and choose the output plot filename:
    python fit_pwm_current.py motor_log.csv --fit-start-s 0.115 --fit-end-s 0.125 \
        --plot pwm_current_fit.png

The fitted model is G(s) = K * exp(-L*s) / (tau*s + 1), with an output offset
for the measured current baseline. Use the whole record only if one linear model
is appropriate over the whole duty/current range. For a local plant model,
include the pre-step baseline, the selected step, and its settling response.
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import least_squares


def load_log(path):
    """Read the first six numeric CSV fields; silently skip headers/blank rows."""
    rows = []
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        for row in csv.reader(f):
            if len(row) < 6:
                continue
            try:
                values = [float(row[i].strip()) for i in range(6)]
            except (ValueError, TypeError):
                continue
            if np.all(np.isfinite(values)):
                rows.append(values)

    if len(rows) < 8:
        raise ValueError("Need at least 8 numeric data rows with six columns.")

    data = np.asarray(rows, dtype=float)
    time_us = data[:, 1]
    pwm = data[:, 4]
    current = data[:, 5]

    # Sort by time, and keep the last row if timestamps are duplicated.
    order = np.argsort(time_us, kind="stable")
    time_us, pwm, current = time_us[order], pwm[order], current[order]
    keep = np.r_[time_us[1:] != time_us[:-1], True]
    time_us, pwm, current = time_us[keep], pwm[keep], current[keep]

    if len(time_us) < 8 or np.any(np.diff(time_us) <= 0):
        raise ValueError("Timestamps must be strictly increasing after cleanup.")

    time_s = (time_us - time_us[0]) * 1e-6
    return time_us, time_s, pwm, current


def simulate_fopdt(time_s, pwm, K, tau_s, delay_s, bias):
    """Simulate y = bias + K*z, where tau*dz/dt = u(t-delay)-z.

    PWM is treated as zero-order-held between log samples. Integration is exact
    between delayed PWM transitions, even when timestamps are not evenly spaced.
    The initial filtered input is set to the first logged PWM value.
    """
    n = len(time_s)
    z = np.empty(n, dtype=float)
    z[0] = pwm[0]

    # Input-change events after the first sample.
    changed = np.flatnonzero(np.diff(pwm) != 0) + 1
    event_t = time_s[changed]
    event_u = pwm[changed]
    shifted_events = event_t + delay_s

    for i in range(n - 1):
        left, right = time_s[i], time_s[i + 1]
        first = np.searchsorted(shifted_events, left, side="right")
        last = np.searchsorted(shifted_events, right, side="left")
        cuts = np.r_[left, shifted_events[first:last], right]

        state = z[i]
        for a, b in zip(cuts[:-1], cuts[1:]):
            if b <= a:
                continue
            midpoint_delayed_time = 0.5 * (a + b) - delay_s
            event_index = np.searchsorted(event_t, midpoint_delayed_time, side="right") - 1
            input_value = pwm[0] if event_index < 0 else event_u[event_index]
            decay = np.exp(-(b - a) / tau_s)
            state = input_value + (state - input_value) * decay
        z[i + 1] = state

    return bias + K * z


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", help="CSV log with time in column 2, PWM in 5, current in 6")
    parser.add_argument("--fit-start-s", type=float, default=None,
                        help="fit start time in seconds relative to the first log sample")
    parser.add_argument("--fit-end-s", type=float, default=None,
                        help="fit end time in seconds relative to the first log sample")
    parser.add_argument("--plot", default=None, help="output PNG filename (default: <csv>_fit.png)")
    args = parser.parse_args()

    time_us, t, pwm, current = load_log(args.csv_file)
    fit_mask = np.ones(len(t), dtype=bool)
    if args.fit_start_s is not None:
        fit_mask &= t >= args.fit_start_s
    if args.fit_end_s is not None:
        fit_mask &= t <= args.fit_end_s
    if fit_mask.sum() < 8:
        raise ValueError("Fit interval contains fewer than 8 samples.")

    dt = np.diff(t)
    dt_med = float(np.median(dt))
    duration = float(t[-1] - t[0])
    u_span = max(float(np.ptp(pwm)), 1.0)
    y_span = max(float(np.ptp(current)), 1.0)

    # Parameter vector: [K, tau, delay, current_offset].
    # A broad delay bound is allowed; the data must determine whether it is useful.
    tau_min = max(dt_med / 100.0, 1e-7)
    tau_max = max(duration, 10.0 * dt_med)
    delay_max = min(max(duration / 4.0, 5.0 * dt_med), 0.05)
    K0 = y_span / u_span
    b0 = float(np.median(current[:min(20, len(current))]))
    x0 = np.array([K0, max(2.0 * dt_med, tau_min * 10.0), dt_med, b0])
    lower = np.array([-100.0 * K0, tau_min, 0.0, np.min(current) - 3.0 * y_span])
    upper = np.array([100.0 * K0, tau_max, delay_max, np.max(current) + 3.0 * y_span])
    x0 = np.minimum(np.maximum(x0, lower + 1e-12), upper - 1e-12)

    def residual(params):
        K, tau_s, delay_s, bias = params
        prediction = simulate_fopdt(t, pwm, K, tau_s, delay_s, bias)
        return prediction[fit_mask] - current[fit_mask]

    result = least_squares(
        residual, x0, bounds=(lower, upper), x_scale="jac",
        loss="soft_l1", f_scale=max(0.05 * y_span, 1.0), max_nfev=3000
    )
    K, tau_s, delay_s, bias = result.x
    fitted = simulate_fopdt(t, pwm, K, tau_s, delay_s, bias)
    err = fitted[fit_mask] - current[fit_mask]
    rmse = float(np.sqrt(np.mean(err ** 2)))
    denom = float(np.sum((current[fit_mask] - np.mean(current[fit_mask])) ** 2))
    r_squared = 1.0 - float(np.sum(err ** 2)) / denom if denom > 0 else float("nan")

    print("Fit: G(s) = K * exp(-L*s) / (tau*s + 1)")
    print(f"K      = {K:.6g} current-units / PWM-unit")
    print(f"tau    = {tau_s * 1e3:.6g} ms")
    print(f"L      = {delay_s * 1e3:.6g} ms")
    print(f"offset = {bias:.6g} current-units")
    print(f"RMSE   = {rmse:.6g} current-units")
    print(f"R^2    = {r_squared:.5f}")
    print(f"median sample interval = {dt_med * 1e6:.1f} us")
    if tau_s < 3.0 * dt_med:
        print("WARNING: fitted tau has fewer than 3 samples per time constant; "
              "increase current sampling rate for a trustworthy dynamic fit.")
    if not result.success:
        print("Optimizer note:", result.message)

    # Plot PWM command, measured/fitted current, and residual.
    plot_path = args.plot or str(Path(args.csv_file).with_suffix("")) + "_fit.png"
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True,
                             gridspec_kw={"height_ratios": [1, 2, 1]})
    x_ms = t * 1e3
    axes[0].step(x_ms, pwm, where="post", color="tab:blue")
    axes[0].set_ylabel("PWM command")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(x_ms, current, ".", ms=3, label="measured current", color="black")
    axes[1].plot(x_ms, fitted, "-", lw=1.6, label="FOPDT fit", color="tab:red")
    axes[1].set_ylabel("Current (logged units)")
    axes[1].legend(loc="best")
    axes[1].grid(True, alpha=0.3)

    residual_all = current - fitted
    axes[2].axhline(0, color="black", lw=0.8)
    axes[2].plot(x_ms, residual_all, color="tab:green", lw=0.9)
    axes[2].set_ylabel("Residual")
    axes[2].set_xlabel("Time from first sample (ms)")
    axes[2].grid(True, alpha=0.3)

    if args.fit_start_s is not None:
        for ax in axes:
            ax.axvline(args.fit_start_s * 1e3, color="gray", ls="--", lw=0.8)
    if args.fit_end_s is not None:
        for ax in axes:
            ax.axvline(args.fit_end_s * 1e3, color="gray", ls="--", lw=0.8)

    fig.suptitle(f"PWM-to-current fit: K={K:.3g}, tau={tau_s*1e3:.3g} ms, "
                 f"delay={delay_s*1e3:.3g} ms")
    fig.tight_layout()
    fig.savefig(plot_path, dpi=160)
    print("Graph saved to:", plot_path)
    plt.show()


if __name__ == "__main__":
    main()
