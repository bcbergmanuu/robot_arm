"""Fit a continuous-time transfer function to the logged motor data.

The log (``output.txt``) is comma separated, one row per sample:

    0: timestamp [us]   1: position   2: speed   3: drive (pwm/torque)   4: current [adc]

``fit_transfer_function`` feeds two of those columns to the ``tfest`` library
(input -> output) and returns both the estimated transfer function and the
curve it produces when the measured input is replayed through it.

Note: ``tfest`` builds its wheel by importing itself, so it needs
``pip install --no-build-isolation tfest`` in an environment that already has
matplotlib, numpy and scipy.
"""

from typing import NamedTuple, Optional

import numpy as np
import tfest
from scipy import signal

DATA_FILE = "output.txt"

TIME_COL = 0
POSITION_COL = 1
SPEED_COL = 2
DRIVE_COL = 3
CURRENT_COL = 4


class Fit(NamedTuple):
    """Result of :func:`fit_transfer_function`."""

    tf: signal.lti  #: the fitted continuous-time transfer function
    t: np.ndarray  #: sample times [s]
    u: np.ndarray  #: measured input
    y: np.ndarray  #: measured output
    y_fit: np.ndarray  #: the computed curve, tf replayed on u
    rmse: float  #: root mean square error between y and y_fit
    estimator: tfest.tfest  #: the tfest object, for .plot() / .plot_bode()

    def __str__(self):
        num = np.array2string(self.tf.num, precision=4)
        den = np.array2string(self.tf.den, precision=4)
        return f"H(s) = {num} / {den}   (rmse={self.rmse:.4g})"


def load_data(path=DATA_FILE):
    """Read the log into an (n_samples, n_columns) array."""
    return np.loadtxt(path, delimiter=",")


def fit_transfer_function(
    path=DATA_FILE,
    x_col=TIME_COL,
    y_col=SPEED_COL,
    n_zeros=1,
    n_poles=2,
    time_col=TIME_COL,
    time_unit=1e-6,
    method="h1",
    restarts=5,
    seed=0,
    verbose=False,
):
    """Fit a transfer function from column ``x_col`` to column ``y_col``.

    ``x_col`` is the input u, ``y_col`` the output y (speed by default).  The
    drive signal that actually excites the motor is column ``DRIVE_COL``; pass
    ``x_col=DRIVE_COL`` to fit the physical speed step response instead.

    ``time_col`` only sets the time base (converted to seconds with
    ``time_unit``); pass ``time_col=None`` to assume a unit sample interval.

    ``tfest`` starts its optimiser from a random guess, so the fit is repeated
    ``restarts`` times and the lowest-loss run is kept.

    Returns a :class:`Fit`: the transfer function plus the computed curve.
    """
    data = load_data(path)
    u = data[:, x_col].astype(float)
    y = data[:, y_col].astype(float)

    if time_col is None:
        t = np.arange(len(u), dtype=float)
    else:
        t = data[:, time_col].astype(float) * time_unit
        t -= t[0]
    duration = (t[-1] - t[0]) * len(t) / (len(t) - 1)  # tfest uses dt = time/len(u)

    best, best_estimator = None, None
    rng = np.random.default_rng(seed)
    state = np.random.get_state()  # tfest draws its x0 from the global np.random
    try:
        for _ in range(max(1, restarts)):
            np.random.seed(int(rng.integers(2**31)))
            estimator = tfest.tfest(u, y)
            res = estimator.estimate(
                n_zeros,
                n_poles,
                method=method,
                time=duration,
                options={"xatol": 1e-3, "disp": verbose},
            )
            if best is None or res.fun < best.fun:
                best, best_estimator = res, estimator
    finally:
        np.random.set_state(state)

    tf = best_estimator.get_transfer_function()
    _, y_fit, _ = signal.lsim(tf, U=u, T=t)
    rmse = float(np.sqrt(np.mean((y - y_fit) ** 2)))
    return Fit(tf=tf, t=t, u=u, y=y, y_fit=y_fit, rmse=rmse, estimator=best_estimator)


def plot(fit: Fit, path: Optional[str] = None):
    """Plot the measured output against the computed curve."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots()
    ax.plot(fit.t, fit.y, label="measured")
    ax.plot(fit.t, fit.y_fit, label="fitted")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("speed")
    ax.set_title(str(fit))
    ax.legend()
    ax.grid(True)
    if path:
        fig.savefig(path, dpi=120, bbox_inches="tight")
    else:
        plt.show()
    return fig, ax


if __name__ == "__main__":
    fit = fit_transfer_function()
    print(fit)
