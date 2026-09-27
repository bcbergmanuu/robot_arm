"""Wall-clock pacing shared by the two real-time SimWorld drivers:
`robotarm.sim.server.run_realtime` (TCP) and the background thread
`robotarm.bus.open_bus("sim")` starts for an in-process bus.

Both step a SimWorld in 1 ms increments to keep pace with wall-clock time,
capping how many steps they'll do in one iteration so a stall (a slow viewer
frame, a busy host) can't make either driver charge arbitrarily far ahead
before it next checks a stop event / delivers inbound frames / syncs a viewer.
"""

from __future__ import annotations

DEFAULT_CATCH_UP_CAP = 50  # 1 ms steps per iteration, shared default for both real-time drivers


def ms_behind(wall0: float, sim0: float, now: float, sim_time: float) -> int:
    """How many ms wall-clock time is ahead of simulated time (negative if sim is ahead).

    `wall0`/`sim0` are the wall-clock and simulated time at some reference
    point (typically when the driver started); `now`/`sim_time` are their
    current values.
    """
    return round((now - wall0 - (sim_time - sim0)) * 1000)


def steps_to_catch_up(wall0: float, sim0: float, now: float, sim_time: float, cap: int) -> int:
    """1 ms steps to run right now so `sim_time` keeps up with wall-clock `now`, clamped to [0, cap]."""
    return max(0, min(ms_behind(wall0, sim0, now, sim_time), cap))
