"""Unit tests for the wall-clock pacing helper shared by sim.server.run_realtime
and robotarm.bus's in-process real-time thread (fix round 1, Task 13)."""

from robotarm.sim.pacing import DEFAULT_CATCH_UP_CAP, ms_behind, steps_to_catch_up


def test_ms_behind_when_wall_clock_has_run_ahead_of_sim_time():
    # 1 s of wall time has passed but sim time hasn't moved at all: 1000 ms behind.
    assert ms_behind(wall0=0.0, sim0=0.0, now=1.0, sim_time=0.0) == 1000


def test_ms_behind_is_negative_when_sim_is_ahead_of_wall_clock():
    assert ms_behind(wall0=0.0, sim0=0.0, now=0.0, sim_time=0.01) == -10


def test_steps_to_catch_up_is_never_negative():
    assert steps_to_catch_up(wall0=0.0, sim0=0.0, now=0.0, sim_time=0.01, cap=50) == 0


def test_steps_to_catch_up_matches_ms_behind_below_the_cap():
    assert steps_to_catch_up(wall0=0.0, sim0=0.0, now=0.01, sim_time=0.0, cap=50) == 10


def test_steps_to_catch_up_is_clamped_at_the_cap():
    assert steps_to_catch_up(wall0=0.0, sim0=0.0, now=1.0, sim_time=0.0, cap=DEFAULT_CATCH_UP_CAP) == \
        DEFAULT_CATCH_UP_CAP
