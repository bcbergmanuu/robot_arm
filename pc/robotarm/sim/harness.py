"""Deterministic lockstep driver for SimWorld + SimBus (tests, tuning)."""

from __future__ import annotations

from collections.abc import Callable

from robotarm.sim.world import SimBus, SimWorld


def run_lockstep(world: SimWorld, bus: SimBus, seconds: float,
                 on_ms: Callable[[float], None] | None = None) -> None:
    """Advance sim time deterministically. Every ms: world.step(1); bus.pump(); on_ms(world.time) if given.
    `on_ms` is where tests call ArmClient.poll(now) (Task 14) or send frames themselves."""
    for _ in range(round(seconds * 1000)):
        world.step(1)
        bus.pump()
        if on_ms is not None:
            on_ms(world.time)
