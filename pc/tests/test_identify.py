import pytest

from robotarm import protocol as p
from robotarm.analysis.steptest import load_step_csv
from robotarm.config import load_arm_config
from robotarm.master.arm_client import ArmClient
from robotarm.master.identify import IdentifyRun, _duty_refusal, _state_refusal
from robotarm.sim.harness import run_lockstep
from robotarm.sim.world import SimBus, SimWorld


@pytest.fixture
def cfg():
    return load_arm_config()


@pytest.fixture
def sim(cfg):
    """Factory for (world, bus) pairs; closes every bus and world after the test."""
    created = []

    def make(initial_q=None):
        world = SimWorld(cfg, initial_q=initial_q)
        bus = SimBus(world)
        created.append((world, bus))
        return world, bus

    yield make
    for world, bus in created:
        bus.shutdown()
        world.close()


def test_identify_records_step_csv(cfg, sim, tmp_path):
    """Node 2 (shoulder) starting mid-range: run the step experiment in lockstep,
    check the recorded samples and the written CSV round-trips through stepfit's loader."""
    axis = cfg.axis_by_name("shoulder")
    # 0 rad is the arm's low-inertia, near-balanced "all-zero" pose (docs/simulator.md); well
    # inside the shoulder's soft limits ([-25, 100] deg), so this is a mid-range start, not one
    # pressed against a stop -- but far enough from either stop that a 20 deg on-phase can't hit one.
    world, bus = sim({axis.joint: 0.0})

    client = ArmClient(bus, cfg)
    run = IdentifyRun(client, axis.node, duty=axis.max_duty, pre_s=0.08, on_s=0.08, post_s=0.04)

    def on_ms(t: float) -> None:
        client.poll(t)
        run.update(t)

    run_lockstep(world, bus, 0.3, on_ms=on_ms)

    assert len(run.samples) >= 180

    pre_end = round(0.08 / 0.001)
    on_end = pre_end + round(0.08 / 0.001)
    positions = [s[1] for s in run.samples]
    pre_positions = positions[:pre_end]
    assert max(pre_positions) - min(pre_positions) <= 2  # flat (duty 0) during the pre-phase
    assert abs(positions[on_end - 1] - positions[pre_end]) > 5  # moved during the on-phase

    csv_path = tmp_path / "output.csv"
    run.write_csv(csv_path)
    data = load_step_csv(csv_path)
    assert len(data.pos) == len(run.samples)
    assert data.duty.max() == pytest.approx(axis.max_duty, abs=1e-6)
    assert list(data.pos) == positions


def test_identify_run_disables_axis_when_finished(cfg, sim):
    axis = cfg.axis_by_name("shoulder")
    world, bus = sim()
    client = ArmClient(bus, cfg)
    run = IdentifyRun(client, axis.node, duty=axis.max_duty, pre_s=0.02, on_s=0.02, post_s=0.01)

    def on_ms(t: float) -> None:
        client.poll(t)
        run.update(t)

    run_lockstep(world, bus, 0.2, on_ms=on_ms)

    assert client.joints[axis.node].state == p.AxisState.DISABLED


def test_duty_refusal_exceeds_max_duty(cfg):
    axis = cfg.axis_by_name("shoulder")
    assert _duty_refusal(axis, axis.max_duty + 0.01) is not None
    assert _duty_refusal(axis, axis.max_duty) is None
    assert _duty_refusal(axis, -axis.max_duty) is None


def test_state_refusal_only_disabled_or_ready_allowed(cfg):
    axis = cfg.axis_by_name("shoulder")
    assert _state_refusal(axis, p.AxisState.DISABLED) is None
    assert _state_refusal(axis, p.AxisState.READY) is None
    assert _state_refusal(axis, p.AxisState.FAULT) is not None
    assert _state_refusal(axis, p.AxisState.HOMING) is not None
