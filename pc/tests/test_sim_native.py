import numpy as np
import pytest

from robotarm.sim import native


def _params(tau_coulomb=0.0):
    motor = native.MotorParams(R=1.14, L=0.00012, kt=0.0191, supply_v=12.0, gear_ratio=1.0,
                               gear_efficiency=1.0, counts_per_motor_rev=256.0,
                               sense_mv_per_a=528.0, adc_max_mv=2500.0)
    return native.BenchParams(motor=motor, j_total=5e-6, b_viscous=0.0, tau_coulomb=tau_coulomb)


def test_run_bench_reaches_no_load_speed():
    duty = np.ones(2000, dtype=np.float32)
    res = native.run_bench(_params(), duty, 1e-3)
    assert res.omega[-1] == pytest.approx(12.0 / 0.0191, rel=0.01)
    assert res.pos.dtype == np.int32 and len(res.pos) == 2000
    assert res.pos[-1] > 0 and res.current_ma[0] == 0.0


def test_run_bench_stalls_against_coulomb():
    res = native.run_bench(_params(tau_coulomb=1.0), np.ones(100), 1e-3)
    assert not res.pos.any() and not res.omega.any()


def test_missing_library_is_reported(monkeypatch, tmp_path):
    monkeypatch.setenv("ROBOTARM_SIMAXIS_LIB", str(tmp_path / "nope.dylib"))
    with pytest.raises(FileNotFoundError, match="make host"):
        native.load_library()
