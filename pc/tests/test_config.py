import math
import subprocess
import sys

import pytest

from robotarm.config import load_arm_config


@pytest.fixture(scope="module")
def cfg():
    return load_arm_config()


def test_six_axes_with_unique_nodes(cfg):
    assert [a.node for a in cfg.axes] == [1, 2, 3, 4, 5, 6]


def test_counts_per_rad_shoulder(cfg):
    shoulder = cfg.axis_by_name("shoulder")
    assert shoulder.counts_per_rad == pytest.approx(4 * 64 * 370 / (2 * math.pi))


def test_twelve_volt_motor_duty_is_capped(cfg):
    assert cfg.axis_by_name("shoulder").max_duty == pytest.approx(0.5)
    assert cfg.axis_by_name("hip").max_duty == pytest.approx(1.0)


def test_rad_counts_roundtrip(cfg):
    elbow = cfg.axis_by_name("elbow")
    assert elbow.counts_to_rad(elbow.rad_to_counts(0.5)) == pytest.approx(0.5, abs=1e-4)


def test_home_position_defaults_to_hard_limit(cfg):
    hip = cfg.axis_by_name("hip")
    assert hip.home.direction == -1
    assert hip.home.position_rad == pytest.approx(math.radians(-169))


def test_soft_limits_inside_hard_limits(cfg):
    for a in cfg.axes:
        assert a.hard_limits_rad[0] < a.soft_limits_rad[0] < a.soft_limits_rad[1] < a.hard_limits_rad[1]


def test_generated_c_table_is_up_to_date():
    result = subprocess.run([sys.executable, "-m", "robotarm", "gen-config", "--check"], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
