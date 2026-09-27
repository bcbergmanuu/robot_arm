import math

import mujoco
import numpy as np
import pytest

from robotarm.config import load_arm_config
from robotarm.sim.model import JOINT_NAMES, load_model


@pytest.fixture(scope="module")
def cfg():
    return load_arm_config()


@pytest.fixture(scope="module")
def model(cfg):
    return load_model(cfg)


def test_joints_and_limits(model, cfg):
    for axis, name in zip(cfg.axes, JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert jid >= 0
        assert model.jnt_range[jid] == pytest.approx(axis.hard_limits_rad, abs=1e-6)
        assert model.dof_armature[model.jnt_dofadr[jid]] == pytest.approx(axis.motor.j_rotor * axis.gear_ratio**2)


def test_tcp_height_at_zero_pose(model, cfg):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    g = cfg.geometry
    tcp = data.site("tcp").xpos
    assert tcp[2] == pytest.approx(g.base_height + g.upper_arm + g.forearm + g.wrist + g.gripper, abs=1e-6)


def test_positive_shoulder_tilts_toward_plus_x(model):
    data = mujoco.MjData(model)
    data.joint("j2").qpos = math.radians(30)
    mujoco.mj_forward(model, data)
    assert data.site("tcp").xpos[0] > 0.1


def test_gravity_torque_on_shoulder_is_plausible(model, cfg):
    data = mujoco.MjData(model)
    data.joint("j2").qpos = math.pi / 2          # arm horizontal
    mujoco.mj_forward(model, data)
    tau = data.qfrc_bias[model.jnt_dofadr[model.joint("j2").id]]
    total_mass = sum(a.link_mass_kg for a in cfg.axes[1:])
    assert 0.5 < abs(tau) < 9.81 * total_mass * 0.6  # between tiny and "all mass at max reach"
