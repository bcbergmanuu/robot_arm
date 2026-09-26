import math
import random

import mujoco
import pytest

from robotarm.config import load_arm_config
from robotarm.master.kinematics import Kinematics, Pose
from robotarm.sim.model import load_model

ARM_JOINTS = ["j1", "j2", "j3", "j4", "j5"]


@pytest.fixture(scope="module")
def cfg():
    return load_arm_config()


@pytest.fixture(scope="module")
def kin(cfg):
    return Kinematics(cfg)


def _soft_limits(cfg):
    by_joint = {a.joint: a for a in cfg.axes}
    return [by_joint[j].soft_limits_rad for j in ARM_JOINTS]


def _random_q(rng, limits, margin=0.0):
    return [rng.uniform(lo + margin, hi - margin) for lo, hi in limits]


def _pose_close(a: Pose, b: Pose, lin=1e-6, ang=1e-6):
    return (abs(a.x - b.x) < lin and abs(a.y - b.y) < lin and abs(a.z - b.z) < lin
            and abs(a.pitch - b.pitch) < ang and abs(a.roll - b.roll) < ang)


def test_fk_zero_pose_is_straight_up(kin, cfg):
    g = cfg.geometry
    p = kin.forward([0.0] * 5)
    assert p.x == pytest.approx(0.0, abs=1e-12)
    assert p.y == pytest.approx(0.0, abs=1e-12)
    assert p.z == pytest.approx(g.base_height + g.upper_arm + g.forearm + g.wrist + g.gripper)
    assert p.pitch == 0.0 and p.roll == 0.0


def test_fk_matches_mujoco_tcp_site(kin, cfg):
    model = load_model(cfg)
    data = mujoco.MjData(model)
    rng = random.Random(17)
    limits = _soft_limits(cfg)
    for _ in range(50):
        q = _random_q(rng, limits)
        for name, value in zip(ARM_JOINTS, q, strict=True):
            data.joint(name).qpos = value
        mujoco.mj_forward(model, data)
        tcp = data.site("tcp").xpos
        p = kin.forward(q)
        assert (p.x, p.y, p.z) == pytest.approx(tuple(tcp), abs=1e-3), q
        assert p.pitch == pytest.approx(q[1] + q[2] + q[3])
        assert p.roll == pytest.approx(q[4])


def test_ik_inverts_fk(kin, cfg):
    rng = random.Random(1717)
    limits = _soft_limits(cfg)
    checked = 0
    while checked < 200:
        q = _random_q(rng, limits, margin=math.radians(1))
        if abs(q[2]) < math.radians(2):  # elbow nearly straight: the two IK branches coincide
            continue
        pose = kin.forward(q)
        if math.hypot(pose.x, pose.y) < 1e-3:  # on the J1 axis q1 is undefined
            continue
        seed = [v + rng.uniform(-0.02, 0.02) for v in q]
        sol = kin.inverse(pose, seed)
        assert sol is not None, q
        assert sol == pytest.approx(q, abs=1e-6), q
        assert _pose_close(kin.forward(sol), pose)
        checked += 1


def test_ik_picks_elbow_branch_nearest_seed(kin):
    q = [0.3, 0.4, 0.9, -0.5, 0.0]
    pose = kin.forward(q)
    other = kin.inverse(pose, [0.3, 1.0, -0.9, 0.2, 0.0])
    assert other is not None and other[2] < 0.0  # the elbow-flipped solution of the same pose
    assert _pose_close(kin.forward(other), pose)
    assert kin.inverse(pose, q) == pytest.approx(q, abs=1e-9)


def test_ik_on_j1_axis_keeps_seed_q1(kin, cfg):
    g = cfg.geometry
    pose = Pose(0.0, 0.0, g.base_height + g.upper_arm + g.forearm + g.wrist + g.gripper, 0.0, 0.4)
    sol = kin.inverse(pose, [0.7, 0.0, 0.0, 0.0, 0.0])
    assert sol is not None
    assert sol == pytest.approx([0.7, 0.0, 0.0, 0.0, 0.4], abs=1e-6)


def test_ik_unreachable_is_none(kin):
    assert kin.inverse(Pose(2.0, 0.0, 0.3, 0.0, 0.0), [0.0] * 5) is None


def test_ik_outside_soft_limits_is_none(kin, cfg):
    # Reachable geometrically, but needs q1 = 170 deg (soft limit 160).
    q = [math.radians(170), math.radians(40), math.radians(40), 0.0, 0.0]
    pose = kin.forward(q)
    assert kin.inverse(pose, q) is None
    # Needs roll beyond the J5 soft limit.
    ok = kin.forward([0.2, 0.5, 0.5, 0.2, 0.0])
    assert kin.inverse(ok, [0.2, 0.5, 0.5, 0.2, 0.0]) is not None
    assert kin.inverse(Pose(ok.x, ok.y, ok.z, ok.pitch, math.radians(170)), [0.2, 0.5, 0.5, 0.2, 0.0]) is None


def test_ik_arm_leaning_back_past_j1_axis(kin):
    """q2 < 0 with the elbow bent back: r < 0 along q1. IK keeps q1 instead of flipping it by pi."""
    q = [0.5, math.radians(-20), math.radians(-60), math.radians(-40), 0.1]
    pose = kin.forward(q)
    sol = kin.inverse(pose, q)
    assert sol == pytest.approx(q, abs=1e-9)
