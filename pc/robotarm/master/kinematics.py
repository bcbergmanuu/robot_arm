"""Forward/inverse kinematics of the 5-DOF arm (J1..J5; the gripper J6 is ignored).

Joint convention (see robotarm/sim/model.py): q = 0 is the arm straight up;
J2/J3/J4 hinge about +y and a positive angle tilts toward +x; J1 and J5
rotate about +z. The TCP sits on the J5 axis, so L4 = wrist + gripper.

    r = L2 sin q2 + L3 sin(q2+q3) + L4 sin(q2+q3+q4)
    z = h + L2 cos q2 + L3 cos(q2+q3) + L4 cos(q2+q3+q4)
    x = r cos q1 ; y = r sin q1 ; pitch = q2+q3+q4 ; roll = q5

r is signed: with q2 < 0 the arm can lean back past the J1 axis. The inverse
takes whichever of (atan2(y, x), +r) and (atan2(y, x) + pi, -r) has q1 nearest
the seed, then the elbow branch (q3 = +/-acos D) nearest the seed; it never
falls back to the other branch (that would be a sudden elbow flip).
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from robotarm.config import ArmConfig

ARM_JOINTS = ("j1", "j2", "j3", "j4", "j5")
_ON_AXIS_R = 1e-6  # below this radius q1 is undefined: keep the seed's
_D_EPS = 1e-9  # |D| up to 1 + this counts as exactly reachable (fully stretched/folded elbow)


@dataclass(frozen=True)
class Pose:
    """Gripper pose. Metres; pitch = angle of the gripper from vertical (q2+q3+q4); roll = q5."""

    x: float
    y: float
    z: float
    pitch: float
    roll: float


class Kinematics:
    def __init__(self, cfg: ArmConfig) -> None:
        g = cfg.geometry
        self.h = g.base_height
        self.l2 = g.upper_arm
        self.l3 = g.forearm
        self.l4 = g.wrist + g.gripper
        by_joint = {a.joint: a for a in cfg.axes}
        self.soft_limits = [by_joint[j].soft_limits_rad for j in ARM_JOINTS]

    def forward(self, q: Sequence[float]) -> Pose:
        """Pose of the TCP for q = [q1..q5] (extra entries, e.g. the gripper, are ignored)."""
        q1, q2, q3, q4, q5 = q[:5]
        a3 = q2 + q3
        pitch = a3 + q4
        r = self.l2 * math.sin(q2) + self.l3 * math.sin(a3) + self.l4 * math.sin(pitch)
        z = self.h + self.l2 * math.cos(q2) + self.l3 * math.cos(a3) + self.l4 * math.cos(pitch)
        return Pose(r * math.cos(q1), r * math.sin(q1), z, pitch, q5)

    def inverse(self, pose: Pose, q_seed: Sequence[float]) -> list[float] | None:
        """Joint angles [q1..q5] reaching `pose`; q1 and the elbow branch nearest `q_seed`.

        None if the pose is out of reach or any joint would leave its soft limits.
        """
        r = math.hypot(pose.x, pose.y)
        if r < _ON_AXIS_R:
            q1 = q_seed[0]
        else:
            # r is signed along q1: the arm may lean back past the J1 axis (q2 < 0), so
            # q1 = atan2(y, x) with +r and q1 + pi with -r are both candidates -- take
            # the one nearest the seed so q1 never jumps by half a turn.
            q1 = math.atan2(pose.y, pose.x)
            flipped = _wrap(q1 + math.pi)
            if abs(_wrap(flipped - q_seed[0])) < abs(_wrap(q1 - q_seed[0])):
                q1, r = flipped, -r
        rw = r - self.l4 * math.sin(pose.pitch)
        zw = pose.z - self.h - self.l4 * math.cos(pose.pitch)
        d = (rw * rw + zw * zw - self.l2 ** 2 - self.l3 ** 2) / (2 * self.l2 * self.l3)
        if abs(d) > 1 + _D_EPS:
            return None
        q3 = math.acos(max(-1.0, min(1.0, d)))
        if abs(-q3 - q_seed[2]) < abs(q3 - q_seed[2]):
            q3 = -q3
        q2 = _wrap(math.atan2(rw, zw) - math.atan2(self.l3 * math.sin(q3), self.l2 + self.l3 * math.cos(q3)))
        q4 = _wrap(pose.pitch - q2 - q3)
        q = [q1, q2, q3, q4, pose.roll]
        return q if self._within_soft_limits(q) else None

    def _within_soft_limits(self, q: Sequence[float]) -> bool:
        return all(lo <= v <= hi for v, (lo, hi) in zip(q, self.soft_limits, strict=True))


def _wrap(angle: float) -> float:
    """Angle wrapped to [-pi, pi)."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
