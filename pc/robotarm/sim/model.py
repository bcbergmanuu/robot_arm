"""MuJoCo model of the 6-axis arm, generated from config/arm.yaml.

Chain: fixed base -> link1 (J1, hip/yaw, axis z) -> link2 (J2, shoulder,
axis y; upper-arm capsule) -> link3 (J3, elbow, axis y; forearm capsule) ->
link4 (J4, wrist bend, axis y; wrist capsule) -> link5 (J5, wrist roll, axis
z) -> gripper body (site "tcp") with two finger bodies (J6 + passive
j6_mirror, hinge axis x).

Joint convention (binding for the Task 17 kinematics): q = 0 has the arm
pointing straight up. J2/J3/J4 are hinges about +y so a positive angle tilts
the arm toward +x. J1 and J5 are hinges about +z. The tcp site sits exactly
on the J5 axis (no x/y offset anywhere between link5 and tcp) so rolling J5
never moves the tcp, and L4 = wrist + gripper for the kinematics.
"""

from __future__ import annotations

import mujoco

from robotarm.config import ArmConfig

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

_BASE_RADIUS = 0.05
_LINK_RADIUS = 0.025
_FINGER_LENGTH = 0.05
_FINGER_OFFSET = 0.02  # off-axis (x/y only) offset of each finger; never shifts tcp


def _joint_attrs(axis) -> str:
    """`range`/`armature`/`damping`/`frictionloss` XML attrs for one axis's joint."""
    lo, hi = axis.hard_limits_rad
    armature = axis.motor.j_rotor * axis.gear_ratio**2
    return (
        f'range="{lo:.9g} {hi:.9g}" limited="true" '
        f'armature="{armature:.9g}" damping="{axis.friction.viscous_nm_s:.9g}" '
        f'frictionloss="{axis.friction.coulomb_nm:.9g}"'
    )


def build_mjcf(cfg: ArmConfig) -> str:
    """Render the MJCF XML string for the arm described by `cfg`."""
    axes_by_joint = {ax.joint: ax for ax in cfg.axes}
    g = cfg.geometry
    j1, j2, j3, j4, j5, j6 = (axes_by_joint[name] for name in JOINT_NAMES)

    j6_lo, j6_hi = j6.hard_limits_rad
    mirror_armature = j6.motor.j_rotor * j6.gear_ratio**2

    return f"""
<mujoco model="katana6m_arm">
  <compiler angle="radian"/>
  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>

  <default>
    <geom contype="0" conaffinity="0"/>
  </default>

  <worldbody>
    <light pos="0.5 0.5 1.5" dir="-0.3 -0.3 -1" directional="true"/>
    <camera name="viewer" pos="1.0 -1.0 0.8" xyaxes="1 1 0 -0.3 0.3 1"/>
    <geom name="floor" type="plane" size="1 1 0.01" pos="0 0 0"
          contype="1" conaffinity="0" rgba="0.6 0.6 0.6 1"/>

    <body name="base" pos="0 0 0">
      <geom name="base_geom" type="cylinder" fromto="0 0 0 0 0 {g.base_height:.9g}"
            size="{_BASE_RADIUS:.9g}" rgba="0.3 0.3 0.3 1"/>

      <body name="link1" pos="0 0 0">
        <joint name="j1" type="hinge" axis="0 0 1" pos="0 0 0" {_joint_attrs(j1)}/>
        <geom name="link1_geom" type="sphere" size="{_LINK_RADIUS * 1.4:.9g}"
              mass="{j1.link_mass_kg:.9g}" rgba="0.7 0.2 0.2 1"/>

        <body name="link2" pos="0 0 {g.base_height:.9g}">
          <joint name="j2" type="hinge" axis="0 1 0" pos="0 0 0" {_joint_attrs(j2)}/>
          <geom name="link2_geom" type="capsule" fromto="0 0 0 0 0 {g.upper_arm:.9g}"
                size="{_LINK_RADIUS:.9g}" mass="{j2.link_mass_kg:.9g}" rgba="0.2 0.4 0.8 1"/>

          <body name="link3" pos="0 0 {g.upper_arm:.9g}">
            <joint name="j3" type="hinge" axis="0 1 0" pos="0 0 0" {_joint_attrs(j3)}/>
            <geom name="link3_geom" type="capsule" fromto="0 0 0 0 0 {g.forearm:.9g}"
                  size="{_LINK_RADIUS * 0.85:.9g}" mass="{j3.link_mass_kg:.9g}" rgba="0.2 0.6 0.4 1"/>

            <body name="link4" pos="0 0 {g.forearm:.9g}">
              <joint name="j4" type="hinge" axis="0 1 0" pos="0 0 0" {_joint_attrs(j4)}/>
              <geom name="link4_geom" type="capsule" fromto="0 0 0 0 0 {g.wrist:.9g}"
                    size="{_LINK_RADIUS * 0.7:.9g}" mass="{j4.link_mass_kg:.9g}" rgba="0.8 0.6 0.2 1"/>

              <body name="link5" pos="0 0 {g.wrist:.9g}">
                <joint name="j5" type="hinge" axis="0 0 1" pos="0 0 0" {_joint_attrs(j5)}/>
                <geom name="link5_geom" type="sphere" size="{_LINK_RADIUS * 0.9:.9g}"
                      mass="{j5.link_mass_kg:.9g}" rgba="0.5 0.5 0.2 1"/>

                <body name="gripper" pos="0 0 0">
                  <geom name="gripper_geom" type="capsule" fromto="0 0 0 0 0 {g.gripper:.9g}"
                        size="{_LINK_RADIUS * 0.6:.9g}" mass="{j6.link_mass_kg:.9g}" rgba="0.6 0.2 0.6 1"/>
                  <site name="tcp" pos="0 0 {g.gripper:.9g}" size="0.005"/>

                  <body name="finger_l" pos="{_FINGER_OFFSET:.9g} 0 {g.gripper * 0.5:.9g}">
                    <joint name="j6" type="hinge" axis="1 0 0" pos="0 0 0" {_joint_attrs(j6)}/>
                    <geom name="finger_l_geom" type="capsule" fromto="0 0 0 0 0 {_FINGER_LENGTH:.9g}"
                          size="{_LINK_RADIUS * 0.3:.9g}" mass="0.01" rgba="0.1 0.1 0.1 1"/>
                  </body>

                  <body name="finger_r" pos="{-_FINGER_OFFSET:.9g} 0 {g.gripper * 0.5:.9g}">
                    <joint name="j6_mirror" type="hinge" axis="1 0 0" pos="0 0 0"
                           range="{-j6_hi:.9g} {-j6_lo:.9g}" limited="true"
                           armature="{mirror_armature:.9g}" damping="{j6.friction.viscous_nm_s:.9g}"
                           frictionloss="{j6.friction.coulomb_nm:.9g}"/>
                    <geom name="finger_r_geom" type="capsule" fromto="0 0 0 0 0 {_FINGER_LENGTH:.9g}"
                          size="{_LINK_RADIUS * 0.3:.9g}" mass="0.01" rgba="0.1 0.1 0.1 1"/>
                  </body>

                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <equality>
    <joint joint1="j6_mirror" joint2="j6" polycoef="0 -1 0 0 0"/>
  </equality>
</mujoco>
"""


def load_model(cfg: ArmConfig) -> mujoco.MjModel:
    """Compile the generated MJCF for `cfg` into a MuJoCo model."""
    return mujoco.MjModel.from_xml_string(build_mjcf(cfg))
