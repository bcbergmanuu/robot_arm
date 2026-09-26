"""Shared arm configuration: single source of truth loaded from config/arm.yaml.

Loaded by the Python master/simulator and used by `robotarm gen-config` to
render the C config table (components/axis_core/src/config_table.c) that
firmware links against. Keep this module and the generator in agreement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

DEFAULT_CONFIG_RELATIVE_PATH = "config/arm.yaml"


def _repo_root() -> Path:
    """Repo root, found relative to this file's location (pc/robotarm/config.py)."""
    return Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class MotorConfig:
    name: str
    nominal_v: float
    R: float
    L: float
    kt: float
    j_rotor: float
    sense_mv_per_a: float


@dataclass(frozen=True)
class HomeConfig:
    direction: int
    velocity_rad_s: float
    current_ma: float
    timeout_s: float
    position_rad: float


@dataclass(frozen=True)
class GainsConfig:
    pos_kp: float
    pos_ki: float
    vel_kp: float
    vel_ki: float
    vel_i_limit: float


@dataclass(frozen=True)
class FrictionConfig:
    coulomb_nm: float
    viscous_nm_s: float


@dataclass(frozen=True)
class Geometry:
    base_height: float
    upper_arm: float
    forearm: float
    wrist: float
    gripper: float


@dataclass(frozen=True)
class AxisConfig:
    node: int
    name: str
    joint: str
    encoder_cpr: int
    gear_ratio: float
    motor: MotorConfig
    motor_sign: int
    encoder_sign: int
    soft_limits_rad: tuple[float, float]
    hard_limits_rad: tuple[float, float]
    max_velocity_rad_s: float
    max_accel_rad_s2: float
    home: HomeConfig
    gains: GainsConfig
    friction: FrictionConfig
    link_mass_kg: float
    gear_efficiency: float
    max_current_ma: float
    max_following_error_rad: float
    watchdog_ms: int
    supply_voltage: float  # denormalized from ArmConfig, needed by max_duty/vel_ff

    @property
    def counts_per_rad(self) -> float:
        """Encoder counts per joint radian (quadrature x4, through the gearbox)."""
        return 4 * self.encoder_cpr * self.gear_ratio / (2 * math.pi)

    @property
    def max_duty(self) -> float:
        """Motor nominal voltage / supply voltage, capped at 1."""
        return min(1.0, self.motor.nominal_v / self.supply_voltage)

    @property
    def vel_ff(self) -> float:
        """Duty per counts/s feed-forward: 1 / (encoder counts/s at duty 1)."""
        no_load_counts_per_s = self.supply_voltage / self.motor.kt / (2 * math.pi) * 4 * self.encoder_cpr
        return 1.0 / no_load_counts_per_s

    def rad_to_counts(self, rad: float) -> int:
        return round(rad * self.counts_per_rad)

    def counts_to_rad(self, counts: int) -> float:
        return counts / self.counts_per_rad


@dataclass(frozen=True)
class ArmConfig:
    supply_voltage: float
    control_hz: int
    axes: list[AxisConfig]
    motors: dict[str, MotorConfig]
    geometry: Geometry
    bench: dict | None = None

    def axis(self, node: int) -> AxisConfig:
        for a in self.axes:
            if a.node == node:
                return a
        raise KeyError(f"no axis with node {node}")

    def axis_by_name(self, name: str) -> AxisConfig:
        for a in self.axes:
            if a.name == name:
                return a
        raise KeyError(f"no axis named {name!r}")


def _load_motors(raw: dict) -> dict[str, MotorConfig]:
    sense_mv_per_a = float(raw["current_sense"]["mv_per_a"])
    motors: dict[str, MotorConfig] = {}
    for name, m in raw["motors"].items():
        motors[name] = MotorConfig(
            name=name,
            nominal_v=float(m["nominal_v"]),
            R=float(m["R"]),
            L=float(m["L"]),
            kt=float(m["kt"]),
            j_rotor=float(m["j_rotor"]),
            sense_mv_per_a=sense_mv_per_a,
        )
    return motors


def _load_geometry(raw: dict) -> Geometry:
    geom = raw["geometry"]
    return Geometry(
        base_height=float(geom["base_height"]),
        upper_arm=float(geom["upper_arm"]),
        forearm=float(geom["forearm"]),
        wrist=float(geom["wrist"]),
        gripper=float(geom["gripper"]),
    )


def _load_axis(
    ax: dict,
    motors: dict[str, MotorConfig],
    defaults: dict,
    supply_voltage: float,
) -> AxisConfig:
    node = int(ax["node"])
    name = ax["name"]

    motor_name = ax["motor"]
    if motor_name not in motors:
        raise ValueError(f"unknown motor {motor_name!r} for axis {name!r}")
    motor = motors[motor_name]

    soft_lo_deg, soft_hi_deg = ax["soft_limits_deg"]
    hard_lo_deg, hard_hi_deg = ax["hard_limits_deg"]
    soft_limits_rad = (math.radians(soft_lo_deg), math.radians(soft_hi_deg))
    hard_limits_rad = (math.radians(hard_lo_deg), math.radians(hard_hi_deg))
    if not (hard_limits_rad[0] < soft_limits_rad[0] < soft_limits_rad[1] < hard_limits_rad[1]):
        raise ValueError(f"soft limits not inside hard limits for axis {name!r}")

    default_gains = defaults.get("gains", {})
    gains_raw = dict(default_gains)
    gains_raw.update(ax.get("gains", {}))
    gains = GainsConfig(
        pos_kp=float(gains_raw["pos_kp"]),
        pos_ki=float(gains_raw["pos_ki"]),
        vel_kp=float(gains_raw["vel_kp"]),
        vel_ki=float(gains_raw["vel_ki"]),
        vel_i_limit=float(gains_raw["vel_i_limit"]),
    )

    friction_raw = ax["friction"]
    friction = FrictionConfig(
        coulomb_nm=float(friction_raw["coulomb_nm"]),
        viscous_nm_s=float(friction_raw["viscous_nm_s"]),
    )

    home_raw = ax["home"]
    direction = int(home_raw["direction"])
    if "position_deg" in home_raw:
        position_rad = math.radians(float(home_raw["position_deg"]))
    else:
        position_rad = hard_limits_rad[0] if direction < 0 else hard_limits_rad[1]
    home = HomeConfig(
        direction=direction,
        velocity_rad_s=math.radians(float(home_raw["velocity_deg_s"])),
        current_ma=float(home_raw["current_ma"]),
        timeout_s=float(home_raw["timeout_s"]),
        position_rad=position_rad,
    )

    gear_efficiency = float(ax.get("gear_efficiency", defaults.get("gear_efficiency", 1.0)))
    watchdog_ms = int(ax.get("watchdog_ms", defaults.get("watchdog_ms", 0)))
    following_error_deg = float(ax.get("max_following_error_deg", defaults.get("max_following_error_deg", 0.0)))

    return AxisConfig(
        node=node,
        name=name,
        joint=ax["joint"],
        encoder_cpr=int(ax["encoder_cpr"]),
        gear_ratio=float(ax["gear_ratio"]),
        motor=motor,
        motor_sign=int(ax["motor_sign"]),
        encoder_sign=int(ax["encoder_sign"]),
        soft_limits_rad=soft_limits_rad,
        hard_limits_rad=hard_limits_rad,
        max_velocity_rad_s=math.radians(float(ax["max_velocity_deg_s"])),
        max_accel_rad_s2=math.radians(float(ax["max_accel_deg_s2"])),
        home=home,
        gains=gains,
        friction=friction,
        link_mass_kg=float(ax["link_mass_kg"]),
        gear_efficiency=gear_efficiency,
        max_current_ma=float(ax["max_current_ma"]),
        max_following_error_rad=math.radians(following_error_deg),
        watchdog_ms=watchdog_ms,
        supply_voltage=supply_voltage,
    )


def load_arm_config(path: Path | None = None) -> ArmConfig:
    """Load and validate config/arm.yaml (default: repo_root/config/arm.yaml)."""
    if path is None:
        path = _repo_root() / DEFAULT_CONFIG_RELATIVE_PATH
    path = Path(path)
    with path.open("r") as f:
        raw = yaml.safe_load(f)

    supply_voltage = float(raw["supply_voltage"])
    control_hz = int(raw["control_hz"])
    motors = _load_motors(raw)
    geometry = _load_geometry(raw)
    defaults = raw.get("defaults", {})

    axes: list[AxisConfig] = []
    seen_nodes: set[int] = set()
    for ax in raw["axes"]:
        axis = _load_axis(ax, motors, defaults, supply_voltage)
        if axis.node in seen_nodes:
            raise ValueError(f"duplicate node {axis.node} (axis {axis.name!r})")
        seen_nodes.add(axis.node)
        axes.append(axis)

    return ArmConfig(
        supply_voltage=supply_voltage,
        control_hz=control_hz,
        axes=axes,
        motors=motors,
        geometry=geometry,
        bench=raw.get("bench"),
    )
