"""SimWorld: the MuJoCo arm driven by six simulated axis nodes, plus an in-process CAN bus.

Every simulated millisecond each NativeAxis runs one axis_core tick (and its DC
motor model) against the current joint state, returns a joint torque that is
applied as a generalized force, and MuJoCo advances one 1 ms step. Frames the
axes transmit are collected with the sim time as their timestamp.
"""

from __future__ import annotations

import queue
import threading

import can
import mujoco
import numpy as np

from robotarm.config import ArmConfig
from robotarm.sim.model import JOINT_NAMES, load_model
from robotarm.sim.native import NativeAxis

_MIRROR_JOINT = "j6_mirror"  # passive finger, coupled to j6 as q = -q6 by an equality


class SimWorld:
    """MuJoCo arm + one NativeAxis per joint, stepped in lockstep at 1 kHz.

    Thread-safe: step/deliver/take_outgoing may be called from different threads.
    """

    def __init__(self, cfg: ArmConfig, initial_q: dict[str, float] | None = None) -> None:
        self.cfg = cfg
        self.model = load_model(cfg)
        self.data = mujoco.MjData(self.model)
        self._dt = self.model.opt.timestep
        self._lock = threading.RLock()
        self._outgoing: list[can.Message] = []
        self._ms = 0

        axes_by_joint = {a.joint: a for a in cfg.axes}
        self.axes: list[NativeAxis] = [NativeAxis(axes_by_joint[j].node, cfg) for j in JOINT_NAMES]
        self._qpos_adr = np.array([self.model.joint(j).qposadr[0] for j in JOINT_NAMES])
        self._dof_adr = np.array([self.model.joint(j).dofadr[0] for j in JOINT_NAMES])
        self._mirror_qpos_adr = self.model.joint(_MIRROR_JOINT).qposadr[0]

        if initial_q:
            self.set_joint_positions(initial_q)
        mujoco.mj_forward(self.model, self.data)

    @property
    def time(self) -> float:
        """Seconds of simulated time (exact multiple of 1 ms)."""
        return self._ms * self._dt

    def step(self, n_ms: int = 1) -> None:
        with self._lock:
            data = self.data
            for _ in range(n_ms):
                q = data.qpos[self._qpos_adr]
                qd = data.qvel[self._dof_adr]
                for i, axis in enumerate(self.axes):
                    data.qfrc_applied[self._dof_adr[i]] = axis.step(1, float(q[i]), float(qd[i]))
                mujoco.mj_step(self.model, data)
                self._ms += 1
                # Drain every ms: an axis's tx queue is only AXIS_TX_QUEUE_LEN frames deep.
                now = self.time
                for axis in self.axes:
                    for msg in axis.recv_all():
                        msg.timestamp = now
                        self._outgoing.append(msg)

    def deliver(self, msg: can.Message) -> None:
        """Put a frame on the bus: every node sees it and filters by id itself."""
        with self._lock:
            for axis in self.axes:
                axis.send(msg)

    def take_outgoing(self) -> list[can.Message]:
        with self._lock:
            out, self._outgoing = self._outgoing, []
        return out

    def joint_positions(self) -> np.ndarray:
        with self._lock:
            return self.data.qpos[self._qpos_adr].copy()

    def set_joint_positions(self, q: dict[str, float]) -> None:
        """Teleport joints (by name, rad) and zero their velocities; the gripper mirror follows j6."""
        with self._lock:
            for name, value in q.items():
                i = JOINT_NAMES.index(name)
                self.data.qpos[self._qpos_adr[i]] = value
                self.data.qvel[self._dof_adr[i]] = 0.0
                if name == "j6":
                    self.data.qpos[self._mirror_qpos_adr] = -value
            mujoco.mj_forward(self.model, self.data)

    def close(self) -> None:
        for axis in self.axes:
            axis.close()


class SimBus(can.BusABC):
    """Thread-safe in-process python-can bus bound to a SimWorld.

    send() delivers straight to the simulated nodes; frames the nodes transmit
    reach recv() only after pump() moves them from the world into the queue.
    """

    def __init__(self, world: SimWorld, channel: str = "sim", **kwargs) -> None:
        super().__init__(channel=channel, **kwargs)
        self.channel_info = f"SimBus({channel})"
        self._world = world
        self._rx: queue.SimpleQueue[can.Message] = queue.SimpleQueue()

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        self._world.deliver(msg)

    def pump(self) -> None:
        for msg in self._world.take_outgoing():
            self._rx.put(msg)

    def _recv_internal(self, timeout: float | None) -> tuple[can.Message | None, bool]:
        try:
            if timeout is not None and timeout <= 0:
                return self._rx.get_nowait(), False
            return self._rx.get(timeout=timeout), False
        except queue.Empty:
            return None, False
