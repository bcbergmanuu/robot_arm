"""ArmClient: joint-level master API over any python-can bus.

Sans-IO core (process/poll) plus an optional background thread (start/close)
that drives it with wall-clock time. The same client drives the simulator
(over SimBus/TcpBus) and the real robot (over slcan/gs_usb/socketcan) --
only the bus differs.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Iterable
from dataclasses import dataclass, field

import can

from robotarm import protocol
from robotarm.config import ArmConfig
from robotarm.protocol import AxisState, Fault

_POLL_PERIOD_S = 0.005  # background thread poll interval


@dataclass
class JointState:
    node: int
    name: str
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    current_a: float = 0.0
    state: AxisState = AxisState.DISABLED
    faults: Fault = field(default_factory=lambda: Fault(0))
    homed: bool = False
    last_seen: float | None = None  # timestamp of the last STATUS


class ArmClient:
    """Sans-IO core: feed frames with process(msg), call poll(now) periodically.

    The threaded runner (start/close) does both with wall-clock time.
    """

    def __init__(self, bus: can.BusABC, cfg: ArmConfig, heartbeat_hz: float = 20.0) -> None:
        self.bus = bus
        self.cfg = cfg
        self._heartbeat_period_s = 1.0 / heartbeat_hz

        self._lock = threading.Lock()
        self.joints: dict[int, JointState] = {a.node: JointState(node=a.node, name=a.name) for a in cfg.axes}

        self._seq = 0
        self._last_heartbeat_t = float("-inf")

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    # -- sans-IO core ----------------------------------------------------

    def process(self, msg: can.Message) -> None:
        """Feed one received frame. Never raises; unknown/garbage frames are ignored."""
        decoded = protocol.decode(msg)
        if isinstance(decoded, protocol.Status):
            self._on_status(decoded, msg.timestamp)
        elif isinstance(decoded, protocol.Telemetry):
            self._on_telemetry(decoded)
        # Estop/Heartbeat/CommandMsg/Setpoint are master->node traffic; nothing to update.

    def poll(self, now: float) -> None:
        """Send a HEARTBEAT if due, then drain bus.recv(timeout=0) into process()."""
        if now - self._last_heartbeat_t >= self._heartbeat_period_s:
            self.bus.send(protocol.encode_heartbeat(self._seq))
            self._seq = (self._seq + 1) & 0xFF
            self._last_heartbeat_t = now
        while True:
            msg = self.bus.recv(timeout=0)
            if msg is None:
                break
            self.process(msg)

    def _on_status(self, status: protocol.Status, timestamp: float) -> None:
        axis = self._axis_for_node(status.node)
        if axis is None:
            return
        with self._lock:
            js = self.joints.get(status.node)
            if js is None:
                return
            js.position_rad = axis.counts_to_rad(status.position)
            js.state = status.state
            js.faults = status.faults
            js.homed = status.homed
            js.last_seen = timestamp

    def _on_telemetry(self, telemetry: protocol.Telemetry) -> None:
        axis = self._axis_for_node(telemetry.node)
        if axis is None:
            return
        with self._lock:
            js = self.joints.get(telemetry.node)
            if js is None:
                return
            js.velocity_rad_s = axis.counts_to_rad(telemetry.velocity)
            js.current_a = telemetry.current_ma / 1000.0

    def _axis_for_node(self, node: int):
        try:
            return self.cfg.axis(node)
        except KeyError:
            return None

    # -- background thread ------------------------------------------------

    def start(self) -> None:
        """Run poll(time.monotonic()) every 5 ms on a background thread."""
        if self._thread is not None:
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="ArmClient", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self.poll(time.monotonic())
            time.sleep(_POLL_PERIOD_S)

    def close(self) -> None:
        """Stop the background thread (if any) and send a DISABLE broadcast.

        Does not shut the bus down -- the caller owns it.
        """
        if self._thread is not None:
            self._stop_event.set()
            self._thread.join(timeout=1.0)
            self._thread = None
        self.bus.send(protocol.encode_command(protocol.NODE_BROADCAST, protocol.Command.DISABLE))

    # -- commands ----------------------------------------------------------

    def estop(self) -> None:
        self.bus.send(protocol.encode_estop())

    def enable(self, nodes: Iterable[int] | None = None) -> None:
        self._send_command(protocol.Command.ENABLE, nodes)

    def disable(self, nodes: Iterable[int] | None = None) -> None:
        self._send_command(protocol.Command.DISABLE, nodes)

    def home(self, nodes: Iterable[int] | None = None) -> None:
        self._send_command(protocol.Command.HOME, nodes)

    def clear_faults(self, nodes: Iterable[int] | None = None) -> None:
        self._send_command(protocol.Command.CLEAR_FAULT, nodes)

    def _send_command(self, cmd: protocol.Command, nodes: Iterable[int] | None) -> None:
        if nodes is None:
            self.bus.send(protocol.encode_command(protocol.NODE_BROADCAST, cmd))
        else:
            for node in nodes:
                self.bus.send(protocol.encode_command(node, cmd))

    def set_velocity(self, node: int, rad_s: float) -> None:
        axis = self.cfg.axis(node)
        counts_per_s = round(rad_s * axis.counts_per_rad)
        self.bus.send(protocol.encode_setpoint(node, protocol.SetpointKind.VELOCITY, counts_per_s))

    def set_position(self, node: int, rad: float) -> None:
        axis = self.cfg.axis(node)
        lo, hi = axis.soft_limits_rad
        clamped = min(max(rad, lo), hi)
        self.bus.send(protocol.encode_setpoint(node, protocol.SetpointKind.POSITION, axis.rad_to_counts(clamped)))

    def set_duty(self, node: int, duty: float) -> None:
        clamped = min(max(duty, -1.0), 1.0)
        value = round(clamped * protocol.DUTY_SCALE)
        self.bus.send(protocol.encode_setpoint(node, protocol.SetpointKind.DUTY, value))

    # -- queries ------------------------------------------------------------

    def all_ready(self) -> bool:
        with self._lock:
            return all(js.state == AxisState.READY for js in self.joints.values())

    def all_homed(self) -> bool:
        with self._lock:
            return all(js.homed for js in self.joints.values())

    def any_fault(self) -> bool:
        with self._lock:
            return any(js.faults for js in self.joints.values())

    def connected(self, now: float, stale_s: float = 0.5) -> bool:
        """True iff every configured node has sent a STATUS within stale_s."""
        with self._lock:
            return all(js.last_seen is not None and now - js.last_seen <= stale_s for js in self.joints.values())
