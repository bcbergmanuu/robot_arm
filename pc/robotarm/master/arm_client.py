"""ArmClient: joint-level master API over any python-can bus.

Sans-IO core (process/poll) plus an optional background thread (start/close)
that drives it with wall-clock time. The same client drives the simulator
(over SimBus/TcpBus) and the real robot (over slcan/gs_usb/socketcan) --
only the bus differs.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Iterable
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
    last_seen: float | None = None  # poll()'s `now` when the last STATUS was processed (R18)


class ArmClient:
    """Sans-IO core: feed frames with process(msg), call poll(now) periodically.

    The threaded runner (start/close) does both with wall-clock time.

    Liveness uses exactly ONE clock: the caller's (controller ruling R18).
    `JointState.last_seen` is stamped with the `now` most recently passed to
    `poll()`, never with the received frame's own `msg.timestamp` -- TcpBus
    frames carry no timestamp (0.0), real python-can backends may stamp epoch
    `time.time()` (or 0.0), and any of those mixed with `poll()`'s wall clock
    (`time.monotonic()` from the background thread, or whatever clock the
    caller drives `poll()` with) would make `connected()` meaningless outside
    the in-process sim. Before the first `poll()` call, `last_seen` stays
    `None`, so `connected()` is False.
    """

    def __init__(self, bus: can.BusABC, cfg: ArmConfig, heartbeat_hz: float = 20.0,
                 keepalive_timeout: float | None = None) -> None:
        """`keepalive_timeout` (R23): None = always send HEARTBEATs from poll(). When
        set, poll() sends them only while keepalive(now) was called within that many
        seconds -- so a stalled application loop stops feeding the axis watchdogs
        even though this client's own poll()/runner keeps going. Same clock as poll() (R18).
        """
        self.bus = bus  # bus.send is assumed thread-safe (SimBus/TcpBus and typical python-can backends are)
        self.cfg = cfg
        self._heartbeat_period_s = 1.0 / heartbeat_hz
        self._keepalive_timeout = keepalive_timeout
        self._last_keepalive: float | None = None

        self._lock = threading.Lock()
        self.joints: dict[int, JointState] = {a.node: JointState(node=a.node, name=a.name) for a in cfg.axes}

        self._seq = 0
        self._last_heartbeat_t = float("-inf")
        self._now: float | None = None  # the `now` of the most recent poll() call (R18)

        self._listeners: list[Callable[[protocol.DecodedMessage], None]] = []

        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        # Set when the background runner stops on an exception (R19): a can.CanError /
        # OSError means the bus failed under it (broken TCP pipe, unplugged USB-CAN
        # adapter, ...); anything else is an internal error. Callers poll alive().
        self.error: BaseException | None = None

    # -- sans-IO core ----------------------------------------------------

    def process(self, msg: can.Message) -> None:
        """Feed one received frame. Never raises; unknown/garbage frames are ignored.

        Liveness (JointState.last_seen) is stamped with the most recent poll()
        `now`, not with msg.timestamp -- see the class docstring (R18).
        """
        decoded = protocol.decode(msg)
        if decoded is None:
            return
        # process() runs on whichever thread calls poll() (the caller's, or the background
        # runner's); add_listener() may be called concurrently from another thread (e.g. the
        # CLI thread, after client.start()) -- snapshot under the lock rather than iterating
        # self._listeners directly, and call back outside it so a listener can itself touch
        # ArmClient (e.g. read .joints) without risking a deadlock on this same lock.
        with self._lock:
            listeners = list(self._listeners)
        for listener in listeners:
            listener(decoded)
        if isinstance(decoded, protocol.Status):
            self._on_status(decoded)
        elif isinstance(decoded, protocol.Telemetry):
            self._on_telemetry(decoded)
        # Estop/Heartbeat/CommandMsg/Setpoint are master->node traffic; nothing to update.

    def add_listener(self, callback: Callable[[protocol.DecodedMessage], None]) -> None:
        """Register `callback(decoded)`, called from process() for every successfully
        decoded frame (Task 18: IdentifyRun uses this to timestamp raw STATUS/TELEMETRY
        pairs by tick index rather than by poll()'s `now`, see R26)."""
        with self._lock:
            self._listeners.append(callback)

    def keepalive(self, now: float) -> None:
        """Tell the client the application loop is alive (R23; see keepalive_timeout)."""
        self._last_keepalive = now

    def _keepalive_fresh(self, now: float) -> bool:
        if self._keepalive_timeout is None:
            return True
        last = self._last_keepalive
        return last is not None and now - last <= self._keepalive_timeout

    def poll(self, now: float) -> None:
        """Send a HEARTBEAT if due (and the keepalive is fresh), then drain bus.recv(timeout=0) into process()."""
        self._now = now
        if now - self._last_heartbeat_t >= self._heartbeat_period_s and self._keepalive_fresh(now):
            self.bus.send(protocol.encode_heartbeat(self._seq))
            self._seq = (self._seq + 1) & 0xFF
            self._last_heartbeat_t = now
        while True:
            msg = self.bus.recv(timeout=0)
            if msg is None:
                break
            self.process(msg)

    def _on_status(self, status: protocol.Status) -> None:
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
            js.last_seen = self._now

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
        try:
            while not self._stop_event.is_set():
                self.poll(time.monotonic())
                time.sleep(_POLL_PERIOD_S)
        except (can.CanError, OSError) as exc:  # the bus failed under us (R19)
            self.error = exc
        except Exception as exc:  # noqa: BLE001 -- a bug, not a bus failure: still stop and surface it
            self.error = exc

    def alive(self) -> bool:
        """True while the background runner is running and has not hit a bus error.

        False before start(), after close(), and once the bus failed under the
        runner (see `error`). Without the runner there are no HEARTBEATs, so
        the axes watchdog-fault on their own within `watchdog_ms`.
        """
        return self._thread is not None and self._thread.is_alive() and self.error is None

    def close(self) -> None:
        """Stop the background thread (if any) and send a DISABLE broadcast.

        Best effort: if the bus is already gone the DISABLE cannot be sent; the
        failure is recorded in `error` (if none is yet) instead of raised --
        the axes then watchdog-fault on their own. Does not shut the bus down
        -- the caller owns it.
        """
        if self._thread is not None:
            self._stop_event.set()
            self._thread.join(timeout=1.0)
            self._thread = None
        try:
            self.bus.send(protocol.encode_command(protocol.NODE_BROADCAST, protocol.Command.DISABLE))
        except (can.CanError, OSError) as exc:
            if self.error is None:
                self.error = exc

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
        """True iff every configured node has sent a STATUS within stale_s of `now`.

        `now` must be on the same clock as the `now` passed to poll() (R18).
        """
        with self._lock:
            return all(js.last_seen is not None and now - js.last_seen <= stale_s for js in self.joints.values())
