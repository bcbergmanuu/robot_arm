"""robotarm.bus: turns a bus URL into a ready-to-use can.BusABC.

Schemes:
  tcp://host:port         TcpBus -- connects to a `robotarm sim` (or any
                          SimServer) process over TCP.
  sim                     A fresh in-process SimWorld + SimBus, stepped at
                          wall-clock speed in a background daemon thread.
                          Convenient for one-shot scripts and `robotarm
                          monitor`/tests that don't need a separate `robotarm
                          sim` process or a viewer.
  slcan:/dev/tty...[@N]   Real hardware via python-can (CAN bus at 1 Mbit/s).
  gs_usb:0                (gs_usb: the channel is the adapter's integer index;
  socketcan:can0          slcan and gs_usb need the python-can `serial` / `gs-usb`
  pcan:PCAN_USBBUS1       extras, which pyproject.toml installs.)

Opening a bus can fail in many ways (no adapter, missing backend library, bad
channel): every CLI catches OPEN_ERRORS around open_bus() and prints a single
`error: ...` line.

Also registers the `robotarm monitor --bus URL` CLI command, which prints
axis state/position/faults once a second until interrupted.
"""

from __future__ import annotations

import argparse
import gc
import logging
import math
import sys
import threading
import time

import can

from robotarm import protocol as p
from robotarm.config import ArmConfig, load_arm_config
from robotarm.sim.pacing import DEFAULT_CATCH_UP_CAP, steps_to_catch_up
from robotarm.sim.world import SimBus, SimWorld
from robotarm.transport.tcp_bus import TcpBus

_REAL_INTERFACES = {"slcan", "gs_usb", "socketcan", "pcan"}
_REAL_BITRATE = 1_000_000
_PRINT_INTERVAL_S = 1.0

# What open_bus() can raise for a bad URL or a missing/unplugged adapter: ValueError (bad URL or
# channel), ImportError (backend library missing), OSError (incl. SimNotRunningError, serial port
# errors), can.CanError (python-can's CanInitializationError, ...), TypeError (bad backend argument).
OPEN_ERRORS: tuple[type[BaseException], ...] = (can.CanError, OSError, ValueError, TypeError, ImportError)


def open_bus(url: str) -> can.BusABC:
    """Build the right can.BusABC for `url`. Raises ValueError for an unknown scheme."""
    if url.startswith("tcp://"):
        return TcpBus(url[len("tcp://"):])
    if url == "sim":
        return _open_in_process_sim()

    scheme, sep, channel = url.partition(":")
    if sep and scheme in _REAL_INTERFACES:
        return _open_real_bus(scheme, channel)

    raise ValueError(f"unknown bus URL scheme: {url!r}")


def _open_real_bus(scheme: str, channel: str) -> can.BusABC:
    kwargs: dict = {"interface": scheme, "channel": channel, "bitrate": _REAL_BITRATE}
    if scheme == "gs_usb":
        try:
            kwargs["index"] = int(channel)  # python-can's gs_usb compares it with a device count
        except ValueError:
            raise ValueError(f"gs_usb channel must be the adapter index (e.g. gs_usb:0), got {channel!r}") from None

    # A backend that fails half-way through its __init__ leaves a bus object whose __del__ logs
    # "<Bus> was not properly shut down" once the exception (and its traceback) is released. Release
    # it here, with python-can's logger quietened, so a failed open is just the one exception.
    can_log = logging.getLogger("can")
    level = can_log.level
    can_log.setLevel(logging.ERROR)
    try:
        try:
            return can.Bus(**kwargs)
        except Exception as exc:  # noqa: BLE001 -- re-raised below, without the traceback
            error = exc
        error.__traceback__ = None
        error.__context__ = None
        error.__cause__ = None
        gc.collect()
        raise error
    finally:
        can_log.setLevel(level)


class _RealtimeSimBus(SimBus):
    """A SimBus whose world is stepped at wall-clock speed by a background thread."""

    def __init__(self, world: SimWorld) -> None:
        super().__init__(world)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        wall0 = time.monotonic()
        sim0 = self._world.time
        while not self._stop.is_set():
            steps = steps_to_catch_up(wall0, sim0, time.monotonic(), self._world.time, DEFAULT_CATCH_UP_CAP)
            if steps:
                self._world.step(steps)
                self.pump()
            else:
                time.sleep(0.0005)

    def shutdown(self) -> None:
        super().shutdown()
        self._stop.set()
        self._thread.join(timeout=2)
        self._world.close()


def _open_in_process_sim() -> can.BusABC:
    world = SimWorld(load_arm_config())
    return _RealtimeSimBus(world)


def _format_faults(faults: p.Fault) -> str:
    if not faults:
        return "NONE"
    return "|".join(f.name for f in p.Fault if f in faults and f.name)


def _print_status_lines(cfg: ArmConfig, latest: dict[int, p.Status]) -> None:
    for axis in cfg.axes:
        st = latest.get(axis.node)
        if st is None:
            print(f"{axis.name:12s} node={axis.node}  (no status yet)")
            continue
        pos_deg = math.degrees(axis.counts_to_rad(st.position))
        print(f"{axis.name:12s} node={axis.node}  state={st.state.name:8s}  "
              f"pos={pos_deg:8.2f} deg  homed={st.homed!s:5s}  faults={_format_faults(st.faults)}")
    print()


def _run_monitor(args: argparse.Namespace) -> int:
    try:
        bus = open_bus(args.bus)
    except OPEN_ERRORS as exc:
        print(f"error: {args.bus}: {exc}", file=sys.stderr)
        return 2

    cfg = load_arm_config()
    latest: dict[int, p.Status] = {}
    try:
        next_print = time.monotonic() + _PRINT_INTERVAL_S
        while True:
            msg = bus.recv(timeout=0.1)
            decoded = p.decode(msg) if msg is not None else None
            if isinstance(decoded, p.Status):
                latest[decoded.node] = decoded
            now = time.monotonic()
            if now >= next_print:
                _print_status_lines(cfg, latest)
                next_print = now + _PRINT_INTERVAL_S
    except KeyboardInterrupt:
        return 0
    finally:
        bus.shutdown()


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("monitor", help="print live axis state/position/faults from a bus URL")
    parser.add_argument("--bus", required=True,
                        help="bus URL: tcp://host:port, sim, slcan:/dev/tty..., gs_usb:0, socketcan:can0, ...")
    parser.set_defaults(func=_run_monitor)
