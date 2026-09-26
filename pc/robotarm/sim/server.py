"""SimServer: TCP front end for SimWorld, and the real-time driving loop.

Architecture (see docs/simulator.md):

    TcpBus(master) ─┐                          ┌─ TcpBus(monitor)
                     ├─ SimServer ── run_realtime ── SimWorld (MuJoCo + 6 axis_core)
    TcpBus(...)  ────┘   (accept + per-client        (stepped in 1 ms increments)
                          reader threads)

Each connected client's inbound frames are queued by SimServer and delivered
to the world only -- clients never see each other's frames, only the world's
STATUS/TELEMETRY output (broadcast to every client). `run_realtime` paces the
world at wall-clock speed and, if given a viewer handle, syncs it at ~60 Hz.
"""

from __future__ import annotations

import argparse
import contextlib
import logging
import math
import os
import queue
import signal
import socket
import sys
import threading
import time

import can

from robotarm.config import load_arm_config
from robotarm.sim.pacing import DEFAULT_CATCH_UP_CAP, ms_behind, steps_to_catch_up
from robotarm.sim.world import SimWorld
from robotarm.transport.tcp_bus import FRAME_SIZE, decode_frame, encode_frame, recv_exact

logger = logging.getLogger(__name__)

_VIEWER_SYNC_HZ = 60.0
_MJPYTHON_ENV_VAR = "MJPYTHON_BIN"  # set by the mjpython launcher (mujoco package) on execve


class SimServer:
    """Listens for TCP clients and shuttles CAN frames between them and a SimWorld."""

    def __init__(self, world: SimWorld, host: str = "127.0.0.1", port: int = 29536) -> None:
        self._world = world
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((host, port))
        self._sock.listen()
        self.port: int = self._sock.getsockname()[1]

        self._inbound: queue.SimpleQueue[can.Message] = queue.SimpleQueue()
        self._clients_lock = threading.Lock()
        self._clients: list[socket.socket] = []
        self._closed = False

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def _accept_loop(self) -> None:
        while True:
            try:
                conn, _addr = self._sock.accept()
            except OSError as exc:
                if not self._closed:
                    logger.warning("SimServer accept() failed: %s", exc, exc_info=True)
                return
            with self._clients_lock:
                self._clients.append(conn)
            threading.Thread(target=self._client_loop, args=(conn,), daemon=True).start()

    def _client_loop(self, conn: socket.socket) -> None:
        try:
            while True:
                header = recv_exact(conn, FRAME_SIZE)
                if header is None:
                    return
                self._inbound.put(decode_frame(header))
        except OSError:
            return
        finally:
            with self._clients_lock:
                if conn in self._clients:
                    self._clients.remove(conn)
            try:
                conn.close()
            except OSError:
                pass

    def take_inbound(self) -> list[can.Message]:
        """Drain every frame received from any client since the last call."""
        out: list[can.Message] = []
        while True:
            try:
                out.append(self._inbound.get_nowait())
            except queue.Empty:
                break
        return out

    def broadcast(self, msgs: list[can.Message]) -> None:
        """Send world-originated frames to every connected client."""
        if not msgs:
            return
        payload = b"".join(encode_frame(m) for m in msgs)
        with self._clients_lock:
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.sendall(payload)
            except OSError:
                with self._clients_lock:
                    if conn in self._clients:
                        self._clients.remove(conn)
                try:
                    conn.close()
                except OSError:
                    pass

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._sock.close()
        except OSError:
            pass
        with self._clients_lock:
            clients, self._clients = self._clients, []
        for conn in clients:
            try:
                conn.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                conn.close()
            except OSError:
                pass


def run_realtime(world: SimWorld, server: SimServer, viewer: bool, stop_event: threading.Event) -> None:
    """Pace `world` at wall-clock speed, routing frames through `server`.

    Each iteration: figure out how many 1 ms steps real time now calls for
    (capped at 50, to keep interleaving inbound delivery / viewer sync / the
    stop_event even under heavy catch-up), deliver any inbound frames before
    each step, then broadcast whatever the world transmitted.
    """
    handle = None
    if viewer:
        import mujoco.viewer

        handle = mujoco.viewer.launch_passive(world.model, world.data)

    try:
        wall0 = time.monotonic()
        sim0 = world.time
        last_warn = 0.0
        next_sync = 0.0
        while not stop_event.is_set():
            if handle is not None and not handle.is_running():
                stop_event.set()
                break

            now = time.monotonic()
            sim_time = world.time
            steps = steps_to_catch_up(wall0, sim0, now, sim_time, DEFAULT_CATCH_UP_CAP)
            behind_ms = ms_behind(wall0, sim0, now, sim_time)

            # mj_step (inside world.step) mutates mjData; the passive viewer's own thread
            # reads/writes it too (rendering, perturbations), so every step -- like every
            # sync -- must run under the viewer's own lock, not just world._lock.
            step_guard = handle.lock() if handle is not None else contextlib.nullcontext()
            with step_guard:
                for _ in range(steps):
                    for msg in server.take_inbound():
                        world.deliver(msg)
                    world.step(1)
            if steps:
                server.broadcast(world.take_outgoing())

            if behind_ms > DEFAULT_CATCH_UP_CAP and now - last_warn >= 1.0:
                logger.warning("sim is %d ms behind real time", behind_ms)
                last_warn = now

            if handle is not None and now >= next_sync:
                with world._lock:  # noqa: SLF001 -- same package; viewer must sync a consistent qpos/qvel
                    handle.sync()
                next_sync = now + 1.0 / _VIEWER_SYNC_HZ

            if steps == 0:
                time.sleep(0.0005)
    finally:
        if handle is not None:
            handle.close()


def _running_under_mjpython() -> bool:
    return _MJPYTHON_ENV_VAR in os.environ


def _initial_positions(cfg, start: str) -> dict[str, float] | None:
    if start == "zero":
        return None
    if start == "near-home":
        return {a.joint: a.home.position_rad - a.home.direction * math.radians(5) for a in cfg.axes}
    raise ValueError(f"unknown --start value: {start!r}")


def _run(args: argparse.Namespace) -> int:
    if args.viewer and sys.platform == "darwin" and not _running_under_mjpython():
        print("run with: make sim  (or: uv run mjpython -m robotarm sim)", file=sys.stderr)
        return 2

    logging.basicConfig(level=logging.INFO)
    cfg = load_arm_config()
    world = SimWorld(cfg, initial_q=_initial_positions(cfg, args.start))
    server = SimServer(world, host=args.host, port=args.port)
    print(f"sim server listening on {args.host}:{server.port} (start={args.start}, viewer={args.viewer})")

    stop = threading.Event()

    def _on_sigint(_signum, _frame) -> None:
        stop.set()

    old_handler = signal.signal(signal.SIGINT, _on_sigint)
    try:
        run_realtime(world, server, args.viewer, stop)
    finally:
        signal.signal(signal.SIGINT, old_handler)
        server.close()
        world.close()
    return 0


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser("sim", help="run the real-time MuJoCo simulator server (TCP + optional viewer)")
    parser.add_argument("--host", default="127.0.0.1", help="listen address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=29536, help="listen port (default: 29536)")
    parser.add_argument("--no-viewer", dest="viewer", action="store_false", help="run headless, no MuJoCo viewer")
    parser.add_argument("--start", choices=["near-home", "zero"], default="near-home",
                        help="initial joint positions: 5 deg before every home stop (default), or all zero")
    parser.set_defaults(func=_run, viewer=True)
