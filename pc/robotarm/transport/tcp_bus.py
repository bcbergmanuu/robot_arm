"""TcpBus: a python-can BusABC that carries CAN frames over a plain TCP socket.

Lets the simulator (with an optional MuJoCo viewer) and the master run as
separate processes: `SimServer` (robotarm.sim.server) listens, `TcpBus`
connects. On the robot the master instead talks to real hardware through
python-can's slcan/gs_usb/socketcan/pcan interfaces (see robotarm.bus).

Wire format: each frame is exactly 13 bytes,
`struct.pack("<IB8s", arbitration_id, dlc, data.ljust(8, b"\\0"))`.
"""

from __future__ import annotations

import queue
import socket
import struct
import threading

import can

FRAME_STRUCT = struct.Struct("<IB8s")
FRAME_SIZE = FRAME_STRUCT.size  # 13


class SimNotRunningError(ConnectionError):
    """Raised by TcpBus when it cannot reach a simulator server."""


def encode_frame(msg: can.Message) -> bytes:
    data = bytes(msg.data)
    return FRAME_STRUCT.pack(msg.arbitration_id, len(data), data.ljust(8, b"\0"))


def decode_frame(buf: bytes) -> can.Message:
    arbitration_id, dlc, data = FRAME_STRUCT.unpack(buf)
    return can.Message(arbitration_id=arbitration_id, dlc=dlc, data=data[:dlc], is_extended_id=False)


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    """Read exactly n bytes from sock, or None if the peer closed before n bytes arrived."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


class TcpBus(can.BusABC):
    """Connects to a robotarm.sim.server.SimServer and exchanges 13-byte CAN frames."""

    def __init__(self, channel: str = "127.0.0.1:29536", connect_timeout: float = 2.0, **kwargs) -> None:
        host, sep, port_str = channel.rpartition(":")
        if not sep or not port_str.isdigit():
            raise ValueError(f"TcpBus channel must be 'host:port', got {channel!r}")
        port = int(port_str)
        super().__init__(channel=channel, **kwargs)
        self.channel_info = f"TcpBus({channel})"
        try:
            self._sock = socket.create_connection((host, port), timeout=connect_timeout)
        except OSError as exc:
            raise SimNotRunningError(
                f"no simulator at {host}:{port} -- start it with `make sim`"
            ) from exc
        self._sock.settimeout(None)
        self._rx: queue.SimpleQueue[can.Message] = queue.SimpleQueue()
        self._send_lock = threading.Lock()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    def _read_loop(self) -> None:
        try:
            while True:
                header = recv_exact(self._sock, FRAME_SIZE)
                if header is None:
                    return
                self._rx.put(decode_frame(header))
        except OSError:
            return

    def send(self, msg: can.Message, timeout: float | None = None) -> None:
        with self._send_lock:
            self._sock.sendall(encode_frame(msg))

    def _recv_internal(self, timeout: float | None) -> tuple[can.Message | None, bool]:
        try:
            if timeout is None:
                return self._rx.get(), False
            if timeout <= 0:
                return self._rx.get_nowait(), False
            return self._rx.get(timeout=timeout), False
        except queue.Empty:
            return None, False

    def shutdown(self) -> None:
        super().shutdown()
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self._sock.close()
        except OSError:
            pass
        self._reader.join(timeout=1)
