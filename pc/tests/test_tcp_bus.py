import threading
import time

import pytest

from robotarm import protocol as p
from robotarm.config import load_arm_config
from robotarm.sim.server import SimServer, run_realtime
from robotarm.sim.world import SimWorld
from robotarm.transport.tcp_bus import SimNotRunningError, TcpBus


def test_connect_without_server_gives_clear_error():
    with pytest.raises(SimNotRunningError, match="make sim"):
        TcpBus("127.0.0.1:1", connect_timeout=0.2)


@pytest.fixture
def running_sim():
    world = SimWorld(load_arm_config())
    server = SimServer(world, port=0)          # port 0: OS picks; server.port has the real one
    stop = threading.Event()
    thread = threading.Thread(target=run_realtime, args=(world, server, False, stop), daemon=True)
    thread.start()
    yield world, server
    stop.set()
    thread.join(2)
    server.close()


def recv_status(bus, node, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        d = p.decode(msg) if msg else None
        if isinstance(d, p.Status) and d.node == node:
            return d
    return None


def test_status_frames_arrive_over_tcp(running_sim):
    _, server = running_sim
    bus = TcpBus(f"127.0.0.1:{server.port}")
    try:
        assert recv_status(bus, 1) is not None
    finally:
        bus.shutdown()


def wait_for_status(bus, node, pred, timeout=1.0):
    """Ruling R3: poll STATUS frames from `node` until one satisfies `pred` (or the deadline);
    returns the last STATUS seen, so a failing assert shows the state actually reached."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        st = recv_status(bus, node, timeout=max(0.0, deadline - time.monotonic()))
        if st is None:
            break
        last = st
        if pred(st):
            break
    return last


def test_disconnecting_master_trips_watchdog(running_sim):
    _, server = running_sim
    bus = TcpBus(f"127.0.0.1:{server.port}")
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.ENABLE))
    st = wait_for_status(bus, 4, lambda s: s.state == p.AxisState.READY)
    assert st is not None and st.state == p.AxisState.READY
    bus.shutdown()                             # master "crashes"
    bus2 = TcpBus(f"127.0.0.1:{server.port}")
    try:
        st = wait_for_status(bus2, 4, lambda s: s.state == p.AxisState.FAULT, timeout=2.0)
        assert st is not None and st.state == p.AxisState.FAULT and p.Fault.WATCHDOG in st.faults
    finally:
        bus2.shutdown()


# --- Fix round 1 additions (controller findings) ---


def test_malformed_channel_raises_clear_value_error():
    for bad_channel in ("no-colon-here", "host:notaport", "host:"):
        with pytest.raises(ValueError, match="host:port"):
            TcpBus(bad_channel)
