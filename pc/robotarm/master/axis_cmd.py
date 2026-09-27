"""`robotarm axis {home,enable,disable,clear} --node N --bus URL`: one command to one axis.

For bring-up (docs/bringup.md): home the axes one at a time, check that an axis enables and
holds, clear a fault -- without starting teleop. Each action sends its COMMAND, keeps the
axis's watchdog fed (ArmClient runner, keepalive-gated like teleop, R23) and waits, bounded,
for the resulting state:

  home     DISABLED/READY, no faults -> HOMING -> READY + homed  (timeout: home.timeout_s + 2 s)
  enable   DISABLED, no faults -> READY
  disable  -> DISABLED (FAULT also counts: the motor is unpowered either way)
  clear    FAULT -> DISABLED (no-op if the axis is not faulted)

`--hold S` keeps the axis in that state (heartbeats flowing) for S more seconds, Ctrl-C ends
early. On exit the client is closed, which broadcasts DISABLE -- an enabled axis would
otherwise watchdog-fault once the heartbeats stop. The homed flag survives DISABLE (until the
board resets), so axes homed one at a time stay homed for a later `robotarm teleop`.

Exit codes: 0 when the target state was reached (also after Ctrl-C during --hold); 130 for
Ctrl-C before that; 2 on any failure (config, bus open, no STATUS, wrong start state, the axis
faulting, timeout, bus lost), reported as a single `error: ...` line on stderr.
"""

from __future__ import annotations

import argparse
import threading
import time
from collections.abc import Callable

import can
import yaml

from robotarm.bus import OPEN_ERRORS, open_bus
from robotarm.config import ArmConfig, AxisConfig, load_arm_config
from robotarm.master.arm_client import ArmClient, JointState
from robotarm.master.cli_util import fail as _fail
from robotarm.master.cli_util import install_stop_handlers as _install_stop_handlers
from robotarm.master.cli_util import restore_handlers as _restore_handlers
from robotarm.master.cli_util import runner_failure as _runner_failure
from robotarm.protocol import AxisState, Fault

ACTIONS = ("home", "enable", "disable", "clear")

_POLL_S = 0.01
_FIRST_STATUS_TIMEOUT_S = 2.0
_STATE_TIMEOUT_S = 1.0          # enable/disable/clear: the axis reacts within one status period
_HOME_TIMEOUT_MARGIN_S = 2.0    # on top of the axis's own home.timeout_s (it faults HOMING itself)
_KEEPALIVE_TIMEOUT_S = 0.1


class _Abort(Exception):
    """stop was set (Ctrl-C / SIGTERM) while waiting."""


class _Failed(Exception):
    """A one-line reason for `error: ...`."""


def _fault_names(faults: Fault) -> str:
    return "|".join(f.name for f in Fault if f in faults and f.name) or "NONE"


def _wait(client: ArmClient, node: int, stop: threading.Event, timeout_s: float,
          done: Callable[[JointState], bool], what: str, fail_on_fault: bool = True) -> JointState:
    deadline = time.monotonic() + timeout_s
    while True:
        now = time.monotonic()
        client.keepalive(now)
        if stop.is_set():
            raise _Abort
        if not client.alive():
            raise _Failed(_runner_failure(client.error))
        js = client.joint(node)
        if js.last_seen is not None:
            if done(js):
                return js
            if fail_on_fault and js.state == AxisState.FAULT:
                raise _Failed(f"node {node} ({js.name}) faulted: {_fault_names(js.faults)}")
        if now >= deadline:
            state = js.state.name if js.last_seen is not None else "silent"
            raise _Failed(f"node {node} ({js.name}): timed out after {timeout_s:g} s waiting for {what} "
                          f"(state {state})")
        time.sleep(_POLL_S)


def _do_action(client: ArmClient, axis: AxisConfig, action: str, stop: threading.Event) -> str:
    """Send the command and wait for its result. Returns the success message."""
    node = axis.node
    js = _wait(client, node, stop, _FIRST_STATUS_TIMEOUT_S, lambda j: True, "the first STATUS")

    if action == "home":
        if js.state not in (AxisState.DISABLED, AxisState.READY) or js.faults:
            raise _Failed(f"node {node} ({axis.name}) is {js.state.name} (faults {_fault_names(js.faults)}): "
                          "home needs DISABLED or READY without faults -- `robotarm axis clear` first")
        client.home([node])
        # HOME clears the homed flag at once; wait for that first so a stale READY+homed STATUS
        # from before the command is not mistaken for the result.
        _wait(client, node, stop, _STATE_TIMEOUT_S, lambda j: not j.homed or j.state == AxisState.HOMING,
              "HOMING")
        _wait(client, node, stop, axis.home.timeout_s + _HOME_TIMEOUT_MARGIN_S,
              lambda j: j.state == AxisState.READY and j.homed, "READY + homed")
        return f"node {node} ({axis.name}) homed"

    if action == "enable":
        if js.state == AxisState.READY:
            return f"node {node} ({axis.name}) already READY"
        if js.state != AxisState.DISABLED or js.faults:
            raise _Failed(f"node {node} ({axis.name}) is {js.state.name} (faults {_fault_names(js.faults)}): "
                          "enable needs DISABLED without faults -- `robotarm axis clear` first")
        client.enable([node])
        _wait(client, node, stop, _STATE_TIMEOUT_S, lambda j: j.state == AxisState.READY, "READY")
        return f"node {node} ({axis.name}) READY"

    if action == "disable":
        client.disable([node])
        js = _wait(client, node, stop, _STATE_TIMEOUT_S,
                   lambda j: j.state in (AxisState.DISABLED, AxisState.FAULT), "DISABLED", fail_on_fault=False)
        if js.state == AxisState.FAULT:
            return f"node {node} ({axis.name}) is FAULT ({_fault_names(js.faults)}), motor unpowered"
        return f"node {node} ({axis.name}) DISABLED"

    # clear
    if js.state != AxisState.FAULT:
        return f"node {node} ({axis.name}) not faulted ({js.state.name})"
    client.clear_faults([node])
    _wait(client, node, stop, _STATE_TIMEOUT_S, lambda j: j.state == AxisState.DISABLED, "DISABLED",
          fail_on_fault=False)
    return f"node {node} ({axis.name}) faults cleared, DISABLED"


def _hold(client: ArmClient, axis: AxisConfig, hold_s: float, stop: threading.Event) -> None:
    """Keep the heartbeats flowing for hold_s (Ctrl-C ends early); the axis must not fault meanwhile."""
    end = time.monotonic() + hold_s
    while time.monotonic() < end and not stop.is_set():
        client.keepalive(time.monotonic())
        if not client.alive():
            raise _Failed(_runner_failure(client.error))
        js = client.joint(axis.node)
        if js.state == AxisState.FAULT:
            raise _Failed(f"node {axis.node} ({axis.name}) faulted during --hold: {_fault_names(js.faults)}")
        time.sleep(_POLL_S)


def run_axis_command(bus_url: str, action: str, node: int, hold_s: float = 0.0) -> int:
    """Body of `robotarm axis`. See the module docstring for exit codes."""
    try:
        arm_cfg = load_arm_config()
    except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError) as exc:
        return _fail(f"config: {exc}")
    try:
        axis = arm_cfg.axis(node)
    except KeyError as exc:
        return _fail(str(exc.args[0]))
    if hold_s < 0:
        return _fail(f"--hold must be >= 0, got {hold_s:g}")

    try:
        bus = open_bus(bus_url)
    except OPEN_ERRORS as exc:
        return _fail(f"{bus_url}: {exc}")
    except KeyboardInterrupt:
        return 130
    try:
        stop = threading.Event()
        old_handlers = _install_stop_handlers(stop)
        try:
            return _axis_loop(bus, arm_cfg, axis, action, hold_s, stop)
        finally:
            _restore_handlers(old_handlers)
    finally:
        bus.shutdown()


def _axis_loop(bus: can.BusABC, arm_cfg: ArmConfig, axis: AxisConfig, action: str, hold_s: float,
               stop: threading.Event) -> int:
    client = ArmClient(bus, arm_cfg, keepalive_timeout=_KEEPALIVE_TIMEOUT_S)
    client.keepalive(time.monotonic())
    client.start()
    try:
        try:
            message = _do_action(client, axis, action, stop)
            print(message, flush=True)
            if hold_s > 0:
                _hold(client, axis, hold_s, stop)
        except _Abort:
            return 130
        except _Failed as exc:
            return _fail(str(exc))
        except (can.CanError, OSError) as exc:
            return _fail(f"bus lost: {exc}")
        except Exception as exc:  # noqa: BLE001 -- a bug: still one line, still cleaned up
            return _fail(f"internal error: {exc!r}")
    finally:
        client.close()  # broadcast DISABLE (best effort)
    return 0


def _run(args: argparse.Namespace) -> int:
    return run_axis_command(args.bus, args.action, args.node, hold_s=args.hold)


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "axis", help="send one command (home/enable/disable/clear) to one axis and wait for the result"
    )
    parser.add_argument("action", choices=ACTIONS)
    parser.add_argument("--node", type=int, required=True, help="axis node id (see config/arm.yaml)")
    parser.add_argument("--bus", required=True,
                        help="bus URL: tcp://host:port, sim, slcan:/dev/tty..., gs_usb:0, socketcan:can0, ...")
    parser.add_argument("--hold", type=float, default=0.0,
                        help="keep the axis in the resulting state for this many seconds (heartbeats "
                             "flowing) before exiting, which disables it (default 0)")
    parser.set_defaults(func=_run)
