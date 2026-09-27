"""`robotarm identify`: open-loop step experiment over CAN for `robotarm stepfit`.

Replaces the firmware's old hard-coded step test: records the same CSV columns
(`time_us,position,velocity,pwm_ticks,current`, see `output.txt`) from live
STATUS/TELEMETRY frames on one axis, so `robotarm.analysis.steptest.load_step_csv`
can read it straight into `robotarm stepfit` -- on the simulator now, on the real
robot later (the encoders are incremental and DUTY mode needs no closed loop, so
no homing is required).

`IdentifyRun.update(now)` is sans-IO, like `ArmClient`/`Teleop`: state machine
enable -> duty 0 for `pre_s` -> `duty` for `on_s` -> duty 0 for `post_s` -> disable.
Samples come from `ArmClient.add_listener`, one per STATUS+TELEMETRY pair of `node`,
timestamped by pair index x 1 ms rather than by the `now` passed to update() --
the axis emits one pair per 1 kHz tick while READY in DUTY mode (controller ruling
R26); host receive time would be wrong on real hardware, where the ArmClient
runner only polls every 5 ms.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import can
import yaml

from robotarm import protocol
from robotarm.analysis.steptest import PWM_TICK_MAX
from robotarm.bus import OPEN_ERRORS, open_bus
from robotarm.config import ArmConfig, AxisConfig, load_arm_config
from robotarm.master.arm_client import ArmClient
from robotarm.master.cli_util import fail as _fail
from robotarm.master.cli_util import install_stop_handlers as _install_stop_handlers
from robotarm.master.cli_util import restore_handlers as _restore_handlers
from robotarm.master.cli_util import runner_failure as _runner_failure
from robotarm.protocol import AxisState

_UPDATE_POLL_S = 0.001       # CLI loop sleep between IdentifyRun.update() calls
_STATUS_TIMEOUT_S = 2.0      # wait for the first STATUS before checking the axis's state
_MIN_SAMPLE_FRACTION = 0.9   # warn if fewer than this fraction of the expected 1 kHz samples arrive


class IdentifyRun:
    """Open-loop step test for one axis. See the module docstring for the state machine.

    `samples`: list of (t_s, position_counts, velocity_counts_s, duty, current_ma),
    one per STATUS+TELEMETRY pair received for `node`.
    """

    _PHASES = ("pre", "on", "post")

    def __init__(self, client: ArmClient, node: int, duty: float = 1.0,
                 pre_s: float = 0.08, on_s: float = 0.08, post_s: float = 0.04) -> None:
        self.client = client
        self.node = node
        self.duty = duty
        self._durations = {"pre": pre_s, "on": on_s, "post": post_s}
        self.samples: list[tuple[float, int, int, float, float]] = []

        self._phase_i = 0  # index into _PHASES; == len(_PHASES) once finished
        self._phase_start: float | None = None
        self._finished = False
        self._pending_status: protocol.Status | None = None
        self._tick = 0  # count of recorded STATUS+TELEMETRY pairs (R26: sample time = tick x 1 ms)

        client.add_listener(self._on_message)
        client.enable([node])

    @property
    def _phase(self) -> str | None:
        return self._PHASES[self._phase_i] if self._phase_i < len(self._PHASES) else None

    def _commanded_duty(self) -> float:
        return self.duty if self._phase == "on" else 0.0

    def update(self, now: float) -> bool:
        """Advance the state machine one driving-loop tick. Returns True once finished
        (the axis has been sent DISABLE); further calls just keep returning True."""
        if self._finished:
            return True
        if self._phase_start is None:
            self._phase_start = now
        phase = self._phase
        if now - self._phase_start >= self._durations[phase]:
            self._phase_i += 1
            self._phase_start = now
            phase = self._phase
            if phase is None:
                self.client.disable([self.node])
                self._finished = True
                return True
        self.client.set_duty(self.node, self._commanded_duty())
        return False

    def _on_message(self, decoded: protocol.DecodedMessage) -> None:
        if isinstance(decoded, protocol.Status) and decoded.node == self.node:
            self._pending_status = decoded
        elif isinstance(decoded, protocol.Telemetry) and decoded.node == self.node:
            status, self._pending_status = self._pending_status, None
            if status is None:
                return  # an orphaned TELEMETRY (its STATUS was dropped): skip rather than mis-pair
            t_s = self._tick * 1e-3
            self._tick += 1
            self.samples.append(
                (t_s, status.position, decoded.velocity, self._commanded_duty(), float(decoded.current_ma))
            )

    def write_csv(self, path: str | Path) -> None:
        """Write `samples` as time_us,position,velocity,pwm_ticks,current -- output.txt's format,
        readable by `robotarm.analysis.steptest.load_step_csv`."""
        with Path(path).open("w", newline="") as f:
            for t_s, pos, vel, duty, current_ma in self.samples:
                time_us = round(t_s * 1e6)
                pwm_ticks = round(duty * PWM_TICK_MAX)
                f.write(f"{time_us},{pos},{vel},{pwm_ticks},{current_ma:.6f}\r\n")


# -- CLI -------------------------------------------------------------------------------


def _duty_refusal(axis: AxisConfig, duty: float) -> str | None:
    """None if `duty` is within the axis's max_duty; otherwise the `error: ...` message."""
    if abs(duty) > axis.max_duty:
        return f"--duty {duty:g} exceeds axis {axis.name!r} max_duty {axis.max_duty:.3f}"
    return None


def _state_refusal(axis: AxisConfig, state: AxisState) -> str | None:
    """None if `state` is DISABLED/READY (identify may proceed); otherwise the `error: ...` message."""
    if state not in (AxisState.DISABLED, AxisState.READY):
        return f"node {axis.node} ({axis.name}) is {state.name}: identify needs DISABLED or READY"
    return None


def _wait_for_state(client: ArmClient, node: int, stop: threading.Event,
                     timeout_s: float = _STATUS_TIMEOUT_S) -> AxisState | None:
    """Block (briefly) for the first STATUS from `node`, then return its axis state.

    None if `stop` was set first (an operator abort, not a timeout -- the caller tells the
    two apart). Raises TimeoutError if neither happens within `timeout_s`."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if stop.is_set():
            return None
        js = client.joints[node]
        if js.last_seen is not None:
            return js.state
        time.sleep(0.01)
    raise TimeoutError(f"no STATUS received from node {node} within {timeout_s:g} s")


def default_out_path(node: int) -> Path:
    """identify_node<N>.csv in the current directory: never the committed bench recording output.txt."""
    return Path(f"identify_node{node}.csv")


def run_identify(bus_url: str, node: int, duty: float = 1.0, out: str | Path | None = None,
                  pre_s: float = 0.08, on_s: float = 0.08, post_s: float = 0.04) -> int:
    """Body of `robotarm identify`. Returns the process exit code:

    0 on a full recording, and also after Ctrl-C/SIGTERM once the run has started (matching
    `robotarm teleop`'s convention) -- whatever was recorded up to that point is still written,
    with a note on stderr that it's partial; 130 for Ctrl-C while the bus is still opening,
    before the run started at all; 2 on any failure (bad config, bus open, an axis not
    DISABLED/READY, --duty beyond the axis's max_duty, or the bus/runner dying mid-run),
    reported as a single `error: ...` line on stderr.

    On every exit path once the run has started, the axis is sent duty 0 then DISABLE
    (best effort -- if the bus itself just died there is nothing left to send to).

    `out` defaults to default_out_path(node)."""
    if out is None:
        out = default_out_path(node)
    try:
        arm_cfg = load_arm_config()
    except (OSError, ValueError, KeyError, TypeError, AttributeError, yaml.YAMLError) as exc:
        return _fail(f"config: {exc}")

    try:
        axis = arm_cfg.axis(node)
    except KeyError as exc:
        return _fail(str(exc))
    duty_error = _duty_refusal(axis, duty)
    if duty_error is not None:
        return _fail(duty_error)

    try:
        bus = open_bus(bus_url)
    except OPEN_ERRORS as exc:
        return _fail(f"{bus_url}: {exc}")
    except KeyboardInterrupt:  # Ctrl-C during a slow open (our handler isn't installed yet)
        return 130

    try:
        stop = threading.Event()
        old_handlers = _install_stop_handlers(stop)
        try:
            return _identify_loop(bus, arm_cfg, axis, node, duty, out, pre_s, on_s, post_s, stop)
        finally:
            _restore_handlers(old_handlers)
    finally:
        bus.shutdown()


def _identify_loop(bus: can.BusABC, arm_cfg: ArmConfig, axis: AxisConfig, node: int, duty: float, out: str | Path,
                    pre_s: float, on_s: float, post_s: float, stop: threading.Event) -> int:
    client = ArmClient(bus, arm_cfg)
    client.start()
    run: IdentifyRun | None = None
    aborted = False
    failure: str | None = None
    try:
        try:
            state = _wait_for_state(client, node, stop)
        except TimeoutError as exc:
            failure = str(exc)
        else:
            if state is None:
                aborted = True  # stop was set while waiting for the first STATUS
            else:
                state_error = _state_refusal(axis, state)
                if state_error is not None:
                    failure = state_error
                else:
                    run = IdentifyRun(client, node, duty, pre_s=pre_s, on_s=on_s, post_s=post_s)
                    try:
                        while not run.update(time.monotonic()):
                            if stop.is_set():
                                aborted = True
                                break
                            if not client.alive():
                                failure = _runner_failure(client.error)
                                break
                            time.sleep(_UPDATE_POLL_S)
                    except (can.CanError, OSError) as exc:
                        failure = f"bus lost: {exc}"
    finally:
        if aborted or failure is not None:
            try:  # best effort: brake, then DISABLE, before ArmClient.close()'s own DISABLE broadcast
                client.set_duty(node, 0.0)
                client.disable([node])
            except (can.CanError, OSError):
                pass  # the bus is already gone; nothing left to send to
        client.close()

    n = len(run.samples) if run is not None else 0
    if n:
        run.write_csv(out)
        if aborted or failure is not None:
            print(f"partial recording: wrote {out} ({n} samples)", file=sys.stderr)
        else:
            expected = round((pre_s + on_s + post_s) * 1000)
            if n < _MIN_SAMPLE_FRACTION * expected:
                print(f"warning: recorded {n} samples, expected ~{expected} at 1 kHz (dropped frames?)",
                      file=sys.stderr)
            print(f"wrote {out} ({n} samples, node {node} {axis.name!r}, duty {duty:g})")
    elif aborted or failure is not None:
        print("no samples recorded", file=sys.stderr)

    if failure is not None:
        return _fail(failure)
    return 0


def _run(args: argparse.Namespace) -> int:
    return run_identify(args.bus, args.node, duty=args.duty, out=args.out,
                         pre_s=args.pre, on_s=args.on, post_s=args.post)


def register(subparsers: argparse._SubParsersAction) -> None:
    parser = subparsers.add_parser(
        "identify", help="open-loop step experiment over CAN, for `robotarm stepfit` (bench motor model)"
    )
    parser.add_argument("--bus", required=True,
                        help="bus URL: tcp://host:port, sim, slcan:/dev/tty..., gs_usb:0, socketcan:can0, ...")
    parser.add_argument("--node", type=int, required=True, help="axis node id (see config/arm.yaml)")
    parser.add_argument("--duty", type=float, default=1.0, help="duty applied during the step (-1..1, default 1.0)")
    parser.add_argument("--out", type=Path, default=None,
                        help="CSV path (default: identify_node<N>.csv in the current directory)")
    parser.add_argument("--pre", type=float, default=0.08, help="seconds at duty 0 before the step (default 0.08)")
    parser.add_argument("--on", type=float, default=0.08, help="seconds at --duty (default 0.08)")
    parser.add_argument("--post", type=float, default=0.04, help="seconds at duty 0 after the step (default 0.04)")
    parser.set_defaults(func=_run)
