"""Shared plumbing for `robotarm teleop`/`robotarm identify`-style CLI loops: a single
`error: ...`-line-and-exit-2 convention, turning a dead ArmClient runner's `.error` into a
one-line message, and SIGINT/SIGTERM handling that lets the main loop stop and run its own
cleanup (DISABLE, closing the bus, writing a partial recording, ...) instead of being killed
outright or raising a KeyboardInterrupt/traceback partway through it.
"""

from __future__ import annotations

import signal
import sys
import threading
from typing import Any

import can


def fail(message: str) -> int:
    """Print a single `error: ...` line to stderr and return exit code 2."""
    print(f"error: {message}", file=sys.stderr)
    return 2


def runner_failure(error: BaseException | None) -> str:
    """A one-line message for `ArmClient.error` once `ArmClient.alive()` is False."""
    if error is None:
        return "bus lost: runner stopped"
    if isinstance(error, (can.CanError, OSError)):
        return f"bus lost: {error}"
    return f"internal error: {error!r}"


def install_stop_handlers(stop: threading.Event) -> dict[int, Any]:
    """SIGINT/SIGTERM set `stop` instead of the default (raise/kill), so the caller's loop
    can exit through its own cleanup path. No-op off the main thread (signal handlers can only
    be installed there); returns the previous handlers, for restore_handlers()."""
    if threading.current_thread() is not threading.main_thread():
        return {}

    def _on_signal(_signum, _frame) -> None:
        stop.set()

    return {sig: signal.signal(sig, _on_signal) for sig in (signal.SIGINT, signal.SIGTERM)}


def restore_handlers(old: dict[int, Any]) -> None:
    for sig, handler in old.items():
        signal.signal(sig, handler)
