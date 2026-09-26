#!/usr/bin/env bash
# Run the real-time simulator (`make sim`). On macOS the MuJoCo viewer must run
# under mjpython (its passive viewer needs the Cocoa main thread), and mjpython
# execve's into a native binary that dlopens the interpreter's libpython. uv's
# standalone CPython keeps that library in sysconfig's LIBDIR rather than next
# to the python executable, so without help mjpython fails with something like
# "Library not loaded: @rpath/libpython3.12.dylib" -- exporting
# DYLD_LIBRARY_PATH to LIBDIR fixes it. Linux has no such requirement (no
# mjpython there), so it just runs plain `python -m robotarm sim`.
#
# If the viewer window itself crashes/segfaults on your machine, rerun with
# --no-viewer (see docs/simulator.md).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

if [[ "$(uname -s)" == "Darwin" ]]; then
    LIBDIR="$(uv run python -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR'))")"
    export DYLD_LIBRARY_PATH="${LIBDIR}${DYLD_LIBRARY_PATH:+:${DYLD_LIBRARY_PATH}}"
    exec uv run mjpython -m robotarm sim "$@"
else
    exec uv run python -m robotarm sim "$@"
fi
