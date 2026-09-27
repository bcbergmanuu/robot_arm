#!/usr/bin/env bash
# Run idf.py inside the official ESP-IDF v6.1 container. Usage: scripts/idf.sh build
# Interactive subcommands (e.g. `menuconfig`) need a tty allocated (`docker run -it`); non-interactive
# uses (e.g. `build` from a script or CI, where stdin isn't a terminal) must not pass -it, or `docker
# run` fails outright with no tty available. Detect it instead of hard-coding either way.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TTY_FLAG=""
if [ -t 0 ]; then
    TTY_FLAG="-it"
fi
# $TTY_FLAG is intentionally unquoted below: it is always either empty or the literal `-it` (no
# spaces/globs to mis-split), and this avoids an `unbound variable`/empty-array pitfall under
# `set -u` with macOS's default bash 3.2.
exec docker run --rm $TTY_FLAG -v "$ROOT":/project -w /project -e HOME=/tmp \
    espressif/idf:v6.1 idf.py "$@"
