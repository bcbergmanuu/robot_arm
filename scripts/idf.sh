#!/usr/bin/env bash
# Run idf.py inside the official ESP-IDF v6.1 container. Usage: scripts/idf.sh build
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec docker run --rm -v "$ROOT":/project -w /project -e HOME=/tmp \
    espressif/idf:v6.1 idf.py "$@"
