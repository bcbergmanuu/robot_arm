# Robot Arm Simulator & PlayStation Teleop — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Develop and test the complete control stack of the Katana refurbish arm on a laptop — axis firmware logic, a physics simulator of the 6-axis arm, and a PlayStation-controller teleop master — such that the *same* firmware core and the *same* master program run unchanged on the real robot (ESP32-S3 axis boards on CAN + PC/Pi master with a USB-CAN adapter).

**Architecture:** "Functional core, imperative shell." All axis control logic lives in a portable C library (`components/axis_core`) that has no hardware or ESP-IDF dependencies: it consumes encoder/current samples and CAN frames and produces a PWM duty and CAN frames. The ESP32 firmware (`main/`) and the simulator (`host/sim` + Python/MuJoCo) are two thin shells around that core. The master (Python) talks CAN through any `python-can` bus: an in-process/TCP bus into the simulator, or a USB-CAN adapter on the real robot.

**Tech Stack:** C11 + CMake (host) / ESP-IDF v6.1 (target, built in docker `espressif/idf:v6.1`); Python 3.12 via `uv`; MuJoCo 3.x; python-can 4.x; pygame 2.6 (SDL2 game controller API); numpy, scipy, matplotlib, PyYAML, pytest.

**Spec:** The "Design" section of this document (derived from the design discussion of 2026-09-26; no separate spec file). Task 1 copies it into `docs/architecture.md`.

---

## Design

### System overview

```
                   ┌───────────── identical code on sim and robot ─────────────┐
 PS4/PS5 pad ──► MASTER (python, PC or Raspberry Pi)        AXIS CORE (C, ×6 nodes)
 (USB/BT, SDL)   robotarm.master: Teleop → ArmClient  CAN  axis_core: state machine,
                 heartbeats, deadman, e-stop, IK      ◄──► trajectory, PID cascade,
                                                          limits, watchdog, homing
                   └───────────────────────────────────────────────────────────┘
      SIM shell:  TcpBus/SimBus  ──  SimWorld (MuJoCo arm + libsimaxis: 6× axis_core + DC motor model)
      REAL shell: python-can slcan/gs_usb ── CAN 1 Mbit ── ESP32-S3 board: main/ (MCPWM, PCNT, ADC, TWAI)
```

Why the master is on a PC/Pi and not on an ESP32-S3: the S3 only has Bluetooth LE; DualShock 4 / DualSense controllers need Bluetooth Classic. A PC/Pi reads the controller via SDL and reaches the axes via a USB-CAN adapter (e.g. CANable, `slcan`/`gs_usb`).

### Decisions (made on the user's behalf overnight — revisit in the morning)

1. **Control loop at 1 kHz** (`AXIS_TICK_HZ`), cascaded **position → velocity → duty** with velocity feed-forward. The previous torque (current) PID is dropped as a loop: the TB9051FTG OCM output is magnitude-only and valid only while the bridge drives, so it cannot close a signed current loop. Current is used for overcurrent protection, homing stall detection and telemetry.
2. **Axis units are encoder counts** (position, counts; velocity, counts/s; duty ∈ [−1, 1]; current, mA). The master works in SI (rad, rad/s) and converts with per-axis `counts_per_rad` from the shared config.
3. **One config file `config/arm.yaml`** is the single source of truth. Python reads it directly; a generator writes `components/axis_core/src/config_table.c` (committed, so the ESP-IDF build needs no Python YAML).
4. **Homing against the mechanical end stop** (incremental encoders): drive slowly toward the stop, detect stall (current above threshold or speed collapse), define the stop as `home_position`, then back off into the soft limits. The original Katana homes the same way.
5. **Safety lives in the axis**: command watchdog (200 ms), soft limits, following-error fault, overcurrent fault, broadcast E-STOP. The master adds deadman (L1) and gamepad-loss handling.
6. **Simulator:** MuJoCo for rigid-body arm dynamics (gravity, joint friction, mechanical stops as joint limits, reflected rotor inertia as joint `armature`); the DC motor electrical model and the six axis cores run in C (`libsimaxis`) called once per 1 ms step via ctypes.
7. **Sim ↔ master transport:** in-process `SimBus` (lockstep, deterministic — used by tests) and a TCP bus (`TcpBus`, 13-byte frames) so the simulator (with viewer) and the teleop run as separate processes. On the robot the master uses `slcan`/`gs_usb`/`socketcan` through `python-can`.
8. **Firmware bench identification** (open-loop duty step, currently hard-coded in `motor_pid.c`) becomes a protocol feature: `SETPOINT kind=DUTY`, telemetry at 1 kHz in duty mode, recorded by `robotarm identify` on sim or robot.

### CAN protocol (1 Mbit/s, 11-bit IDs, little-endian)

`id = (type << 3) | node`, node 1–6 = axes, node 0 = broadcast. Lower id = higher priority.

| type | name | dir | len | payload |
|---|---|---|---|---|
| 0x00 | ESTOP | master→all (node 0) | 0 | — |
| 0x01 | HEARTBEAT | master→all (node 0) | 1 | u8 seq |
| 0x02 | COMMAND | master→node/all | 1 | u8 cmd: 0 DISABLE, 1 ENABLE, 2 HOME, 3 CLEAR_FAULT |
| 0x03 | SETPOINT | master→node | 5 | u8 kind (0 POSITION counts, 1 VELOCITY counts/s, 2 DUTY ×10000), i32 value |
| 0x10 | STATUS | node→master | 7 | i32 position counts, u8 state, u8 faults, u8 flags (bit0 homed) |
| 0x11 | TELEMETRY | node→master | 6 | i32 velocity counts/s, i16 current mA |

States: 0 DISABLED, 1 HOMING, 2 READY, 3 FAULT. Faults bitmask: 1 WATCHDOG, 2 OVERCURRENT, 4 FOLLOWING, 8 ESTOP, 16 HOMING. STATUS+TELEMETRY at 100 Hz (every tick while in DUTY mode). Any HEARTBEAT/COMMAND/SETPOINT addressed to the node or broadcast feeds the watchdog.

### Repository layout after this plan

```
CMakeLists.txt                  ESP-IDF project (unchanged role; VS Code IDF setup keeps working)
main/                           ESP32 shell: board.h, hal_*.c, axis_task.c, main.c, Kconfig.projbuild
components/axis_core/           portable C core (IDF component AND host static lib)
  include/axis/{pid,protocol,axis_config,config_table,axis}.h
  src/{pid,protocol,axis,config_table}.c        (config_table.c is generated)
host/                           host CMake project
  CMakeLists.txt
  tests/{tinytest.h,test_config.h,test_*.c}
  sim/{motor_model,bench,simaxis}.{c,h}       → build/host/libsimaxis.{dylib,so}
config/arm.yaml, config/teleop.yaml, config/bench_identified.yaml
pc/robotarm/                    python package (uv project at repo root: pyproject.toml)
  config.py protocol.py bus.py cli.py __main__.py
  transport/tcp_bus.py
  sim/{native,model,world,harness,server}.py
  master/{arm_client,gamepad,teleop,kinematics,identify}.py
  analysis/steptest.py
  tools/gen_config.py
pc/tests/                       pytest suite
scripts/idf.sh                  docker wrapper for idf.py
Makefile                        host / test / firmware / sim / teleop
docs/architecture.md docs/protocol.md docs/simulator.md docs/teleop.md docs/bringup.md
```

## Global Constraints

- Branch `feature/sim-teleop`; commit after every task (conventional message + trailer `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`).
- `components/axis_core` is C11, includes no ESP-IDF/FreeRTOS headers, uses no heap allocation, uses `float` only (no `double`) — the ESP32-S3 FPU is single precision.
- `AXIS_TICK_HZ` = 1000. CAN: 1 Mbit/s, 11-bit ids, layout exactly as the protocol table above.
- Python: `requires-python = ">=3.12,<3.14"`, run everything via `uv run` from the repo root. Package root `pc/`.
- ESP-IDF builds via `scripts/idf.sh` (docker `espressif/idf:v6.1`, repo mounted at `/project`; the repo is under `$HOME` so colima can mount it).
- Root `CMakeLists.txt` stays the ESP-IDF project file. The host build lives in `host/` and builds to `build/host/` (git-ignored).
- Values not known from the hardware (link lengths, masses, wrist-bend motor, limits, friction) are marked `# (assumed)` in `config/arm.yaml` and listed in `docs/bringup.md` as "measure on robot".
- `make test` (ctest + pytest) must pass at the end of every task.

## Review Focus

1. **Master dies / link drops mid-motion** (teleop crashes, TCP socket closes, USB-CAN unplugged) → every enabled axis drops to duty 0 and reports FAULT_WATCHDOG within `watchdog_ms` (200 ms). Tests: Task 5 (core), Task 13 (TCP disconnect).
2. **Gamepad disconnects or goes out of BT range while a stick is deflected** → teleop immediately commands zero velocity on all axes and does not move again until the deadman is released and pressed again. Test: Task 16.
3. **Commands beyond reach** (stick held into a soft limit, position setpoint outside limits, IK target out of workspace, position command on an unhomed axis) → motion stops at/inside the limit without fault; out-of-reach IK keeps the last reachable target; unhomed position setpoints are ignored. Tests: Task 6, Task 17.
4. **Stick drift / noise around center** → radial deadzone, no creeping joints. Test: Task 15.
5. **Starting teleop with no simulator running / wrong adapter path** → single clear error line, exit code 2, no traceback. Tests: Task 13, Task 16.

---

# Phase 0 — Scaffolding

### Task 1: Project scaffolding (host C build, Python project, Makefile, docs)

**Files:**
- Create: `host/CMakeLists.txt`, `host/tests/tinytest.h`, `host/tests/test_smoke.c`
- Create: `components/axis_core/CMakeLists.txt`, `components/axis_core/include/axis/version.h`, `components/axis_core/src/version.c`
- Create: `pyproject.toml`, `pc/robotarm/__init__.py`, `pc/robotarm/__main__.py`, `pc/robotarm/cli.py`, `pc/tests/test_smoke.py`, `pc/tests/conftest.py`
- Create: `Makefile`, `scripts/idf.sh`, `docs/architecture.md`
- Modify: `.gitignore` (add `build/`, `.venv/` already present; add `docs/img/*.tmp.png`)

**Interfaces:**
- Produces: `make host` → `build/host/` with ctest tests; `make test`; `uv run robotarm --help`; `scripts/idf.sh <idf.py args>`; conftest fixture `host_build` (session, autouse) that runs `make host` once so pytest can load `libsimaxis`.

- [ ] **Step 1: Create the axis_core component skeleton that builds both under ESP-IDF and on the host**

`components/axis_core/CMakeLists.txt`:
```cmake
set(AXIS_CORE_SRCS
    src/version.c
)

if(ESP_PLATFORM)
    idf_component_register(SRCS ${AXIS_CORE_SRCS} INCLUDE_DIRS include)
else()
    add_library(axis_core STATIC ${AXIS_CORE_SRCS})
    target_include_directories(axis_core PUBLIC include)
    target_compile_features(axis_core PUBLIC c_std_11)
    target_compile_options(axis_core PRIVATE -Wall -Wextra -Werror -Wdouble-promotion)
    target_link_libraries(axis_core PUBLIC m)
endif()
```
Later tasks append their `.c` files to `AXIS_CORE_SRCS`.

`include/axis/version.h`:
```c
#pragma once
#define AXIS_CORE_VERSION "0.1.0"
const char *axis_core_version(void);
```
`src/version.c`:
```c
#include "axis/version.h"
const char *axis_core_version(void) { return AXIS_CORE_VERSION; }
```

- [ ] **Step 2: Host CMake project + tiny test framework**

`host/tests/tinytest.h`:
```c
#pragma once
#include <math.h>
#include <stdio.h>

static int tt_fail = 0, tt_count = 0;

#define TT_CHECK(cond) do { tt_count++; if (!(cond)) { tt_fail++; \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define TT_NEAR(a, b, eps) do { double _a = (double)(a), _b = (double)(b); tt_count++; \
    if (fabs(_a - _b) > (double)(eps)) { tt_fail++; \
    printf("  FAIL %s:%d: %s = %g, expected %g (eps %g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(eps)); } } while (0)
#define TT_RUN(fn) do { printf("%s\n", #fn); fn(); } while (0)
#define TT_DONE() (printf("%d checks, %d failed\n", tt_count, tt_fail), tt_fail ? 1 : 0)
```

`host/tests/test_smoke.c`:
```c
#include <string.h>
#include "axis/version.h"
#include "tinytest.h"

static void test_version(void) { TT_CHECK(strcmp(axis_core_version(), "0.1.0") == 0); }

int main(void) { TT_RUN(test_version); return TT_DONE(); }
```

`host/CMakeLists.txt`:
```cmake
cmake_minimum_required(VERSION 3.22)
project(robot_arm_host C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../components/axis_core axis_core)

function(axis_test name)
    add_executable(${name} tests/${name}.c)
    target_link_libraries(${name} PRIVATE axis_core)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Werror)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

axis_test(test_smoke)
```

- [ ] **Step 3: Makefile and IDF docker wrapper**

`Makefile`:
```make
.PHONY: host test ctest pytest firmware sim teleop gen-config

host:
	cmake -S host -B build/host -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
	cmake --build build/host -j

ctest: host
	ctest --test-dir build/host --output-on-failure

pytest: host
	uv run pytest -q pc/tests

test: ctest pytest

gen-config:
	uv run robotarm gen-config

firmware:
	scripts/idf.sh build

sim:
	uv run mjpython -m robotarm sim

teleop:
	uv run robotarm teleop --bus tcp://127.0.0.1:29536
```

`scripts/idf.sh` (chmod +x):
```bash
#!/usr/bin/env bash
# Run idf.py inside the official ESP-IDF v6.1 container. Usage: scripts/idf.sh build
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec docker run --rm -v "$ROOT":/project -w /project -e HOME=/tmp \
    espressif/idf:v6.1 idf.py "$@"
```

- [ ] **Step 4: Python project**

`pyproject.toml`:
```toml
[project]
name = "robotarm"
version = "0.1.0"
description = "Simulator and PlayStation teleop master for the Katana refurbish robot arm"
requires-python = ">=3.12,<3.14"
dependencies = [
    "mujoco>=3.2",
    "numpy>=2.0",
    "scipy>=1.13",
    "matplotlib>=3.9",
    "python-can>=4.4",
    "pygame>=2.6",
    "pyyaml>=6.0",
]

[project.scripts]
robotarm = "robotarm.cli:main"

[dependency-groups]
dev = ["pytest>=8"]

[build-system]
requires = ["uv_build>=0.8,<0.12"]
build-backend = "uv_build"

[tool.uv.build-backend]
module-root = "pc"

[tool.pytest.ini_options]
testpaths = ["pc/tests"]
```
Run `uv python pin 3.12` (creates `.python-version`), then `uv sync`. If the `uv_build` version bound is rejected, use whatever bound `uv init --build-backend uv` generates.

`pc/robotarm/cli.py`:
```python
"""Command line entry point: `uv run robotarm <command>`."""
import argparse
import sys


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="robotarm", description=__doc__)
    parser.add_subparsers(dest="command", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
```
Each later task registers its subcommand in `build_parser()` via a `register(subparsers)` function in its module (e.g. `robotarm.sim.server.register`). The sub-parser sets `func`.

`pc/robotarm/__main__.py`:
```python
import sys
from robotarm.cli import main

sys.exit(main())
```

`pc/tests/conftest.py`:
```python
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="session", autouse=True)
def host_build():
    """Build the C host targets (libsimaxis, tests) once per test session."""
    subprocess.run(["make", "-s", "host"], cwd=REPO_ROOT, check=True)
    return REPO_ROOT / "build" / "host"
```

`pc/tests/test_smoke.py`:
```python
import subprocess
import sys


def test_cli_help_runs():
    out = subprocess.run([sys.executable, "-m", "robotarm", "--help"], capture_output=True, text=True)
    assert out.returncode == 0
    assert "robotarm" in out.stdout
```

- [ ] **Step 5: `docs/architecture.md`** — copy the "Design" section of this plan (overview diagram, decisions, protocol summary pointing to `docs/protocol.md`, repo layout) and add a "Development workflow" section listing the Makefile targets.

- [ ] **Step 6: Verify**

Run: `make test`
Expected: `test_smoke` passes in ctest; pytest `1 passed`.
Run: `scripts/idf.sh build 2>&1 | tail -3`
Expected: `Project build complete` (the empty axis_core component is picked up by IDF automatically from `components/`).

- [ ] **Step 7: Commit**

```bash
git add -A host components pc pyproject.toml uv.lock .python-version Makefile scripts docs/architecture.md .gitignore
git commit -m "build: scaffold host C build, python project and docker IDF wrapper"
```

---

# Phase 1 — Portable axis core

### Task 2: PID controller

**Files:**
- Create: `components/axis_core/include/axis/pid.h`, `components/axis_core/src/pid.c`
- Test: `host/tests/test_pid.c`
- Modify: `components/axis_core/CMakeLists.txt` (add `src/pid.c`), `host/CMakeLists.txt` (`axis_test(test_pid)`)

**Interfaces:**
- Produces:
```c
typedef struct { float kp, ki, kd, out_min, out_max, i_min, i_max; } pid_gains_t;
typedef struct { pid_gains_t g; float integ; float prev_err; int has_prev; } pidc_t;
void  pidc_init(pidc_t *p, const pid_gains_t *g);
void  pidc_reset(pidc_t *p);
float pidc_update(pidc_t *p, float err, float dt);
```
(`pidc_t`, not `pid_t`, to avoid the POSIX `pid_t` clash.) Gains are continuous-time: `ki` in 1/s, `kd` in s; `i_min/i_max` clamp the integral term in output units.

- [ ] **Step 1: Write the failing tests** — `host/tests/test_pid.c`:
```c
#include "axis/pid.h"
#include "tinytest.h"

static pid_gains_t gains(float kp, float ki, float kd) {
    pid_gains_t g = {kp, ki, kd, -1.0f, 1.0f, -0.5f, 0.5f};
    return g;
}

static void test_proportional_only(void) {
    pidc_t p; pid_gains_t g = gains(0.1f, 0, 0); pidc_init(&p, &g);
    TT_NEAR(pidc_update(&p, 2.0f, 0.001f), 0.2f, 1e-6);
    TT_NEAR(pidc_update(&p, -3.0f, 0.001f), -0.3f, 1e-6);
}

static void test_integral_accumulates_with_dt(void) {
    pidc_t p; pid_gains_t g = gains(0, 10.0f, 0); pidc_init(&p, &g);
    float out = 0;
    for (int i = 0; i < 10; i++) out = pidc_update(&p, 1.0f, 0.001f);
    TT_NEAR(out, 0.1f, 1e-5);   /* 10 * 1.0 * 0.001 * 10 */
}

static void test_integral_clamped(void) {
    pidc_t p; pid_gains_t g = gains(0, 100.0f, 0); pidc_init(&p, &g);
    float out = 0;
    for (int i = 0; i < 1000; i++) out = pidc_update(&p, 1.0f, 0.001f);
    TT_NEAR(out, 0.5f, 1e-6);   /* i_max */
}

static void test_output_clamped(void) {
    pidc_t p; pid_gains_t g = gains(10.0f, 0, 0); pidc_init(&p, &g);
    TT_NEAR(pidc_update(&p, 5.0f, 0.001f), 1.0f, 1e-6);
    TT_NEAR(pidc_update(&p, -5.0f, 0.001f), -1.0f, 1e-6);
}

static void test_anti_windup_recovers_quickly(void) {
    pid_gains_t g = {2.0f, 50.0f, 0, -1.0f, 1.0f, -10.0f, 10.0f};
    pidc_t p; pidc_init(&p, &g);
    for (int i = 0; i < 2000; i++) pidc_update(&p, 5.0f, 0.001f);   /* saturated high */
    /* error flips sign: output must leave saturation immediately, not after unwinding 10 units */
    TT_CHECK(pidc_update(&p, -0.6f, 0.001f) < 0.0f);
}

static void test_derivative_skips_first_sample(void) {
    pidc_t p; pid_gains_t g = gains(0, 0, 0.01f); pidc_init(&p, &g);
    TT_NEAR(pidc_update(&p, 1.0f, 0.001f), 0.0f, 1e-6);
    TT_NEAR(pidc_update(&p, 1.02f, 0.001f), 0.2f, 1e-4);   /* 0.01 * 0.02 / 0.001 */
}

static void test_reset_clears_state(void) {
    pidc_t p; pid_gains_t g = gains(0, 10.0f, 0); pidc_init(&p, &g);
    for (int i = 0; i < 10; i++) pidc_update(&p, 1.0f, 0.001f);
    pidc_reset(&p);
    TT_NEAR(pidc_update(&p, 0.0f, 0.001f), 0.0f, 1e-6);
}

int main(void) {
    TT_RUN(test_proportional_only);
    TT_RUN(test_integral_accumulates_with_dt);
    TT_RUN(test_integral_clamped);
    TT_RUN(test_output_clamped);
    TT_RUN(test_anti_windup_recovers_quickly);
    TT_RUN(test_derivative_skips_first_sample);
    TT_RUN(test_reset_clears_state);
    return TT_DONE();
}
```

- [ ] **Step 2: Run to verify it fails** — `make ctest` → compile error (`axis/pid.h` not found).

- [ ] **Step 3: Implement** — `pid.h` with the interface above (include guard `#pragma once`, doc comments on units). `pid.c`:
```c
#include "axis/pid.h"

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void pidc_init(pidc_t *p, const pid_gains_t *g) { p->g = *g; pidc_reset(p); }

void pidc_reset(pidc_t *p) { p->integ = 0.0f; p->prev_err = 0.0f; p->has_prev = 0; }

float pidc_update(pidc_t *p, float err, float dt) {
    const pid_gains_t *g = &p->g;
    float d = 0.0f;
    if (p->has_prev && dt > 0.0f) d = g->kd * (err - p->prev_err) / dt;
    p->prev_err = err;
    p->has_prev = 1;

    float integ_new = clampf(p->integ + g->ki * err * dt, g->i_min, g->i_max);
    float unclamped = g->kp * err + integ_new + d;
    /* Anti-windup: refuse integral growth that pushes further into saturation. */
    int winding_up = (unclamped > g->out_max && err > 0.0f) || (unclamped < g->out_min && err < 0.0f);
    if (!winding_up) p->integ = integ_new;
    return clampf(g->kp * err + p->integ + d, g->out_min, g->out_max);
}
```

- [ ] **Step 4: Run** — `make ctest` → all pass.
- [ ] **Step 5: Commit** — `git commit -m "feat(core): add PID controller with anti-windup"`

### Task 3: CAN protocol (C + Python, shared golden vectors)

**Files:**
- Create: `components/axis_core/include/axis/protocol.h`, `components/axis_core/src/protocol.c`, `pc/robotarm/protocol.py`, `docs/protocol.md`
- Test: `host/tests/test_protocol.c`, `pc/tests/test_protocol.py`

**Interfaces:**
- Produces (C):
```c
typedef struct { uint16_t id; uint8_t len; uint8_t data[8]; } can_frame_t;
enum { PROTO_NODE_BROADCAST = 0 };
enum { PROTO_MSG_ESTOP = 0x00, PROTO_MSG_HEARTBEAT = 0x01, PROTO_MSG_COMMAND = 0x02,
       PROTO_MSG_SETPOINT = 0x03, PROTO_MSG_STATUS = 0x10, PROTO_MSG_TELEMETRY = 0x11 };
enum { PROTO_CMD_DISABLE = 0, PROTO_CMD_ENABLE = 1, PROTO_CMD_HOME = 2, PROTO_CMD_CLEAR_FAULT = 3 };
enum { PROTO_SP_POSITION = 0, PROTO_SP_VELOCITY = 1, PROTO_SP_DUTY = 2 };
#define PROTO_DUTY_SCALE 10000
#define PROTO_ID(type, node) ((uint16_t)(((type) << 3) | ((node) & 0x7)))
static inline uint8_t proto_type(uint16_t id) { return (uint8_t)(id >> 3); }
static inline uint8_t proto_node(uint16_t id) { return (uint8_t)(id & 0x7); }
void proto_encode_estop(can_frame_t *f);
void proto_encode_heartbeat(can_frame_t *f, uint8_t seq);
void proto_encode_command(can_frame_t *f, uint8_t node, uint8_t cmd);
void proto_encode_setpoint(can_frame_t *f, uint8_t node, uint8_t kind, int32_t value);
void proto_encode_status(can_frame_t *f, uint8_t node, int32_t pos, uint8_t state, uint8_t faults, uint8_t flags);
void proto_encode_telemetry(can_frame_t *f, uint8_t node, int32_t vel, int16_t current_ma);
bool proto_decode_command(const can_frame_t *f, uint8_t *cmd);
bool proto_decode_setpoint(const can_frame_t *f, uint8_t *kind, int32_t *value);
bool proto_decode_status(const can_frame_t *f, int32_t *pos, uint8_t *state, uint8_t *faults, uint8_t *flags);
bool proto_decode_telemetry(const can_frame_t *f, int32_t *vel, int16_t *current_ma);
```
Decoders return false when the type in the id or `len` does not match.
- Produces (Python, `robotarm.protocol`): same constants as `IntEnum`s `MsgType`, `Command`, `SetpointKind`, `AxisState` (DISABLED=0, HOMING=1, READY=2, FAULT=3), `Fault` (`IntFlag`: WATCHDOG=1, OVERCURRENT=2, FOLLOWING=4, ESTOP=8, HOMING=16); `DUTY_SCALE = 10000`; functions `make_id(type, node) -> int`, `encode_estop() -> can.Message`, `encode_heartbeat(seq)`, `encode_command(node, cmd)`, `encode_setpoint(node, kind, value)`, `encode_status(node, pos, state, faults, flags)`, `encode_telemetry(node, vel, current_ma)`, and `decode(msg: can.Message) -> Estop | Heartbeat | CommandMsg | Setpoint | Status | Telemetry | None` returning frozen dataclasses (`Status(node, position, state: AxisState, faults: Fault, homed: bool)`, `Telemetry(node, velocity, current_ma)`, `Setpoint(node, kind, value)`, `CommandMsg(node, command)`, `Heartbeat(seq)`, `Estop()`). All messages have `is_extended_id=False`.

- [ ] **Step 1: Write failing C test** — the golden vectors (identical literals appear in the Python test — that is what keeps the two implementations in agreement):
```c
#include <string.h>
#include "axis/protocol.h"
#include "tinytest.h"

static int frame_eq(const can_frame_t *f, uint16_t id, uint8_t len, const uint8_t *data) {
    return f->id == id && f->len == len && memcmp(f->data, data, len) == 0;
}

static void test_golden_vectors(void) {
    can_frame_t f;
    proto_encode_estop(&f);
    TT_CHECK(f.id == 0x000 && f.len == 0);
    proto_encode_heartbeat(&f, 7);
    TT_CHECK(frame_eq(&f, 0x008, 1, (const uint8_t[]){0x07}));
    proto_encode_command(&f, 1, PROTO_CMD_HOME);
    TT_CHECK(frame_eq(&f, 0x011, 1, (const uint8_t[]){0x02}));
    proto_encode_setpoint(&f, 3, PROTO_SP_VELOCITY, -1000);
    TT_CHECK(frame_eq(&f, 0x01B, 5, (const uint8_t[]){0x01, 0x18, 0xFC, 0xFF, 0xFF}));
    proto_encode_status(&f, 2, 123456, 2, 0x05, 0x01);
    TT_CHECK(frame_eq(&f, 0x082, 7, (const uint8_t[]){0x40, 0xE2, 0x01, 0x00, 0x02, 0x05, 0x01}));
    proto_encode_telemetry(&f, 6, -250000, 1500);
    TT_CHECK(frame_eq(&f, 0x08E, 6, (const uint8_t[]){0x70, 0x2F, 0xFC, 0xFF, 0xDC, 0x05}));
}

static void test_roundtrip_and_rejects(void) {
    can_frame_t f; uint8_t kind; int32_t value;
    proto_encode_setpoint(&f, 4, PROTO_SP_POSITION, 2147483647);
    TT_CHECK(proto_decode_setpoint(&f, &kind, &value) && kind == PROTO_SP_POSITION && value == 2147483647);
    int32_t pos; uint8_t st, fl, fg;
    TT_CHECK(!proto_decode_status(&f, &pos, &st, &fl, &fg));        /* wrong type */
    f.len = 4;
    TT_CHECK(!proto_decode_setpoint(&f, &kind, &value));             /* wrong length */
    int32_t vel; int16_t cur;
    proto_encode_telemetry(&f, 1, 42, -3);
    TT_CHECK(proto_decode_telemetry(&f, &vel, &cur) && vel == 42 && cur == -3);
    TT_CHECK(proto_node(f.id) == 1 && proto_type(f.id) == PROTO_MSG_TELEMETRY);
}

int main(void) { TT_RUN(test_golden_vectors); TT_RUN(test_roundtrip_and_rejects); return TT_DONE(); }
```

- [ ] **Step 2: Write failing Python test** — `pc/tests/test_protocol.py`:
```python
import can
import pytest

from robotarm import protocol as p


def frame(msg: can.Message):
    return msg.arbitration_id, bytes(msg.data)


def test_golden_vectors():
    assert frame(p.encode_estop()) == (0x000, b"")
    assert frame(p.encode_heartbeat(7)) == (0x008, bytes([0x07]))
    assert frame(p.encode_command(1, p.Command.HOME)) == (0x011, bytes([0x02]))
    assert frame(p.encode_setpoint(3, p.SetpointKind.VELOCITY, -1000)) == (0x01B, bytes([0x01, 0x18, 0xFC, 0xFF, 0xFF]))
    assert frame(p.encode_status(2, 123456, p.AxisState.READY, 0x05, 0x01)) == (
        0x082, bytes([0x40, 0xE2, 0x01, 0x00, 0x02, 0x05, 0x01]))
    assert frame(p.encode_telemetry(6, -250000, 1500)) == (0x08E, bytes([0x70, 0x2F, 0xFC, 0xFF, 0xDC, 0x05]))


def test_decode_status():
    msg = p.decode(p.encode_status(2, 123456, p.AxisState.READY, 0x05, 0x01))
    assert msg == p.Status(node=2, position=123456, state=p.AxisState.READY,
                           faults=p.Fault.WATCHDOG | p.Fault.FOLLOWING, homed=True)


def test_decode_rejects_bad_length_and_unknown_type():
    bad = can.Message(arbitration_id=p.make_id(p.MsgType.STATUS, 2), data=b"\x00\x01", is_extended_id=False)
    assert p.decode(bad) is None
    assert p.decode(can.Message(arbitration_id=0x7F0, data=b"", is_extended_id=False)) is None


@pytest.mark.parametrize("value", [-(2**31), -1, 0, 1, 2**31 - 1])
def test_setpoint_roundtrip(value):
    assert p.decode(p.encode_setpoint(5, p.SetpointKind.POSITION, value)) == p.Setpoint(5, p.SetpointKind.POSITION, value)
```

- [ ] **Step 3: Run both** — `make test` → C compile fails; pytest ImportError.

- [ ] **Step 4: Implement** `protocol.c` with little-endian helpers `put_i32/get_i32/put_i16/get_i16` (byte shifts, never `memcpy` of structs), and `robotarm/protocol.py` with `struct` formats `"<Bi"` (setpoint), `"<iBBB"` (status), `"<ih"` (telemetry). Encoders validate `node` in 0..7 (Python raises `ValueError`; C masks with `& 0x7`).

- [ ] **Step 5: Write `docs/protocol.md`** — the protocol table from the Design section, id formula, state/fault tables, rates, watchdog rule, and the golden vectors table.

- [ ] **Step 6: Run** — `make test` → pass. **Commit** — `git commit -m "feat(protocol): CAN protocol in C and Python with shared golden vectors"`

### Task 4: Shared arm configuration and C config generator

**Files:**
- Create: `config/arm.yaml`, `pc/robotarm/config.py`, `pc/robotarm/tools/__init__.py`, `pc/robotarm/tools/gen_config.py`
- Create: `components/axis_core/include/axis/axis_config.h`, `components/axis_core/include/axis/config_table.h`, `components/axis_core/src/config_table.c` (generated)
- Test: `pc/tests/test_config.py`, `host/tests/test_config_table.c`

**Interfaces:**
- Produces (C) `axis_config.h`:
```c
#include <stdint.h>
#include "axis/pid.h"
#define AXIS_TICK_HZ 1000
#define AXIS_DT (1.0f / (float)AXIS_TICK_HZ)
typedef struct {
    uint8_t node_id;
    const char *name;
    float counts_per_rad;        /* encoder counts per joint radian (positive) */
    float max_duty;              /* motor nominal voltage / supply voltage, <= 1 */
    int32_t pos_min, pos_max;    /* soft limits, counts relative to home zero */
    float max_vel;               /* counts/s */
    float max_acc;               /* counts/s^2 */
    float max_current_ma;        /* sustained above this for overcurrent_ms -> fault */
    uint32_t overcurrent_ms;
    int32_t max_following_error; /* counts */
    pid_gains_t pos_pid;         /* error counts -> velocity correction counts/s */
    pid_gains_t vel_pid;         /* error counts/s -> duty */
    float vel_ff;                /* duty per counts/s (feed-forward) */
    int8_t home_dir;             /* -1 or +1: direction of the homing end stop */
    float home_vel;              /* counts/s, positive */
    int32_t home_pos;            /* joint position (counts) at the end stop */
    float home_current_ma;       /* stall threshold */
    uint32_t home_timeout_ms;
    int8_t motor_sign, encoder_sign;
    uint32_t watchdog_ms;
} axis_config_t;
```
`config_table.h`: `extern const axis_config_t AXIS_CONFIGS[]; extern const unsigned AXIS_CONFIG_COUNT; const axis_config_t *axis_config_for_node(uint8_t node);` (returns NULL if unknown).
- Produces (Python) `robotarm.config`: `load_arm_config(path: Path | None = None) -> ArmConfig` (default `config/arm.yaml` relative to repo root); dataclasses `ArmConfig(supply_voltage, control_hz, axes: list[AxisConfig], motors: dict[str, MotorConfig], geometry: Geometry, bench: dict | None)`, `AxisConfig(node, name, joint, encoder_cpr, gear_ratio, motor: MotorConfig, motor_sign, encoder_sign, soft_limits_rad: tuple[float,float], hard_limits_rad: tuple[float,float], max_velocity_rad_s, max_accel_rad_s2, home: HomeConfig, gains: GainsConfig, friction: FrictionConfig, link_mass_kg, gear_efficiency, max_current_ma, max_following_error_rad, watchdog_ms)` with properties `counts_per_rad` (`4*encoder_cpr*gear_ratio/(2π)`), `max_duty` (`min(1, motor.nominal_v/supply_voltage)`), `vel_ff` (duty per counts/s: `1 / (counts/s at duty 1)` where counts/s at duty 1 = `supply_voltage / motor.kt / (2π) * 4*encoder_cpr` — the no-load motor speed at full supply), and methods `rad_to_counts(rad) -> int`, `counts_to_rad(counts) -> float`. `MotorConfig(name, nominal_v, R, L, kt, j_rotor, sense_mv_per_a)`. `ArmConfig.axis(node)` and `ArmConfig.axis_by_name(name)`.
- `robotarm gen-config [--check]` writes `components/axis_core/src/config_table.c`; with `--check` exits 1 if the file on disk differs.

- [ ] **Step 1: Write `config/arm.yaml`** (full content):
```yaml
# Single source of truth for per-axis parameters (firmware + simulator + master).
# Anything marked "(assumed)" is an estimate: measure it on the robot (see docs/bringup.md).
# Joint convention: q = 0 is the arm pointing straight up; J2/J3/J4 positive tilts toward +x.
supply_voltage: 24.0
control_hz: 1000

motors:
  # Faulhaber datasheet-class values (assumed) -- replace with exact datasheet numbers.
  faulhaber_2657cr_24v: {nominal_v: 24.0, R: 4.2,  L: 0.00046, kt: 0.0347, j_rotor: 2.0e-6}
  faulhaber_2657cr_12v: {nominal_v: 12.0, R: 1.14, L: 0.00012, kt: 0.0191, j_rotor: 2.0e-6}
  faulhaber_2642cr_12v: {nominal_v: 12.0, R: 1.9,  L: 0.00018, kt: 0.0164, j_rotor: 1.1e-6}
  faulhaber_2224sr_12v: {nominal_v: 12.0, R: 8.7,  L: 0.00030, kt: 0.0143, j_rotor: 0.26e-6}

current_sense:
  # TB9051FTG OCM mirror (~0.24 % of motor current) into R3 = 220 ohm -> ~528 mV/A (assumed, verify)
  mv_per_a: 528.0
  adc_max_mv: 2500.0      # with ADC_ATTEN_DB_12 on the ESP32-S3 (usable range ~2.5 V)

geometry:                 # Katana 6M180-like, metres (assumed, measure)
  base_height: 0.2015
  upper_arm: 0.190
  forearm: 0.139
  wrist: 0.060            # wrist pitch axis -> wrist roll body
  gripper: 0.130          # wrist roll body -> gripper tip

defaults:
  gear_efficiency: 0.7            # (assumed)
  watchdog_ms: 200
  max_following_error_deg: 10.0
  gains:
    pos_kp: 20.0                  # 1/s
    pos_ki: 0.0
    vel_kp: 0.00004               # duty per count/s -- retuned in Task 12
    vel_ki: 0.002
    vel_i_limit: 0.3              # duty

axes:
  - node: 1
    name: hip
    joint: j1
    encoder_cpr: 128
    gear_ratio: 100.0
    motor: faulhaber_2657cr_24v
    motor_sign: 1
    encoder_sign: 1
    soft_limits_deg: [-160, 160]  # (assumed)
    hard_limits_deg: [-169, 169]  # mechanical stops (assumed)
    max_velocity_deg_s: 60
    max_accel_deg_s2: 180
    max_current_ma: 2500
    home: {direction: -1, velocity_deg_s: 15, current_ma: 1800, timeout_s: 40}
    link_mass_kg: 1.2             # (assumed)
    friction: {coulomb_nm: 0.8, viscous_nm_s: 0.5}   # at the joint (assumed)
  - node: 2
    name: shoulder
    joint: j2
    encoder_cpr: 64
    gear_ratio: 370.0             # 3.7 planetary x strainwave 100
    motor: faulhaber_2657cr_12v
    motor_sign: 1
    encoder_sign: 1
    soft_limits_deg: [-25, 100]
    hard_limits_deg: [-31, 104]
    max_velocity_deg_s: 45
    max_accel_deg_s2: 120
    max_current_ma: 2500
    home: {direction: 1, velocity_deg_s: 10, current_ma: 1800, timeout_s: 30}
    link_mass_kg: 1.0
    friction: {coulomb_nm: 1.2, viscous_nm_s: 0.8}
  - node: 3
    name: elbow
    joint: j3
    encoder_cpr: 360
    gear_ratio: 370.0
    motor: faulhaber_2642cr_12v
    motor_sign: 1
    encoder_sign: 1
    soft_limits_deg: [-115, 115]
    hard_limits_deg: [-122, 122]
    max_velocity_deg_s: 60
    max_accel_deg_s2: 180
    max_current_ma: 2000
    home: {direction: -1, velocity_deg_s: 10, current_ma: 1500, timeout_s: 30}
    link_mass_kg: 0.8
    friction: {coulomb_nm: 0.8, viscous_nm_s: 0.5}
  - node: 4
    name: wrist_bend
    joint: j4
    encoder_cpr: 128              # (assumed: not documented in README)
    gear_ratio: 100.0             # (assumed)
    motor: faulhaber_2224sr_12v   # (assumed)
    motor_sign: 1
    encoder_sign: 1
    soft_limits_deg: [-110, 110]
    hard_limits_deg: [-117, 117]
    max_velocity_deg_s: 90
    max_accel_deg_s2: 360
    max_current_ma: 800
    home: {direction: 1, velocity_deg_s: 15, current_ma: 600, timeout_s: 30}
    link_mass_kg: 0.4
    friction: {coulomb_nm: 0.2, viscous_nm_s: 0.1}
  - node: 5
    name: wrist_rotate
    joint: j5
    encoder_cpr: 128
    gear_ratio: 100.0
    motor: faulhaber_2224sr_12v
    motor_sign: 1
    encoder_sign: 1
    soft_limits_deg: [-160, 160]
    hard_limits_deg: [-168, 168]
    max_velocity_deg_s: 120
    max_accel_deg_s2: 480
    max_current_ma: 800
    home: {direction: -1, velocity_deg_s: 20, current_ma: 600, timeout_s: 30}
    link_mass_kg: 0.3
    friction: {coulomb_nm: 0.1, viscous_nm_s: 0.05}
  - node: 6
    name: gripper
    joint: j6
    encoder_cpr: 128
    gear_ratio: 100.0
    motor: faulhaber_2224sr_12v
    motor_sign: 1
    encoder_sign: 1
    soft_limits_deg: [0, 55]      # 0 = closed
    hard_limits_deg: [-3, 60]
    max_velocity_deg_s: 60
    max_accel_deg_s2: 360
    max_current_ma: 800
    home: {direction: -1, velocity_deg_s: 10, current_ma: 500, timeout_s: 20}
    link_mass_kg: 0.2
    friction: {coulomb_nm: 0.05, viscous_nm_s: 0.02}
```
Per-axis `gains:` may override any key of `defaults.gains`. `home.position` defaults to the hard limit on the homing side (`hard_limits_deg[0]` when direction −1, else `[1]`). Motor current sense `sense_mv_per_a` comes from `current_sense.mv_per_a`.

- [ ] **Step 2: Failing Python test** — `pc/tests/test_config.py`:
```python
import math
import subprocess
import sys

import pytest

from robotarm.config import load_arm_config


@pytest.fixture(scope="module")
def cfg():
    return load_arm_config()


def test_six_axes_with_unique_nodes(cfg):
    assert [a.node for a in cfg.axes] == [1, 2, 3, 4, 5, 6]


def test_counts_per_rad_shoulder(cfg):
    shoulder = cfg.axis_by_name("shoulder")
    assert shoulder.counts_per_rad == pytest.approx(4 * 64 * 370 / (2 * math.pi))


def test_twelve_volt_motor_duty_is_capped(cfg):
    assert cfg.axis_by_name("shoulder").max_duty == pytest.approx(0.5)
    assert cfg.axis_by_name("hip").max_duty == pytest.approx(1.0)


def test_rad_counts_roundtrip(cfg):
    elbow = cfg.axis_by_name("elbow")
    assert elbow.counts_to_rad(elbow.rad_to_counts(0.5)) == pytest.approx(0.5, abs=1e-4)


def test_home_position_defaults_to_hard_limit(cfg):
    hip = cfg.axis_by_name("hip")
    assert hip.home.direction == -1
    assert hip.home.position_rad == pytest.approx(math.radians(-169))


def test_soft_limits_inside_hard_limits(cfg):
    for a in cfg.axes:
        assert a.hard_limits_rad[0] < a.soft_limits_rad[0] < a.soft_limits_rad[1] < a.hard_limits_rad[1]


def test_generated_c_table_is_up_to_date():
    result = subprocess.run([sys.executable, "-m", "robotarm", "gen-config", "--check"], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
```

- [ ] **Step 3: Failing C test** — `host/tests/test_config_table.c`:
```c
#include "axis/config_table.h"
#include "tinytest.h"

static void test_lookup(void) {
    TT_CHECK(AXIS_CONFIG_COUNT == 6);
    const axis_config_t *c = axis_config_for_node(2);
    TT_CHECK(c != NULL && c->node_id == 2);
    TT_NEAR(c->max_duty, 0.5f, 1e-6);
    TT_CHECK(c->pos_min < 0 && c->pos_max > 0);
    TT_CHECK(c->home_dir == 1 && c->home_pos > c->pos_max);
    TT_CHECK(axis_config_for_node(0) == NULL && axis_config_for_node(7) == NULL);
}

int main(void) { TT_RUN(test_lookup); return TT_DONE(); }
```

- [ ] **Step 4: Implement** `config.py` (dataclasses, yaml loading, defaults merge, derived properties, validation raising `ValueError` with the axis name for: duplicate nodes, soft limits not inside hard limits, unknown motor name) and `tools/gen_config.py`. The generator renders one C initializer per axis, converting: limits deg→counts (`round(rad*counts_per_rad)`), velocities/accels deg/s→counts/s, `home_pos` counts, `pos_pid = {kp, ki, 0, -max_vel, max_vel, -max_vel*0.2, max_vel*0.2}`, `vel_pid = {vel_kp, vel_ki, 0, -max_duty, max_duty, -vel_i_limit, vel_i_limit}`, `overcurrent_ms = 200`, following error deg→counts. Floats are printed with `repr(float(x)) + "f"` (`1e-05f` is valid C). Header comment: `/* GENERATED by `uv run robotarm gen-config` from config/arm.yaml -- do not edit. */`. Register the `gen-config` subcommand in `cli.py`. Add `src/config_table.c` to the component sources and `axis_test(test_config_table)`.

- [ ] **Step 5: Generate and run** — `uv run robotarm gen-config && make test` → pass.
- [ ] **Step 6: Commit** — `git commit -m "feat(config): arm.yaml single source of truth with generated C config table"`

### Task 5: Axis core — state machine, commands, watchdog, E-STOP, status

**Files:**
- Create: `components/axis_core/include/axis/axis.h`, `components/axis_core/src/axis.c`
- Create: `host/tests/test_config.h` (test fixture config), `host/tests/test_axis_states.c`

**Interfaces:**
- Consumes: `pidc_*` (Task 2), `proto_*` (Task 3), `axis_config_t` (Task 4).
- Produces (`axis.h`):
```c
typedef enum { AXIS_DISABLED = 0, AXIS_HOMING = 1, AXIS_READY = 2, AXIS_FAULT = 3 } axis_state_t;
enum { AXIS_FAULT_WATCHDOG = 1u << 0, AXIS_FAULT_OVERCURRENT = 1u << 1, AXIS_FAULT_FOLLOWING = 1u << 2,
       AXIS_FAULT_ESTOP = 1u << 3, AXIS_FAULT_HOMING = 1u << 4 };
enum { AXIS_FLAG_HOMED = 1u << 0 };
typedef struct { int32_t encoder_raw; float current_ma; } axis_inputs_t;   /* raw = accumulated hw count */
typedef struct { float duty; } axis_outputs_t;                             /* signed, already x motor_sign */
#define AXIS_TX_QUEUE_LEN 8
#define AXIS_VEL_WINDOW 8
#define AXIS_STATUS_DIVIDER 10   /* 100 Hz status at 1 kHz tick */
typedef struct {
    const axis_config_t *cfg;
    axis_state_t state;
    uint8_t faults;
    bool homed;
    int32_t zero_offset;       /* raw (sign-corrected) count that corresponds to position 0 */
    int32_t pos;               /* counts relative to home zero */
    float vel;                 /* counts/s, estimated */
    int32_t pos_hist[AXIS_VEL_WINDOW];
    uint8_t hist_idx, hist_fill;
    uint8_t sp_kind;           /* PROTO_SP_* */
    float target;              /* counts | counts/s | duty, per sp_kind */
    float sp_pos, sp_vel;      /* trajectory generator state */
    pidc_t pos_pid, vel_pid;
    uint32_t ms_since_rx, overcurrent_ms, home_ms, stall_ms, tick;
    float current_ma, duty;
    can_frame_t txq[AXIS_TX_QUEUE_LEN];
    uint8_t tx_head, tx_count;
} axis_t;
void axis_init(axis_t *a, const axis_config_t *cfg);
void axis_tick(axis_t *a, const axis_inputs_t *in, axis_outputs_t *out);
void axis_on_frame(axis_t *a, const can_frame_t *f);
bool axis_pop_tx(axis_t *a, can_frame_t *out);
void axis_set_home(axis_t *a, int32_t raw_at_zero);   /* marks homed; raw is sign-corrected */
```

Behaviour contract (this task implements the state/command/safety parts; Tasks 6–7 fill in motion and homing):
- `axis_on_frame`: ignore frames whose node is neither `cfg->node_id` nor 0. `ESTOP` → `faults |= ESTOP`, state FAULT. `HEARTBEAT`, `COMMAND`, `SETPOINT` reset `ms_since_rx`. Commands: `DISABLE` → DISABLED unless FAULT; `ENABLE` → READY only from DISABLED with `faults == 0`, with `sp_kind = VELOCITY, target = 0`; `HOME` → HOMING from DISABLED or READY (Task 7); `CLEAR_FAULT` → from FAULT clears faults, state DISABLED. `SETPOINT` is accepted only in READY: DUTY → `target = value / 10000`; VELOCITY → `target = value`; POSITION → only if homed. Changing `sp_kind` makes the transition bumpless (`sp_pos = pos`, `sp_vel = vel`, PIDs reset).
- `axis_tick`: `raw = in->encoder_raw * encoder_sign`; `pos = raw - zero_offset`; update velocity estimate; `ms_since_rx++`; if state is READY or HOMING and `ms_since_rx > watchdog_ms` → fault WATCHDOG. Overcurrent: `current_ma > max_current_ma` for more than `overcurrent_ms` consecutive ticks → fault OVERCURRENT (any state except DISABLED/FAULT). In DISABLED/FAULT: duty 0, `sp_pos = pos`, `sp_vel = 0`, PIDs reset. READY with DUTY: `duty = clamp(target, ±max_duty)`. Output `out->duty = duty * motor_sign`. Every `AXIS_STATUS_DIVIDER` ticks (every tick when READY and `sp_kind == DUTY`) queue STATUS and TELEMETRY (`current_ma` rounded, clamped to int16). Queue full → drop the oldest frame.

- [ ] **Step 1: Test fixture config** — `host/tests/test_config.h`:
```c
#pragma once
#include "axis/axis_config.h"

/* Small, round numbers so the tests are easy to reason about. */
static const axis_config_t TEST_CFG = {
    .node_id = 3, .name = "test",
    .counts_per_rad = 1000.0f, .max_duty = 0.8f,
    .pos_min = -20000, .pos_max = 20000,
    .max_vel = 10000.0f, .max_acc = 50000.0f,
    .max_current_ma = 2000.0f, .overcurrent_ms = 200,
    .max_following_error = 3000,
    .pos_pid = {20.0f, 0.0f, 0.0f, -10000.0f, 10000.0f, -2000.0f, 2000.0f},
    .vel_pid = {0.0002f, 0.004f, 0.0f, -0.8f, 0.8f, -0.3f, 0.3f},
    .vel_ff = 1.0f / 20000.0f,
    .home_dir = -1, .home_vel = 2000.0f, .home_pos = -21000,
    .home_current_ma = 1500.0f, .home_timeout_ms = 20000,
    .motor_sign = 1, .encoder_sign = 1, .watchdog_ms = 200,
};
```

- [ ] **Step 2: Failing tests** — `host/tests/test_axis_states.c`:
```c
#include "axis/axis.h"
#include "test_config.h"
#include "tinytest.h"

static axis_inputs_t in0 = {0, 0.0f};

static void send_cmd(axis_t *a, uint8_t node, uint8_t cmd) { can_frame_t f; proto_encode_command(&f, node, cmd); axis_on_frame(a, &f); }
static void send_sp(axis_t *a, uint8_t kind, int32_t v) { can_frame_t f; proto_encode_setpoint(&f, 3, kind, v); axis_on_frame(a, &f); }
static float tick(axis_t *a, const axis_inputs_t *in) { axis_outputs_t o; axis_tick(a, in, &o); return o.duty; }
static void drain(axis_t *a) { can_frame_t f; while (axis_pop_tx(a, &f)) {} }

static void test_starts_disabled_with_zero_duty(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    TT_CHECK(a.state == AXIS_DISABLED);
    TT_NEAR(tick(&a, &in0), 0.0f, 1e-9);
}

static void test_enable_and_duty_setpoint_clamped(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_READY);
    send_sp(&a, PROTO_SP_DUTY, 5000);
    TT_NEAR(tick(&a, &in0), 0.5f, 1e-6);
    send_sp(&a, PROTO_SP_DUTY, -10000);
    TT_NEAR(tick(&a, &in0), -0.8f, 1e-6);            /* max_duty */
}

static void test_motor_sign_applied(void) {
    axis_config_t cfg = TEST_CFG; cfg.motor_sign = -1;
    axis_t a; axis_init(&a, &cfg);
    send_cmd(&a, 3, PROTO_CMD_ENABLE); send_sp(&a, PROTO_SP_DUTY, 2500);
    TT_NEAR(tick(&a, &in0), -0.25f, 1e-6);
}

static void test_frames_for_other_nodes_ignored(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 4, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_DISABLED);
    send_cmd(&a, PROTO_NODE_BROADCAST, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_READY);
}

static void test_watchdog_trips_without_master(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE); send_sp(&a, PROTO_SP_DUTY, 5000);
    for (int i = 0; i < 200; i++) tick(&a, &in0);
    TT_CHECK(a.state == AXIS_READY);
    tick(&a, &in0);
    TT_CHECK(a.state == AXIS_FAULT && (a.faults & AXIS_FAULT_WATCHDOG));
    TT_NEAR(tick(&a, &in0), 0.0f, 1e-9);
}

static void test_heartbeat_keeps_axis_alive(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    for (int i = 0; i < 2000; i++) {
        if (i % 100 == 0) { can_frame_t f; proto_encode_heartbeat(&f, (uint8_t)i); axis_on_frame(&a, &f); }
        tick(&a, &in0);
    }
    TT_CHECK(a.state == AXIS_READY);
}

static void test_watchdog_ignored_while_disabled(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    for (int i = 0; i < 1000; i++) tick(&a, &in0);
    TT_CHECK(a.state == AXIS_DISABLED && a.faults == 0);
}

static void test_estop_and_clear(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE); send_sp(&a, PROTO_SP_DUTY, 5000);
    can_frame_t f; proto_encode_estop(&f); axis_on_frame(&a, &f);
    TT_CHECK(a.state == AXIS_FAULT && (a.faults & AXIS_FAULT_ESTOP));
    TT_NEAR(tick(&a, &in0), 0.0f, 1e-9);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_FAULT);                  /* cannot enable while faulted */
    send_cmd(&a, 3, PROTO_CMD_CLEAR_FAULT);
    TT_CHECK(a.state == AXIS_DISABLED && a.faults == 0);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    TT_CHECK(a.state == AXIS_READY);
}

static void test_overcurrent_fault(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    send_cmd(&a, 3, PROTO_CMD_ENABLE);
    axis_inputs_t hot = {0, 2500.0f};
    for (int i = 0; i < 150; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&a, &f); }
        tick(&a, &hot);
    }
    TT_CHECK(a.state == AXIS_READY);                  /* short spikes tolerated */
    for (int i = 0; i < 100; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&a, &f); }
        tick(&a, &hot);
    }
    TT_CHECK(a.state == AXIS_FAULT && (a.faults & AXIS_FAULT_OVERCURRENT));
}

static void test_status_frames_at_100hz(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    drain(&a);
    axis_inputs_t in = {1234, 321.0f};
    int status = 0, telem = 0;
    for (int i = 0; i < 100; i++) {
        tick(&a, &in);
        can_frame_t f;
        while (axis_pop_tx(&a, &f)) {
            int32_t pos, vel; uint8_t st, fl, fg; int16_t cur;
            if (proto_decode_status(&f, &pos, &st, &fl, &fg)) { status++; TT_CHECK(pos == 1234 && st == AXIS_DISABLED && fg == 0); }
            if (proto_decode_telemetry(&f, &vel, &cur)) { telem++; TT_CHECK(cur == 321); }
            TT_CHECK(proto_node(f.id) == 3);
        }
    }
    TT_CHECK(status == 10 && telem == 10);
}

static void test_tx_queue_drops_oldest_when_full(void) {
    axis_t a; axis_init(&a, &TEST_CFG);
    for (int i = 0; i < 1000; i++) tick(&a, &in0);   /* never drained */
    int n = 0; can_frame_t f;
    while (axis_pop_tx(&a, &f)) n++;
    TT_CHECK(n == AXIS_TX_QUEUE_LEN);
}

int main(void) {
    TT_RUN(test_starts_disabled_with_zero_duty);
    TT_RUN(test_enable_and_duty_setpoint_clamped);
    TT_RUN(test_motor_sign_applied);
    TT_RUN(test_frames_for_other_nodes_ignored);
    TT_RUN(test_watchdog_trips_without_master);
    TT_RUN(test_heartbeat_keeps_axis_alive);
    TT_RUN(test_watchdog_ignored_while_disabled);
    TT_RUN(test_estop_and_clear);
    TT_RUN(test_overcurrent_fault);
    TT_RUN(test_status_frames_at_100hz);
    TT_RUN(test_tx_queue_drops_oldest_when_full);
    return TT_DONE();
}
```
- [ ] **Step 3: Run** — `make ctest` → fails to compile.
- [ ] **Step 4: Implement** `axis.h`/`axis.c` per the contract. Structure `axis.c` as small static functions: `update_velocity_estimate`, `check_safety`, `enter_fault(a, bits)`, `handle_command`, `handle_setpoint`, `queue_tx`, `queue_status`, `run_ready` (DUTY only for now; VELOCITY/POSITION output 0 until Task 6), `run_homing` (stub → stays in HOMING with duty 0 until Task 7). Velocity estimate: `vel = (pos - pos_hist[oldest]) * AXIS_TICK_HZ / filled_window` over the `AXIS_VEL_WINDOW` ring buffer.
- [ ] **Step 5: Run** — `make test` → pass.
- [ ] **Step 6: Commit** — `git commit -m "feat(core): axis state machine with watchdog, e-stop, overcurrent and status telemetry"`

### Task 6: Axis core — trajectory generator, cascade control, soft limits, following error

**Files:**
- Modify: `components/axis_core/src/axis.c`
- Create: `host/tests/plant.h` (first-order test plant), `host/tests/test_axis_motion.c`

**Interfaces:**
- Consumes: Task 5 `axis_t` fields `sp_pos`, `sp_vel`, `target`, `sp_kind`.
- Produces: READY-state behaviour for VELOCITY and POSITION setpoints; `test/plant.h` API `plant_t`, `plant_init(plant_t*, float gain_counts_per_s_per_duty, float tau_s)`, `plant_step(plant_t*, float duty, float dt)`, fields `pos` (double), `vel`, `stall_at` (optional hard stop: `has_stop`, `stop_pos`), `current_ma` (= |duty|*1000 normally, 2500 when pushing into the stop).

Algorithm (per tick, `dt = AXIS_DT`):
```
if sp_kind == POSITION:
    goal = clamp(target, pos_min, pos_max)
    dist = goal - sp_pos
    v_des = sign(dist) * min(max_vel, sqrtf(2 * max_acc * fabsf(dist)))
    if fabsf(dist) < 0.5 && fabsf(sp_vel) < max_acc * dt: sp_pos = goal; sp_vel = 0; v_des = 0
elif sp_kind == VELOCITY:
    vmax = homed ? max_vel : home_vel          # unhomed jogging is slow
    v_des = clamp(target, -vmax, vmax)
    if homed:                                  # brake toward soft limits, never push outward
        v_des = min(v_des, sqrtf(2 * max_acc * max(0, pos_max - sp_pos)))
        v_des = max(v_des, -sqrtf(2 * max_acc * max(0, sp_pos - pos_min)))
sp_vel += clamp(v_des - sp_vel, -max_acc*dt, max_acc*dt)
sp_pos += sp_vel * dt
vel_cmd = sp_vel + pidc_update(&pos_pid, sp_pos - pos, dt)
duty    = vel_ff * vel_cmd + pidc_update(&vel_pid, vel_cmd - vel, dt)
if |sp_pos - pos| > max_following_error: fault FOLLOWING
```

- [ ] **Step 1: Test plant** — `host/tests/plant.h`:
```c
#pragma once
#include <math.h>

/* First-order velocity plant: tau * dv/dt = gain * duty - v. Optional hard stop. */
typedef struct {
    double pos; float vel; float gain, tau;
    int has_stop; double stop_pos; int stop_dir;   /* stop_dir -1: stop below, +1: stop above */
    float current_ma;
} plant_t;

static void plant_init(plant_t *p, float gain, float tau) {
    p->pos = 0; p->vel = 0; p->gain = gain; p->tau = tau; p->has_stop = 0; p->stop_pos = 0; p->stop_dir = 0; p->current_ma = 0;
}

static void plant_step(plant_t *p, float duty, float dt) {
    p->vel += (p->gain * duty - p->vel) * dt / p->tau;
    p->pos += p->vel * dt;
    p->current_ma = fabsf(duty) * 1000.0f;
    if (p->has_stop && ((p->stop_dir < 0 && p->pos <= p->stop_pos) || (p->stop_dir > 0 && p->pos >= p->stop_pos))) {
        p->pos = p->stop_pos; p->vel = 0;
        if (duty * p->stop_dir > 0.05f) p->current_ma = 2500.0f;   /* stalled against the stop */
    }
}

static int32_t plant_count(const plant_t *p) { return (int32_t)floor(p->pos); }
```

- [ ] **Step 2: Failing tests** — `host/tests/test_axis_motion.c`:
```c
#include <math.h>
#include "axis/axis.h"
#include "plant.h"
#include "test_config.h"
#include "tinytest.h"

typedef struct { axis_t a; plant_t p; float max_err; float max_pos; } rig_t;

static void rig_init(rig_t *r, const axis_config_t *cfg) {
    axis_init(&r->a, cfg); plant_init(&r->p, 20000.0f, 0.02f); r->max_err = 0; r->max_pos = -1e9f;
}
static void cmd(rig_t *r, uint8_t c) { can_frame_t f; proto_encode_command(&f, 3, c); axis_on_frame(&r->a, &f); }
static void sp(rig_t *r, uint8_t kind, int32_t v) { can_frame_t f; proto_encode_setpoint(&f, 3, kind, v); axis_on_frame(&r->a, &f); }
static void run_ms(rig_t *r, int ms) {
    for (int i = 0; i < ms; i++) {
        if (i % 50 == 0) { can_frame_t f; proto_encode_heartbeat(&f, 0); axis_on_frame(&r->a, &f); }
        axis_inputs_t in = {plant_count(&r->p), r->p.current_ma};
        axis_outputs_t out; axis_tick(&r->a, &in, &out);
        plant_step(&r->p, out.duty, AXIS_DT);
        float err = fabsf(r->a.sp_pos - (float)r->a.pos);
        if (err > r->max_err) r->max_err = err;
        if ((float)r->p.pos > r->max_pos) r->max_pos = (float)r->p.pos;
        can_frame_t f; while (axis_pop_tx(&r->a, &f)) {}
    }
}

static void test_velocity_mode_tracks_target(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 5000);
    run_ms(&r, 50);
    TT_CHECK(r.a.sp_vel < 5000.0f);                         /* accel-limited ramp: 50000 * 0.05 = 2500 */
    run_ms(&r, 950);
    TT_NEAR(r.a.vel, 5000.0f, 250.0f);
    TT_CHECK(r.a.state == AXIS_READY);
}

static void test_velocity_clamped_to_max(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 1000000);
    run_ms(&r, 1000);
    TT_NEAR(r.a.sp_vel, 10000.0f, 1.0f);
}

static void test_position_move_settles_without_big_overshoot(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_POSITION, 10000);
    run_ms(&r, 3000);
    TT_NEAR(r.a.pos, 10000, 5);
    TT_CHECK(r.max_pos < 10050.0f);
    TT_CHECK(r.a.state == AXIS_READY);
}

static void test_position_target_clamped_to_soft_limit(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_POSITION, 50000);
    run_ms(&r, 6000);
    TT_NEAR(r.a.pos, 20000, 10);
    TT_CHECK(r.a.state == AXIS_READY && r.a.faults == 0);
}

static void test_velocity_jog_stops_at_soft_limit(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 10000);
    run_ms(&r, 5000);                                       /* keeps pushing into the limit */
    TT_CHECK(r.a.sp_pos <= 20000.5f);
    TT_CHECK(r.max_pos <= 20060.0f);
    TT_CHECK(r.a.state == AXIS_READY && r.a.faults == 0);
    sp(&r, PROTO_SP_VELOCITY, -5000);                       /* can drive back out */
    run_ms(&r, 500);
    TT_CHECK(r.a.pos < 19000);
}

static void test_unhomed_position_setpoint_ignored_and_jog_is_slow(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_POSITION, 10000);
    run_ms(&r, 500);
    TT_NEAR(r.a.pos, 0, 5);
    sp(&r, PROTO_SP_VELOCITY, 10000);
    run_ms(&r, 1000);
    TT_NEAR(r.a.sp_vel, TEST_CFG.home_vel, 1.0f);
}

static void test_following_error_fault_when_stalled(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    r.p.gain = 0.0f;                                        /* motor does not move */
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_VELOCITY, 5000);
    run_ms(&r, 2000);
    TT_CHECK(r.a.state == AXIS_FAULT && (r.a.faults & AXIS_FAULT_FOLLOWING));
}

static void test_mode_switch_is_bumpless(void) {
    rig_t r; rig_init(&r, &TEST_CFG); axis_set_home(&r.a, 0);
    cmd(&r, PROTO_CMD_ENABLE); sp(&r, PROTO_SP_DUTY, 2000);
    run_ms(&r, 300);
    int32_t p0 = r.a.pos;
    sp(&r, PROTO_SP_POSITION, p0);
    run_ms(&r, 5);
    TT_CHECK(fabsf(r.a.sp_pos - (float)p0) < 200.0f);
    run_ms(&r, 1000);
    TT_CHECK(r.a.state == AXIS_READY);
}

int main(void) {
    TT_RUN(test_velocity_mode_tracks_target);
    TT_RUN(test_velocity_clamped_to_max);
    TT_RUN(test_position_move_settles_without_big_overshoot);
    TT_RUN(test_position_target_clamped_to_soft_limit);
    TT_RUN(test_velocity_jog_stops_at_soft_limit);
    TT_RUN(test_unhomed_position_setpoint_ignored_and_jog_is_slow);
    TT_RUN(test_following_error_fault_when_stalled);
    TT_RUN(test_mode_switch_is_bumpless);
    return TT_DONE();
}
```
- [ ] **Step 3: Run** — fails (READY non-duty outputs 0).
- [ ] **Step 4: Implement** the algorithm above in `run_ready`. Use `fminf/fmaxf/sqrtf/fabsf` (float math only). If a test needs different `TEST_CFG` gains to pass, tune the fixture gains — not the tolerances.
- [ ] **Step 5: Run** — `make test` → pass. **Commit** — `git commit -m "feat(core): trajectory generator, cascaded position/velocity control and soft limits"`

### Task 7: Axis core — homing against the mechanical end stop

**Files:**
- Modify: `components/axis_core/src/axis.c`
- Test: `host/tests/test_axis_homing.c`

**Interfaces:**
- Produces: HOME command behaviour. Constants in `axis.h`: `AXIS_HOME_SETTLE_MS 300` (ignore stall detection right after start), `AXIS_HOME_STALL_MS 100`.

Algorithm:
```
on HOME (from DISABLED or READY, faults == 0): state = HOMING; homed = false; home_ms = stall_ms = 0;
    sp_pos = pos; sp_vel = vel; reset PIDs
HOMING tick:
    v_des = home_dir * home_vel; ramp sp_vel with max_acc; duty = vel_ff*sp_vel + pidc_update(vel_pid, sp_vel - vel)  (no position loop, no following-error check)
    home_ms++
    if home_ms > home_timeout_ms: fault HOMING
    if home_ms > AXIS_HOME_SETTLE_MS and (current_ma >= home_current_ma or fabsf(vel) < 0.2*home_vel): stall_ms++ else stall_ms = 0
    if stall_ms >= AXIS_HOME_STALL_MS:
        axis_set_home(a, raw - home_pos)       -> pos == home_pos at the stop
        state = READY; sp_kind = POSITION; target = home_pos (clamped by the generator into [pos_min,pos_max] => backs off)
        sp_pos = pos; sp_vel = 0; reset PIDs; duty = 0 this tick
```
The overcurrent fault must not trigger during homing as long as `home_current_ma < max_current_ma` and the stall is detected within 100 ms — keep `overcurrent_ms` (200) > `AXIS_HOME_STALL_MS`.

- [ ] **Step 1: Failing tests** — `host/tests/test_axis_homing.c` (reuse the rig from Task 6 by copying `rig_t`, `cmd`, `sp`, `run_ms` into this file — each test file is standalone):
```c
static void test_homes_against_stop_and_backs_off(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.pos = 3000.0; r.p.has_stop = 1; r.p.stop_pos = -2500.0; r.p.stop_dir = -1;  /* home_dir = -1 */
    cmd(&r, PROTO_CMD_HOME);
    TT_CHECK(r.a.state == AXIS_HOMING);
    run_ms(&r, 5000);
    TT_CHECK(r.a.homed && r.a.state == AXIS_READY);
    /* the stop defines home_pos (-21000); soft limit pos_min = -20000 is 1000 counts further in */
    run_ms(&r, 2000);
    TT_NEAR(r.a.pos, TEST_CFG.pos_min, 10);
    TT_NEAR(r.p.pos, -2500.0 + 1000.0, 10);
    TT_CHECK(r.a.faults == 0);
}

static void test_homing_times_out_without_stop(void) {
    axis_config_t cfg = TEST_CFG; cfg.home_timeout_ms = 1000;
    rig_t r; rig_init(&r, &cfg);
    cmd(&r, PROTO_CMD_HOME);
    run_ms(&r, 1100);
    TT_CHECK(r.a.state == AXIS_FAULT && (r.a.faults & AXIS_FAULT_HOMING) && !r.a.homed);
}

static void test_home_rejected_while_faulted(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    can_frame_t f; proto_encode_estop(&f); axis_on_frame(&r.a, &f);
    cmd(&r, PROTO_CMD_HOME);
    TT_CHECK(r.a.state == AXIS_FAULT);
}

static void test_rehoming_from_ready(void) {
    rig_t r; rig_init(&r, &TEST_CFG);
    r.p.has_stop = 1; r.p.stop_pos = -2500.0; r.p.stop_dir = -1;
    cmd(&r, PROTO_CMD_HOME); run_ms(&r, 5000);
    TT_CHECK(r.a.homed);
    cmd(&r, PROTO_CMD_HOME);
    TT_CHECK(r.a.state == AXIS_HOMING && !r.a.homed);
    run_ms(&r, 5000);
    TT_CHECK(r.a.homed && r.a.state == AXIS_READY);
}
```
(plus `main` running all four).
- [ ] **Step 2: Run** — fails. **Step 3: Implement.** **Step 4: Run** `make test` → pass.
- [ ] **Step 5: Commit** — `git commit -m "feat(core): homing against the mechanical end stop with stall detection"`

---

# Phase 2 — Motor model and validation against the bench data

### Task 8: DC motor model (C)

**Files:**
- Create: `host/sim/motor_model.h`, `host/sim/motor_model.c`
- Test: `host/tests/test_motor_model.c`
- Modify: `host/CMakeLists.txt` — add `add_library(simcore STATIC sim/motor_model.c)` (later `sim/bench.c`, `sim/simaxis.c`), `target_include_directories(simcore PUBLIC sim)`, link `axis_core m`; tests link `simcore`.

**Interfaces:**
- Produces:
```c
typedef struct {
    float R, L, kt;            /* ohm, henry, Nm/A (ke == kt in SI) */
    float supply_v;
    float gear_ratio, gear_efficiency;
    float counts_per_motor_rev;/* quadrature counts = 4 x encoder lines */
    float sense_mv_per_a, adc_max_mv;
} motor_params_t;
typedef struct { motor_params_t p; float i; } motor_t;
void    motor_init(motor_t *m, const motor_params_t *p);
float   motor_step(motor_t *m, float duty, float omega_motor, float dt);  /* returns shaft torque Nm */
int32_t motor_encoder_count(const motor_t *m, double motor_angle_rad);
float   motor_sensed_current_ma(const motor_t *m, float duty);
```
Model: `V = duty * supply_v`; `i_ss = (V - kt*omega)/R`; exact first-order update `i = i_ss + (i - i_ss) * expf(-dt * R / L)` (stable for any dt); torque `kt * i`. Duty 0 means both low-side switches on (bdc_motor brake), i.e. a shorted winding → braking current. Sensed current: `0` if `|duty| < 1e-3` (OCM mirrors only while the bridge drives), else `min(|i| * sense_mv_per_a, adc_max_mv) / sense_mv_per_a * 1000`. Encoder: `floor(angle / 2π * counts_per_motor_rev)`.

- [ ] **Step 1: Failing tests**:
```c
#include <math.h>
#include "motor_model.h"
#include "tinytest.h"

static motor_params_t P = {4.0f, 0.0004f, 0.035f, 24.0f, 100.0f, 0.7f, 512.0f, 528.0f, 2500.0f};

static void test_stall_current_and_torque(void) {
    motor_t m; motor_init(&m, &P);
    float tq = 0; for (int i = 0; i < 100; i++) tq = motor_step(&m, 1.0f, 0.0f, 1e-4f);
    TT_NEAR(m.i, 6.0f, 1e-3);            /* 24 V / 4 ohm */
    TT_NEAR(tq, 0.21f, 1e-4);
}

static void test_no_load_speed_zero_current(void) {
    motor_t m; motor_init(&m, &P);
    float omega = 24.0f / 0.035f;       /* back-EMF equals supply */
    for (int i = 0; i < 100; i++) motor_step(&m, 1.0f, omega, 1e-4f);
    TT_NEAR(m.i, 0.0f, 1e-3);
}

static void test_electrical_time_constant(void) {
    motor_t m; motor_init(&m, &P);
    motor_step(&m, 1.0f, 0.0f, 0.0001f);  /* tau = L/R = 100 us */
    TT_NEAR(m.i, 6.0f * (1.0f - expf(-1.0f)), 1e-3);
}

static void test_large_dt_is_stable(void) {
    motor_t m; motor_init(&m, &P);
    motor_step(&m, 1.0f, 0.0f, 0.01f);
    TT_NEAR(m.i, 6.0f, 1e-3);
}

static void test_brake_current_opposes_motion(void) {
    motor_t m; motor_init(&m, &P);
    for (int i = 0; i < 100; i++) motor_step(&m, 0.0f, 100.0f, 1e-4f);
    TT_CHECK(m.i < 0.0f);
    TT_NEAR(motor_sensed_current_ma(&m, 0.0f), 0.0f, 1e-6);   /* not visible on OCM */
}

static void test_sensed_current_magnitude_and_clip(void) {
    motor_t m; motor_init(&m, &P);
    m.i = -1.0f; TT_NEAR(motor_sensed_current_ma(&m, -0.5f), 1000.0f, 0.5f);
    m.i = 10.0f; TT_NEAR(motor_sensed_current_ma(&m, 1.0f), 2500.0f / 528.0f * 1000.0f, 0.5f);
}

static void test_encoder_quantization(void) {
    motor_t m; motor_init(&m, &P);
    TT_CHECK(motor_encoder_count(&m, 2.0 * M_PI) == 512);
    TT_CHECK(motor_encoder_count(&m, -0.001) == -1);
}
```
- [ ] **Step 2–4:** run (fail) → implement → `make test` (pass).
- [ ] **Step 5: Commit** — `git commit -m "feat(sim): DC motor electrical model with current-sense and encoder emulation"`

### Task 9: Bench model, step-response fit against `output.txt`

**Files:**
- Create: `host/sim/bench.h`, `host/sim/bench.c`, `host/sim/simlib_api.c` (exports for ctypes; extended in Task 10), shared library target `simaxis` in `host/CMakeLists.txt` (`add_library(simaxis SHARED sim/simlib_api.c)` linking `simcore axis_core`, output `build/host/libsimaxis.dylib|.so`)
- Create: `pc/robotarm/sim/__init__.py`, `pc/robotarm/sim/native.py`, `pc/robotarm/analysis/__init__.py`, `pc/robotarm/analysis/steptest.py`
- Create: `config/bench_identified.yaml` (output of the fit), `docs/img/stepfit.png`, `docs/simulator.md` (section "Motor model validation")
- Test: `host/tests/test_bench.c`, `pc/tests/test_steptest.py`

**Interfaces:**
- Produces (C):
```c
typedef struct { motor_params_t motor; float j_total;   /* kg m^2 at the motor shaft */
                 float b_viscous;                        /* Nm s/rad at the motor shaft */
                 float tau_coulomb; } bench_params_t;    /* Nm at the motor shaft */
typedef struct { bench_params_t p; motor_t m; double angle; float omega; } bench_t;
void bench_init(bench_t *b, const bench_params_t *p);
void bench_step(bench_t *b, float duty, float dt);   /* 10 substeps internally; Coulomb with stiction */
/* exported for ctypes: run a whole duty profile; outputs arrays of length n */
int bench_run(const bench_params_t *p, const float *duty, int n, float dt, int32_t *pos_out, float *omega_out, float *current_ma_out);
```
Stiction rule: if `|omega| < 1e-3` and `|motor torque| <= tau_coulomb` → `omega = 0`; else `J domega = T - b omega - tau_coulomb sign(omega)`, with a zero-crossing clamp (friction must not reverse the direction within one substep).
- Produces (Python):
  - `robotarm.sim.native`: `load_library() -> ctypes.CDLL` (search `ROBOTARM_SIMAXIS_LIB` env var, then `<repo>/build/host/libsimaxis.{dylib,so}`; raise `FileNotFoundError("libsimaxis not built -- run `make host`")`), ctypes mirrors `MotorParams`, `BenchParams`, function `run_bench(params: BenchParams, duty: np.ndarray, dt: float) -> BenchResult(pos, omega, current_ma)`.
  - `robotarm.analysis.steptest`: `StepData(t_s, pos, duty, current_raw)`; `load_step_csv(path) -> StepData` (columns `time_us,position,velocity,pwm_ticks,current`; duty = pwm/400; the file stores positions negated — keep them as recorded since the pwm was applied in reverse, i.e. treat recorded position as the response to +duty); `simulate(params, data) -> np.ndarray` (positions at the data sample times); `fit_bench(data, motor: MotorConfig, supply_v, counts_per_rev) -> (BenchParams, FitReport(rmse_counts, final_error_pct))` using `scipy.optimize.least_squares` over `(log j_total, log b, log tau_coulomb)` on the position trace; CLI `robotarm stepfit [csv] [--cpr 256] [--motor faulhaber_2657cr_12v] [--supply 12] [--plot docs/img/stepfit.png] [--out config/bench_identified.yaml]`.

Known facts about `output.txt` (commit b7c24a4): 1000 samples, `time_us = 200 * index`; pwm 0 → 400 (= 100 % duty, `BDC_MCPWM_DUTY_TICK_MAX` = 400) at sample 401, back to 0 at sample 801; recorded position rises 0 → ~903 counts while driven and coasts to 1010; steady speed ≈ 17800 counts/s (from the position slope); the recorded velocity column is counts-per-window × 1000 and is ignored. The axis/motor under test is not recorded — defaults assume `--cpr 256` (64-line encoder) and `faulhaber_2657cr_12v` at `--supply 12`; the fit absorbs errors into J/b/Tc. Deceleration after switch-off is faster than acceleration: the brake (shorted winding) plus Coulomb friction explains that — the model must reproduce it.

- [ ] **Step 1: Failing C test** `test_bench.c`: (a) with `tau_coulomb` above stall torque the rotor never moves; (b) with zero friction the steady speed at duty 1 equals `supply/kt` within 1 %; (c) after duty goes to 0 the rotor stops and never reverses direction.
- [ ] **Step 2: Failing Python tests** `pc/tests/test_steptest.py`:
```python
from pathlib import Path

import numpy as np
import pytest

from robotarm.analysis import steptest

REPO = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def data():
    return steptest.load_step_csv(REPO / "output.txt")


def test_load_step_csv(data):
    assert len(data.t_s) == 1000
    assert data.duty.max() == pytest.approx(1.0)
    assert np.argmax(data.duty > 0) == 401
    assert data.pos[-1] == 1010


def test_committed_identification_reproduces_the_recording(data):
    params = steptest.load_identified(REPO / "config" / "bench_identified.yaml")
    sim = steptest.simulate(params, data)
    final = data.pos[-1]
    assert abs(sim[-1] - final) / final < 0.05                     # end position within 5 %
    assert np.sqrt(np.mean((sim - data.pos) ** 2)) / final < 0.05   # whole trace within 5 % RMS
    # coast-down (brake + friction) must also match: position gained after switch-off
    off = 801
    assert abs((sim[-1] - sim[off]) - (data.pos[-1] - data.pos[off])) < 30
```
- [ ] **Step 3: Run** — fail.
- [ ] **Step 4: Implement** bench (C), `simlib_api.c` exporting `bench_run` with `__attribute__((visibility("default")))`, native.py, steptest.py (+ `load_identified`, `save_identified`), `stepfit` CLI.
- [ ] **Step 5: Fit** — `uv run robotarm stepfit output.txt --plot docs/img/stepfit.png --out config/bench_identified.yaml`. Inspect the plot (Read the PNG). If the 5 % criteria are not met, first try other `--cpr/--motor/--supply` combos (the README lists 64, 128 and 360 CPR encoders), and document the best-fitting assumption in `docs/simulator.md`. Do not loosen the thresholds.
- [ ] **Step 6: Write `docs/simulator.md`** section "Motor model validation": model equations, the fitted parameters, the plot, assumptions about which motor was on the bench, and how to redo this with `robotarm identify` (Task 18) once the robot is back.
- [ ] **Step 7: Run** `make test` → pass. **Commit** — `git commit -m "feat(sim): bench motor model fitted and validated against recorded step response"`

---

# Phase 3 — Arm simulator

### Task 10: `simaxis` — axis core + motor model as a native sim node

**Files:**
- Create: `host/sim/simaxis.h`, `host/sim/simaxis.c`; extend `host/sim/simlib_api.c`
- Modify: `pc/robotarm/sim/native.py` (add `NativeAxis`)
- Test: `host/tests/test_simaxis.c`, `pc/tests/test_native_axis.py`

**Interfaces:**
- Produces (C, exported):
```c
typedef struct simaxis simaxis_t;
simaxis_t *simaxis_create(uint8_t node_id, const motor_params_t *motor);   /* NULL if node unknown */
void   simaxis_destroy(simaxis_t *s);
/* Advance n 1-kHz ticks. joint_q (rad) / joint_qd (rad/s) come from the physics engine and are
   held constant during the call (q is extrapolated with qd inside). Returns the mean joint torque (Nm). */
double simaxis_step(simaxis_t *s, int n_ticks, double joint_q, double joint_qd);
void   simaxis_rx(simaxis_t *s, uint16_t id, uint8_t len, const uint8_t *data);
int    simaxis_tx(simaxis_t *s, uint16_t *id, uint8_t *len, uint8_t *data);  /* 1 if a frame was popped */
typedef struct { float duty, current_ma, motor_torque; int32_t pos; float vel; uint8_t state, faults, homed; } simaxis_debug_t;
void   simaxis_get_debug(const simaxis_t *s, simaxis_debug_t *out);
```
Per tick: `motor_angle = joint_q * gear_ratio` (+ `joint_qd * gear_ratio * t` within the call); `encoder_raw = encoder_sign * motor_encoder_count(...)`; run `axis_tick`; the hardware sees `duty_hw = out.duty`, and the motor voltage direction is `motor_sign * duty_hw` (the simulated wiring matches the configured signs, so positive commands move the joint positively); 10 electrical substeps of 100 µs with `omega_motor = joint_qd * gear_ratio`; `joint_torque = kt * i * gear_ratio * gear_efficiency * motor_sign`; sensed current averaged over the tick feeds the next tick's `current_ma`.
- Produces (Python): `NativeAxis(node: int, cfg: ArmConfig)` with `.step(n_ticks, q, qd) -> float`, `.send(msg: can.Message)`, `.recv_all() -> list[can.Message]`, `.debug() -> SimAxisDebug` (dataclass mirror).

- [ ] **Step 1: Failing C test** — create axis node 2 with a pure-inertia 1-DOF loop inside the test (`J_joint = j_rotor*N^2 + 0.05`, explicit Euler at 1 ms): send ENABLE + DUTY 0.3 → joint moves positive; send heartbeats; after 1 s joint velocity > 0; STATUS frames come out of `simaxis_tx` with node 2.
- [ ] **Step 2: Failing Python test** — the same in Python via `NativeAxis` plus: without heartbeats the node reports `AxisState.FAULT` with `Fault.WATCHDOG` in its STATUS after 250 ms.
- [ ] **Step 3–4:** implement → `make test` pass.
- [ ] **Step 5: Commit** — `git commit -m "feat(sim): simaxis native node wrapping axis core and motor model"`

### Task 11: MuJoCo model of the arm generated from the config

**Files:**
- Create: `pc/robotarm/sim/model.py`
- Test: `pc/tests/test_model.py`

**Interfaces:**
- Produces: `build_mjcf(cfg: ArmConfig) -> str` and `load_model(cfg) -> mujoco.MjModel`. Joint names `j1..j6` (from `axis.joint`), plus a passive `j6_mirror` coupled to `j6` by an `<equality><joint joint1="j6_mirror" joint2="j6" polycoef="0 -1 0 0 0"/>`. Site `tcp` at the gripper tip. `JOINT_NAMES = ["j1", ..., "j6"]`.

Model details: `<option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"/>`; chain: fixed base (cylinder, `base_height`) → `link1` (J1 hinge axis z) → `link2` at z=`base_height` (J2 hinge axis y, capsule of length `upper_arm` along +z) → `link3` at z=`upper_arm` (J3, y) → `link4` at z=`forearm` (J4, y, capsule `wrist`) → `link5` at z=`wrist` (J5 hinge axis z) → gripper body with two finger bodies (J6 and mirror, hinge axis x) and site `tcp` at z=`gripper`. Each joint: `range` = hard limits (radians, `limited="true"`), `armature = j_rotor * gear_ratio**2`, `damping = friction.viscous_nm_s`, `frictionloss = friction.coulomb_nm`. Link geoms get `mass = link_mass_kg`. `<contact>` excludes all arm-arm pairs (use `contype="0" conaffinity="0"` on arm geoms; a floor plane with contype 1 for looks). Add a light and a camera looking at the arm for the viewer.

- [ ] **Step 1: Failing tests**:
```python
import math

import mujoco
import numpy as np
import pytest

from robotarm.config import load_arm_config
from robotarm.sim.model import JOINT_NAMES, load_model


@pytest.fixture(scope="module")
def cfg():
    return load_arm_config()


@pytest.fixture(scope="module")
def model(cfg):
    return load_model(cfg)


def test_joints_and_limits(model, cfg):
    for axis, name in zip(cfg.axes, JOINT_NAMES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert jid >= 0
        assert model.jnt_range[jid] == pytest.approx(axis.hard_limits_rad, abs=1e-6)
        assert model.dof_armature[model.jnt_dofadr[jid]] == pytest.approx(axis.motor.j_rotor * axis.gear_ratio**2)


def test_tcp_height_at_zero_pose(model, cfg):
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    g = cfg.geometry
    tcp = data.site("tcp").xpos
    assert tcp[2] == pytest.approx(g.base_height + g.upper_arm + g.forearm + g.wrist + g.gripper, abs=1e-6)


def test_positive_shoulder_tilts_toward_plus_x(model):
    data = mujoco.MjData(model)
    data.joint("j2").qpos = math.radians(30)
    mujoco.mj_forward(model, data)
    assert data.site("tcp").xpos[0] > 0.1


def test_gravity_torque_on_shoulder_is_plausible(model, cfg):
    data = mujoco.MjData(model)
    data.joint("j2").qpos = math.pi / 2          # arm horizontal
    mujoco.mj_forward(model, data)
    tau = data.qfrc_bias[model.jnt_dofadr[model.joint("j2").id]]
    total_mass = sum(a.link_mass_kg for a in cfg.axes[1:])
    assert 0.5 < abs(tau) < 9.81 * total_mass * 0.6  # between tiny and "all mass at max reach"
```
- [ ] **Step 2–4:** run (fail) → implement → pass.
- [ ] **Step 5: Commit** — `git commit -m "feat(sim): MuJoCo arm model generated from arm.yaml"`

### Task 12: `SimWorld` + in-process `SimBus` + lockstep harness; tune gains

**Files:**
- Create: `pc/robotarm/sim/world.py`, `pc/robotarm/sim/harness.py`
- Modify: `config/arm.yaml` (tuned gains), regenerate `components/axis_core/src/config_table.c`
- Test: `pc/tests/test_world.py`

**Interfaces:**
- Produces:
```python
class SimWorld:
    def __init__(self, cfg: ArmConfig, initial_q: dict[str, float] | None = None): ...
    model: mujoco.MjModel; data: mujoco.MjData; axes: list[NativeAxis]; time: float  # seconds of sim time
    def step(self, n_ms: int = 1) -> None       # per ms: tau_i = axis_i.step(1, q_i, qd_i); data.qfrc_applied[dof_i] = tau_i; mj_step
    def deliver(self, msg: can.Message) -> None # to every axis (bus semantics: all nodes see all frames)
    def take_outgoing(self) -> list[can.Message]  # frames produced by axes since last call, msg.timestamp = sim time
    def joint_positions(self) -> np.ndarray     # rad, j1..j6
    def set_joint_positions(self, q: dict[str, float]) -> None

class SimBus(can.BusABC):                       # thread-safe in-process bus bound to a SimWorld
    def __init__(self, world: SimWorld, channel="sim", **kwargs): ...
    def send(self, msg, timeout=None) -> None   # world.deliver(msg)
    def _recv_internal(self, timeout) -> tuple[can.Message | None, bool]
    def pump(self) -> None                      # move world.take_outgoing() into the receive queue
```
`harness.py`:
```python
def run_lockstep(world: SimWorld, bus: SimBus, seconds: float, on_ms=None) -> None:
    """Advance sim time deterministically. Every ms: world.step(1); bus.pump(); on_ms(world.time) if given.
    `on_ms` is where tests call ArmClient.poll(now) (Task 14) or send frames themselves."""
```
Gain tuning procedure (write it as a script `pc/robotarm/sim/tune.py` with CLI `robotarm tune --axis NAME` that prints overshoot, settle time and steady error for a 20° position step from the homed-and-backed-off pose, using lockstep): per axis, raise `vel_kp` until the velocity step response overshoots > 10 % then take 50 %; set `vel_ki` so the integral time `vel_kp/vel_ki` ≈ 20–50 ms; set `pos_kp` for ≈ 1–2 % overshoot. Record the tuned numbers in `config/arm.yaml` and regenerate C (`robotarm gen-config`), then rerun `make test` (the C tests use `TEST_CFG` and are unaffected).

- [ ] **Step 1: Failing tests** `pc/tests/test_world.py` — send frames directly (no ArmClient yet):
```python
import math

import pytest

from robotarm import protocol as p
from robotarm.config import load_arm_config
from robotarm.sim.harness import run_lockstep
from robotarm.sim.world import SimBus, SimWorld


@pytest.fixture
def cfg():
    return load_arm_config()


def heartbeat(bus):
    seq = [0]

    def on_ms(t):
        if round(t * 1000) % 50 == 0:
            bus.send(p.encode_heartbeat(seq[0] & 0xFF))
            seq[0] += 1
    return on_ms


def latest_status(bus, node):
    status = None
    while (msg := bus.recv(timeout=0)) is not None:
        d = p.decode(msg)
        if isinstance(d, p.Status) and d.node == node:
            status = d
    return status


def test_axes_report_disabled_after_power_up(cfg):
    world = SimWorld(cfg)
    bus = SimBus(world)
    run_lockstep(world, bus, 1.0)
    for node in range(1, 7):
        assert latest_status(bus, node).state == p.AxisState.DISABLED  # drains the queue per node; fine for a check


def test_all_axes_home_and_back_off(cfg):
    start = {a.joint: a.home.position_rad - a.home.direction * math.radians(5) for a in cfg.axes}
    world = SimWorld(cfg, initial_q=start)
    bus = SimBus(world)
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.HOME))
    run_lockstep(world, bus, 8.0, on_ms=heartbeat(bus))
    q = world.joint_positions()
    for i, a in enumerate(cfg.axes):
        edge = a.soft_limits_rad[0] if a.home.direction < 0 else a.soft_limits_rad[1]
        assert q[i] == pytest.approx(edge, abs=math.radians(1.0)), a.name


def test_position_move_on_shoulder(cfg):
    shoulder = cfg.axis_by_name("shoulder")
    start = {a.joint: a.home.position_rad - a.home.direction * math.radians(5) for a in cfg.axes}
    world = SimWorld(cfg, initial_q=start)
    bus = SimBus(world)
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.HOME))
    run_lockstep(world, bus, 8.0, on_ms=heartbeat(bus))
    target = math.radians(30)
    bus.send(p.encode_setpoint(shoulder.node, p.SetpointKind.POSITION, shoulder.rad_to_counts(target)))
    run_lockstep(world, bus, 6.0, on_ms=heartbeat(bus))
    assert world.joint_positions()[1] == pytest.approx(target, abs=math.radians(0.5))
    st = latest_status(bus, shoulder.node)
    assert st.state == p.AxisState.READY and st.faults == 0


def test_axes_fault_when_heartbeats_stop(cfg):
    world = SimWorld(cfg)
    bus = SimBus(world)
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.ENABLE))
    run_lockstep(world, bus, 0.5)
    st = latest_status(bus, 3)
    assert st.state == p.AxisState.FAULT and p.Fault.WATCHDOG in st.faults
```
(`p.NODE_BROADCAST = 0` is added to `protocol.py` if not already there.) `latest_status` drains the whole receive queue, so in `test_axes_report_disabled_after_power_up` collect all statuses once into a dict keyed by node instead of calling it six times. A DISABLED axis outputs duty 0 = brake; the shoulder under gravity may creep slowly through the brake — assert only the state, not the position.
- [ ] **Step 2–4:** run (fail) → implement world/bus/harness → tune gains (`robotarm tune`) until the tests pass (the home test requires every axis to finish homing within 8 s from 5° before its stop).
- [ ] **Step 5: Commit** — `git commit -m "feat(sim): SimWorld coupling MuJoCo with six simulated axis nodes; tuned gains"`

### Task 13: TCP transport, real-time sim server, viewer, `robotarm sim`

**Files:**
- Create: `pc/robotarm/transport/__init__.py`, `pc/robotarm/transport/tcp_bus.py`, `pc/robotarm/bus.py`, `pc/robotarm/sim/server.py`
- Test: `pc/tests/test_tcp_bus.py`, `pc/tests/test_bus_url.py`

**Interfaces:**
- Produces:
  - Wire format: each frame is 13 bytes `struct.pack("<IB8s", arbitration_id, dlc, data.ljust(8, b"\0"))`.
  - `TcpBus(can.BusABC)`: `__init__(self, channel: str = "127.0.0.1:29536", connect_timeout=2.0, **kw)`; raises `SimNotRunningError(ConnectionError)` with message `"no simulator at {host}:{port} -- start it with `make sim`"`; `send`, `_recv_internal`, `shutdown`.
  - `SimServer(world, host="127.0.0.1", port=29536)`: accepts multiple clients (thread per client reading frames into a queue); `broadcast(msgs)`; frames from one client are delivered to the world only (clients do not see each other's frames — master is the only sender of commands).
  - `run_realtime(world, server, viewer: bool, stop_event)`: loop: compute wall-clock target sim time; step the world in 1 ms increments until caught up (max 50 per iteration; log a warning once per second if falling behind); deliver inbound frames before each step; broadcast `world.take_outgoing()` after; if viewer, `viewer.sync()` at ~60 Hz. Viewer via `mujoco.viewer.launch_passive(model, data)`; on macOS this requires running under `mjpython` — if `sys.platform == "darwin"` and not under mjpython, print `run with: uv run mjpython -m robotarm sim` and exit 2 (or use `--no-viewer`).
  - `robotarm.bus.open_bus(url: str) -> can.BusABC`: `tcp://host:port` → `TcpBus`; `sim` → a fresh in-process `SimWorld` + `SimBus` running `run_realtime` in a background thread without viewer; `slcan:/dev/tty...[@bitrate]`, `gs_usb:0`, `socketcan:can0`, `pcan:PCAN_USBBUS1` → `can.Bus(interface=..., channel=..., bitrate=1_000_000)`. Unknown scheme → `ValueError`. CLI commands catch `SimNotRunningError`/`can.CanError`/`OSError` from `open_bus`, print `error: <message>` to stderr and return 2.
  - CLI `robotarm sim [--host 127.0.0.1] [--port 29536] [--no-viewer] [--start near-home|zero]` (`near-home` = 5° before every home stop, default) and `robotarm monitor --bus URL` (prints one line per second with state/position/fault per axis).
- [ ] **Step 1: Failing tests**:
```python
# pc/tests/test_tcp_bus.py
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


def test_disconnecting_master_trips_watchdog(running_sim):
    _, server = running_sim
    bus = TcpBus(f"127.0.0.1:{server.port}")
    bus.send(p.encode_command(p.NODE_BROADCAST, p.Command.ENABLE))
    time.sleep(0.05)
    assert recv_status(bus, 4).state == p.AxisState.READY
    bus.shutdown()                             # master "crashes"
    time.sleep(0.4)
    bus2 = TcpBus(f"127.0.0.1:{server.port}")
    try:
        st = recv_status(bus2, 4)
        assert st.state == p.AxisState.FAULT and p.Fault.WATCHDOG in st.faults
    finally:
        bus2.shutdown()
```
```python
# pc/tests/test_bus_url.py
import subprocess
import sys

import pytest

from robotarm.bus import open_bus


def test_unknown_scheme():
    with pytest.raises(ValueError):
        open_bus("carrier-pigeon://x")


def test_monitor_without_sim_exits_2_with_one_line_error():
    r = subprocess.run([sys.executable, "-m", "robotarm", "monitor", "--bus", "tcp://127.0.0.1:1"],
                       capture_output=True, text=True, timeout=20)
    assert r.returncode == 2
    assert r.stderr.strip().startswith("error:") and "Traceback" not in r.stderr
```
- [ ] **Step 2–4:** fail → implement → pass.
- [ ] **Step 5: Manual smoke** — `uv run robotarm sim --no-viewer & sleep 2; uv run robotarm monitor --bus tcp://127.0.0.1:29536` shows 6 DISABLED axes; kill both.
- [ ] **Step 6: Docs** — `docs/simulator.md`: running the sim (with/without viewer, mjpython note), architecture diagram of SimWorld/SimServer, time handling (lockstep vs real-time).
- [ ] **Step 7: Commit** — `git commit -m "feat(sim): TCP sim server with real-time loop, viewer and bus URL factory"`

---

# Phase 4 — Master and PlayStation teleop

### Task 14: `ArmClient` — joint-level master API

**Files:**
- Create: `pc/robotarm/master/__init__.py`, `pc/robotarm/master/arm_client.py`
- Test: `pc/tests/test_arm_client.py`

**Interfaces:**
- Produces:
```python
@dataclass
class JointState:
    node: int; name: str
    position_rad: float = 0.0; velocity_rad_s: float = 0.0; current_a: float = 0.0
    state: AxisState = AxisState.DISABLED; faults: Fault = Fault(0); homed: bool = False
    last_seen: float | None = None        # timestamp of the last STATUS

class ArmClient:
    """Sans-IO core: feed frames with process(msg), call poll(now) periodically.
    The threaded runner (start/close) does both with wall-clock time."""
    def __init__(self, bus: can.BusABC, cfg: ArmConfig, heartbeat_hz: float = 20.0): ...
    joints: dict[int, JointState]          # by node
    def process(self, msg: can.Message) -> None
    def poll(self, now: float) -> None      # sends HEARTBEAT when due; drains bus.recv(timeout=0) into process()
    def start(self) -> None                 # background thread: poll(time.monotonic()) every 5 ms
    def close(self) -> None                 # stops thread; sends DISABLE broadcast; does not shut the bus
    def estop(self) -> None
    def enable(self, nodes: Iterable[int] | None = None) -> None     # None = broadcast
    def disable(self, nodes=None) -> None
    def home(self, nodes=None) -> None
    def clear_faults(self, nodes=None) -> None
    def set_velocity(self, node: int, rad_s: float) -> None          # converts with counts_per_rad
    def set_position(self, node: int, rad: float) -> None            # clamps to soft limits (rad) before sending
    def set_duty(self, node: int, duty: float) -> None
    def all_ready(self) -> bool; def all_homed(self) -> bool; def any_fault(self) -> bool
    def connected(self, now: float, stale_s: float = 0.5) -> bool    # every node sent STATUS within stale_s
```
- [ ] **Step 1: Failing tests** (lockstep with `SimWorld`/`SimBus`, calling `client.poll(t)` from `on_ms`): (a) heartbeats keep 6 enabled axes READY for 3 s; (b) `home()` then `all_homed()` is true within 8 s from the near-home start; (c) `set_position(2, 0.5)` reaches 0.5 rad ± 0.01; (d) `set_position(2, 10.0)` ends at the upper soft limit, no fault; (e) `estop()` → all FAULT with `Fault.ESTOP`, then `clear_faults()` + `enable()` → READY; (f) `connected()` false before any STATUS, true after 50 ms.
- [ ] **Step 2–4:** fail → implement → pass.
- [ ] **Step 5: Commit** — `git commit -m "feat(master): ArmClient joint-level API over any python-can bus"`

### Task 15: Gamepad input (PlayStation via SDL) with deadzone

**Files:**
- Create: `pc/robotarm/master/gamepad.py`, `config/teleop.yaml`
- Test: `pc/tests/test_gamepad.py`

**Interfaces:**
- Produces:
```python
@dataclass(frozen=True)
class GamepadState:
    connected: bool = False
    lx: float = 0.0; ly: float = 0.0; rx: float = 0.0; ry: float = 0.0   # [-1, 1], +y = stick pushed UP
    l2: float = 0.0; r2: float = 0.0                                     # [0, 1]
    buttons: frozenset[str] = frozenset()   # names: cross circle square triangle l1 r1 share options ps
                                            #        dpad_up dpad_down dpad_left dpad_right l3 r3
def apply_deadzone(x: float, y: float, deadzone: float) -> tuple[float, float]   # radial, rescaled to reach 1.0
class Gamepad:                         # real device via pygame._sdl2.controller (GameController API = PS-agnostic names)
    def __init__(self, deadzone: float = 0.12): ...
    def poll(self) -> GamepadState     # pumps SDL events; handles hot-plug (connect/disconnect) without raising
class FakeGamepad:                     # for tests and headless runs
    def __init__(self): self.state = GamepadState(connected=True)
    def set(self, **changes) -> None
    def poll(self) -> GamepadState
```
SDL controller mapping (SDL names → PlayStation names): `a→cross, b→circle, x→square, y→triangle, leftshoulder→l1, rightshoulder→r1, back→share, start→options, guide→ps, leftstick→l3, rightstick→r3, dpup/dpdown/dpleft/dpright→dpad_*`; axes `leftx, lefty, rightx, righty` (int16; SDL +y is DOWN → negate), `lefttrigger/righttrigger` (0..32767 → [0,1]). Initialise pygame with `os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")` so the pad keeps working when the terminal is not focused, and `pygame.display` is not required. If `pygame._sdl2.controller` is unavailable, fall back to `pygame.joystick` with the DualSense/DS4 raw index map in `config/teleop.yaml`.

`config/teleop.yaml`:
```yaml
deadzone: 0.12
loop_hz: 50
speed_scale: {normal: 0.5, slow: 0.15}      # fraction of each joint's max velocity; R1 held = slow
buttons:                                     # PlayStation names (see gamepad.py)
  deadman: l1
  slow: r1
  enable: cross
  estop: circle
  home: triangle
  toggle_mode: square
  clear_faults: options
joint_mode:                                  # stick/button -> joint; sign flips direction
  j1: {axis: lx, sign: -1}
  j2: {axis: ly, sign: -1}
  j3: {axis: ry, sign: 1}
  j5: {axis: rx, sign: 1}
  j4: {buttons: [dpad_up, dpad_down]}        # + / -
  j6: {triggers: [r2, l2]}                   # close (r2) / open (l2)
cartesian_mode:
  linear_speed_m_s: 0.05
  pitch_speed_rad_s: 0.5
raw_joystick_fallback:                       # DualSense on macOS without GameController support
  buttons: {cross: 0, circle: 1, square: 2, triangle: 3, share: 4, ps: 5, options: 6, l3: 7, r3: 8, l1: 9, r1: 10,
            dpad_up: 11, dpad_down: 12, dpad_left: 13, dpad_right: 14}
  axes: {lx: 0, ly: 1, rx: 2, ry: 3, l2: 4, r2: 5}
```
- [ ] **Step 1: Failing tests**:
```python
import pytest

from robotarm.master.gamepad import FakeGamepad, GamepadState, apply_deadzone


def test_small_stick_noise_is_zero():
    assert apply_deadzone(0.05, -0.08, 0.12) == (0.0, 0.0)


def test_deadzone_rescales_to_full_range():
    x, y = apply_deadzone(1.0, 0.0, 0.12)
    assert x == pytest.approx(1.0) and y == 0.0
    x, _ = apply_deadzone(0.56, 0.0, 0.12)
    assert x == pytest.approx(0.5, abs=1e-6)


def test_diagonal_is_radial_not_per_axis():
    x, y = apply_deadzone(0.1, 0.1, 0.12)       # |v| = 0.141 > 0.12 -> not zero
    assert x > 0 and y > 0


def test_fake_gamepad_set():
    pad = FakeGamepad()
    pad.set(lx=0.5, buttons=frozenset({"l1"}))
    s = pad.poll()
    assert s.connected and s.lx == 0.5 and "l1" in s.buttons


def test_real_gamepad_without_device_reports_disconnected():
    from robotarm.master.gamepad import Gamepad
    state = Gamepad().poll()       # CI/laptop without a pad: must not raise
    assert isinstance(state, GamepadState)
```
- [ ] **Step 2–4:** fail → implement → pass. Also add CLI `robotarm gamepad-test` printing the live `GamepadState` 10×/s (for the user to verify their controller mapping in the morning).
- [ ] **Step 5: Commit** — `git commit -m "feat(master): PlayStation gamepad input via SDL with radial deadzone"`

### Task 16: Teleop — joint jog mode, deadman, e-stop, gamepad loss, `robotarm teleop`

**Files:**
- Create: `pc/robotarm/master/teleop.py`
- Test: `pc/tests/test_teleop.py`

**Interfaces:**
- Consumes: `ArmClient` (Task 14), `GamepadState`, `FakeGamepad` (Task 15), `config/teleop.yaml`.
- Produces:
```python
class Mode(Enum): JOINT = "joint"; CARTESIAN = "cartesian"
@dataclass
class TeleopConfig: ...                   # loaded from config/teleop.yaml by load_teleop_config(path=None)
class Teleop:
    def __init__(self, client: ArmClient, arm_cfg: ArmConfig, cfg: TeleopConfig): ...
    mode: Mode; armed: bool               # armed = deadman pressed while connected, since last loss/release
    def update(self, pad: GamepadState, now: float) -> None
    def status_line(self) -> str          # one-line human readable status for the CLI
def run_teleop(bus_url: str, mode: str, gamepad=None) -> int   # CLI body; returns exit code
```
Rules for `update` (called at `loop_hz`):
1. Button *edges* (pressed now, not in the previous state): `estop` → `client.estop()` (works even when not armed); `enable` → `clear_faults()` is NOT implicit — `enable` sends `enable()`; `clear_faults` → `clear_faults()`; `home` → `home()`; `toggle_mode` → switch mode (only when not armed).
2. `armed` becomes true on a rising edge of `deadman` while `pad.connected`; becomes false when deadman is released or `pad.connected` is false. After a loss, the user must release and press deadman again.
3. Not armed → send `set_velocity(node, 0.0)` for all jog axes (every update, so the axis watchdog is also fed); do nothing else.
4. Armed + JOINT: per `joint_mode` entry: stick axis value × sign × joint max velocity × speed scale (`slow` when the slow button is held); dpad pair → ±0.5 of max; trigger pair → (r2 − l2) × max. Send `set_velocity` for every mapped joint each update.
5. Armed + CARTESIAN: see Task 17 (until then, CARTESIAN behaves as not armed).

CLI `robotarm teleop --bus URL [--mode joint|cartesian] [--fake-gamepad]`: opens the bus (errors → `error: ...`, exit 2), starts `ArmClient`, loops at `loop_hz` calling `Gamepad.poll()` + `Teleop.update()`, reprints `status_line()` in place (`\r`), Ctrl-C → `client.close()` (DISABLE) and exit 0.

- [ ] **Step 1: Failing tests** (lockstep sim + `ArmClient` + `FakeGamepad`; helper that calls `teleop.update(pad.poll(), t)` every 20 ms inside `on_ms`, and `client.poll(t)` every ms):
  - `test_no_motion_without_deadman`: enabled + homed, `ly=1.0` without l1 for 1 s → shoulder moves < 0.2°.
  - `test_deadman_jogs_shoulder_up`: l1 + `ly=1.0` for 1 s → shoulder moved positive by > 10°.
  - `test_releasing_deadman_stops`: after jogging, release l1 → within 0.5 s shoulder velocity < 1°/s.
  - `test_gamepad_disconnect_stops_and_requires_rearm`: while jogging set `connected=False` → stops within 0.5 s; set `connected=True` with l1 still held and stick deflected → still no motion for 1 s; release and press l1 → motion resumes.
  - `test_circle_is_estop_even_when_not_armed`: press circle → all axes FAULT with ESTOP.
  - `test_triggers_drive_gripper`: l1 + r2=1 → j6 moves toward its "close" direction (positive velocity per mapping) and stops at the soft limit without fault.
  - `test_teleop_cli_without_sim_exits_2`: subprocess `robotarm teleop --bus tcp://127.0.0.1:1 --fake-gamepad` → exit code 2, stderr starts with `error:`.
- [ ] **Step 2–4:** fail → implement → pass.
- [ ] **Step 5: Docs** — `docs/teleop.md`: controller layout diagram (ASCII) with the button map, the safety rules, how to pair a DualSense/DS4 on macOS (System Settings → Bluetooth, hold PS + Create/Share until the light bar flashes), `robotarm gamepad-test`, and running `make sim` + `make teleop`.
- [ ] **Step 6: Commit** — `git commit -m "feat(master): PlayStation teleop with deadman, e-stop and gamepad-loss handling"`

### Task 17: Kinematics and Cartesian jog mode

**Files:**
- Create: `pc/robotarm/master/kinematics.py`
- Modify: `pc/robotarm/master/teleop.py`
- Test: `pc/tests/test_kinematics.py`, extend `pc/tests/test_teleop.py`

**Interfaces:**
- Produces:
```python
@dataclass(frozen=True)
class Pose:  x: float; y: float; z: float; pitch: float; roll: float   # metres; pitch = angle of the gripper from vertical (q2+q3+q4); roll = q5
class Kinematics:
    def __init__(self, cfg: ArmConfig): ...
    def forward(self, q: Sequence[float]) -> Pose            # q = [q1..q5] (gripper ignored)
    def inverse(self, pose: Pose, q_seed: Sequence[float]) -> list[float] | None   # None if unreachable or outside soft limits
```
Equations (q = 0 is straight up; L2 = upper_arm, L3 = forearm, L4 = wrist + gripper, h = base_height):
```
r = L2 sin q2 + L3 sin(q2+q3) + L4 sin(q2+q3+q4)
z = h + L2 cos q2 + L3 cos(q2+q3) + L4 cos(q2+q3+q4)
x = r cos q1 ; y = r sin q1 ; pitch = q2+q3+q4 ; roll = q5
IK: q1 = atan2(y, x) (if r < 1e-6 keep seed q1); r = hypot(x, y)
    rw = r - L4 sin(pitch) ; zw = z - h - L4 cos(pitch)
    D = (rw² + zw² - L2² - L3²) / (2 L2 L3) ; |D| > 1 → None
    q3 = ±acos(D)  (try both; pick the one closest to seed q3)
    q2 = atan2(rw, zw) - atan2(L3 sin q3, L2 + L3 cos q3)
    q4 = pitch - q2 - q3 ; q5 = roll ; any joint outside soft limits → None
```
Cartesian teleop (armed + CARTESIAN): hold a target `Pose` initialised from FK of the measured joints when entering the mode / arming. Each update integrate: `ly` → radial in/out (along the direction of q1), `lx` → q1 rotation (target pose rotated about z), `ry` → z, `rx` → roll, dpad up/down → pitch, at `cartesian_mode` speeds × speed scale. Compute IK with seed = current measured joints; if None, keep the previous target (Review Focus #3); otherwise send `set_position` for j1..j5 each update. Gripper triggers work as in joint mode (velocity).

- [ ] **Step 1: Failing tests** — FK at zero pose equals `(0, 0, h+L2+L3+L4, 0, 0)`; FK matches the MuJoCo `tcp` site for 50 random joint vectors inside soft limits (tolerance 1 mm; `wrist+gripper` = L4 must match the model); IK(FK(q)) ≈ q for 200 random reachable q (seed = q + small noise), and `forward(inverse(p))` ≈ p; unreachable pose (x = 2 m) → None; pose needing a joint outside soft limits → None. Teleop test: Cartesian mode, l1 + `ry=1` for 1 s → TCP z (from MuJoCo) rises > 2 cm while x/y change < 5 mm; pushing radial outward for 10 s stops at the workspace edge with no axis fault.
- [ ] **Step 2–4:** fail → implement → pass.
- [ ] **Step 5: Commit** — `git commit -m "feat(master): forward/inverse kinematics and Cartesian jog mode"`

### Task 18: `robotarm identify` — bench step experiment over CAN

**Files:**
- Create: `pc/robotarm/master/identify.py`
- Test: `pc/tests/test_identify.py`

**Interfaces:**
- Produces: `class IdentifyRun: __init__(self, client: ArmClient, node: int, duty: float = 1.0, pre_s=0.08, on_s=0.08, post_s=0.04)`; `update(now) -> bool` (True when finished; state machine: enable → duty 0 for `pre_s` → `duty` for `on_s` → 0 for `post_s` → disable); `samples: list[(t, pos_counts, vel_counts_s, duty, current_ma)]` recorded from every STATUS/TELEMETRY pair of that node (1 kHz in duty mode); `write_csv(path)` in the same column format as `output.txt` (`time_us,position,velocity,pwm_ticks,current` with pwm_ticks = duty × 400) so `robotarm stepfit` can read it. CLI `robotarm identify --bus URL --node N [--duty 1.0] [--out FILE]`. Refuses (exit 2, `error:` line) if the axis is not DISABLED/READY or if `--duty` exceeds the axis `max_duty`.
- [ ] **Step 1: Failing test** — lockstep sim, node 2 starting mid-range: run the experiment; the CSV has ≥ 180 rows; position increases during the on-phase and is flat (±2 counts) during the pre-phase; `steptest.load_step_csv` reads it back.
- [ ] **Step 2–4:** fail → implement → pass.
- [ ] **Step 5: Commit** — `git commit -m "feat(master): identify command running open-loop step tests over CAN"`

---

# Phase 5 — ESP32 firmware shell

### Task 19: ESP32 HAL + axis task on top of `axis_core`, TWAI, console move

**Files:**
- Create: `main/board.h`, `main/hal_motor.{c,h}`, `main/hal_encoder.{c,h}`, `main/hal_current.{c,h}`, `main/hal_can.{c,h}`, `main/axis_task.{c,h}`, `main/Kconfig.projbuild`, `sdkconfig.defaults`
- Modify: `main/main.c`, `main/CMakeLists.txt`, `main/idf_component.yml` (drop `espressif/pid_ctrl`), `sdkconfig` (regenerated by the build), `dependencies.lock`
- Delete: `main/motor_pid.{c,h}`, `main/motion_control.{c,h}`, `main/adc_continuous_read.{c,h}`, `main/storage.{c,h}`, `main/main.h`

**Interfaces:**
- Consumes: `axis_core` (`axis_init/axis_tick/axis_on_frame/axis_pop_tx`, `axis_config_for_node`).
- Produces:
```c
/* board.h -- pinout of the bcbergmanuu/dc-motor-driver PCB (XIAO ESP32-S3) */
#define BOARD_PWM_A_GPIO        7    /* MOT_PWM2 */
#define BOARD_PWM_B_GPIO        8    /* MOT_PWM1 */
#define BOARD_ENC_A_GPIO        1    /* ENCODER_B net */
#define BOARD_ENC_B_GPIO        9    /* ENCODER_A net */
#define BOARD_CURRENT_ADC_UNIT  ADC_UNIT_1
#define BOARD_CURRENT_ADC_CH    ADC_CHANNEL_3   /* GPIO4, MOT_OCM */
#define BOARD_CAN_TX_GPIO       43
#define BOARD_CAN_RX_GPIO       44
#define BOARD_OCC_GPIO          2    /* TB9051 OCC: left unconfigured (see docs/bringup.md) */
#define BOARD_PWM_FREQ_HZ       25000
#define BOARD_PWM_RES_HZ        10000000
#define BOARD_CURRENT_MV_PER_A  528.0f   /* mirrors current_sense.mv_per_a in config/arm.yaml */

void  hal_motor_init(void);                 void hal_motor_set_duty(float duty);   /* [-1,1], 0 = brake */
void  hal_encoder_init(void);               int32_t hal_encoder_read(void);        /* accumulated, never wraps at ±limit */
void  hal_current_init(void);               float hal_current_ma(void);            /* latest averaged, lock-free */
void  hal_can_init(void);                   bool hal_can_recv(can_frame_t *f);     bool hal_can_send(const can_frame_t *f);
void  axis_task_start(uint8_t node_id);
```
Implementation notes:
- `hal_encoder`: PCNT unit limits ±10000 with watch points at both limits and `flags.accum_count = 1` so `pcnt_unit_get_count` returns the accumulated count; glitch filter 1000 ns; same edge/level actions as the existing code.
- `hal_current`: ADC continuous 20 kHz on ADC1 CH3 with `ADC_ATTEN_DB_12`; conversion-done callback notifies a small task that averages the frame, converts raw → mV with `adc_cali` (curve fitting scheme), → mA with `BOARD_CURRENT_MV_PER_A`, stores into a `volatile float`.
- `hal_can`: IDF 6.1 TWAI node API (`esp_twai.h`, `esp_twai_onchip.h`: `twai_new_node_onchip`, `twai_node_enable`, `twai_node_transmit`, rx via `on_rx_done` callback + `twai_node_receive_from_isr` into a FreeRTOS queue). 1 Mbit/s. Read the headers in the container (`/opt/esp/idf/components/esp_driver_twai/include/`) for exact names.
- `axis_task`: `gptimer` at 1 kHz → ISR `vTaskNotifyGiveFromISR` → task (priority `configMAX_PRIORITIES-2`, pinned to core 1): drain `hal_can_recv` into `axis_on_frame`; `axis_tick` with `{hal_encoder_read(), hal_current_ma()}`; `hal_motor_set_duty(out.duty)`; drain `axis_pop_tx` into `hal_can_send` (non-blocking).
- `Kconfig.projbuild`: `menu "Robot arm axis"`, `config AXIS_NODE_ID int "CAN node id of this axis (1-6)" range 1 6 default 1`. `main.c` looks up `axis_config_for_node(CONFIG_AXIS_NODE_ID)` and aborts with a log error if NULL.
- Console on USB-Serial/JTAG so GPIO43/44 are free for CAN: `sdkconfig.defaults` with `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` and `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`; also update `sdkconfig` accordingly (delete the conflicting UART console lines and let the build regenerate).
- `main/CMakeLists.txt`: `REQUIRES axis_core esp_driver_mcpwm esp_driver_pcnt esp_driver_gptimer esp_driver_twai esp_adc esp_timer` (no `bt`, no `nvs_flash`).

- [ ] **Step 1: Write the HAL modules, axis_task, main.c, Kconfig, board.h; delete the replaced files.**
- [ ] **Step 2: Build** — `scripts/idf.sh build 2>&1 | tail -20` → `Project build complete`, no warnings from `main/` or `components/axis_core` (fix any).
- [ ] **Step 3: Confirm console config** — `grep -E "^CONFIG_ESP_CONSOLE_(USB_SERIAL_JTAG|UART)=" sdkconfig` → only `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`.
- [ ] **Step 4: Host tests still pass** — `make test`.
- [ ] **Step 5: Commit** — `git commit -m "feat(firmware): ESP32 shell on axis_core with TWAI CAN; console moved to USB-JTAG"`

### Task 20: Documentation, bring-up guide, final verification

**Files:**
- Create: `docs/bringup.md`
- Modify: `README.md` (add "Software" section: architecture summary, pinout table, quick start, links to docs), `docs/architecture.md` (final state)

- [ ] **Step 1: `docs/bringup.md`** — step-by-step for when the robot is back: hardware shopping list (USB-CAN adapter such as CANable 2.0, 120 Ω termination at both bus ends, CAN wiring), flashing each board with its node id (`scripts/idf.sh menuconfig` → Robot arm axis → node id, or `-DCONFIG...` via `sdkconfig.defaults`), first power-up checklist (motor disconnected; `robotarm monitor`), sign check per axis (`identify` with small duty; if position counts go negative flip `encoder_sign`/`motor_sign` in `arm.yaml`, regenerate, reflash), current-sense calibration (compare `current_ma` with a bench supply reading), running `identify` + `stepfit` per axis and updating `config/arm.yaml` motor/friction values, homing tuning (thresholds), gain retune with the real motors, then teleop at `slow` speed with a hand on the E-stop. Include the full "(assumed)" values list from `config/arm.yaml` as a "measure on robot" checklist, and the open hardware questions: OCC pin behaviour, OCM magnitude-only, ADC attenuation, whether duty 0 should brake or coast.
- [ ] **Step 2: README** — keep the existing hardware content; add the software section with the pinout table (from Task 19 `board.h`) and quick start: `make test`, `make sim`, `make teleop`, `robotarm gamepad-test`.
- [ ] **Step 3: Full verification** — `make test` (all green), `scripts/idf.sh build` (complete), headless sim smoke test (`robotarm sim --no-viewer` + `robotarm teleop --fake-gamepad --bus tcp://…` for 3 s, exits 0 on SIGINT).
- [ ] **Step 4: Commit** — `git commit -m "docs: bring-up guide, README software section"`

---

## Self-review notes

- Spec coverage: core/HAL split (T2–T7, T19), host build + tests (T1–T8), motor model validated vs `output.txt` (T8–T9), CAN protocol (T3), simulator (T10–T13), master + PS controller (T14–T17), identification over CAN (T18), safety in firmware (T5–T7) and master (T16), ESP32 bring-up prep incl. TWAI + console conflict (T19–T20).
- Review Focus → tests: #1 T5 `test_watchdog_trips_without_master`, T13 `test_disconnecting_master_trips_watchdog`; #2 T16 `test_gamepad_disconnect_stops_and_requires_rearm`; #3 T6 soft-limit tests, T14 (d), T17 workspace-edge test; #4 T15 deadzone tests; #5 T13 `test_monitor_without_sim_exits_2_with_one_line_error`, T16 `test_teleop_cli_without_sim_exits_2`.
- Names used across tasks: `axis_config_t`, `axis_t`, `pidc_t`, `can_frame_t`, `proto_*`, `NativeAxis`, `SimWorld`, `SimBus`, `run_lockstep`, `TcpBus`, `SimServer`, `run_realtime`, `open_bus`, `ArmClient`, `JointState`, `GamepadState`, `FakeGamepad`, `Teleop`, `Kinematics`, `Pose`, `IdentifyRun` — each is defined in exactly one task's Interfaces block.
