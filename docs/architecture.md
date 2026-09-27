# Architecture

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
5. **Safety lives in the axis**: command watchdog (200 ms), soft limits, following-error fault, overcurrent fault (not counted during the first 500 ms of HOMING, so an axis that starts pressed against its stop is recognised as stalled first), broadcast E-STOP. The master adds deadman (L1) and gamepad-loss handling.
6. **Simulator:** MuJoCo for rigid-body arm dynamics (gravity, joint friction, mechanical stops as joint limits, reflected rotor inertia as joint `armature`); the DC motor electrical model and the six axis cores run in C (`libsimaxis`) called once per 1 ms step via ctypes.
7. **Sim ↔ master transport:** in-process `SimBus` (lockstep, deterministic — used by tests) and a TCP bus (`TcpBus`, 13-byte frames) so the simulator (with viewer) and the teleop run as separate processes. On the robot the master uses `slcan`/`gs_usb`/`socketcan` through `python-can`.
8. **Firmware bench identification** (open-loop duty step, hard-coded in the original firmware's `motor_pid.c`, since deleted along with the rest of the pre-rewrite `main/` in favour of the HAL split below) becomes a protocol feature: `SETPOINT kind=DUTY`, telemetry at 1 kHz in duty mode, recorded by `robotarm identify` on sim or robot.

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

See `docs/protocol.md` for the full protocol reference (message framing, timing guarantees, and worked examples) once it lands.

### Repository layout (final state)

```
CMakeLists.txt                  ESP-IDF project (unchanged role; VS Code IDF setup keeps working)
main/                            ESP32 shell (each board flashed with its own CONFIG_AXIS_NODE_ID)
  board.h                          pinout (main/board.h, see the README's Software section for the table)
  Kconfig.projbuild                CONFIG_AXIS_NODE_ID (1-6)
  hal_motor.{c,h}                  TB9051FTG H-bridge via MCPWM (duty -> two compare registers)
  hal_encoder.{c,h}                quadrature encoder via PCNT (x4 decode)
  hal_current.{c,h}                OCM current sense via ADC continuous mode
  hal_can.{c,h}                    TWAI (CAN), RX ISR + queue, TX task off the control path, bus-off recovery
  axis_task.{c,h}                  1 kHz control loop (gptimer ISR), stall watchdog, task WDT, status log
  main.c                           app_main: hal_*_init, axis_task_start
components/axis_core/             portable C core (IDF component AND host static lib)
  include/axis/{pid,protocol,axis_config,config_table,axis,version}.h
  src/{pid,protocol,axis,config_table,version}.c        (config_table.c is generated, committed)
host/                            host CMake project (build/host/, via `make host`)
  CMakeLists.txt
  tests/{tinytest.h,test_config.h,rig.h,plant.h,test_*.c}   (ctest per host/CMakeLists.txt's axis_test())
  sim/{motor_model,bench,simaxis}.{c,h} sim/simlib_api.c (no .h; ctypes entry points only)       -> build/host/libsimaxis.{dylib,so}
config/arm.yaml, config/teleop.yaml, config/bench_identified.yaml
pc/robotarm/                    python package (uv project at repo root: pyproject.toml)
  config.py protocol.py bus.py cli.py __main__.py
  transport/tcp_bus.py
  sim/{native,model,world,harness,server,pacing,tune}.py
  master/{arm_client,gamepad,teleop,kinematics,identify,cli_util}.py
  analysis/steptest.py
  tools/gen_config.py
pc/tests/                       pytest suite (mirrors pc/robotarm/*, run via `make pytest`)
scripts/idf.sh                  docker wrapper for idf.py (build only; flashing needs a serial port)
scripts/sim.sh                  what `make sim` runs: sets up mjpython/DYLD_LIBRARY_PATH on macOS
Makefile                        host / test / firmware / sim / teleop
docs/architecture.md docs/protocol.md docs/simulator.md docs/teleop.md docs/bringup.md
```

## Development workflow

Build and test everything through the `Makefile` targets:

| Target | What it does |
|---|---|
| `make host` | Configures and builds the host CMake project (`host/` → `build/host/`), including `axis_core` as a static library and its ctest executables. |
| `make ctest` | Runs `make host`, then `ctest --test-dir build/host --output-on-failure`. |
| `make pytest` | Runs `make host`, then `uv run pytest -q pc/tests`. |
| `make test` | Runs `ctest` and `pytest` — the required gate before every commit. |
| `make gen-config` | Runs `uv run robotarm gen-config` to regenerate `components/axis_core/src/config_table.c` from `config/arm.yaml`. |
| `make firmware` | Builds the ESP32-S3 firmware via `scripts/idf.sh build` (Docker, `espressif/idf:v6.1`). |
| `make sim` | Launches the MuJoCo simulator (`scripts/sim.sh`, which runs `uv run mjpython -m robotarm sim` and sets up `DYLD_LIBRARY_PATH` on macOS; `mjpython` is required for the viewer). |
| `make teleop` | Launches the PlayStation teleop master against the TCP sim/robot bus. |

Other useful commands:

- `uv run robotarm sim --no-viewer` — the headless equivalent of `make sim`, works under plain `python`/`uv run` on any platform; see `docs/simulator.md`.
- `uv run robotarm monitor --bus <URL>` — print live axis state/position/faults from any bus (`tcp://…`, `sim`, or a real interface).
- `uv run robotarm gamepad-test` — print live gamepad state until Ctrl-C, to check a controller before teleop.
- `uv run robotarm identify --bus <URL> --node <N> [--duty D]` — open-loop step experiment over CAN for `stepfit`; see `docs/simulator.md` and `docs/bringup.md`.
- `uv run robotarm stepfit <recording> --motor <name> --supply <V> --cpr <N> --out <config.yaml>` — fit the bench motor model to a step recording.
- `uv run robotarm tune --axis <name> [--velocity]` — step-response metrics for one axis in the simulator, used to derive `config/arm.yaml`'s gains.

- `scripts/idf.sh <idf.py args>` — run any `idf.py` subcommand (e.g. `build`, `menuconfig`, `size`) inside the official ESP-IDF v6.1 Docker image, with the repo mounted at `/project`. Requires Docker (or Colima) running, and the repo must live under `$HOME` so it can be mounted.
- `uv run robotarm --help` — the Python CLI entry point; each later task registers its own subcommand.
- `uv sync` — install/update the Python virtual environment (`.venv/`) from `pyproject.toml`; run `uv python pin 3.12` once beforehand to pin the interpreter.

`build/` and `.venv/` are git-ignored; the ESP-IDF Docker build may also touch `sdkconfig`/`dependencies.lock` at the repo root — those are restored (not committed) unless a task explicitly changes IDF configuration.
