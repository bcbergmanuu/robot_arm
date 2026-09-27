# Bring-up guide

Everything in this repo (host tests, the simulator, teleop, `identify`/`stepfit`) was built and
verified against MuJoCo, not the real arm — the real bring-up starts here. Read `docs/teleop.md`
(safety rules) and `docs/simulator.md` (motor model / bench identification) alongside this.

## First things to try on the laptop (before touching the robot)

Do these first, without any hardware connected, to shake out anything environmental (Bluetooth
pairing, Docker, the MuJoCo viewer) before it's mixed up with real-robot debugging.

1. **Verify the MuJoCo viewer.** `make sim` was only exercised headless overnight — the build
   machine's shell is sandboxed and can't open windows, so the viewer itself is unverified on a
   real display. Run:
   ```
   make sim
   ```
   A window should open showing the arm resting near its home stops. If it segfaults or the
   window never appears, see the "macOS + viewer" note in `docs/simulator.md`; the fallback that
   is known to work everywhere is `uv run robotarm sim --no-viewer` (or `scripts/sim.sh
   --no-viewer`) plus `uv run robotarm monitor --bus tcp://127.0.0.1:29536` in a second terminal
   to watch axis state without a viewer.
2. **Pair the controller and check it.** Pair a DualShock 4 / DualSense as in "Pairing a
   controller on macOS" in `docs/teleop.md`, then:
   ```
   uv run robotarm gamepad-test
   ```
   Confirm every stick, trigger and button reads correctly (values move as you'd expect, names
   match the physical buttons) before trusting it to drive anything.
3. **Teleop against the simulator.** With `make sim` still running:
   ```
   make teleop
   ```
   Home (Triangle), enable (Cross), hold L1 and jog each joint a little, then Circle (E-STOP) and
   confirm every axis stops and faults. This is the same program and the same `config/teleop.yaml`
   that will drive the real robot — only `--bus` changes.

Once all three work, move to the real hardware below.

## Hardware shopping list

- **USB-CAN adapter**, e.g. [CANable 2.0](https://canable.io/) (`slcan` or `gs_usb` firmware —
  either works with `python-can`; `gs_usb` needs no serial driver on macOS/Linux).
- **120 Ω termination resistors, one at each physical end of the CAN bus** (not one per node —
  a straight or star topology wants exactly two terminators total, at the two ends of the trunk).
  Six axis boards in the middle of the bus need none.
- CAN wiring: a twisted pair for CANH/CANL, plus a shared ground between every board and the
  USB-CAN adapter. Keep stubs off the trunk short.
- A bench power supply (current-limited) is strongly recommended for the first power-up of each
  axis, instead of the full 24 V arm supply — see "First power-up" below.

## Flashing each board with its node id

Each ESP32-S3 board is flashed with a single `CONFIG_AXIS_NODE_ID` (1–6, see `main/Kconfig.projbuild`
and the `node:` field per axis in `config/arm.yaml`: 1 hip, 2 shoulder, 3 elbow, 4 wrist_bend,
5 wrist_rotate, 6 gripper). Use `scripts/build_node.sh`, one build directory per node:

```
scripts/build_node.sh 3 build                        # -> build/node3 (idf.py in Docker)
scripts/build_node.sh 3 flash -p /dev/cu.usbmodemXXXX
scripts/build_node.sh 3 monitor -p /dev/cu.usbmodemXXXX
```

Each node gets its own `build/nodeN/sdkconfig`, generated from the committed `sdkconfig.defaults`
(USB-Serial-JTAG console, task-watchdog panic, MCPWM control in IRAM) plus a generated
`build/nodeN/sdkconfig.node` holding `CONFIG_AXIS_NODE_ID=N`. Check it after a build:
`grep AXIS_NODE_ID build/node3/sdkconfig`. The tracked top-level `sdkconfig` stays at node 1: it is
what `scripts/idf.sh build`/`make firmware` and the VS Code ESP-IDF extension use. Do **not**
`echo` into `sdkconfig.defaults` or set the node id in `menuconfig` for the top-level build. The
existing `sdkconfig` overrides the defaults, and overwriting `sdkconfig.defaults` would drop the
console/watchdog/IRAM settings.

Flashing and the console need the board's USB serial port. Docker Desktop on macOS cannot pass
USB serial ports into a container, so on macOS `flash` and `monitor` run on the host:
- `flash` runs `uvx esptool --chip esp32s3 -p PORT ... write-flash @flash_args` inside
  `build/nodeN` (the file lists the bootloader, partition table and app offsets of that build).
  You can also run that command yourself from `build/nodeN`.
- `monitor` runs `pyserial-miniterm` (`uvx --from pyserial`). Exit with Ctrl-].

On Linux, both run `idf.py` in Docker with the port passed through (`--device`).

Keep a written note of which physical board carries which node id. Nothing on the board itself
says so once it is flashed.

## First power-up checklist

1. **Motor disconnected** from the H-bridge output (leave the encoder and CAN connected). This
   way a wiring or sign mistake can't drive the arm into a limit or its own frame.
2. Power the board (3.3 V logic from USB is enough to boot; the TB9051FTG driver stage needs its
   own supply per `config/arm.yaml`'s `supply_voltage: 24.0`, or a bench 12 V/24 V supply matching
   whichever motor is on that axis).
3. From a laptop with the USB-CAN adapter attached:
   ```
   uv run robotarm monitor --bus slcan:/dev/tty.usbmodemXXXX     # or gs_usb:0
   ```
   Confirm the node appears, in state `DISABLED`, with a plausible (not wildly jumping) position —
   the encoder should read 0 until homed and change smoothly if you turn the shaft by hand.
4. Watch the firmware's own status line (`idf.py monitor`, once per second per axis — see
   `main/axis_task.c`): `stall_trips=0`, `cur_stale=0`, CAN `buserr=0`/`rxdrop=0`/`txdrop=0`. Any
   of those non-zero on a quiet bus with the motor disconnected point at wiring before code.
5. Only once state, faults and CAN stats all look sane, reconnect the motor and repeat with a
   bench-limited supply before trusting the axis with the arm's own 24 V rail.

## Sign check per axis

With the motor connected and the axis clear to move a little:

```
uv run robotarm identify --bus <URL> --node <N> --duty 0.15
```
This runs `identify`'s open-loop step (short, small duty — see `docs/simulator.md`) and writes a
CSV. Watch `robotarm monitor` (or the CSV's `position` column) while it runs:

- **Position counts should increase for a positive duty**, and the direction should match the
  physical motion you'd call "positive" for that joint (see the joint convention comment at the
  top of `config/arm.yaml`: q = 0 is straight up, J2/J3/J4 positive tilts toward +x).
- If **position counts go negative for a positive duty**, or the physical motion is backwards
  from the convention, flip that axis's `motor_sign` in `config/arm.yaml` (this reverses which way
  a positive duty drives the shaft without touching the encoder reading) — or `encoder_sign` if
  the *encoder* reads backwards relative to a *correctly-signed* motor (rotate the shaft by hand
  with the motor unpowered and check the position sign, which isolates encoder wiring from motor
  wiring). Then:
  ```
  make gen-config       # regenerates components/axis_core/src/config_table.c from arm.yaml
  scripts/build_node.sh <N> build
  scripts/build_node.sh <N> flash -p <port>
  ```
  Re-run `identify` to confirm.

## Current-sense calibration

`config/arm.yaml`'s `current_sense.mv_per_a: 528.0` is a calculated-not-measured value (see the
open question below). To calibrate: drive a **known**, steady current through the motor (e.g. hold
it stalled against a soft limit at a small duty, or use a bench supply with its own ammeter) and
compare against the `current_ma` telemetry (`robotarm monitor`, or the `current` column from
`identify`'s CSV). If the reported current doesn't match the true current, correct `mv_per_a`
(the ADC-to-mA conversion is `current_ma = adc_mv / mv_per_a * 1000`, so `current_ma` is inversely
proportional to `mv_per_a`):
```
mv_per_a_new = mv_per_a_old × (reported_current / true_current)
```
and rerun `make gen-config` + reflash.

## Per-axis identification and friction fit

Once wiring and current sense are confirmed for an axis, capture a real step response and refit
the bench model — the exact command `docs/simulator.md`'s "Redoing this on the robot" section
already documents:
```
uv run robotarm identify --bus <URL> --node <N> --duty 1.0 --out output_<axis>.txt
uv run robotarm stepfit output_<axis>.txt --motor <real-motor-name> --supply <V> \
    --cpr <4 x encoder lines> --plot docs/img/stepfit_<axis>.png --out config/bench_identified_<axis>.yaml
```
`stepfit` fits `j_total`, `b_viscous`, `tau_coulomb` on the *motor shaft*; referred to the joint
they scale **up** by the gear ratio (motor-shaft values are the small ones): `× gear_ratio²` for
`j_total`/`b_viscous`, `× gear_ratio` for `tau_coulomb` (see `docs/simulator.md`'s "Redoing this on
the robot" section and `armature = motor.j_rotor * gear_ratio**2` in `pc/robotarm/sim/model.py`).
`config/arm.yaml`'s `friction:` block is already joint-side and has no inertia field — the
rotor's own inertia is reflected into MuJoCo's joint `armature` straight from the config's
`motors.*.j_rotor` × `gear_ratio²`, not from a fit — so in practice only `b_viscous` and
`tau_coulomb` (→ `friction.viscous_nm_s` and `friction.coulomb_nm`) actually get transferred into
that axis's `friction:` block in `config/arm.yaml`, replacing the placeholder numbers; `j_total`
is a sanity check against the config's own `j_rotor`, not a value to write anywhere. Apply
`gear_efficiency` too if the fit was done electrically (torque in vs. torque delivered at the
joint differ by that factor). Repeat per axis — every axis currently shares friction numbers derived from
one bench recording of unknown provenance (see Ruling R10 / "(assumed)" list below), not six real
fits.

## Homing tuning

Homing constants live in each axis's `home:` block in `config/arm.yaml` (`direction`,
`velocity_deg_s`, `current_ma` stall threshold, `timeout_s`) plus two firmware constants in
`components/axis_core/include/axis/axis.h`: `AXIS_HOME_SETTLE_MS` (300 ms grace before stall
detection starts) and `AXIS_HOME_STALL_MS` (100 ms of stall evidence before the stop is declared
found). If an axis homes too early (false stall from static friction) or too late/never (the
stall threshold never trips), raise/lower that axis's `home.current_ma` first — it's a per-axis
config value and needs no reflash-and-regenerate round trip beyond `make gen-config` +
`scripts/build_node.sh <N> build` + flash. Watch `robotarm monitor`'s `faults` column: a real jam
should show `HOMING` stall-then-found; a timeout shows the axis still `HOMING` past `timeout_s`.

Home **one axis at a time** during bring-up:
```
uv run robotarm axis home --node <N> --bus <URL>      # waits for READY + homed, then disables it again
uv run robotarm axis clear --node <N> --bus <URL>     # after a fault: FAULT -> DISABLED
uv run robotarm axis enable --node <N> --bus <URL> --hold 5   # enable and hold position for 5 s
uv run robotarm axis disable --node <N> --bus <URL>
```
Each prints one line and exits 0, or prints one `error: ...` line (for example the fault that
stopped homing, or a timeout) and exits 2. The homed flag survives the DISABLE at exit until the
board resets, so axes homed one at a time are still homed when you start `robotarm teleop`.

**Wrong direction during homing.** If the motor or encoder sign is wrong, homing drives *away*
from `home.direction`. The velocity loop then winds up to `max_duty` and would hit the opposite
stop and accept that stall as home. The firmware prevents this: after the first 50 ms of homing,
moving against `home.direction` faster than half `home.velocity_deg_s` for 40 consecutive ms
faults the axis with `HOMING` (`AXIS_HOME_DIR_GRACE_MS` / `AXIS_HOME_WRONG_DIR_MS` in `axis.h`).
A `HOMING` fault within about 0.1 s of the start therefore means "check the signs" (see "Sign
check per axis"), not "tune the homing".

## Gain retune with the real motors

`config/arm.yaml`'s `gains` (per axis `vel_kp`/`vel_ki`, shared `pos_kp`) were tuned in the
simulator against the *assumed* motor/friction parameters (`docs/architecture.md`'s Gain tuning
section, `pc/robotarm/sim/tune.py`). Once identification above gives real `J`/`b`/`T_c` per axis,
rerun the same procedure against the corrected model, or retune empirically on the robot: start
from the committed gains (they're conservative — 50 % of the measured instability threshold in
sim), watch for oscillation or slow settling with `robotarm monitor` while jogging in teleop, and
adjust `vel_kp` (and `vel_ki = vel_kp / 30ms` to keep the same integral time) up or down.

## Teleop at slow speed, hand on the E-stop

Only once every axis above has been checked individually:
```
uv run robotarm teleop --bus <URL>
```
Hold **R1** (slow, 0.3x speed) for the first session. Keep a hand near a way to cut power (the
E-STOP button, Circle, sends a CAN broadcast and works even if the PC itself locks up — but it is
not a substitute for a physical kill switch on a first run with a real arm). Home one axis at a
time with `robotarm axis home --node <N>` (see "Homing tuning") rather than all six with
Triangle, until you trust each one's sign and limits. Use a USB cable for the controller, not
Bluetooth (see "Losing the controller" in `docs/teleop.md`).

## "(assumed)" values in `config/arm.yaml` — measure on the robot

Everything below is tagged `(assumed)` in the committed `config/arm.yaml` and should be replaced
once measured:

| location | value | how to measure |
|---|---|---|
| `motors.*` (all four Faulhaber entries): `R`, `L`, `kt`, `j_rotor` | "datasheet-class" placeholders, not the exact datasheet numbers | Look up the exact part number stamped on each motor (README's per-axis motor table) in Faulhaber's datasheet, or measure `R` with a multimeter (winding resistance) and `kt` from the no-load speed / stall current ratio. |
| `current_sense.mv_per_a` | 528.0 (calculated from the TB9051FTG OCM mirror ratio and R3, never measured) | See "Current-sense calibration" above. |
| `geometry.*` (`base_height`, `upper_arm`, `forearm`, `wrist`, `gripper`) | Katana 6M180-like guesses, metres | Measure the physical link lengths (pivot to pivot) with calipers/tape and the arm at q = 0. Kinematics (`pc/robotarm/master/kinematics.py`, cartesian teleop mode) is only as accurate as these. |
| `defaults.gear_efficiency` | 0.7 for every axis | Best measured indirectly: compare a known applied duty/current against the resulting joint torque (e.g. holding a known mass), or accept the guess if cartesian-mode accuracy is good enough. |
| `axes.hip.soft_limits_deg` | [-160, 160] | Move the hip by hand (motor disconnected, or DISABLED) to its physical limits and read the encoder-derived angle from `robotarm monitor`; set soft limits a few degrees inside the hard stops. |
| `axes.hip.hard_limits_deg` | [-169, 169] | Same, but the mechanical stop itself — the point where the joint physically cannot move further. |
| `axes.hip.link_mass_kg` | 1.2 | Weigh the link (or estimate from the assembly's known parts) if precise dynamics matter; teleop and homing don't depend on it, only gain tuning in simulation did. |
| `axes.hip.friction.{coulomb_nm,viscous_nm_s}` | 0.8 / 0.5, at the joint | Replace with a real per-axis fit — see "Per-axis identification" above. |
| `axes.wrist_bend.encoder_cpr` | 128 ("not documented in README") | Confirm the wrist_bend encoder's actual line count (README lists 128CPR for wrist rotate and fingers but leaves wrist bend blank); if it's a different encoder, correct `encoder_cpr`. |
| `axes.wrist_bend.gear_ratio` | 200.0 (Ruling R13: raised from the README's 100:1 because 100:1 couldn't hold the wrist near horizontal within its 800 mA current limit in simulation) | Confirm the real wrist_bend gear ratio (strainwave stage x any secondary reduction) once the hardware is accessible; if it really is 100:1, the axis will need a higher `max_current_ma` or a counterbalance instead. |
| `axes.wrist_bend.motor` | `faulhaber_2224sr_12v` (guessed — the README leaves the wrist_bend motor blank) | Read the part number off the physical motor. |

Every other axis's `friction:` and `link_mass_kg` values are not individually tagged `(assumed)`
in the file but are equally unmeasured placeholders (the only real data point is the single bench
recording behind `config/bench_identified.yaml` — see the next section); treat the whole
`friction:`/`link_mass_kg` column as provisional until each axis has its own `identify` +
`stepfit` run.

### Which motor was actually on the bench (Ruling R10)

`config/bench_identified.yaml` (the model validated against `output.txt`, see
`docs/simulator.md`) assumes the recording came from a `faulhaber_2224sr_12v` at 12 V with a
64-line (256 counts/rev) encoder — the only assumption consistent with both the recorded current
and no-load speed, but still an assumption, since the recording's provenance (axis/motor/encoder/
supply) was never logged. Once any axis is identified on the real robot with a known motor, this
assumption is superseded for that axis; it's only a stand-in for "some small 12 V Faulhaber
motor" until then.

## Open hardware questions

These came out of the firmware bring-up work (Task 19) and are not resolved by anything in this
repo — they need the real board and, in a few cases, an oscilloscope:

- **TB9051FTG OCC pin.** `main/board.h` defines `BOARD_OCC_GPIO` (GPIO2) but the firmware leaves
  it unconfigured — OCC is the driver's over-current *comparator* output (a separate, faster,
  analog-threshold protection from the OCM current-sense mirror the firmware does use). Decide
  whether to wire it as a GPIO input (e.g. an immediate hardware-triggered stop, faster than the
  ~1 ms software overcurrent check) or leave it unused, and update `hal_current.c`/`board.h`
  accordingly.
- **EN wiring / PWM pins floating before init.** `hal_motor_init()` is the first call in
  `app_main()` specifically so the bridge starts at brake as early as possible, but GPIO7/GPIO8
  (`BOARD_PWM_A_GPIO`/`BOARD_PWM_B_GPIO`) are still floating (default input, no defined pull)
  between power-on/reset and that call — during that window it is up to the TB9051FTG's own
  EN pin state and default input thresholds whether the bridge could see either input as high.
  Check with a scope whether the driver's EN pin(s) are tied in a safe state (disabled) until
  firmware configures the PWM outputs, and whether GPIO7/8 need an external pull-down for the
  gap between reset and `hal_motor_init()`.
- **`motor_sign`/`encoder_sign` per axis.** All six axes currently default to `motor_sign: 1,
  encoder_sign: 1` in `config/arm.yaml` — nobody has confirmed any of them yet. Run the sign check
  above per axis before trusting direction.
- **The OCM current signal is magnitude-only, and only valid while the bridge drives.** The
  TB9051FTG's OCM output is a *magnitude* of motor current with no sign, and it only reflects
  current while the bridge is actively driving (duty ≠ 0) — during brake (duty 0, both inputs low)
  the sensed current reads ~0 regardless of any back-EMF/decay current actually still flowing in
  the winding. This is why `axis_core`'s cascade dropped torque (current) as a closed *signed*
  loop entirely (`docs/architecture.md`'s Decisions #1): current is only used open-loop, for
  overcurrent protection, homing stall detection, and telemetry — all of which only need a
  magnitude and only care about it while driving. It also means homing's stall threshold
  (`home.current_ma`) and any overcurrent threshold are being compared against a magnitude that
  can momentarily read near-zero right after a duty change even while the motor is still loaded,
  until the bridge is driving steadily again.

  **Duty weighting (R31).** OCM follows the motor current only during the PWM on-phase and reads
  ~0 in the off-phase. `hal_current.c` averages the ADC over each whole 1 ms tick, so the raw
  average is `|i| × |duty|`. The control task converts it back with
  `axis_motor_current_from_avg()` (`components/axis_core/include/axis/current_sense.h`), using
  the duty applied during that millisecond: it divides by `|duty|` when `|duty| ≥ 0.1`. Below 0.1
  it passes the raw average through unscaled, because dividing would only amplify ADC noise. The
  result is clamped to 30 A. The simulator models the same raw average and uses the same helper,
  so thresholds tuned in the sim mean the same thing on the board. Consequence: **below 10 % duty
  the reported current under-reads** (by the factor `|duty|`). A stall at very small duty will
  not trip `max_current_ma` or `home.current_ma` by current alone; the homing stall detector's
  low-velocity criterion still catches it. For current-sense calibration, compare at a duty of
  0.3 or more. Whether the average really scales with duty like this (OCM settling time, PWM
  frequency vs. ADC sampling) needs a scope check on the real board.
- **Compare-0 brake behaviour.** `hal_motor.c` assumes writing both MCPWM comparators to 0 drives
  both TB9051FTG inputs low, which the datasheet calls the brake state (both low, or IN1=IN2). Put
  a scope on PWM_A/PWM_B during a duty-0 command and during a stall trip to confirm both lines
  are actually low (not, say, one line stuck high from a comparator/generator misconfiguration)
  and that the transition happens within the ≤40 µs the code comment claims.
- **Gravity creep of unpowered axes after a fault: brake vs hold.** In simulation, a
  FAULTed/DISABLED axis (duty forced to 0 = winding short/brake) still creeps under gravity —
  e.g. the shoulder drifts ~1–2°/s (see `docs/teleop.md`'s open design note). Whether the real
  TB9051FTG's brake state resists this any better, and whether any axis needs a mechanical brake
  or a "hold last position" fault behaviour instead of a pure brake, is unknown until the real
  arm is faulted under its own load.
- **Current-sense scale (528 mV/A) and ADC attenuation.** `mv_per_a: 528.0` is calculated from the
  OCM mirror ratio (~0.24 % of motor current) through a 220 Ω resistor, not measured; see
  "Current-sense calibration" above. Separately, `hal_current.c` configures `ADC_ATTEN_DB_12`
  (~2.5 V usable range, `current_sense.adc_max_mv: 2500.0`) — confirm the ADC never clips at the
  currents this scale predicts for a stalled motor (`max_current_ma` per axis), and reduce the
  attenuation (or the resistor) if it does.
- **Per-board node id build.** Each board needs its own `CONFIG_AXIS_NODE_ID` baked in at flash
  time (`scripts/build_node.sh`, see "Flashing each board" above); there is currently no other way to tell two flashed
  boards apart (no serial number, no DIP switch), so a mislabeled board is silent until it
  responds to the wrong CAN node id.

## Known limitations (open items)

- **`identify` duty-edge labelling.** Each CSV row's `pwm_ticks` is the duty the host was
  *commanding* when that STATUS+TELEMETRY pair arrived, not the duty the axis applied on that
  tick. The recorded step edge can therefore be off by a few samples (up to the host's poll
  latency, about 5 ms on real hardware). This matters for `stepfit` on short steps. Check the edge
  against the position trace, or trim the first samples after it.
- **Gamepad fast reconnect.** `Gamepad` forgets its device when SDL reports it removed, matching
  on the SDL instance id. A pad that drops and reconnects very quickly (Bluetooth) might be
  re-added before the removal is seen, or come back with a different instance id. The
  consequences have not been observed on hardware. Teleop requires a fresh L1 press after any
  loss either way, so the failure mode is "does not reconnect" (restart teleop), not "moves".
