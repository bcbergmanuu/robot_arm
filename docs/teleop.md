# Teleop with a PlayStation controller

`robotarm teleop` jogs the arm joint by joint with a DualSense (PS5) or DualShock 4 (PS4)
controller. The same program drives the simulator and the real robot; only the `--bus` URL
differs. The button map lives in `config/teleop.yaml`, and the code is in
`pc/robotarm/master/teleop.py`.

## Quick start (simulator)

```
make sim                    # terminal 1: simulator + MuJoCo viewer on tcp://127.0.0.1:29536
make teleop                 # terminal 2: uv run robotarm teleop --bus tcp://127.0.0.1:29536
```

Then, on the controller:

1. **Triangle** homes every axis. Each axis drives into its home stop, then backs off to its
   soft limit.
2. **Cross** enables any axis that is still DISABLED. Homing leaves axes READY already.
3. **Hold L1** (the deadman) and move the sticks. Let go of L1 and the arm stops.

Other ways to run it:

```
uv run robotarm teleop --bus tcp://127.0.0.1:29536                 # same as make teleop
uv run robotarm teleop --bus sim                                   # in-process sim, no viewer
uv run robotarm teleop --bus slcan:/dev/tty.usbmodem1234           # real robot (also gs_usb:0, socketcan:can0, pcan:...)
uv run robotarm teleop --bus tcp://127.0.0.1:29536 --fake-gamepad  # headless: a pad that is connected but idle
```

Teleop prints one status line and updates it in place about 10 times a second:

```
JOINT ARMED pad:ok arm:ok READY homed:6/6 | j1 +12 j2 +35 j3 -40 j4 +0 j5 +0 j6 +27
```

It shows the mode, whether you are armed, the controller link, the arm link (every node sent a
STATUS in the last 0.5 s), the axis states, how many axes are homed, and each joint angle in
degrees. When an axis has a fault, `| FAULT j2:FOLLOWING` is added to the end.

## Controller layout (joint mode)

```
            L2  open gripper (j6 +)                 R2  close gripper (j6 -)
            L1  DEADMAN: hold to move               R1  hold for slow speed
         _______________________________________________________
        /     [Create]                         [Options]         \
       /   ^ d-pad up:                          clear faults      \
      /  <   >  wrist bend + (j4)          (Triangle)  HOME         \
     |     v  d-pad down:                (Square)   (Circle)         |
     |        wrist bend - (j4)         toggle mode  E-STOP          |
     |                                      (Cross)  ENABLE          |
     |      ( L )                  [PS]            ( R )             |
     |   left stick                             right stick          |
      \  up/down:    shoulder j2              up/down:    elbow j3   /
       \ left/right: hip j1                   left/right: wrist      /
        \_________________________________    rotate j5  __________/
```

| Input | Action |
|---|---|
| L1 (hold) | **Deadman**. Nothing moves unless it is held. |
| R1 (hold) | Slow: every jog input runs at 0.3x its normal speed (`speed_scale.slow` / `speed_scale.normal`). Sticks drop from 50 % to 15 % of max, the d-pad from 50 % to 15 %, and the triggers from 100 % to 30 %. |
| Left stick up/down | j2 shoulder. Up = +q2 (tilts forward) |
| Left stick left/right | j1 hip. Right = -q1 (clockwise seen from above) |
| Right stick up/down | j3 elbow. Up = +q3 |
| Right stick left/right | j5 wrist rotate. Right = +q5 |
| D-pad up / down | j4 wrist bend +/- at half its max speed |
| L2 / R2 | j6 gripper: velocity = (L2 - R2) x max. L2 opens, R2 closes toward 0 (closed) |
| Circle | **E-STOP** for every axis while held, whether or not you are armed |
| Cross | ENABLE (DISABLED to READY). It does not clear faults. Only works while not armed. |
| Options | Clear faults. Faulted axes go to DISABLED, then press Cross. |
| Triangle | HOME every axis. Only works while not armed. |
| Square | Switch between joint and cartesian mode. Only works while not armed. |

The sticks have a radial deadzone of 0.12 (`deadzone` in `config/teleop.yaml`). The loop runs
at 50 Hz (`loop_hz`).

### Cartesian mode

Press Square (not armed) to switch; the status line starts with `CART`. You can also start in it
with `robotarm teleop --mode cartesian`. With L1 held you move the gripper tip instead of single
joints:

| Input | Action |
|---|---|
| Left stick up/down | Out / in, horizontally along the direction the hip points |
| Left stick left/right | Swing the arm about the vertical axis. Left = counter-clockwise seen from above |
| Right stick up/down | Up / down (z) |
| Right stick left/right | Roll the gripper (j5). Right = +roll |
| D-pad up / down | Pitch the gripper (the angle from vertical, q2+q3+q4) +/- |
| L2 / R2 | Gripper, as in joint mode |

Speeds come from `cartesian_mode` in `config/teleop.yaml`: `linear_speed_m_s` (0.05 m/s) for
moves and `pitch_speed_rad_s` (0.5 rad/s) for pitch, roll and the swing. R1 slows everything by
the same 0.3x as in joint mode. When you arm, the target starts at the gripper's current pose,
so nothing jumps. If you push toward a pose the arm cannot reach, or one that needs a joint past
its soft limit, the target stops at the last reachable pose and the arm stops there. Pull back
to move again. Cartesian mode sends position setpoints, which the axes ignore until they are
homed: if any of j1–j5 is not homed, the arm holds still and the status line shows
`HOLD: home j1-j5 for cartesian`. Press Triangle (not armed) first.

## Safety rules

- **Deadman.** You become *armed* only when you *press* L1 while the controller is connected.
  Releasing L1 disarms. Holding L1 while the program starts does not arm you: let go and
  press it again.
- **Releasing the deadman stops the arm**, in either mode. When jogging stops (L1 released,
  controller lost, E-STOP), every jog axis gets a zero-velocity setpoint on the next 3 loops (60 ms), repeated in
  case a frame is lost. The axes then decelerate to a stop on their own ramps. The same happens
  when the controller drops out. At all other times while not armed, teleop sends no setpoints,
  so it never interferes with HOME or anything else. Heartbeats alone keep the axis watchdogs fed.
- **E-STOP (Circle) always works.** It acts while the button is held, not just when you press
  it: E-STOP is sent on every loop while Circle is down, armed or not, including when Circle was
  already held at start-up or through a controller reconnect. It also disarms you, and you
  cannot arm while it is held, and Options, Cross and Triangle are ignored while it is held.
  To recover: release Circle, press Options (clear faults), then
  Cross (enable), then press L1 again.
- **Other buttons act on the press.** Holding a button does nothing more. A button held while
  the program starts, or held through a controller reconnect, does nothing until you release it.
  Cross (enable), Triangle (home) and Square (mode) only work while you are **not** armed.
- **Losing the controller** disarms, and the arm stops. Teleop notices the loss in one of two ways:
  - SDL reports the controller removed (cable pulled, and usually a clean Bluetooth disconnect):
    on the next loop.
  - A Bluetooth pad that goes out of range or runs flat is not always reported as removed. It
    can keep returning its last report, including a stick held forward. So teleop also treats
    the controller as lost when a stick is pushed past the deadzone and **every** input (sticks,
    triggers, buttons) has stayed exactly the same for `stale_input_s` (config/teleop.yaml,
    default 0.5 s; 0 turns the check off). A thumb on a stick always jitters a little, so this
    does not fire in normal use. It can fire if you hold a stick perfectly still against its rim.
    That is safe: the arm stops, and you release and press L1 again.

  Until one of those happens (up to `stale_input_s` for a silent Bluetooth pad), the last stick
  position keeps jogging. **Use a USB cable for the first sessions on the real robot.**
  When the controller comes back you must **release and press L1 again**.
  Holding it through the reconnect is not enough.
- **A hung teleop loop stops the arm.** Heartbeats are only sent while the teleop loop is alive:
  it checks in on every iteration, and the check-in expires after 100 ms (or two loop periods,
  if `loop_hz` is below 20). If the loop stalls
  (a stuck gamepad driver, a paused terminal, a bug), the heartbeats stop and every axis
  watchdog-faults 200 ms later. The last jog command does not keep running.
- **Losing the bus** (sim closed, USB-CAN adapter unplugged): teleop prints
  `error: bus lost: <reason>` and exits with code 2. Without heartbeats, every axis
  watchdog-faults within 200 ms by itself.
- **Any other failure** (bad `config/teleop.yaml`, gamepad start-up or read error, internal
  error): teleop prints a single `error: ...` line and exits with code 2. Once the loop has
  started, it still sends DISABLE on the way out.
- **Ctrl-C** (or SIGTERM) sends DISABLE to every axis and exits with code 0. Ctrl-C while the bus
  is still connecting exits with code 130.
- A FAULTed or DISABLED axis is unpowered. In the sim, the shoulder then creeps down under
  gravity at about 1–2°/s. Expect the same on the real robot for any joint that can be driven
  backwards by its load.
- The axes enforce their own soft limits, velocity and acceleration limits, and following-error
  and overcurrent faults in firmware. Teleop never relies on the PC to stay inside them. A
  homed axis brakes to a stop at its soft limit instead of faulting. Before homing there are no
  soft limits and the axis moves at its slow homing speed.

## Pairing a controller on macOS

1. Open **System Settings → Bluetooth**.
2. Put the controller in pairing mode: hold **PS + Create** (DualSense) or **PS + Share**
   (DualShock 4) until the light bar flashes quickly.
3. Choose *DualSense Wireless Controller* / *Wireless Controller* in the device list.

A USB-C (DualSense) or micro-USB (DS4) cable also works and needs no pairing. To check the
controller, run:

```
uv run robotarm gamepad-test
```

It prints the live state 10 times a second: stick values (up = +y), triggers from 0 to 1, and
the pressed buttons by their PlayStation names. The output says `disconnected` if SDL cannot see
the controller. Press Ctrl-C to stop. If buttons show up under the wrong names, pygame fell back
to raw joystick mode, and the index map is `raw_joystick_fallback` in `config/teleop.yaml`.
