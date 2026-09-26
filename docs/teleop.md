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

1. **Triangle** homes every axis. In the sim the wrist bend will usually fault with
   `OVERCURRENT` here, because it has already sagged onto its end stop. See *Known issue*
   below for the recovery steps.
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
| R1 (hold) | Slow: sticks at 15 % of each joint's max speed instead of 50 % |
| Left stick up/down | j2 shoulder. Up = +q2 (tilts forward) |
| Left stick left/right | j1 hip. Right = -q1 (clockwise seen from above) |
| Right stick up/down | j3 elbow. Up = +q3 |
| Right stick left/right | j5 wrist rotate. Right = +q5 |
| D-pad up / down | j4 wrist bend +/- at half its max speed |
| L2 / R2 | j6 gripper: velocity = (L2 - R2) x max. L2 opens, R2 closes toward 0 (closed) |
| Circle | **E-STOP** for every axis, whether or not you are armed |
| Cross | ENABLE (DISABLED to READY). It does not clear faults. |
| Options | Clear faults. Faulted axes go to DISABLED, then press Cross. |
| Triangle | HOME every axis |
| Square | Switch between joint and cartesian mode. Only works while not armed. |

The sticks have a radial deadzone of 0.12 (`deadzone` in `config/teleop.yaml`). The loop runs
at 50 Hz (`loop_hz`). Cartesian mode arrives with a later task. Until then the arm holds still
in cartesian mode, even with L1 held.

## Safety rules

- **Deadman.** You become *armed* only when you *press* L1 while the controller is connected.
  Releasing L1 disarms. Holding L1 while the program starts does not arm you: let go and
  press it again.
- **Not armed means stopped.** While not armed, every jog axis gets a zero-velocity setpoint on
  every loop (50 Hz), so it stops within one deceleration ramp. There is one exception: an axis
  that has just homed and is still backing off its end stop into the soft range is left alone
  until it gets there.
- **Buttons act on the press.** Holding a button does nothing more. A button held while the
  program starts, or held through a controller reconnect, does nothing until you release it.
- **E-STOP (Circle) always works**, armed or not, and it also disarms you. To recover: Options
  (clear faults), then Cross (enable), then press L1 again.
- **Losing the controller** (Bluetooth drops out, battery dies, cable pulled) disarms at once,
  and the arm stops. When the controller comes back you must **release and press L1 again**.
  Holding it through the reconnect is not enough.
- **Losing the bus** (sim closed, USB-CAN adapter unplugged): teleop prints
  `error: bus lost: <reason>` and exits with code 2. Without heartbeats, every axis
  watchdog-faults within 200 ms by itself.
- **Ctrl-C** (or SIGTERM) sends DISABLE to every axis and exits with code 0.
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

## Known issue: homing an axis that already rests on its end stop

The firmware ignores stall detection for the first `AXIS_HOME_SETTLE_MS` (300 ms) of a HOME.
Declaring home found then takes another `AXIS_HOME_STALL_MS` (100 ms), but the overcurrent
fault trips after `overcurrent_ms` (200 ms). So when HOME starts with an axis already pressed
against its home stop, or within a few degrees of it, that axis faults with `OVERCURRENT`
instead of homing.

In the sim this happens in two ways:

- If you wait more than about 0.15 s after `make sim` starts, the wrist bend sags onto its stop
  under gravity.
- If you home a second time, some axes (for example the shoulder, 4° from its stop at the soft
  limit) are still close to their stops.

To recover:

1. Press Options, then Cross.
2. Jog each affected axis at least 10° away from its home stop. Unhomed axes jog slowly, with no
   soft limits.
3. Press Triangle again.
