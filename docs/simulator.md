# Simulator

## Motor model validation

The simulator is only as trustworthy as its motor model, so the model is checked against the
one open-loop recording we have from real hardware: `output.txt` in the repo root
(1000 samples at 200 µs, 100 % duty from sample 401 to 800, then duty 0).

### Model

`host/sim/bench.c` (built into `build/host/libsimaxis`, driven from Python through
`robotarm.sim.native`) simulates one DC motor driving an inertia, everything referred to the
motor shaft:

- Electrical (`host/sim/motor_model.c`): `L di/dt = duty·V_supply − R·i − k_t·ω`, solved
  exactly per substep; torque `T = k_t·i`. Duty 0 shorts the winding (the TB9051FTG brakes),
  so the back-EMF drives a braking current.
- Mechanical: `J dω/dt = T − b·ω − T_c·sign(ω)`, integrated with 10 substeps per sample.
- Stiction: while `|ω| < 1e-3 rad/s` and `|T| ≤ T_c` the rotor stays at rest. Friction may bring
  the rotor to rest but never reverses it (zero-crossing clamp).
- Encoder: `counts = floor(angle / 2π · counts_per_rev)`.

The motor's `R`, `L`, `k_t` come from `config/arm.yaml`; `J`, `b` and `T_c` are fitted by
`scipy.optimize.least_squares` over `(log J, log b, log T_c)` on the position trace.

Acceleration and deceleration share the same electrical damping `k_t²/R` (driven and
shorted winding alike), so the faster stop after switch-off comes from the Coulomb term:
the rotor decays towards `−T_c/(b + k_t²/R)` instead of towards zero and hits standstill early.
The model reproduces the coast (107 counts recorded) to within a few counts.

### Result

```
uv run robotarm stepfit output.txt --motor faulhaber_2224sr_12v --supply 12 --cpr 256 \
    --plot docs/img/stepfit.png --out config/bench_identified.yaml
```

| parameter | value (motor shaft) |
|---|---|
| `j_total` | 6.28e-7 kg m² |
| `b_viscous` | 7.48e-6 Nm s/rad |
| `tau_coulomb` | 8.35e-3 Nm |
| position RMS error | 3.1 counts (0.31 % of the 1010-count stroke) |
| end-position error | 2 counts (0.20 %) |

![Recorded vs simulated step response](img/stepfit.png)

The remaining ±10-count sawtooth in the error trace is the recording itself: the logged
position only updates every 3–4 samples. `pc/tests/test_steptest.py` re-runs the committed
`config/bench_identified.yaml` against `output.txt` and requires end position and RMS within
5 % and the coast-down within 30 counts.

### Which motor was on the bench (assumption)

The recording does not say which axis/motor/encoder/supply was used. The position trace alone
cannot tell: every combination of motor (`faulhaber_2657cr_24v/12v`, `2642cr_12v`,
`2224sr_12v`), supply (12 V, 24 V) and encoder (64, 128, 360 lines = 256, 512, 1440 counts)
fits it to ~0.31 % RMS, because `J`, `b`, `T_c` absorb the scale. The recorded **current**
does discriminate:

| assumption | mean model current (driven, after 4 ms) | recorded |
|---|---|---|
| `faulhaber_2657cr_12v` @ 12 V, 256 (the CLI default) | 4.7 A (ADC clipped) — needs `T_c` = 80 mNm | 1.12 A |
| `faulhaber_2224sr_12v` @ 12 V, 256 | 0.91 A | 1.12 A |
| `faulhaber_2224sr_12v` @ 12 V, 512 | 1.14 A | 1.12 A |
| any motor @ 24 V | ≥ 2.3 A | 1.12 A |

Only the small 2224SR 12 V motor at a 12 V supply gives a plausible current (its stall current
is 12 V / 8.7 Ω = 1.4 A; the recorded current extrapolates to ~1.7 A at standstill and ~0 A at
~32 000 counts/s, which matches its no-load speed of 839 rad/s at 256 counts/rev). The committed
identification therefore assumes **`faulhaber_2224sr_12v` at 12 V with a 64-line encoder
(256 counts/rev)**. The 256 vs 512 counts/rev choice is not decisive given the uncalibrated
current sense (`mv_per_a` is itself assumed).

Not modelled: the ~4 A spike in the recorded current in the first 2 ms after switch-on (above
the 2224SR's stall current, so it is likely a sensing/ADC artefact or a lower real `R`), and
the ~2 ms lag of the current reading.

### Redoing this on the robot

Once the arm is back, `robotarm identify` (Task 18) records a step on a known axis with a known
motor, encoder and supply. Feed that CSV (same columns: `time_us,position,velocity,pwm_ticks,current`)
to `robotarm stepfit <csv> --motor <name> --supply <V> --cpr <4 × lines>` to refit `J`, `b`,
`T_c` per axis, and replace the `(assumed)` friction values in `config/arm.yaml` with the
results referred to the joint (`× gear_ratio` for torques, `× gear_ratio²` for `J` and `b`).

## Gain tuning in simulation

`uv run robotarm tune --axis NAME [--velocity]` homes the simulated arm (lockstep, deterministic)
and prints overshoot, settle time and steady error for a 20° position step (or a velocity step)
on one axis; the procedure and the tuned values are in `pc/robotarm/sim/tune.py` and
`config/arm.yaml`. The reported homing offset (0.1–0.4° per axis) is expected: the axis registers
its home while pressed into MuJoCo's slightly compliant joint limit (default `solref`), so its
zero sits off by that penetration depth — deterministic, and well inside the 1° homing tolerance.

Measured velocity-loop thresholds (loop gain = `vel_kp` × no-load counts/s per unit duty, `vel_ki` = 0,
first velocity-step overshoot > 10 %, worse of the homed and the all-zero pose) and the chosen gains
(50 % of the threshold, `vel_ki = vel_kp / 30 ms`, `pos_kp` = 50 for all axes):

| axis | loop gain at > 10 % | `vel_kp` | `vel_ki` |
|---|---|---|---|
| hip | ~8 (all-zero pose; ~28 when homed) | 7.0e-5 | 2.3e-3 |
| shoulder | ~9 | 8.8e-5 | 2.9e-3 |
| elbow | ~9 | 1.4e-5 | 4.7e-4 |
| wrist_bend (gear 200) | ~15 | 5.5e-5 | 1.8e-3 |
| wrist_rotate | ~11 | 4.0e-5 | 1.3e-3 |
| gripper | ~17 | 6.2e-5 | 2.1e-3 |

The simulated encoders are incremental like the real boards: each reads 0 at the first
simulated tick wherever the joint is, so positions only become absolute after homing.
