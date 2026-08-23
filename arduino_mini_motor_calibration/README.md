# mini — motor feed-forward calibration

Bench sketch that measures this robot's PWM-to-speed curve and least-squares
fits it, so the feed-forward constants in
`arduino_mini_base_controller/Config.h` come from data instead of a guess. It
also commissions the two direction signs those constants depend on.

It is a standalone tool: no IMU, no odometry, no PID, no ROS serial protocol.
Flash it, calibrate, paste the constants it prints into the main firmware, then
flash the main firmware back.

Section 7 of the [repository README](../README.md) is the short version. This
is the full reference.

## 1. What it measures, and why

The main firmware drives each wheel with an open-loop feed-forward term plus a
PID correction. The feed-forward inverts a straight-line model of the motor:

```text
|speed_mm_s| = slope * pwm + intercept
```

Four arrays hold it — one slope and one intercept per direction, each with a
value per wheel:

```text
FF_SLOPE_MM_S_PER_PWM_FWD[]   FF_INTERCEPT_MM_S_FWD[]
FF_SLOPE_MM_S_PER_PWM_REV[]   FF_INTERCEPT_MM_S_REV[]
```

Per wheel **and** per direction, because brushed gearmotors differ from each
other by tens of percent and are direction-asymmetric. That asymmetry is what
makes a robot drive straight forward and curve in reverse.

The better this fit is, the less work PID has to do, and the less the robot
depends on integral windup to reach a commanded speed. It does not replace PID
— it removes the part of the error that is predictable.

**The values shipped in the main firmware are a generic straight line, not a
calibration.** Replace them before driving on the floor.

## 2. Before you start

- [ ] **Robot on blocks, both wheels spinning free.** Every measurement here is
      open loop and runs up to `MAX_DRIVE_PWM` (210).
- [ ] **Battery charged, and kept charged for the whole run.** The fit is only
      valid at the bus voltage it was measured at; a battery that sags across
      the sweep bakes a wrong slope into the firmware.
- [ ] **Direction signs commissioned.** Send `DIR` and answer it. No fit can
      repair a sign error, and a wheel with both signs wrong measures perfectly
      while turning the wrong way. `DIR` is step 3 of the procedure below.
- [ ] **`MEASURED_COUNTS_PER_REV` correct.** Every mm/s printed here is scaled
      by it (see step 2 of the procedure).

These constants are duplicated from the main firmware and must match it, or the
numbers you paste back will not mean what the firmware thinks they mean:

| Constant | Default | Why it matters here |
| --- | --- | --- |
| `WHEEL_DIAMETER_MM` | `47.0` | converts encoder counts to mm |
| `MEASURED_COUNTS_PER_REV` | `{520, 520}` | scales every speed in the fit |
| `MAX_DRIVE_PWM` | `210` | the fit must span the range the firmware uses |
| `ENCODER_SIGN` | commissioned by `DIR` | forward must read positive |
| `MOTOR_DIRECTION_SIGN` | commissioned by `DIR` | which driver polarity is forward |

Wiring is identical to the main firmware (TB6612FNG on D10/D5/D4 and D9/D7/D8,
`STBY` on D12, encoders on D2/D6 and D3/D11). The IMU is unused and can stay
connected or be left off.

## 3. Procedure

**1. Upload and connect.** Open the sketch in the Arduino IDE, select Arduino
Uno, upload. Open a serial terminal at **500000 baud** (the same rate the base
controller uses, so one terminal profile covers both), line ending "Newline" or
"Carriage return".

You should see:

```text
READY,mini_motor_ff_calibration_v2
WARN,PUT_THE_ROBOT_ON_BLOCKS_BOTH_WHEELS_MUST_SPIN_FREE
INFO,RUN,DIR,FIRST_THEN,CAL,OR,HELP,FOR_COMMANDS
INFO,GEOMETRY,counts=520.0/520.0,dia=47.00,maxpwm=210
SIGN,1,ENC,1,MOT,1
SIGN,2,ENC,-1,MOT,-1
INFO,COMPARE_WITH,CFG,ON_arduino_mini_base_controller,THEY_MUST_AGREE
INFO,CONFIG,range=10..210,step=10,settle_ms=400,window_ms=600,passes=2
INFO,ESTIMATED_SECONDS,per_direction=42,full_CAL=168
```

**2. Check counts-per-rev.** Send `TICKS,20`, then turn one wheel by hand
exactly one full turn and read the running total. Do it per wheel. Put the
magnitudes in `MEASURED_COUNTS_PER_REV` here **and** in the main firmware.

The shipped 520 is this robot's hardware: a 13 PPR encoder on the motor shaft,
decoded x2 (channel A on CHANGE, so two counts per encoder cycle), behind a
1:20 gearbox — 13 x 2 x 20 = 520. Recompute it for your own encoder and gear
ratio. Get this wrong and every speed in the fit is wrong by the same factor.

**3. Commission the direction signs.** Send `DIR`. It takes about thirty
seconds and it is not optional — see [§8](#which-way-is-forward) for what goes
wrong without it.

`DIR` spins each wheel at a fixed driver polarity, **ignoring both signs**, and
asks you which way the robot tried to roll:

```text
INFO,DIR_START,ROBOT_ON_BLOCKS,BOTH_WHEELS_MUST_SPIN_FREE
INFO,FORWARD_MEANS_THE_TOP_OF_THE_WHEEL_MOVES_TOWARD_THE_FRONT
INFO,DIR_PROBE,MOTOR_1(LEFT),pwm=40,WATCH_THIS_WHEEL_NOW
INFO,DIR_RAW_TICKS,-631
ASK,WHICH_WAY_DID_THE_ROBOT_TRY_TO_ROLL,F=FORWARD,B=BACKWARD,X=ABORT
```

Answer `F` or `B` (or `X` to abort) by watching the **wheel**, not the numbers.
From that one answer it derives both signs at once and confirms them by driving
the wheel forward through them:

```text
INFO,DIR_SOLVED,MOTOR_1(LEFT),ENC,-1,MOT,1
DIR,1,F,90,352.18,OK
```

After both wheels it drives them together so you can check the whole robot, then
prints the pair ready to paste into `MiniRobotGeometry.h` **in both sketch
folders**. Nothing is persisted, so a session that ends without pasting them
measured a robot that does not exist yet.

**4. Calibrate.** Send `CAL`. It sweeps both wheels, both directions, and takes
about two and a half minutes at the default settings. Points stream as they are
measured:

```text
INFO,SWEEP_START,MOTOR_1(LEFT),FORWARD
C,1,F,30,0.00,STALL
C,1,F,40,4.12,STALL
C,1,F,50,21.90,USED
...
R,1,F,1.8734,-71.20,0.9981,32,38.0,322.2
```

**5. Paste the constants.** At the end `CAL` prints the block to copy over the
four `FF_*` lines in `arduino_mini_base_controller/Config.h`:

```text
INFO,PASTE_INTO_arduino_mini_base_controller/Config.h
// ---------------------------------------------------------
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {1.8734f, 1.9102f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-71.20f, -76.85f};
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {1.8012f, 1.8440f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-68.90f, -74.10f};
// ---------------------------------------------------------
```

`RESULT` reprints it at any time.

**6. Verify.** Send `VERIFY`. It drives a few target speeds through the fresh
fit, open loop, and reports how far off each lands:

```text
V,1,F,60.0,58.4,-2.7
V,1,F,120.0,117.1,-2.4
```

Under about 10% is a good fit — that residual is what PID absorbs. Consistently
larger errors mean the fit is describing something other than the motor; see
troubleshooting below.

**7. Re-flash the main firmware** with the pasted constants. Retune the wheel
PID afterwards if you had tuned it against the old feed-forward: the loop now
starts from a much better guess, so the gains that used to be right are
probably too hot.

## 4. Commands

| Command | Effect |
| --- | --- |
| `CAL` | Calibrate both motors, both directions, then print the table and paste block. |
| `CAL,<motor>` | Same for one motor (`1` = left, `2` = right). |
| `SWEEP,<motor>[,R]` | One motor, one direction. `R` = reverse. Fits and stores that one direction. |
| `VERIFY` / `VERIFY,<motor>` | Drive target speeds through the fitted constants and report the open-loop error. |
| `RESULT` | Reprint the fit table and the paste block. |
| `HOLD,<motor>,<pwm>[,R]` | Hold one PWM and stream the measured speed (max 20 s). Spot checks, and watching battery sag. |
| `DIR` | Commission **both** direction signs for both wheels from one look at each. Spins a wheel at a fixed driver polarity, asks which way the robot rolled, derives `MOTOR_DIRECTION_SIGN` and `ENCODER_SIGN` from the answer, and confirms them. Session-only; the values are printed ready to paste. **Run this before `CAL`.** |
| `DIR,<motor>` | Same for one wheel. Skips the both-wheels-together confirmation. |
| `CHECK` | Drive each wheel forward through the current signs and report whether the encoder agrees: `OK`, `BACKWARDS`, or `NO_ENCODER_MOTION`. Four seconds. A consistency check on `DIR`'s answer, not a substitute for it — it cannot see the wheel. |
| `SIGN` | Print the current commissioning signs for every wheel. |
| `TICKS[,<seconds>]` | Motors off; stream raw encoder totals. Default 30 s, max 300. |
| `STEP,<pwm_step>` | Sweep step size, 1–50. Default 10. |
| `RANGE,<min>,<max>` | Sweep range within 0–210, at least 30 wide. Default 10–210. |
| `DWELL,<settle_ms>,<window_ms>` | Settle 50–5000, window 200–5000. Default 400, 600. |
| `PASSES,<1..4>` | Passes per direction. Default 2. |
| `CONFIG` | Print the current settings and the run-time estimate. |
| `STOP` | Coast both motors. |
| `HELP` | Command list. |

Commands are case-insensitive and terminated by `\n`, `\r` or `:`.

**Sending any character while a sweep, `VERIFY`, `HOLD` or `TICKS` is running
aborts it and coasts the motors.** An aborted sweep stores no fit, so the
previous result for that wheel and direction stays untouched.

`DIR`'s probes are the one exception: they last under two seconds each and
swallow whatever you type, because a keystroke while you are being asked to
watch a wheel is far more likely to be an early answer than an abort. Abort
`DIR` by answering `X`.

Sweep settings are not persisted — they reset to the defaults on every reset or
re-upload.

## 5. Output lines

| Line | Fields |
| --- | --- |
| `C,...` | `motor, F\|R, pwm, mm_s, USED\|STALL\|HOLD` — one measured point |
| `R,...` | `motor, F\|R, slope, intercept, r2, samples, deadzone_pwm, mm_s_at_max_pwm` |
| `V,...` | `motor, F\|R, target_mm_s, measured_mm_s, error_pct` |
| `DIR,...` | `motor, F, pwm, mm_s, OK\|FAILED` — `DIR`'s post-commission self check |
| `ASK,...` | `DIR` is waiting for `F`, `B` or `X` on the serial line |
| `CHECK,...` | `motor, F, pwm, mm_s, OK\|BACKWARDS\|NO_ENCODER_MOTION` |
| `SIGN,...` | `motor, ENC, sign, MOT, sign` — one row per wheel |
| `TICKS,...` | `left, right` — raw counts since the command started |
| `INFO,` / `WARN,` / `ERR,` / `ACK,` | progress, diagnostics, failures, completion |

`C` rows are plain CSV, so you can also capture the log and fit offline if you
would rather look at the scatter yourself. Speeds in them are **magnitudes
along the commanded direction** — a reverse sweep prints positive numbers when
the wheel is correctly running backwards, which is the form the firmware's
`speedFeedForwardPWM()` inverts.

## 6. Reading the results

An `R` row for each wheel and direction:

```text
R,1,F,1.8734,-71.20,0.9981,32,38.0,322.2
      slope  intercept  r2   n  deadzone  mm/s at PWM 210
```

- **slope** — mm/s gained per PWM count. The number you came for.
- **intercept** — negative, by construction: extrapolating the linear region
  down to PWM 0 lands below zero because the motor does not move at all until
  it beats stiction.
- **r²** — how straight the measured region actually was. Below `0.97` the
  sketch warns.
- **samples** — points that made it into the fit. With the defaults, expect
  roughly 24–36 (19 steps × 2 passes, minus the stalled ones).
- **deadzone_pwm** — `-intercept/slope`, the PWM where the fitted line crosses
  zero speed. The firmware reproduces exactly this point when it inverts the
  line, which is why there is no shared minimum-PWM floor: each wheel carries
  its own dead zone.
- **mm/s at max PWM** — top speed the model predicts at `MAX_DRIVE_PWM`. Sanity
  check it against what the robot visibly does.

What good looks like: r² ≥ 0.99, forward and reverse slopes within a few
percent of each other on the same wheel, left and right within ~10%, dead zones
in the 30–50 PWM range for typical small gearmotors.

The sketch also prints `INFO,SLOPE_MISMATCH_FORWARD,<pct>%` per direction. A
few percent is normal and is exactly what the per-wheel fit exists to absorb. A
large one is usually mechanical (a dragging wheel) or a wrong counts-per-rev on
one side — the firmware cannot fix that, because it would be correcting to a
wrong measurement.

## 7. Tuning the sweep

Run time is `steps × passes × (settle + window)` per direction, four times that
for a full `CAL`. `CONFIG` prints the estimate.

- **`PASSES,2`** (the default) ramps up on the first pass and down on the
  second. A gearmotor settles at a slightly different speed at the same PWM
  depending on which side it approached from; averaging both directions of
  travel fits the middle of that hysteresis loop instead of one edge of it.
  `PASSES,1` halves the time and is fine for a quick re-check.
- **`DWELL`** — raise `settle_ms` if your gearbox is slow to reach steady
  state; raise `window_ms` if low-PWM points look noisy. A window under ~200 ms
  quantises badly, because only a handful of ticks land inside it.
- **`STEP` / `RANGE`** — a smaller step gives more points and a better-averaged
  fit at the cost of time. Raising the low end of `RANGE` above the dead zone
  saves time otherwise spent measuring stalled points, but the sketch already
  discards those, so it changes nothing about the fit.

## 8. Troubleshooting

| Message or symptom | Cause | Fix |
| --- | --- | --- |
| Nothing, or garbage characters | Terminal at the wrong baud | 500000, not 115200 |
| `ERR,FIT_FAILED,MOTOR_n,used=0,...` + `CAUSE,NO_ENCODER_MOTION` | The wheel may be spinning, but the encoder netted zero counts. Most often channel **B** is floating or on the wrong pin — the ISR then reads a constant B and counts `+1, -1, +1, -1`, netting zero however fast the wheel turns. Also: flat battery, `STBY` low, channel A unplugged, wheel jammed | `DIR` reports the same fault in two seconds. Then `TICKS,20` and turn that wheel by hand: if the count stays near zero while the other wheel counts cleanly, it is the encoder wiring — left B = `D6`, right B = `D11` |
| `ERR,FIT_FAILED,MOTOR_n,...` + `CAUSE,WHEEL_MEASURED_BACKWARDS` | The signs were never commissioned. Normal on a differential drive, where the right motor is mounted mirrored | `DIR`, then `CAL`. Do not try to work out which sign to flip by hand — that is the mistake `DIR` exists to remove |
| `ERR,FIT_FAILED,MOTOR_n,...` + `CAUSE,TOO_SLOW_OR_TOO_FEW_POINTS` | The wheel moved, but never above `MIN_VALID_SPEED_MM_S` | Charge the battery, check `STBY`, confirm the wheel spins free |
| `WARN,SOME_POINTS_RAN_BACKWARDS,...` | Part of the sweep fitted and part ran backwards — one sign cannot describe a wheel that changed its mind. Usually a slipping coupling or an intermittent encoder channel | `DIR`, then look for the mechanical fault before trusting the fit |
| `ERR,DIR_INCONSISTENT,...` | The two wheels were answered against different ends of the robot | Re-run `DIR` and answer both against the same front |
| `ERR,DIR_SELF_CHECK_FAILED,...` | The wheel stalled during the confirmation, or the answer was given for the other wheel | Re-run `DIR,<motor>` and watch the wheel it names |
| `ERR,TOO_FEW_STEPS,n,...` | `RANGE`/`STEP` leave fewer than four distinct PWM points | Widen `RANGE` or shrink `STEP` |
| `ERR,FIT_REJECTED,NON_POSITIVE_SLOPE` | More PWM produced less speed — not a motor curve | Check for a slipping coupling, a failing driver channel, or a collapsing battery |
| `WARN,LOW_R2,<r2>,...` | Noisy or non-linear measurement | Longer `DWELL`, charged battery, check for mechanical binding |
| `WARN,WHEELS_DIFFER_A_LOT,...` | Left and right slopes differ by more than 25% | Look for drag on the slow wheel; re-check `MEASURED_COUNTS_PER_REV` per wheel |
| `0.000f /*NOT_CALIBRATED*/` in the paste block | That wheel/direction has no valid fit — the preceding `WARN,MISSING_FIT` lines name which | Fix the cause the `ERR,FIT_FAILED` line reported, then re-run. **Never paste a block containing `0.000f`** — the firmware treats a non-positive slope as "no feed-forward" and falls back to pure PID, so that wheel will be visibly slow |
| `VERIFY` errors much larger than 10% | Battery sagged between the sweep and the check, or the fit was made at a different voltage | Recharge and re-run `CAL` |
| All speeds off by a constant factor | Wrong `MEASURED_COUNTS_PER_REV` | Re-measure with `TICKS` |

### Which way is forward

<a id="which-way-is-forward"></a>

The encoder cannot tell you which way the **robot** moved. It only knows its own
count went up or down, and both commissioning signs sit between that count and
the robot frame:

```text
reported_speed = raw_ticks(pin polarity chosen by MOTOR_DIRECTION_SIGN) * ENCODER_SIGN
```

Flip either sign on its own and the reported speed changes sign. **Flip both and
it does not — but the wheel now physically turns the other way.** That pair is
the one hardware fault no measurement can find:

| `MOT` | `ENC` | Wheel turns | Serial reports |
| --- | --- | --- | --- |
| `+1` | `+1` | forward | `+352.18` ✅ |
| `-1` | `+1` | forward | `+352.18` ✅ |
| `+1` | `-1` | forward | `+352.18` ✅ |
| `-1` | `-1` | **backward** | `+352.18` ⚠️ |

The last row is a robot that drives away backwards while the fit succeeds, the
PID converges, odometry reports a straight line, and every number in the log
looks healthy. Nothing inside the robot can see it.

So `DIR` does not ask you which sign to flip. It drives the wheel at a fixed
driver polarity with **no signs applied at all**, asks the one question only a
human can answer — which way did the robot try to roll — and derives both signs
from it:

- `MOTOR_DIRECTION_SIGN` = `+1` if that polarity rolled the robot forward, else
  `-1`. That is precisely what the constant means.
- `ENCODER_SIGN` = the sign that makes the counts it just recorded come out
  positive for the direction you said it went.

One observation, both signs, no guess to get wrong. It then drives the wheel
forward through the derived pair and confirms it measures positive, and after
both wheels it runs them together so you can check the whole robot — because
each wheel answered on its own can still be answered against the wrong end.

There is deliberately no command to set a sign by hand. Hand-setting is how a
robot reaches the last row of that table.

When any sign differs from the compiled-in value, the result block ends with:

```text
WARN,MEASURED_WITH_OVERRIDDEN_SIGNS,PASTE_THESE_TOO
SIGN,1,ENC,1,MOT,1
SIGN,2,ENC,1,MOT,-1
INFO,PASTE_INTO_MiniRobotGeometry.h,IN_BOTH_SKETCH_FOLDERS
// ---------------------------------------------------------
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {1, 1};
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {1, -1};
// ---------------------------------------------------------
```

Those two lines and the four `FF_` lines are one package: the fit describes a
robot wired to those signs, so paste both or neither, into
`MiniRobotGeometry.h` in **both** sketch folders.

## 9. How the fit is computed

- **Stalled points are excluded.** Anything below `MIN_VALID_SPEED_MM_S`
  (15 mm/s) is stiction and dead zone, where speed is not linear in PWM.
  Including it would drag the line down and make the firmware under-drive at
  every speed. Those points still print, tagged `STALL`, so you can see the
  dead zone.
- **Both sweep directions feed one fit**, which averages the hysteresis
  described above.
- **Least squares runs streaming**, Welford-style over the means, variances and
  covariance. No sample array — the Uno has 2 KB of SRAM — and no precision
  loss from accumulating large raw sums in 32-bit float.
- **PWM runs at ~31.4 kHz** (Timer1, prescaler 1), the same as the main
  firmware. PWM frequency changes the effective torque at a given duty, so
  measuring at the default 490 Hz would produce a fit the firmware could not
  use.

Build footprint on an Uno: 18854 bytes of flash (58%) and 489 bytes of static
RAM (24%).
