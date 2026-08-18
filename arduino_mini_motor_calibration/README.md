# mini — motor feed-forward calibration

Bench sketch that measures this robot's PWM-to-speed curve and least-squares
fits it, so the feed-forward constants in
`arduino_mini_base_controller/arduino_mini_base_controller.ino` come from data
instead of a guess.

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
- [ ] **Direction signs already correct.** If a wheel runs backwards on a
      forward command, fix `MOTOR_DIRECTION_SIGN` / `ENCODER_SIGN` first — no
      fit can repair a sign error, and the sketch will refuse the points.
- [ ] **`MEASURED_COUNTS_PER_REV` correct.** Every mm/s printed here is scaled
      by it (see step 2 of the procedure).

These constants are duplicated from the main firmware and must match it, or the
numbers you paste back will not mean what the firmware thinks they mean:

| Constant | Default | Why it matters here |
| --- | --- | --- |
| `WHEEL_DIAMETER_MM` | `47.0` | converts encoder counts to mm |
| `MEASURED_COUNTS_PER_REV` | `{720, 720}` | scales every speed in the fit |
| `MAX_DRIVE_PWM` | `210` | the fit must span the range the firmware uses |
| `ENCODER_SIGN` | `{1, 1}` | forward must read positive |
| `MOTOR_DIRECTION_SIGN` | `{1, 1}` | flips a wheel wired backwards |

Wiring is identical to the main firmware (TB6612FNG on D10/D5/D4 and D9/D7/D8,
`STBY` on D12, encoders on D2/D6 and D3/D11). The IMU is unused and can stay
connected or be left off.

## 3. Procedure

**1. Upload and connect.** Open the sketch in the Arduino IDE, select Arduino
Uno, upload. Open a serial terminal at **115200 baud** (not the 500000 the ROS
bridge uses — nothing here is parsed by ROS), line ending "Newline" or
"Carriage return".

You should see:

```text
READY,mini_motor_ff_calibration_v1
WARN,PUT_THE_ROBOT_ON_BLOCKS_BOTH_WHEELS_MUST_SPIN_FREE
INFO,SEND_CAL_TO_START_OR_HELP_FOR_COMMANDS
INFO,CONFIG,range=30..210,step=10,settle_ms=400,window_ms=600,passes=2
INFO,ESTIMATED_SECONDS,per_direction=38,full_CAL=152
```

**2. Check counts-per-rev.** Send `TICKS,20`, then turn one wheel by hand
exactly one full turn and read the running total. Do it per wheel. Put the
magnitudes in `MEASURED_COUNTS_PER_REV` here **and** in the main firmware.

The default 720 assumes a 360 PPR encoder on the output shaft (channel A is
decoded on CHANGE, so two counts per encoder cycle). An encoder on the motor
shaft reads that times the gear ratio — thousands. Get this wrong and every
speed in the fit is wrong by the same factor.

**3. Calibrate.** Send `CAL`. It sweeps both wheels, both directions, and takes
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

**4. Paste the constants.** At the end `CAL` prints the block to copy over the
four `FF_*` lines in `arduino_mini_base_controller.ino`:

```text
INFO,PASTE_INTO_arduino_mini_base_controller.ino
// ---------------------------------------------------------
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {1.8734f, 1.9102f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-71.20f, -76.85f};
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {1.8012f, 1.8440f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-68.90f, -74.10f};
// ---------------------------------------------------------
```

`RESULT` reprints it at any time.

**5. Verify.** Send `VERIFY`. It drives a few target speeds through the fresh
fit, open loop, and reports how far off each lands:

```text
V,1,F,60.0,58.4,-2.7
V,1,F,120.0,117.1,-2.4
```

Under about 10% is a good fit — that residual is what PID absorbs. Consistently
larger errors mean the fit is describing something other than the motor; see
troubleshooting below.

**6. Re-flash the main firmware** with the pasted constants. Retune the wheel
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
| `TICKS[,<seconds>]` | Motors off; stream raw encoder totals. Default 30 s, max 300. |
| `STEP,<pwm_step>` | Sweep step size, 1–50. Default 10. |
| `RANGE,<min>,<max>` | Sweep range within 0–210, at least 30 wide. Default 30–210. |
| `DWELL,<settle_ms>,<window_ms>` | Settle 50–5000, window 200–5000. Default 400, 600. |
| `PASSES,<1..4>` | Passes per direction. Default 2. |
| `CONFIG` | Print the current settings and the run-time estimate. |
| `STOP` | Coast both motors. |
| `HELP` | Command list. |
| `PING` | `PONG`. |

Commands are case-insensitive and terminated by `\n`, `\r` or `:`.

**Sending any character while a sweep, `VERIFY`, `HOLD` or `TICKS` is running
aborts it and coasts the motors.** An aborted sweep stores no fit, so the
previous result for that wheel and direction stays untouched.

Sweep settings are not persisted — they reset to the defaults on every reset or
re-upload.

## 5. Output lines

| Line | Fields |
| --- | --- |
| `C,...` | `motor, F\|R, pwm, mm_s, USED\|STALL\|HOLD` — one measured point |
| `R,...` | `motor, F\|R, slope, intercept, r2, samples, deadzone_pwm, mm_s_at_max_pwm` |
| `V,...` | `motor, F\|R, target_mm_s, measured_mm_s, error_pct` |
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
| Nothing, or garbage characters | Terminal at the wrong baud | 115200, not 500000 |
| `ERR,FIT_FAILED,TOO_FEW_MOVING_POINTS` | Wheel never moved: flat battery, `STBY` low, motor or encoder unplugged, wheel jammed | Check power and wiring, confirm the wheel spins free |
| `WARN,WRONG_DIRECTION,...` | Wheel turns opposite to the command, or the encoder reads inverted | Fix `MOTOR_DIRECTION_SIGN` / `ENCODER_SIGN` (here and in the firmware), then re-run |
| `ERR,FIT_REJECTED,NON_POSITIVE_SLOPE` | More PWM produced less speed — not a motor curve | Check for a slipping coupling, a failing driver channel, or a collapsing battery |
| `WARN,LOW_R2,<r2>,...` | Noisy or non-linear measurement | Longer `DWELL`, charged battery, check for mechanical binding |
| `WARN,WHEELS_DIFFER_A_LOT,...` | Left and right slopes differ by more than 25% | Look for drag on the slow wheel; re-check `MEASURED_COUNTS_PER_REV` per wheel |
| `0.000f /*NOT_CALIBRATED*/` in the paste block | That wheel/direction has no valid fit | Run the missing `SWEEP`. The placeholder is deliberately not a plausible number, so a half-done run fails loudly instead of driving wrong |
| `VERIFY` errors much larger than 10% | Battery sagged between the sweep and the check, or the fit was made at a different voltage | Recharge and re-run `CAL` |
| All speeds off by a constant factor | Wrong `MEASURED_COUNTS_PER_REV` | Re-measure with `TICKS` |

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

Build footprint on an Uno: about 14 KB flash (43%) and 477 bytes of static RAM
(23%).
