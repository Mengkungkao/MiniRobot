# mini — motor feed-forward calibration

Bench sketch that measures this robot's PWM-to-speed curve and least-squares
fits it, so the feed-forward constants in
`arduino_mini_base_controller/Config.h` come from data instead of a guess. It
also commissions the two direction signs those constants depend on.

It is a standalone tool that does exactly five things: drive the TB6612
channels, run motors 1 and 2, count encoder ticks per wheel revolution, work
out which way each motor turns, and print the measured points plus the fitted
forward and reverse curves. No IMU, no odometry, no PID, no ROS serial protocol, no closed-loop
anything. Flash it, calibrate, paste the constants it prints into the main
firmware, then flash the main firmware back.

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
- [ ] **Counts per revolution measured.** Send `CPR` and turn each wheel by
      hand. Every mm/s printed here and in the firmware is scaled by it, so a
      wrong value is wrong by exactly that factor and nothing downstream can
      see it. `CPR` is step 2 of the procedure below.

These constants are duplicated from the main firmware and must match it, or the
numbers you paste back will not mean what the firmware thinks they mean:

| Constant | Default | Why it matters here |
| --- | --- | --- |
| `WHEEL_DIAMETER_MM` | `47.0` | converts encoder counts to mm |
| `MEASURED_COUNTS_PER_REV` | measured by `CPR` | scales every speed in the fit |
| `MAX_DRIVE_PWM` | `210` | the fit must span the range the firmware uses |
| `ENCODER_SIGN` | commissioned by `DIR` | forward must read positive |
| `MOTOR_DIRECTION_SIGN` | commissioned by `DIR` | which driver polarity is forward |

Wiring is identical to the main firmware (TB6612FNG on D10/D5/D4 and D9/D7/D8,
`STBY` on D12, encoders on D2/D6 and D3/D11). The IMU is unused and can stay
connected or be left off.

## 3. Procedure

**1. Upload and connect.** Open the sketch in the Arduino IDE, select Arduino
Uno, upload. Open a serial terminal at **115200 baud** (the same rate the base
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

**2. Measure counts per revolution.** Send `CPR`. This is the first
measurement for a reason: it is the scale factor under every other number in
this sketch and in the firmware.

It takes each wheel in turn, with the motors off. Mark the tyre and the chassis
so a full turn is unambiguous, turn the wheel the requested number of turns by
hand, and press Enter:

```text
INFO,CPR_START,MOTORS_ARE_OFF,THIS_IS_MEASURED_BY_HAND
INFO,CPR_SPIN,MOTOR_1(CH_A),TURN_THIS_WHEEL_EXACTLY_5,FULL_TURNS_BY_HAND
INFO,MARK_THE_TYRE_AND_THE_CHASSIS_SO_ONE_TURN_IS_UNAMBIGUOUS
ASK,PRESS_ENTER_WHEN_DONE,X_THEN_ENTER_TO_ABORT
CPR,LIVE,412,0
CPR,LIVE,1183,0
CPR,LIVE,2601,0
CPR,1,TICKS,2601,TURNS,5,COUNTS_PER_REV,520.2
```

The `CPR,LIVE` rows stream both wheels while you turn one, so a pair of swapped
encoders shows up as the wrong column moving. Default is 5 turns; `CPR,10` for
a careful job, `CPR,1` for a quick re-check, `CPR,<motor>,<turns>` for one
wheel. More turns is strictly better — the error is how precisely you stop on
the mark, a fixed number of counts however far the wheel went, so it divides
down.

The result is used for the rest of the session immediately, and printed ready
to paste into `MiniRobotGeometry.h` in **both** sketch folders.

It has to be by hand. The number is counts per turn of the *wheel*, after the
gearbox, and nothing on the board knows the gear ratio — driving the motor
would only count ticks against a clock, which says nothing about how far the
wheel actually went.

The shipped 520 is this robot's hardware: a 13 PPR encoder on the motor shaft,
decoded x2 (channel A on CHANGE, so two counts per encoder cycle), behind a
1:20 gearbox — 13 x 2 x 20 = 520. That arithmetic is a prediction; `CPR` is the
measurement.

**Then check it with `ROT`.** This does the same sum backwards: it drives what
the sketch now believes is exactly one turn and brakes, so you can see whether
that many counts really is a whole revolution. Line the marks up, send `ROT`,
and watch the wheel:

```text
WARN,ROT_DRIVES_THE_MOTORS,ROBOT_ON_BLOCKS_WHEELS_SPINNING_FREE
INFO,ROT,LINE_A_MARK_ON_EACH_TYRE_UP_WITH_A_MARK_ON_THE_CHASSIS
ASK,PRESS_ENTER_WHEN_READY,X_THEN_ENTER_TO_ABORT
INFO,ROT_RUN,MOTOR_1(CH_A),TURNS,1,TARGET_COUNTS,520
ROT,1,TARGET,520,ACTUAL,526,STOP_ERROR,6,DEG,4.2
```

| The mark… | Means | Fix |
| --- | --- | --- |
| comes back round | the count is right | nothing |
| stops short | `MEASURED_COUNTS_PER_REV` too **low** | re-run `CPR` with more turns |
| goes past | `MEASURED_COUNTS_PER_REV` too **high** | re-run `CPR` with more turns |

`ROT,5` multiplies the error by five and turns a discrepancy you might argue
about into one you cannot miss. `ROT,<motor>,<turns>` does one wheel.

Read `STOP_ERROR` carefully: it is how far past the target **count** the wheel
coasted before the brake caught it, so it measures this command's own stopping
accuracy — a few degrees is normal and healthy. It says nothing about whether
the count is right. That verdict is visual, and it has to be: the encoder
cannot see the tyre.

This is the check that catches a count wrong by a clean factor — a miscounted
hand turn, an x2/x4 decoding mix-up, a gear ratio off by one stage. Every one
of those produces a perfectly self-consistent number that `CPR` reports
happily, and that every speed in the fit is then scaled by.

`ROT` drives raw pin polarity with no signs applied, so it works before `DIR`
has run and whichever way the wheel happens to turn — only the magnitude
matters here.

**3. Commission the direction signs.** Send `DIR`. It takes about thirty
seconds and it is not optional — see [§8](#which-way-is-forward) for what goes
wrong without it.

`DIR` spins each wheel at a fixed driver polarity, **ignoring both signs**, and
asks you which way the robot tried to roll:

```text
INFO,DIR_START,ROBOT_ON_BLOCKS,BOTH_WHEELS_MUST_SPIN_FREE
INFO,FORWARD_MEANS_THE_TOP_OF_THE_WHEEL_MOVES_TOWARD_THE_FRONT
INFO,DIR_PROBE,MOTOR_1(CH_A),pwm=40,WATCH_THIS_WHEEL_NOW
INFO,DIR_RAW_TICKS,-631
ASK,WHICH_WAY_DID_THE_ROBOT_TRY_TO_ROLL,F=FORWARD,B=BACKWARD,X=ABORT
```

Answer `F` or `B` (or `X` to abort) by watching the **wheel**, not the numbers.
From that one answer it derives both signs at once and confirms them by driving
the wheel forward through them:

```text
INFO,DIR_SOLVED,MOTOR_1(CH_A),ENC,-1,MOT,1
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
INFO,SWEEP_START,MOTOR_1(CH_A),FORWARD
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

**6. Re-flash the main firmware** with the pasted constants. Retune the wheel
PID afterwards if you had tuned it against the old feed-forward: the loop now
starts from a much better guess, so the gains that used to be right are
probably too hot.

## 4. Commands

| Command | Effect |
| --- | --- |
| `CAL` | Calibrate both motors, both directions, then print the table and paste block. |
| `CAL,<motor>` | Same for one motor (`1` = TB6612 channel A, `2` = channel B). |
| `SWEEP,<motor>[,R]` | One motor, one direction. `R` = reverse. Fits and stores that one direction. |
| `RESULT` | Reprint the fit table and the paste block. |
| `CPR[,<turns>]` | Hand-spin measurement of encoder counts per wheel revolution, both wheels, motors off. Default 5 turns, max 20. Session-only; printed ready to paste. **Run this first.** |
| `CPR,<motor>,<turns>` | Same for one wheel. |
| `ROT[,<turns>]` | Drive `countsPerRev × turns` counts under power and brake, so you can see whether the mark comes back round. Default 1 turn, max 20. The check that `CPR`'s answer is real. |
| `ROT,<motor>,<turns>` | Same for one wheel. |
| `DIR` | Commission **both** direction signs for both wheels from one look at each. Spins a wheel at a fixed driver polarity, asks which way the robot rolled, derives `MOTOR_DIRECTION_SIGN` and `ENCODER_SIGN` from the answer, and confirms them. Session-only; the values are printed ready to paste. **Run this before `CAL`.** |
| `DIR,<motor>` | Same for one wheel. Skips the both-wheels-together confirmation. |
| `SIGN` | Print the current commissioning signs for every wheel. |
| `STEP,<pwm_step>` | Sweep step size, 1–50. Default 10. |
| `RANGE,<min>,<max>` | Sweep range within 0–210, at least 30 wide. Default 10–210. |
| `DWELL,<settle_ms>,<window_ms>` | Settle 50–5000, window 200–5000. Default 400, 600. |
| `PASSES,<1..4>` | Passes per direction. Default 2. |
| `CONFIG` | Print the current settings and the run-time estimate. |
| `STOP` | Coast both motors. |
| `HELP` | Command list. |

Commands are case-insensitive and terminated by `\n`, `\r` or `:`.

**Sending any character while a sweep or `ROT` is running aborts it and coasts
the motors.** An aborted sweep stores no fit, so the
previous result for that wheel and direction stays untouched.

`CPR` and `DIR` are the exception: they are asking you questions, so they read
what you type as the answer rather than as an abort. Abort either by answering
`X`.

Sweep settings are not persisted — they reset to the defaults on every reset or
re-upload.

## 5. Output lines

| Line | Fields |
| --- | --- |
| `C,...` | `motor, F\|R, pwm, mm_s, USED\|STALL` — one measured point |
| `R,...` | `motor, F\|R, slope, intercept, r2, samples, deadzone_pwm, mm_s_at_max_pwm` |
| `DIR,...` | `motor, F, pwm, mm_s, OK\|FAILED` — `DIR`'s post-commission self check |
| `ASK,...` | waiting for an operator answer on the serial line |
| `CPR,LIVE,...` | `motor 1, motor 2` — running raw totals while you turn a wheel |
| `CPR,...` | `motor, TICKS, n, TURNS, n, COUNTS_PER_REV, n` — one wheel's result |
| `ROT,...` | `motor, TARGET, n, ACTUAL, n, STOP_ERROR, n, DEG, n` |
| `SIGN,...` | `motor, ENC, sign, MOT, sign` — one row per wheel |
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
percent of each other on the same wheel, the two wheels within ~10%, dead zones
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
| Nothing, or garbage characters | Terminal at the wrong baud | 115200 |
| `ERR,FIT_FAILED,MOTOR_n,used=0,...` + `CAUSE,NO_ENCODER_MOTION` | The wheel may be spinning, but the encoder netted zero counts. Most often channel **B** is floating or on the wrong pin — the ISR then reads a constant B and counts `+1, -1, +1, -1`, netting zero however fast the wheel turns. Also: flat battery, `STBY` low, channel A unplugged, wheel jammed | `CPR` catches it first: turn that wheel by hand and if the count stays near zero while the other column moves cleanly, it is the encoder wiring — motor 1 B = `D6`, motor 2 B = `D11`. `DIR` reports the same fault in two seconds |
| `ERR,FIT_FAILED,MOTOR_n,...` + `CAUSE,WHEEL_MEASURED_BACKWARDS` | The signs were never commissioned. Normal on a differential drive, where one motor is mounted mirrored | `DIR`, then `CAL`. Do not try to work out which sign to flip by hand — that is the mistake `DIR` exists to remove |
| `ERR,FIT_FAILED,MOTOR_n,...` + `CAUSE,TOO_SLOW_OR_TOO_FEW_POINTS` | The wheel moved, but never above `MIN_VALID_SPEED_MM_S` | Charge the battery, check `STBY`, confirm the wheel spins free |
| `WARN,SOME_POINTS_RAN_BACKWARDS,...` | Part of the sweep fitted and part ran backwards — one sign cannot describe a wheel that changed its mind. Usually a slipping coupling or an intermittent encoder channel | `DIR`, then look for the mechanical fault before trusting the fit |
| `ERR,DIR_INCONSISTENT,...` | The two wheels were answered against different ends of the robot | Re-run `DIR` and answer both against the same front |
| `ERR,DIR_SELF_CHECK_FAILED,...` | The wheel stalled during the confirmation, or the answer was given for the other wheel | Re-run `DIR,<motor>` and watch the wheel it names |
| `ERR,TOO_FEW_STEPS,n,...` | `RANGE`/`STEP` leave fewer than four distinct PWM points | Widen `RANGE` or shrink `STEP` |
| `ERR,FIT_REJECTED,NON_POSITIVE_SLOPE` | More PWM produced less speed — not a motor curve | Check for a slipping coupling, a failing driver channel, or a collapsing battery |
| `WARN,LOW_R2,<r2>,...` | Noisy or non-linear measurement | Longer `DWELL`, charged battery, check for mechanical binding |
| `WARN,WHEELS_DIFFER_A_LOT,...` | The two motors' slopes differ by more than 25% | Look for drag on the slow wheel; re-check `MEASURED_COUNTS_PER_REV` per wheel |
| `0.000f /*NOT_CALIBRATED*/` in the paste block | That wheel/direction has no valid fit — the preceding `WARN,MISSING_FIT` lines name which | Fix the cause the `ERR,FIT_FAILED` line reported, then re-run. **Never paste a block containing `0.000f`** — the firmware treats a non-positive slope as "no feed-forward" and falls back to pure PID, so that wheel will be visibly slow |
| All speeds off by a constant factor | Wrong `MEASURED_COUNTS_PER_REV` | Re-measure with `CPR`, using more turns |
| `ERR,CPR_NO_COUNTS,...` | The wheel turned but the encoder netted near zero, or it never turned | Channel B floating or miswired (motor 1 `D6`, motor 2 `D11`); if the *other* column moved instead, the two encoders are swapped |
| `WARN,CPR_CROSSTALK,...` | The other wheel counted too | Encoders cross-wired, or the wrong wheel was turned |
| `WARN,WHEELS_SHOULD_MATCH,...` | The two wheels measured more than 5% apart | Same part, same gearbox — one count is wrong. Re-run `CPR` with more turns and stop precisely on the mark |
| `ROT` stops short or overshoots the mark | `MEASURED_COUNTS_PER_REV` is too low / too high | Re-run `CPR` with more turns; use `ROT,5` to magnify what is left |
| `ERR,ROT_NO_PROGRESS,...` | The wheel stopped counting mid-rotation | Jammed wheel, flat battery, or the encoder dropped out — check `STBY` and the encoder wiring |

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
- **Four things are called "direction"** in the source, and three of them are
  numbers. If you edit the sketch, read the block at the top of `Config.h`
  first: `bool reverse` is what was commanded, `dirSlot` (`FWD`/`REV`) is only
  ever an array index, the sign of a speed says which way the robot actually
  moved, and `motorSign`/`encoderSign` are per-robot wiring corrections that are
  `+1`/`-1` purely by coincidence. Converting between the first two goes through
  `dirSlotFor()` / `slotIsReverse()`, and `static_assert`s refuse to build if the
  slots stop lining up with `bool reverse` — the results array indexes on that.
- **PWM runs at ~31.4 kHz** (Timer1, prescaler 1), the same as the main
  firmware. PWM frequency changes the effective torque at a given duty, so
  measuring at the default 490 Hz would produce a fit the firmware could not
  use.

Build footprint on an Uno: 20756 bytes of flash (64%) and 441 bytes of static
RAM (22%).
