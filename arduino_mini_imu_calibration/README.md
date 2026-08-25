# mini — IMU calibration

Bench sketch that measures this robot's MPU9250 / MPU9255 / MPU6500 and prints
the constants ready to paste into `arduino_mini_base_controller/`.

It is a standalone tool: no odometry, no PID, no ROS serial protocol. Flash it,
calibrate, paste the constants it prints into the main firmware, then flash the
main firmware back.

Section 6 of the [repository README](../README.md) describes how the firmware
uses the IMU. This is the reference for measuring it.

## 1. What it measures, and why

The firmware runs a Mahony filter over the accelerometer and gyro (and
optionally the magnetometer) and publishes the fused heading as the robot's
yaw. Four things that filter depends on are not measured anywhere in the
firmware:

| What | Where it lives now | Why it matters |
| --- | --- | --- |
| Gyro zero-rate offset | `calibrateGyro()`, re-measured at every boot | The boot calibration is only as good as the sensor is quiet. Nothing in the firmware tells you whether it is. |
| Accelerometer bias and scale | **nowhere — the firmware applies none** | The accelerometer is the filter's only gravity reference. Its errors go into roll, pitch, and the attitude yaw is computed against. |
| Hard and soft iron | `magOffset*` / `magScale*` in `Imu.cpp`, shipped at `0` / `1` | Uncalibrated, which is one reason nine-axis fusion is off by default. |
| `IMU_YAW_SIGN` | `Config.h`, a commissioning value | A left turn must make yaw increase (REP-103). Until someone turns the robot and looks, it is a guess. |

This sketch measures all four and prints them ready to paste. It also answers
the question the firmware's comments keep deferring — **whether the
magnetometer is usable on this robot at all** — by spinning the wheels and
watching what motor current does to the field.

### The sensor must be configured the way the firmware configures it

Full-scale range sets what one LSB is worth, and the DLPF sets how much noise
survives averaging. A bias measured at one setting does not describe a sensor
running at another. `ImuDriver.cpp` therefore repeats `initMPU()` from `Imu.cpp`
exactly:

| Setting | Value |
| --- | --- |
| Accelerometer range | ±2 g |
| Gyro range | ±250 °/s |
| Accel and gyro DLPF | 41 Hz |
| Sample rate | 200 Hz |
| Magnetometer | AK8963, continuous mode 2, 16-bit, 100 Hz |

**If you change one, change the other.**

## 2. Before you start

- [ ] **IMU wired and powered.** `SDA` → `A4`, `SCL` → `A5`, `GND` common, `VCC`
      to 3.3 V or 5 V depending on your module. Nothing else is needed for any
      command except `MOTORS`.
- [ ] **Warmed up.** Leave the robot powered for a few minutes first. The gyro
      bias moves while the die warms, so a bias measured cold is one the sensor
      has already left behind. `GYRO` reports the temperature at both ends of
      the run and warns if it was still climbing.
- [ ] **A flat, level surface**, and for `ACCEL` something you can rest the
      robot on six different ways — a table edge and a book work fine.
- [ ] **Away from steel and magnets** for `MAG`. A steel desk frame, a laptop,
      or a speaker will be calibrated into the result as if it were part of the
      robot.
- [ ] **On blocks with a charged battery** for `MOTORS`, which spins both
      wheels up to PWM 200.

Wiring for `MOTORS` is identical to the main firmware (TB6612FNG on D10/D5/D4
and D9/D7/D8, `STBY` on D12). Encoders are not used.

## 3. Procedure

**1. Upload and connect.** Open the sketch in the Arduino IDE, select Arduino
Uno, upload. Open a serial terminal at **115200 baud** (the same rate the ROS
bridge uses — nothing here is parsed by ROS), line ending "Newline" or
"Carriage return".

```text
READY,mini_imu_calibration_v1
INFO,IMU,MPU9250,ADDR_0x68,MAG_PRESENT,MAG_ASA,1.176,1.180,1.164
INFO,SEND_CHECK_FIRST_THEN_CAL,OR_HELP_FOR_COMMANDS
INFO,LET_THE_IMU_WARM_UP_FOR_A_FEW_MINUTES_BEFORE_CALIBRATING
INFO,CONFIG,gyro_samples=2000,accel_samples_per_face=400,mag_seconds=30
INFO,ESTIMATED_SECONDS,gyro=10,accel_per_face=2,mag=40
```

`INFO,IMU,NONE` means the IMU is not answering on the bus. Check `A4`/`A5` and
the module's supply voltage before going further.

**2. Check the sensor.** Send `CHECK`. Three seconds:

```text
CHECK,READ_RATE_HZ,780
CHECK,GRAVITY_G,0.998,OK
CHECK,GYRO_NOISE_DEG_S,0.061,0.058,0.055,OK
CHECK,GYRO_BIAS_DEG_S,-1.284,0.451,0.093,QUICK_LOOK_ONLY,RUN,GYRO,FOR_THE_REAL_ONE
CHECK,FIELD_UT,48.2,PLAUSIBLE
CHECK,TEMP_C,29.4
```

Gravity is the magnitude of the accelerometer vector, which is 1 g at rest
whatever the orientation — so it needs no assumption about how the IMU is
mounted, and a bad value means a bad scale or a robot that was moving.

**3. Calibrate.** Send `CAL` and follow the `ASK,` lines. It runs the health
check, the gyro, the six accelerometer faces, the yaw sign and the
magnetometer, then prints everything ready to paste. About ten minutes, most of
it spent moving the robot around. Each stage is also its own command if you
only want one.

At any `ASK,` prompt, **X cancels** and anything else — including a bare Enter —
continues.

**4. Paste the constants.** `CAL` finishes with the blocks to copy:

```text
INFO,PASTE_INTO_arduino_mini_base_controller/Imu.cpp
// ---------------------------------------------------------
static float magOffsetX = 12.44f, magOffsetY = -5.10f, magOffsetZ = 3.87f;
static float magScaleX = 1.0213f, magScaleY = 0.9871f, magScaleZ = 0.9934f;
// ---------------------------------------------------------

INFO,ACCEL_CORRECTION_IS_NEW_CODE,ADD_IT_NEXT_TO_THE_OTHER_STATICS
// ---------------------------------------------------------
static const float accelOffsetX = 0.01230f, accelOffsetY = -0.00451f, accelOffsetZ = 0.02104f;
static const float accelScaleX = 1.00214f, accelScaleY = 0.99873f, accelScaleZ = 1.00431f;
// then in readAccelGyro(), replace the three accel lines with:
//   accelX = ((float)rawAx / ACCEL_SCALE - accelOffsetX) * accelScaleX;
//   accelY = ((float)rawAy / ACCEL_SCALE - accelOffsetY) * accelScaleY;
//   accelZ = ((float)rawAz / ACCEL_SCALE - accelOffsetZ) * accelScaleZ;
// ---------------------------------------------------------

INFO,PASTE_INTO_arduino_mini_base_controller/Config.h
// ---------------------------------------------------------
const float IMU_YAW_SIGN = 1.0f;
// ---------------------------------------------------------
```

The two magnetometer lines **replace** the existing ones in `Imu.cpp`. The
accelerometer block is **new code** — the firmware has no accel correction
today, which is why the three lines it belongs in are printed with it.

`RESULT` reprints all of this at any time.

**5. Check the magnetometer against the motors.** Put the robot on blocks and
send `MOTORS`. See section 6.

**6. Re-flash the main firmware** with the pasted constants.

## 4. Commands

| Command | Effect |
| --- | --- |
| `CAL` | Guided full run: `CHECK`, `GYRO`, `ACCEL`, `YAWSIGN`, `MAG`, then the table and the paste blocks. Stops if a stage fails. |
| `CHECK` | Health check: part, I2C read rate, gravity magnitude, gyro noise and rough bias, field magnitude, temperature. A few seconds. **Run this first.** |
| `GYRO` | Zero-rate offset, per-axis noise, and how far the offset moved across the run. 10 s at the default sample count. Robot still. |
| `ACCEL` | Six-face bias and scale. Prompts for each face, in any order. |
| `FACE` | Measure one face wherever the robot is sitting right now. For redoing a single face that came back `NOT_FLAT`. |
| `MAG[,<seconds>]` | Hard and soft iron from a figure-of-eight sweep, then a check pass that scores it. 10–120 s, default 30 plus a 10 s check. |
| `MOTORS` | Spin both wheels at PWM 80/140/200 and report what the field and the gyro do. **Robot on blocks.** |
| `YAWSIGN` | Asks you to turn the robot left, integrates the Z gyro, and reports `IMU_YAW_SIGN`. |
| `LIVE[,<seconds>]` | Stream corrected accel, gyro, roll, pitch, integrated yaw, and (if fitted) field magnitude and heading. Default 30 s, max 300. |
| `RAW[,<seconds>]` | Stream raw LSB counts, no correction. For wiring and byte-order faults that scaling hides. |
| `RESULT` | Reprint the results table and the paste blocks. |
| `SAMPLES,<gyro>,<accel>` | Sample counts: gyro 200–6000 (total, split in half), accel 50–2000 per face. Default 2000 and 400. |
| `MAGTIME,<seconds>` | Magnetometer sweep length, 10–120. Default 30. |
| `CONFIG` | Print the current settings and time estimates. |
| `STOP` | Coast both motors. |
| `HELP` | Command list. |
| `PING` | `PONG`. |

Commands are case-insensitive and terminated by `\n`, `\r` or `:`.

**Sending any character while a measurement is running aborts it** and coasts
the motors. At an `ASK,` prompt, **X** cancels; anything else continues.

Settings are not persisted — they reset to the defaults on every reset or
re-upload. Neither are results: they live in RAM until you paste them.

## 5. Output lines

| Line | Fields |
| --- | --- |
| `G,...` | `axis, bias_deg_s, noise_deg_s, drift_deg_s` |
| `AF,...` | `±axis, x_g, y_g, z_g, OK\|NOT_FLAT` — one measured face |
| `A,...` | `axis, bias_g, scale` — the fitted result |
| `M,<t>,...` | `seconds, samples, spanX, spanY, spanZ` — magnetometer progress |
| `M,<axis>,...` | `axis, offset_uT, scale` — the fitted result |
| `MI,...` | `pwm, field_deg, field_uT, gyro_z_noise_deg_s` — motor interference |
| `CHECK,...` | `what, values…, verdict` |
| `L,...` / `RAW,...` | live streams; `HDR,` names their columns |
| `ASK,` | wants an answer: X cancels, anything else continues |
| `INFO,` / `WARN,` / `ERR,` / `ACK,` | progress, diagnostics, failures, completion |

## 6. Reading the results

### Gyro

```text
G,X,-1.2840,0.0612,0.0104
G,Y,0.4513,0.0587,-0.0071
G,Z,0.0931,0.0554,0.0088
INFO,TEMP_C,start=28.9,end=29.1
INFO,EXPECTED_YAW_DRIFT_DEG_PER_MIN,0.53
```

- **bias** — the zero-rate offset. A degree or two per axis is normal. The
  firmware re-measures this at every boot, so there is nothing to paste; what
  you are checking is that it is in family and stable.
- **noise** — per-sample standard deviation. Around 0.05–0.15 °/s is healthy at
  the 41 Hz DLPF. Much more means vibration, a noisy supply, or long unshielded
  I2C wires.
- **drift** — second half of the run minus the first. **This is the number that
  matters.** A constant bias costs nothing because the firmware removes it at
  boot; a bias that keeps moving afterwards is exactly what walks the heading
  away. `EXPECTED_YAW_DRIFT_DEG_PER_MIN` converts the Z drift into what you
  will actually see in `/odom`.

Under ~0.05 °/s of drift is good. Over 0.2 °/s and the sketch warns — usually
because the sensor is still warming up.

### Accelerometer

```text
A,X,0.01230,1.00214
A,Y,-0.00451,0.99873
A,Z,0.02104,1.00431
INFO,WORST_OFF_AXIS_G,0.021,APPROX_TILT_DEG,1.2
```

- **bias** — added zero offset, in g. A few hundredths is normal.
- **scale** — multiplier that makes ±1 g read ±1 g. Within a few percent of
  `1.0` is normal; past 10% a face was probably not flat.
- **worst off-axis** — how level the surfaces you used were. It does not enter
  the fit; it tells you how much to trust it.

The correction the firmware applies is `(raw − bias) × scale`, per axis.

### Magnetometer

```text
M,X,12.44,1.0213
M,Y,-5.10,0.9871
M,Z,3.87,0.9934
INFO,SPANS_UT,94.2,97.5,96.9
INFO,FIELD_ESTIMATE_UT,48.1
INFO,CORRECTED_FIELD_UT,48.3,SPREAD_PCT,2.4
```

- **offset** — hard iron: the constant field the robot carries with it, from
  magnets and steel. Subtracted before anything else.
- **scale** — soft iron, as the ratio between axis spans. On a small robot the
  real value is a few percent; a large one usually means an axis was not fully
  rotated rather than real iron.
- **spans** — how far each axis swung during the sweep. Three similar numbers
  mean the rotation covered the sphere. An axis below 25 µT is refused.
- **field** — should land in Earth's 25–65 µT. Outside that, something on the
  robot dominates the measurement.
- **spread** — the quality number, from the check pass. After correction every
  sample should sit on a sphere of one radius however the robot is held, so the
  spread of that radius is how well it worked. Under ~5% is good; over 8% the
  sketch warns.

The min/max method behind the offsets cannot check itself — the extremes it
fits are the samples it fitted — which is exactly why the separate check pass
exists.

### Motor interference

```text
INFO,BASELINE_FIELD_UT,48.3,GYRO_Z_NOISE,0.058
MI,80,1.2,0.4,0.089
MI,140,3.7,1.1,0.143
MI,200,6.9,2.4,0.201
INFO,WORST_FIELD_DEVIATION_DEG,6.9
WARN,MOTORS_MOVE_THE_FIELD,KEEP_MAGNETOMETER_FUSION_OFF
```

`field_deg` is the angle between the field measured with the motors off and
with them running — that is, roughly the heading error the distortion would
cause. **Over about 5°, heading becomes a function of throttle** and nine-axis
fusion will hurt more than it helps, however good the calibration is. Moving
the IMU further from the motors and the battery is the only real fix.

`gyro_z_noise` climbing with PWM is vibration reaching the gyro. It rectifies
into an apparent rotation the firmware never catches, because it re-measures
its bias while parked.

### Yaw sign

`YAWSIGN` integrates the Z gyro while you turn the robot left and reports
`+1.0` if the gyro already agrees with ROS (left turn → yaw increases) or
`-1.0` if it is inverted. It also warns if gravity is not on Z — a side-mounted
IMU is not something a sign flip can fix.

## 7. Tuning the run

- **`SAMPLES,<gyro>,<accel>`** — the gyro run is split in half to measure
  drift, so more samples means both a better average and a longer baseline to
  see drift over. 2000 (10 s) is a reasonable default; 6000 (30 s) gives a much
  better drift number if you are chasing heading problems.
- **`MAGTIME,<seconds>`** — 30 s is enough for an unhurried figure-of-eight.
  Extend it if the `M,` progress lines show a span still growing when time runs
  out.
- Everything else is a threshold in `Config.h`, with a comment saying what it
  is for.

## 8. Troubleshooting

| Message or symptom | Cause | Fix |
| --- | --- | --- |
| Nothing, or garbage characters | Terminal at the wrong baud | 115200 |
| `INFO,IMU,NONE` / `ERR,NO_IMU_FOUND` | Not answering at `0x68` or `0x69` | `SDA` → `A4`, `SCL` → `A5`, check the module's supply voltage and that `GND` is common |
| `CHECK,READ_RATE_HZ` far below ~500 | I2C struggling | Shorter wires, and check the pull-ups on the module |
| `CHECK,GRAVITY_G,...,BAD_SCALE_OR_MOVING` | Robot moving, or the accel range is not ±2 g | Hold it still; if it persists, `ImuDriver.cpp` and `Imu.cpp` disagree about `ACCEL_CONFIG` |
| `ERR,MOVED_DURING_RUN` | Something touched the robot, or it is on a springy surface | Solid table, hands off. The test scales with each axis's own noise, so it is not a noisy sensor |
| `ERR,NEVER_WENT_STILL` | Continuous vibration | Move away from anything running; check nothing is resting against the robot |
| `WARN,STILL_WARMING_UP` | Die temperature moved more than 1.5 °C during the run | Leave it powered a few minutes and re-run `GYRO` |
| `WARN,BIAS_STILL_MOVING` | Bias differs between the two halves | Same fix. If it persists when warm, that drift is real and caps how long yaw stays true between corrections |
| `ERR,NOT_RESTING_ON_ONE_FACE` | Robot propped at an angle, so two axes share gravity | Lie it flat on a level surface; one axis must carry gravity alone |
| `WARN,ODD_SCALE` | A face was not flat | `FACE` re-measures a single face without repeating the sequence |
| `ERR,AXIS_NOT_ROTATED_ENOUGH` | The sweep never turned that axis through the field | Roll and pitch the robot as well as spinning it, then re-run `MAG` |
| `ERR,TOO_FEW_SAMPLES` | AK8963 stopped answering | Check wiring; on an MPU6500 there is no magnetometer at all |
| `WARN,FIELD_OUTSIDE_25_TO_65_uT` | Something magnetic near the IMU | Usually the motors or a steel chassis part. Move the IMU or accept that the magnetometer is unusable |
| `WARN,POOR_FIT` | Corrected field still changes with orientation | Slower, more complete rotation, away from steel furniture |
| `ERR,NOT_ENOUGH_ROTATION` | The `YAWSIGN` turn was too small to read | Turn a full 90° within the window |
| `WARN,IMU_IS_NOT_MOUNTED_FLAT` | Gravity is not on Z | Remount the IMU flat — `IMU_YAW_SIGN` cannot fix a side-mounted sensor |
| Roll/pitch drift in `LIVE` with the robot still | Gyro drift or an uncorrected accelerometer | Re-run `GYRO` warm, then `ACCEL` |

## 9. How the numbers are computed

- **Gyro bias** is the mean of two half-runs, and the drift is the difference
  between them. Noise is the mean of the two halves' standard deviations, so it
  is noise around a steady mean and does not include the drift.
- **Accelerometer bias and scale** come from opposite faces: with one axis up
  and then down, `bias = (up + down) / 2` and `scale = 2 g / (up − down)`. The
  faces are not named in advance — the sketch works out which face it is
  looking at from gravity, so nothing depends on knowing how the IMU is
  mounted, and you can present them in any order.
- **Hard iron** is the midpoint of each axis's extremes over the sweep; **soft
  iron** is each axis's span against the average span. A separate check pass
  then measures the corrected field's spread, because a min/max fit cannot
  score itself.
- **Statistics run streaming**, Welford-style over the means and variances,
  with the extremes tracked alongside. No sample array — the Uno has 2 KB of
  SRAM, and a 30 second magnetometer sweep is 3000 samples.
- **Motion rejection** compares each axis's peak-to-peak against the larger of
  a fixed limit and eight times its own standard deviation, so a noisier module
  is not mistaken for a moving robot.

Build footprint on an Uno: about 30 KB flash (94%) and 791 bytes of static RAM
(39%). It is a bench tool at its final size, but if you need room to extend it,
`RAW` and `MOTORS` are the two commands nothing else depends on.
