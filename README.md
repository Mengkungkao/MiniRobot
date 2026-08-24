# mini — ROS 2 Two-Wheel Differential Drive Robot

A TurtleBot3-style ROS 2 Humble stack for a small two-wheel differential drive
robot: Raspberry Pi 4B + Arduino Uno + TB6612FNG + MPU9250/9255/6500 + COIN-D6
LiDAR, with SLAM Toolbox mapping and Nav2 autonomous navigation.

## 0. Get the code

Clone this repository on both the Raspberry Pi and the Ubuntu workstation
(or copy it over some other way, e.g. `scp`/USB drive) before running any of
the steps below:

```bash
git clone <this-repository-url> MiniRobot
cd MiniRobot
```

Everything below assumes your current directory is the repository root.

## 1. System architecture

```text
Ubuntu workstation
  RViz2 (+ Goto and Teleop panels from mini_rviz_plugins)
  SLAM Toolbox / AMCL
  Nav2 global planner (A*)
  Nav2 Rotation Shim + Regulated Pure Pursuit controller
  Global and local costmaps
  Predefined waypoint client
            |
            | ROS2 DDS over Ethernet/Wi-Fi
            v
Raspberry Pi 4B running Ubuntu Server + ROS2 Humble
  robot_state_publisher
  COIN-D6 -> /scan
  cmd_mux: priority mux over /cmd_vel (Nav2), /cmd_vel_teleop, /cmd_vel_manual,
           plus a front obstacle stop gate driven by /scan -> /cmd_vel_out
  serial bridge: /cmd_vel_out -> Arduino
  Arduino telemetry -> /odom, /imu/data, /joint_states and TF
            |
            | USB serial, 500000 baud
            v
Arduino Uno
  TB6612FNG dual H-bridge, ~31 kHz PWM
  Two quadrature encoders
  MPU9250 / MPU9255 / MPU6500, Mahony fusion (auto-detected, optional)
  Two independent wheel-speed PID loops, with breakaway and stall handling
  500 ms communication watchdog
  Latched emergency stop
```

Every box below the workstation is optional and the rest keeps working without
it: the LiDAR, the mux, the Arduino, the IMU, `robot_state_publisher`. What each
one costs you when it is gone, and how the stack tells you, is in
[Running with parts switched off or missing](#running-with-parts-switched-off-or-missing).

SSH is used to open a terminal and manage the Raspberry Pi. ROS2 topics are
transported directly by DDS across the network; they are not carried inside
the SSH session.

Nav2's `navigation_launch.py` always publishes its final velocity command on
the plain `/cmd_vel` topic (its internal `velocity_smoother` remap to that name
is hardcoded and cannot be renamed from an outer launch file without
namespacing the whole Nav2 stack). Because of that, `cmd_mux` reads `/cmd_vel`
directly as its Nav2 input and republishes the arbitrated, safety-gated result
on `/cmd_vel_out`, which is the topic the serial bridge actually subscribes to.
Manual test commands and teleop are unaffected: they already use
`/cmd_vel_manual` and `/cmd_vel_teleop` (see section 13).

## 2. Important coordinate convention

Odometry follows ROS conventions:

- `+X`: forward
- `+Y`: left
- positive yaw: counter-clockwise/left turn
- TF chain while mapping: `map -> odom -> base_footprint -> base_link -> laser`

### Which end is the front

**The LiDAR end is the front.** The COIN-D6 sits above the caster with its own
zero mark pointing that way, so `base_link +X`, LiDAR angle zero, "drive
forward", a positive `cmd_vel.linear.x` and the Arduino's positive `VEL` all
mean the same physical direction. The drive axle trails.

The Arduino shares that convention. Nothing rotates anything on the way across
the serial link: velocity commands, odometry, wheel speeds and yaw all cross
unmodified, and the bridge is a unit converter. The firmware used to call the
axle end forward and the Pi rotated every `x`/`y` quantity by 180 degrees to
compensate (`reverse_base_direction`); that parameter is gone, and so is the
whole class of "which frame is this number in" bug that came with it.

The body layout is now one line of firmware, in `MiniRobotGeometry.h`:

```cpp
const int8_t WHEEL_Y_SIGN[MOTOR_COUNT] = {-1, +1};
```

That is the wheel's position along `+Y` in units of half the track, which is
exactly what differential-drive kinematics wants:

```text
wheel speed = v - WHEEL_Y_SIGN[i] * omega * TRACK_WIDTH_MM/2
```

### Motors are numbered, not sided

A motor number names a TB6612 channel and nothing else:

| | channel | pins | position |
| --- | --- | --- | --- |
| motor 1 | A | `PWMA`/`AIN1`/`AIN2`, encoder on `D2`/`D6` | `-Y` side (starboard) |
| motor 2 | B | `PWMB`/`BIN1`/`BIN2`, encoder on `D3`/`D11` | `+Y` side (port) |

The side column is **measured, not chosen** — see
[Wheel direction](#wheel-direction) for how, and for why the obvious way of
measuring it is the wrong one.

No motor is called "left" or "right" anywhere in the firmware, the bridge, the
calibration sketches or the validation scripts. This is not cosmetic. The
robot's front has already been redefined once; the wiring did not change with
it, so any channel named after a side was going to end up lying. Faults are
reported by motor number (`m1`/`m2`), which is what a wiring diagram and a
`PIDM` command both use.

Two lines map a number to a side, and they must agree with each other:

| where | what it says |
| --- | --- |
| `MiniRobotGeometry.h` → `WHEEL_Y_SIGN` | motor 1 is at `-Y`, motor 2 at `+Y` — used by the drive kinematics and the encoder heading |
| `config/base.yaml` → `wheel_joint_names` | motor 1 drives `right_wheel_joint`, motor 2 `left_wheel_joint` — used by `/joint_states` only |

Send `CFG` to the board and it prints its own copy as `ysign=-1/1`;
`scripts/validate_motor_control.py` compares that against the source and stops
if the board is running something else.

Turning is still described as left and right — `LEFT`/`RIGHT` serial commands,
teleop keys, "turn left" — because that is a direction of motion and it means
the same thing whichever wheel does it.

### Which files describe the body

Change these together or the map and the robot will disagree:

| where | what it fixes |
| --- | --- |
| `MiniRobotGeometry.h` | `WHEEL_Y_SIGN`, plus `ENCODER_SIGN` / `MOTOR_DIRECTION_SIGN`, which define what "forward" means to each wheel |
| `config/base.yaml` | `wheel_joint_names`, the motor-number-to-joint mapping |
| `urdf/mini_robot.urdf.xacro` | `caster_x`/`body_x` positive so the caster and laser sit at `+X`; `imu_joint` yawed by `pi` because the chip physically faces the axle end |
| `config/nav2_params.yaml` | costmap footprint reaches further ahead of the axle than behind it |

Yaw needs no rotation anywhere. Rotation about `Z` is unchanged by a 180-degree
yaw rotation, so a left turn is a left turn whichever end leads — which is why
the IMU can stay bolted down facing the rear and still supply the heading
odometry integrates along.

## 3. Folder structure

```text
arduino_mini_base_controller/          the firmware to upload to the Arduino Uno
  arduino_mini_base_controller.ino     setup()/loop() and the protocol reference
  MiniRobotPins.h                      pin map and motor indexing        \
  MiniRobotGeometry.h                  wheel size, counts/rev, body       | shared
                                         layout, commissioning signs      | layer
  MiniRobotMotors.h/.cpp               TB6612FNG channels, 31 kHz PWM     |
  MiniRobotEncoders.h/.cpp             quadrature ISRs, atomic reads     /
  Config.h                             timing, drive limits, feed-forward fit,
                                         low-speed torque constants
  Types.h                              WheelPID (fractional-order PID)
  Imu.h/.cpp                           MPU/AK8963 driver + Mahony fusion
  MotorControl.h/.cpp                  kinematics, PID loop, odometry, watchdog
  Sweep.h/.cpp                         built-in SWEEP calibration command
  CommandParser.h/.cpp                 serial command set
  Telemetry.h/.cpp                     the 20 Hz 'T,...' line
arduino_mini_motor_calibration/        bench sketch: measures and fits the
  arduino_mini_motor_calibration.ino    feed-forward motor constants (section 7)
  MiniRobot*.h/.cpp                    byte-identical copies of the six
                                         shared-layer files above; keep them
                                         that way with
                                         scripts/sync_shared_sketch_files.sh
  Config.h                             baud, fit thresholds, CPR, DIR and
                                         sweep settings
  Counts.h/.cpp                        the session counts-per-rev, CPR, ROT
  Direction.h/.cpp                     the session signs and DIR
  Measure.h/.cpp                       abort handling, one speed measurement
  Fit.h/.cpp                           least squares, results table, paste block
  Sweep.h/.cpp                         the PWM sweep and CAL
  Commands.h/.cpp                      serial command set, CONFIG, HELP
  README.md                            full reference for that sketch
  cal_result.md                        analysed report of the CAL run the FF_*
                                         constants in Config.h came from
  cal_raw.log                          that run's raw serial log, kept so the
                                         fit can be re-derived or re-plotted
                                         without re-running the robot
arduino_mini_imu_calibration/          bench sketch: measures the IMU constants
  arduino_mini_imu_calibration.ino      the firmware cannot measure itself
                                         (section 6)
  MiniRobot*.h/.cpp                    the same six shared-layer files; the
                                         encoders are unused here but shipped
                                         anyway, because one copy set is easier
                                         to keep honest than a per-sketch subset
  Config.h                             sample counts, thresholds, limits
  Stats.h                              Welford streaming mean/variance
  ImuDriver.h/.cpp                     detection, register setup, scaled reads
  GyroCal.h/.cpp                       zero-rate offset, noise, drift
  AccelCal.h/.cpp                      six-face bias and scale
  MagCal.h/.cpp                        hard and soft iron
  MotorNoise.h/.cpp                    what the wheels do to the magnetometer
  CalUtil.h/.cpp                       abort handling, prompts, stillness
  Interactive.h/.cpp                   CHECK, YAWSIGN, LIVE, RAW, CONFIG, HELP
  Results.h/.cpp                       results table + paste-ready constants
  CommandParser.h/.cpp                 serial command set
  README.md                            full reference for that sketch
ros2_ws/src/mini_robot/
  mini_robot/serial_bridge.py         Arduino <-> ROS2
  mini_robot/coin_d6_lidar.py         COIN-D6 -> LaserScan
  mini_robot/cmd_mux.py               teleop/manual/Nav2 priority and safety gate
  mini_robot/teleop_keyboard.py       WASD keyboard teleop
  mini_robot/nav_monitor.py           live health monitor for the whole chain
  mini_robot/goto_cli.py              send one x_mm,y_mm,heading_deg target
  mini_robot/waypoint_cli.py          predefined Nav2 routes
  launch/robot.launch.py              Raspberry Pi launch
  launch/desktop_slam.launch.py       RViz + SLAM + Nav2
  launch/desktop_navigation.launch.py saved map + AMCL + Nav2
  config/base.yaml                    serial bridge and cmd_mux parameters
  config/lidar.yaml                   COIN-D6 driver parameters
  config/nav2_params.yaml             costmaps, A* and pure pursuit
  config/slam_toolbox.yaml
  config/waypoints.yaml
  config/mini_view.rviz               RViz layout including the custom panels
  urdf/mini_robot.urdf.xacro
ros2_ws/src/mini_rviz_plugins/        RViz Goto + Teleop panels (C++/Qt,
                                       desktop only)
scripts/
  bootstrap_robot_stack.sh            one-shot installer (desktop or pi mode)
  verify_robot_stack.sh               checks the stack is wired up correctly
  ros_network_env.sh                  source this to set ROS2 DDS network env vars
  detect_serial_ports.sh              works out which USB adapter is the Arduino
                                        and which is the LiDAR, then writes and
                                        installs the udev rules for both
  99-mini-robot.rules.example         udev rules template for stable serial ports
  robot_autostart.sh                  optional: turn boot-time autostart of
                                        robot.launch.py on or off (section 11)
  robot_bringup.sh                    what that systemd unit runs; sources ROS
                                        and waits for the serial ports
  build_arduino.sh                    compile a sketch for the Uno with the
                                        IDE's toolchain and report flash/RAM
  flash_arduino.py                    upload a .hex to the Uno over its
                                        bootloader, from the Pi, with only
                                        pyserial (section 8)
  sync_shared_sketch_files.sh         copy or --check the six MiniRobot* files
                                        across all three sketch folders
  run_motor_calibration.py            drive a full CAL run and log every
                                        sample to a file (section 7)
  analyse_motor_calibration.py        turn a raw CAL log into cal_result.md:
                                        every sample kept, refitted off-board,
                                        residuals, hysteresis and the
                                        breakaway floor (section 7)
  validate_motor_control.py           bench validation of the base controller
                                        over serial, no ROS needed (section 19)
  preflight_slam.py                   end-to-end check of the ROS chain before
                                        mapping (section 16)
```

## 4. Wiring

### TB6612FNG motor driver

| TB6612FNG | Arduino Uno | Notes |
|-----------|-------------|-------|
| `PWMA`    | `D10`       | motor 1 speed  |
| `AIN1`    | `D5`        | motor 1 direction |
| `AIN2`    | `D4`        | motor 1 direction |
| `PWMB`    | `D9`        | motor 2 speed |
| `BIN1`    | `D7`        | motor 2 direction |
| `BIN2`    | `D8`        | motor 2 direction |
| `STBY`    | `D12`       | driver enable, held HIGH |
| `VM`      | motor battery + | motor supply |
| `VCC`     | `5V`        | logic supply |
| `GND`     | `GND`       | must share ground with the battery |

`D9` and `D10` are both on Timer1, so the sketch reprograms that one timer to
run at ~31.4 kHz instead of the Arduino default 490 Hz. 490 Hz sits in the
middle of the audible band and makes the motors sing at low duty. Timer0 is
untouched, so `millis()`/`micros()`/`delay()` behave normally.

### Encoders

| Encoder | Channel A | Channel B |
|---------|-----------|-----------|
| Motor 1 | `D2` (INT0) | `D6`    |
| Motor 2 | `D3` (INT1) | `D11`   |

Only `D2`/`D3` are external-interrupt capable on the Uno, so channel A goes
there and channel B is read inside the ISR to recover the direction. That is
x2 decoding: two counts per encoder cycle.

### IMU

| MPU9250/9255/6500 | Arduino Uno |
|-------------------|-------------|
| `VCC`             | `3.3V` or `5V`, per your module |
| `GND`             | `GND` |
| `SDA`             | `A4`  |
| `SCL`             | `A5`  |

The sketch auto-detects the chip by `WHO_AM_I` at I2C address `0x68` or `0x69`:
`0x70` = MPU6500 (no magnetometer), `0x71` = MPU9250, `0x73` = MPU9255. It
probes at 400 kHz and, if nothing answers, again at 100 kHz — long jumper leads
degrade the edges enough that fast mode fails while standard mode works, and
falling back to encoder-only heading because of cable capacitance would be a
silly way to lose a gyro.

Send `IMU` over serial (or read `/diagnostics`) to see what it found. When it
finds nothing it also walks the whole bus and prints what it saw, because
"NONE" covers three different faults:

```text
INFO,IMU,NONE
INFO,I2C_SCAN,EMPTY_BUS
INFO,WHO_AM_I,0x68=0xFF,0x69=0xFF
```

- `EMPTY_BUS` — nothing acknowledges anywhere. The module has no power, `SDA`
  and `SCL` are swapped or not connected, or the module is dead. This is a
  wiring fault, not a software one.
- addresses listed but no MPU — something is on the bus that this driver does
  not know. The `WHO_AM_I` line says what the two candidate addresses returned;
  an MPU6050 answers `0x68` there and is *not* supported by this sketch.
- nothing at `0x68`/`0x69` but other addresses present — the module is strapped
  to an address nobody probes.

A missing IMU is not fatal: see [Running without an IMU](#running-without-an-imu).

## 5. Arduino serial protocol

The Pi normally sends:

```text
VEL,<linear_mm_s>,<angular_deg_s>
```

Examples:

```text
VEL,100,0
VEL,0,45
VEL,0,-45
VEL,-100,0
STOP
ESTOP
CLEAR_ESTOP
RESET_ODOM
ZERO_YAW
CAL_IMU
MAG,0
MAG,1
IMU
PING
```

Testing aliases are also supported. `FORWARD`/`REVERSE` drive towards and away
from the LiDAR end; `LEFT`/`RIGHT` rotate in place, counter-clockwise and
clockwise. Each answers `ACK,<name>` or `ERR,ESTOP_LATCHED`, so a command
swallowed by a latched e-stop does not look identical to one that worked:

```text
FORWARD,100
REVERSE,100
LEFT,45
RIGHT,45
```

Arduino publishes one telemetry line at 20 Hz, 23 comma-separated fields:

```text
T,time_ms,x_mm,y_mm,yaw_deg,vx_mm_s,wz_deg_s,
  tick_m1,tick_m2,
  speed_m1,speed_m2,
  pwm_m1,pwm_m2,
  roll_deg,pitch_deg,
  gx_deg_s,gy_deg_s,
  ax_g,ay_g,az_g,
  estop,watchdog,status
```

Per-wheel fields are in motor order: motor 1 (TB6612 channel A) then motor 2
(channel B). `yaw_deg`/`wz_deg_s` are the heading the pose was actually
integrated along, which is the IMU's when one is fitted and the encoders' when
one is not — so a host never has to guess which sensor a yaw came from.

`status` is a bitmask:

| bit | meaning |
| --- | --- |
| 0 | motor 1 is commanded to move and is not moving |
| 1 | motor 2 is commanded to move and is not moving |
| 2 | an IMU was detected (clear = heading comes from the encoders) |
| 3 | magnetometer fusion is active |

It was added after the original 22-field line. Both the ROS bridge and
`scripts/validate_motor_control.py` accept 22 or 23 fields, so a board that has
not been reflashed still works and simply reports no status.

`CFG` prints the constants the board was actually compiled with, which is the
only way to tell a fixed source tree from a fixed robot:

```text
INFO,CFG,counts=520.0/520.0,dia=47.00,track=200.0,maxlin=150.0,maxang=86.0,
         maxwheel=150.0,maxpwm=210,enc=-1/1,mot=-1/-1,ysign=-1/1
```

## 6. IMU behaviour and the magnetometer

The sketch runs a Mahony AHRS filter over the accelerometer and gyro, and
optionally the AK8963 magnetometer.

### Running without an IMU

A robot that refuses to move because a sensor is unplugged is a robot you
cannot debug. The firmware used to latch an emergency stop when no MPU
answered, on the reasoning that odometry integrated along a heading stuck at
zero would be wrong in a way that is hard to see. The wrongness was real; the
cure was worse.

Now the wheels supply the heading instead. The two wheel distances differ by
exactly the arc the body swept:

```text
d(yaw) = (distance at -Y - distance at +Y) / TRACK_WIDTH_MM
```

So with the IMU missing the robot still drives, still maps and still navigates.
The heading drifts faster — every wheel slip is a heading error it cannot see,
and it cannot notice the robot being shoved sideways — but SLAM's scan matcher
corrects exactly that kind of drift, and it is told to expect more of it: the
bridge publishes a yaw covariance of 0.15 instead of 0.03 when the heading came
from the wheels, so slam_toolbox weights the scans over the odometry rather
than fighting an over-confident prior.

Nothing about the fallback is silent:

| where | what it says |
| --- | --- |
| boot banner | `ERR,NO_IMU_DETECTED` then `INFO,HEADING_SOURCE,ENCODERS`, plus an I2C scan |
| every telemetry line | `status` bit 2 clear |
| `/diagnostics` | `heading_source: encoders` |
| bridge log | one warning on the transition |
| `/imu/data` | still published, but with `-1` as the leading covariance — the `sensor_msgs/Imu` way of saying "no estimate", so nothing downstream fuses the zeros as though the robot were level and still |

To make a missing IMU fatal again, set `REQUIRE_IMU = true` in `Config.h`.

**Magnetometer fusion is off by default.** On a robot this size the AK8963 sits
centimetres from two brushed motors and a battery, so the field it measures
swings with motor current and yaw becomes a function of throttle. Six-axis
fusion has no absolute heading reference but is smooth and locally accurate,
which is what odometry actually needs — SLAM supplies the absolute correction.

To try nine-axis fusion:

```bash
ros2 service call /base/enable_magnetometer std_srvs/srv/Trigger {}
```

Then park the robot, watch `/imu/data`, and spin the wheels in place. If yaw
moves while the robot does not, turn it back off:

```bash
ros2 service call /base/disable_magnetometer std_srvs/srv/Trigger {}
```

The `magOffset*` / `magScale*` constants in the sketch are hard/soft iron
corrections, and ship at 0/1 -- i.e. uncalibrated.

### Measuring the IMU constants

`arduino_mini_imu_calibration/` is a bench sketch that measures the four things
the firmware cannot measure about itself: the gyro zero-rate offset and drift,
the accelerometer bias and scale (which the firmware applies none of, even
though the accelerometer is the Mahony filter's only gravity reference), the
hard/soft iron correction above, and `IMU_YAW_SIGN`. Upload it, open a serial
terminal at **115200 baud**, and send `CAL`:

```text
CAL
```

It walks through a health check, the gyro, six accelerometer faces, the yaw
sign and a figure-of-eight magnetometer sweep, then prints every constant ready
to paste. About ten minutes, most of it spent moving the robot around.

Send `MOTORS` (robot on blocks) before ever enabling nine-axis fusion: it spins
the wheels and reports how far the measured field moves with motor current. If
that is more than a few degrees, the magnetometer calibration is correct and
still unusable -- heading would become a function of throttle. Leave `MAG`
fusion off.

Full reference:
[`arduino_mini_imu_calibration/README.md`](arduino_mini_imu_calibration/README.md).

## 7. Motor calibration (do this before driving on the floor)

Each wheel's PWM-to-speed relationship is different, and different again in
reverse. The sketch models it as a per-wheel, per-direction straight line
(`FF_SLOPE_*` / `FF_INTERCEPT_*`) and uses it as open-loop feed-forward so PID
only corrects the residual. **The values shipped in the sketch are a generic
guess, not a calibration.**

### The easy way: the calibration sketch

Upload `arduino_mini_motor_calibration/arduino_mini_motor_calibration.ino`,
put the robot on blocks so both wheels spin free, open a serial terminal at
500000 baud and send:

```text
CPR
```

`CPR` measures encoder counts per wheel revolution, by hand, with the motors
off: mark the tyre, turn each wheel the number of turns it asks for, press
Enter. This goes first because it is the scale factor under every other number
in the sketch and in the firmware — get it wrong and every speed is wrong by
exactly that factor, while telemetry, `/odom` and the PID all agree the robot
is on target. Nothing downstream can detect it.

Check it with `ROT`, which does the same sum backwards: it drives what the
sketch now believes is exactly one turn and brakes. Line a mark on the tyre up
with a mark on the chassis, send `ROT`, and the mark should come back round.
Short means the count is too low, past means too high; `ROT,5` magnifies a
small error into an obvious one.

Then send:

```text
DIR
```

`DIR` commissions the two direction signs, and it is not optional. It
spins each wheel at a fixed driver polarity with no signs applied, asks which
way the robot tried to roll, and derives both `MOTOR_DIRECTION_SIGN` and
`ENCODER_SIGN` from your answer. Answer by watching the wheel, not the numbers:
a wheel with **both** signs wrong reports a perfectly healthy positive speed
while turning backwards, and no measurement the robot can make will catch it.
Paste the `SIGN` block it prints into `MiniRobotGeometry.h`, then run
`bash scripts/sync_shared_sketch_files.sh` to push it to all three sketch
folders.

Then send:

```text
CAL
```

It sweeps both wheels in both directions, two passes each, fits the line
on-board, and prints the four constant lines ready to paste over the
`FF_SLOPE_*` / `FF_INTERCEPT_*` definitions in
`arduino_mini_base_controller/Config.h`:

```text
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {6.4639f, 6.2343f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-44.31f, -44.78f};
...
```

Widen and refine the sweep before running it, so the dead zone is captured and
the operating band is densely sampled rather than skipped over:

```text
RANGE,4,72
STEP,2
DWELL,400,600
PASSES,2
CAL
```

That is 280 samples and about five minutes. Send `HELP` for the rest of the
commands (`RESULT` reprints the paste block). **Any character aborts a running
sweep**, so do not type while it works.

#### Keep the samples

The sketch cannot: 2 KB of SRAM does not hold 280 points, so everything except
the fitted answer is gone as soon as the terminal scrolls. And a five-minute
sweep that must not be typed into is an awkward thing to babysit by hand, so
both halves are scripted:

```bash
python3 scripts/run_motor_calibration.py -o cal_raw.log
python3 scripts/analyse_motor_calibration.py cal_raw.log \
    -o arduino_mini_motor_calibration/cal_result.md
```

The first sends the settings above, runs `CAL`, and writes every line to the
log. The second turns it into the report.

That refits the points off-board as a cross-check on the on-board arithmetic,
and adds what the sketch has no room to compute: residuals (is the line
actually straight, or does r² just look good?), the gap between the ascending
and descending passes (stiction and backlash), the measured breakaway per wheel
with the `FF_BREAKAWAY_PWM` line ready to paste, and every sample as CSV for
plotting. Both files are kept in the repo, so the fit can be re-derived without
re-running the robot.

The sketch stops there on purpose: it drives the motor channels, counts the
encoder ticks per wheel turn, commissions the directions, and prints the curve. Checking the fit
against a control law is the firmware's job, not the bench tool's.

Re-flash the main firmware when you are done.

Keep the battery charged for the whole run: the fit is only valid at the bus
voltage it was measured at.

Full reference, including how to read the fit and what to do when it complains:
[`arduino_mini_motor_calibration/README.md`](arduino_mini_motor_calibration/README.md).

### The manual way: SWEEP in the main firmware

The main sketch also has a built-in sweep, if you would rather fit the points
yourself. With the wheels off the floor, connect a plain serial terminal at
500000 baud (not the ROS stack) and run:

```text
SWEEP,1,10,1000
SWEEP,1,10,1000,R
SWEEP,2,10,1000
SWEEP,2,10,1000,R
```

Each step prints `C,<motor>,<F|R>,<pwm>,<measured_mm_s>`. Drop the stalled
points at the bottom of each sweep, least-squares fit the rest, and paste the
slope/intercept into the sketch. A sweep blocks the controller for its whole
duration (no telemetry, no commands), which is why it is a bench operation.

### Also measure, in the same session

- **`MEASURED_COUNTS_PER_REV`** — send `RESET_ODOM`, rotate one wheel by hand
  exactly one full turn, read that wheel's tick field from telemetry. Do it
  per wheel. This robot ships `520`: a 13 PPR encoder on the motor shaft,
  x2 decoding (CHANGE on channel A only), behind a 1:20 gearbox
  (13 x 2 x 20 = 520). Recompute it for your own encoder and gear ratio.
- **`TRACK_WIDTH_MM`** — the distance between the two wheel contact patches.

## 8. Install the Arduino software

The firmware talks to the TB6612FNG through plain
`digitalWrite`/`analogWrite` and to the IMU through raw I2C registers, so **no
libraries are needed at all** -- not third-party ones, and no library from this
repo either. Each sketch folder is self-contained:

1. Open `arduino_mini_base_controller/arduino_mini_base_controller.ino`.
2. Select Arduino Uno and upload.
3. Keep the robot still during startup IMU calibration.

No sketchbook setting, no library install, no symlink.

The price of that is one shared hardware layer kept in three places:

| File | base_controller | motor_calibration | imu_calibration |
| --- | :---: | :---: | :---: |
| `MiniRobotPins.h` | yes | yes | yes |
| `MiniRobotGeometry.h` | yes | yes | yes |
| `MiniRobotMotors.h/.cpp` | yes | yes | yes |
| `MiniRobotEncoders.h/.cpp` | yes | yes | yes (unused there) |

Every copy that exists **must stay byte-identical**. The motor calibration
sketch reports mm/s scaled by `WHEEL_DIAMETER_MM` and `MEASURED_COUNTS_PER_REV`
and the firmware consumes the fit scaled by the same constants, so a mismatch
silently bakes a constant error into every speed the robot drives. After
editing any copy, push it out:

```bash
bash scripts/sync_shared_sketch_files.sh            # copy them out
bash scripts/sync_shared_sketch_files.sh --check    # report drift, change nothing
```

`--check` exits non-zero and prints a diff when anything has drifted, so it
works as a pre-flight or a pre-commit hook. The base controller is the source of
truth — it is the one that flies.

### Building and flashing without the IDE

The Arduino is wired to the Raspberry Pi, so reflashing it through the IDE means
unplugging the robot and carrying it to another machine. Two scripts avoid that.

On any machine that has ever run the Arduino IDE (it uses the toolchain already
installed under `~/.arduino15`):

```bash
bash scripts/build_arduino.sh arduino_mini_base_controller
bash scripts/build_arduino.sh --all
```

```text
arduino_mini_base_controller           flash  30080 / 32256 (93%)   static RAM  1170 / 2048 (57%)
  -> build/arduino_mini_base_controller/arduino_mini_base_controller.hex
```

The size report is the point of running it: the base controller is past 90% of
the Uno's flash, so a change that looks small can be the one that does not fit,
and the IDE only tells you after you have picked a board and a port.

Then, on the Pi, with nothing installed beyond pyserial:

```bash
scp build/arduino_mini_base_controller/*.hex minirobot@<pi>:/tmp/base.hex
ssh minirobot@<pi> 'python3 ~/MiniRobot/scripts/flash_arduino.py /tmp/base.hex'
```

```text
base.hex: 30208 bytes (93% of the application section)
bootloader on /dev/arduino_mini, signature 0x1E 0x95 0x0F (ATmega328P)
wrote 234 pages
verified
done; the board is running the new sketch
```

It speaks STK500v1, which is what the Uno's Optiboot bootloader speaks, reads
back every page it writes, and refuses any image that would reach into the
bootloader — so a failed upload leaves a board that still takes uploads. Stop
the ROS stack first; two processes cannot share the port.

Verified build size on `arduino:avr:uno` (AVR core 1.8.8, avr-gcc 7.3.0, LTO on
as the IDE defaults to):

| Sketch | Flash (of 32256) | SRAM (of 2048) |
| --- | --- | --- |
| `arduino_mini_base_controller` | 30080 B (93%) | 1170 B (57%) |
| `arduino_mini_motor_calibration` | 20754 B (64%) | 441 B (21%) |
| `arduino_mini_imu_calibration` | 30488 B (94%) | 803 B (39%) |

Splitting the firmware into modules cost nothing in SRAM. Duplicating the shared
layer per sketch costs nothing either: `const` file-scope data has internal
linkage, so the compiler folds the scalars and drops whatever a sketch does not
use.

Flash is the tight resource. The base controller has about 2.2 KB spare -- room
for small additions but not for another large subsystem; the cheapest savings
are dropping the `SWEEP` command once the feed-forward fit is measured, and the
nine-axis Mahony path if you never enable the magnetometer. The IMU calibration
sketch has under 2 KB spare and is essentially at its final size: if you need to
extend it, `RAW` and `MOTORS` are the two commands nothing else depends on.

Only one sketch is on the board at a time, so these do not compete -- but each
must fit on its own.

If no IMU is detected at boot the sketch reports `ERR,NO_IMU_DETECTED`, prints
an I2C scan, and carries on with the heading integrated from the wheels — see
[Running without an IMU](#running-without-an-imu). Set `REQUIRE_IMU = true` in
`Config.h` to make it latch the emergency stop instead.

## 9. Install ROS2 on the Raspberry Pi

Use Ubuntu Server 22.04 with ROS2 Humble. From the repository root:

```bash
bash scripts/bootstrap_robot_stack.sh pi
```

This single script (safe to re-run) will:

- add the ROS2 apt repository and key if they are not already configured
- install `ros-humble-ros-base`, `robot_state_publisher`, `xacro` and the
  Python serial/YAML/colcon tooling
- add your user to the `dialout` group (needed for serial port access)
- symlink `ros2_ws/src/mini_robot` from this repo into `~/ros2_ws/src/`, so
  future `git pull`s are picked up without re-copying anything
- run `rosdep install` and `colcon build --symlink-install`
- install the udev rules from section 10 automatically, once you have replaced
  the placeholders in `scripts/99-mini-robot.rules.example`
- append the ROS2 environment (domain ID, `ROS_LOCALHOST_ONLY=0`) to `~/.bashrc`

Log out and back in afterwards so the `dialout` group change takes effect.

`mini_rviz_plugins` is deliberately **not** built on the Pi: it needs
`rviz_common` and Qt5, which `ros-base` does not provide, and adding it would
fail the whole `colcon build`.

## 10. Stable Arduino and LiDAR port names

### The easy way: let the script work it out

With both devices plugged in, and `robot.launch.py` not running:

```bash
bash scripts/detect_serial_ports.sh --install
```

It opens each `/dev/ttyUSB*` in turn and classifies it by what it transmits,
because that is the only thing that separates two CH340s. The Arduino announces
itself in ASCII (`READY,`, `INFO,`, `T,`) at 500000 baud, or 115200 if one of
the calibration sketches is loaded; the COIN-D6 emits `0xAA 0x55` framed binary
at 230400. It then writes `scripts/99-mini-robot.rules` matching each device on
its `ID_PATH` — or on its USB serial number, if the adapter actually has one —
installs it, reloads udev, and confirms both symlinks appeared.

Nothing is written to either port unless you pass `--wake-lidar`, so the LiDAR
motor is never commanded to spin. Opening a port does toggle DTR and reset the
Arduino, which is harmless on a bench but means you should not run this while
the robot is driving. `--dry-run` reports what it found and writes nothing.

`bootstrap_robot_stack.sh pi` runs it automatically when both devices are
present and no rules file exists yet, so on a fresh Pi there is usually nothing
to do by hand.

The generated file encodes which physical USB socket each adapter sits in, so
it is machine-specific and gitignored rather than shared between robots.

### The manual way

Inspect serial device identities (which port is Arduino vs. LiDAR depends on
the USB-serial chip on your Arduino clone — boards with a CH340 adapter
enumerate as `ttyUSB*` rather than `ttyACM*`, so confirm with
`ls -l /dev/serial/by-id/` before assuming a specific node):

```bash
ls -l /dev/serial/by-id/
udevadm info -q property -n /dev/ttyUSB0 | grep ID_PATH=
udevadm info -q property -n /dev/ttyUSB1 | grep ID_PATH=
```

CH340 adapters share one vendor/product ID and have no serial EEPROM, so
`ATTRS{serial}` comes back identical for both and cannot distinguish them.
`scripts/99-mini-robot.rules.example` matches on the physical USB port path
(`ENV{ID_PATH}`) instead, which is stable as long as each adapter stays in the
same USB port. Fill in the two placeholders, then either re-run
`bash scripts/bootstrap_robot_stack.sh pi` or install it directly:

```bash
sudo cp scripts/99-mini-robot.rules.example /etc/udev/rules.d/99-mini-robot.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Expected device links:

```text
/dev/arduino_mini
/dev/coin_d6
```

The launch arguments can be used instead if the links are not configured:

```bash
source ~/.bashrc
ros2 launch mini_robot robot.launch.py \
  arduino_port:=/dev/ttyUSB1 \
  lidar_port:=/dev/ttyUSB0
```

## 11. Start the robot-side ROS2 backbone

`bootstrap_robot_stack.sh` already added ROS2 sourcing and the network env vars
to `~/.bashrc`, so a fresh login shell has everything needed. In that shell, on
the Raspberry Pi:

```bash
ros2 launch mini_robot robot.launch.py
```

In a second shell, verify the stack came up correctly:

```bash
bash scripts/verify_robot_stack.sh
```

This checks that `mini_serial_bridge`, `coin_d6_lidar` and `mini_cmd_mux` are
running, and that `/odom`, `/imu/data`, `/scan`, `/joint_states`,
`/cmd_vel_out` and the `/base/*` services are all present. You can also inspect
things manually:

```bash
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /imu/data
ros2 topic echo /scan
ros2 run tf2_tools view_frames
```

### Running with parts switched off or missing

Every component can be left out, and the rest keeps working. A node that is off
is simply never started — there is no stub, no zero-publisher and nothing
pretending the hardware is there.

```bash
ros2 launch mini_robot robot.launch.py lidar:=false
ros2 launch mini_robot robot.launch.py mux:=false
ros2 launch mini_robot robot.launch.py base:=false
ros2 launch mini_robot robot.launch.py description:=false
ros2 launch mini_robot robot.launch.py lidar:=false mux:=false   # they compose
```

| switched off | what still works | what stops | how you find out |
| --- | --- | --- | --- |
| **LiDAR** (`lidar:=false`, or unplugged) | driving, teleop, odometry, TF | mapping, Nav2, the front stop gate | mux logs `no usable /scan — front stop gate is OFF and forward motion is UNPROTECTED` |
| **mux** (`mux:=false`) | driving — the bridge subscribes to `/cmd_vel` directly | source arbitration and the front stop gate; whatever publishes `/cmd_vel` drives the base unmediated | the bridge logs the topic it is on at startup |
| **base** (`base:=false`, or the Arduino unplugged) | LiDAR, TF, everything perception-side | all motion, odometry, IMU | bridge absent, or `Serial port disconnected` in `/diagnostics` |
| **IMU** (unplugged or faulty) | everything — heading comes from the wheels | absolute attitude; heading drifts faster | `heading_source: encoders`, see [Running without an IMU](#running-without-an-imu) |
| **magnetometer** | everything — six-axis fusion is the default anyway | nothing | `MAG_ABSENT` in `INFO,IMU` |
| **`robot_state_publisher`** (`description:=false`) | driving, LiDAR, odometry | TF, so RViz, SLAM and Nav2 | no `/tf_static`, no robot in RViz |
| **an encoder** | the other wheel | closed-loop control of that wheel: it stalls, boosts, and cools down in a loop | `ERR,STALL,MOTOR_<n>`, `status` bit set, `stalled_motors` in `/diagnostics` |
| **desktop (RViz/SLAM/Nav2)** | the whole robot side, teleop included | mapping and autonomous navigation | nothing publishes `/cmd_vel` |

Two switches are worth knowing about:

- **`require_scan`** in `config/base.yaml` decides what the mux does with no
  usable `/scan`. `false` (the default) passes forward commands through
  unprotected; `true` blocks them until scans return. `false` is the default
  because "the robot does not move and nothing says why" is the worse failure
  on a bench.
- **`REQUIRE_IMU`** in `Config.h` decides whether a missing IMU is fatal.
  `false` (the default) falls back to encoder heading; `true` latches an
  emergency stop.

`bash scripts/verify_robot_stack.sh` reports a missing node as
`not present (fine if that part of the stack isn't launched right now)` rather
than as a failure, so it is still useful with parts switched off.

### Optional: start it automatically at boot

The stack can also come up on its own when the Pi powers on, as a systemd
unit. It is entirely opt-in: nothing is installed until you ask for it, and
while it is off everything above behaves exactly as it did before.

On the Pi:

```bash
bash scripts/robot_autostart.sh on
```

That writes `/etc/systemd/system/mini-robot.service` (generated per machine, so
it is not tracked in the repo), starts it now, and enables it for every later
boot. Turn it off again with:

```bash
bash scripts/robot_autostart.sh off
```

| command | effect |
| --- | --- |
| `on` | run now **and** at every boot |
| `off` | stop now and do not start at boot |
| `start` / `stop` / `restart` | this boot only; the boot setting is unchanged |
| `status` | what is installed, enabled and running |
| `logs [-f]` | the unit's journal, i.e. the launch output |
| `preview` | print the unit file without installing anything |
| `install` | write the unit but leave it off |
| `uninstall` | remove the unit; nothing else is touched |

The unit runs `scripts/robot_bringup.sh`, which exists because systemd gives
its services a non-interactive shell that never reads `~/.bashrc` (section 21).
The wrapper sources `/opt/ros/humble/setup.bash` and
`~/ros2_ws/install/setup.bash` explicitly, sets `ROS_DOMAIN_ID`, waits up to
30 s for `/dev/arduino_mini` and `/dev/coin_d6` to appear (udev creates those
symlinks a moment after the adapters enumerate), and then execs the same
`ros2 launch mini_robot robot.launch.py` you would run by hand.

Two behaviours are worth knowing:

- **It stands aside for a hand-started stack.** If `serial_bridge` or
  `coin_d6_lidar` is already running when the unit starts, it says so in the
  journal and exits cleanly instead of fighting for the serial ports -- two
  processes cannot share them. So with autostart on, run `off` (or `stop`)
  before launching by hand.
- **It brings the nodes up, it does not drive.** Nothing publishes a velocity
  command at boot, so the wheels stay still; the watchdog and the latched
  e-stop of section 20 are unchanged. The LiDAR motor does start spinning.

Everything in the unit is per-machine (user, repo path, workspace,
`ROS_DOMAIN_ID`), so after moving the repo re-run `robot_autostart.sh on`
rather than editing `/etc/systemd/system/mini-robot.service` by hand.

## 12. Configure ROS2 networking

Both the Pi and workstation must:

- be on the same local network
- use the same `ROS_DOMAIN_ID` (this project defaults to `31`)
- use `ROS_LOCALHOST_ONLY=0`
- allow ROS2/DDS UDP traffic through the firewall

On both machines:

```bash
export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=0
```

Test from the workstation:

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom
```

When Wi-Fi blocks multicast, run a Fast DDS discovery server on the Pi:

```bash
fastdds discovery --server-id 0 -l 192.168.1.50 -p 11811
```

Then on both machines:

```bash
export ROS_DISCOVERY_SERVER=192.168.1.50:11811
```

## 13. Install and build on the Ubuntu workstation

From the repository root on the workstation:

```bash
bash scripts/bootstrap_robot_stack.sh desktop
```

This installs `ros-humble-desktop`, Nav2, SLAM Toolbox, `teleop_twist_keyboard`
and the Qt5 development files, symlinks both `mini_robot` and
`mini_rviz_plugins` into `~/ros2_ws/src/`, builds the workspace, and appends the
ROS2 environment to `~/.bashrc`. Open a new shell afterwards, or source the two
`setup.bash` files it printed at the end.

Run `bash scripts/verify_robot_stack.sh` here too to confirm both packages
built and (once `robot.launch.py` is running on the Pi) that the robot's topics
and services are visible over the network.

## 14. Manual movement tests

Raise the wheels off the floor for the first test.

Forward:

```bash
ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist \
"{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Rotate left:

```bash
ros2 topic pub -r 10 /cmd_vel_manual geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Stop the publisher with `Ctrl+C`. The source timeout and Arduino watchdog will
stop the robot.

Keyboard teleoperation (WASD):

```bash
ros2 run mini_robot teleop_keyboard \
  --ros-args --remap cmd_vel:=/cmd_vel_teleop
```

Keys: `w`/`s` forward/backward, `a`/`d` rotate left/right — hold a key to move,
release to stop. Any other key stops the robot immediately. While a key is held
the node streams the command at ~20 Hz so the mux never falls back to a
lower-priority source (e.g. Nav2) mid-drive; after ~0.75 s without a keypress
it publishes a final stop and goes silent. Speeds are set with the `speed`
(m/s) and `turn` (rad/s) parameters.

Live health monitor for the whole Nav2 → mux → Arduino chain:

```bash
ros2 run mini_robot nav_monitor
```

Emergency stop:

```bash
ros2 service call /base/emergency_stop std_srvs/srv/Trigger {}
```

Clear the latched stop:

```bash
ros2 service call /base/clear_emergency_stop std_srvs/srv/Trigger {}
```

Reset odometry:

```bash
ros2 service call /base/reset_odometry std_srvs/srv/Trigger {}
```

Zero the yaw reference and recalibrate the IMU (keep the robot still for the
latter):

```bash
ros2 service call /base/zero_yaw std_srvs/srv/Trigger {}
ros2 service call /base/calibrate_imu std_srvs/srv/Trigger {}
```

## 15. Motor PID behaviour and low-speed torque

For straight motion, both wheels receive the same speed setpoint. Each wheel
has its own PID state and PWM output. This is intentional: equal speed does not
usually require equal PWM, because motor torque, gearbox friction and wheel
loading differ between the two sides.

Starting gains are:

```text
Kp = 0.35
Ki = 0.60
Kd = 0.0015
```

Change both wheels at runtime:

```text
PID,0.35,0.60,0.0015
```

Change one wheel, by motor number:

```text
PIDM,2,0.33,0.60,0.0015
```

The loops are fractional-order PID (PI^lambda D^mu). `lambda`/`mu` default to
`1.0`, which is exactly classic PID and takes a fast path that skips the
Grünwald–Letnikov shaping entirely. Both can be passed as optional 4th and 5th
arguments to `PID`/`PIDM` if you want to experiment.

### Torque at low speed

Slow is the hard case for a brushed gearmotor on a PWM bridge with no current
sensing. Drive it at a low duty and it turns slowly *and* weakly, so the first
sticky patch, rug edge or cable stops it — which is no use for a robot that has
to approach a goal at 50 mm/s.

Four things address it, and they are separate because they fix separate
problems. All four live in the "Low speed" block of
`arduino_mini_base_controller/Config.h`.

**1. The loop could not see the speed it was controlling.** One encoder count
is `pi * 47 / 520 = 0.284 mm`. At 30 mm/s that is 1.06 counts per 10 ms control
period, so a speed measured over one period alternates between 0 and 28.4 mm/s.
A PID cannot hold a speed it cannot see, and the zero samples look exactly like
a stall, so the integrator winds up and the wheel surges.

The measurement window now closes when a count actually arrives rather than on
a fixed schedule (`SPEED_WINDOW_MAX_MS`, capped at 120 ms so a genuinely
stopped wheel is still reported stopped promptly). At speed nothing changes —
counts arrive every tick — and at 20 mm/s the estimate becomes a measurement
instead of a coin flip.

**2. The integrator had no authority.** `Ki` was `0.020`. With a plant of
6.4 mm/s per PWM count that is an integral loop gain of 0.13/s: an eight-second
time constant, correcting a stalled wheel at roughly 0.4 PWM per second. The
loop was not weak, it was slow enough to look weak. `Ki = 0.60` gives a 0.26 s
time constant, and `MAX_INTEGRAL_PWM` rose from 80 to 120 so the correction has
somewhere to go — 120 PWM on top of a feed-forward of ~20 is most of the
bridge, which is the point. A robot that can only apply the torque its
calibration fit predicts cannot climb over a cable.

**3. The bottom of the PWM range is a dead zone the fit does not describe.**
Extrapolating the feed-forward line to zero speed says breakaway is at PWM
6.9–7.6, but the calibration sweep measured a hard stall right up to PWM 8 and
the wheels actually starting at PWM 10, 12 and 14 depending on wheel and
direction (`arduino_mini_motor_calibration/cal_result.md`). Anything commanded
into that gap energises the bridge and produces heat.

`FF_BREAKAWAY_PWM` is a floor on the output of a wheel that is **commanded and
not turning**. Deliberately not a floor on the feed-forward: a wheel already
rolling coasts through speeds far below its breakaway, and flooring it there
would turn every slow turn into a series of lurches.

Read it off the **ascending** leg of a sweep. Coming back down, a turning wheel
keeps turning at PWM values it could never have started from — motor 2 was
still running at PWM 10 on the way down and needed 14 to start on the way up —
and a wheel commanded from rest is always on the ascending side.
`scripts/analyse_motor_calibration.py` derives it and prints the line ready to
paste. On this robot the two wheels genuinely differ, `{10, 14}`, which is the
argument against one shared floor: at 14 the easier wheel lurches on every
start, at 10 the stiffer one sits dead until the stall boost rescues it.

**4. A wheel loaded to a standstill needs more than any fit predicts.** While a
wheel is commanded to move and is not moving, an extra PWM term ramps in on top
of feed-forward and PID at `STALL_BOOST_RATE_PWM_S` (400 PWM/s) up to
`STALL_BOOST_MAX_PWM`. It is deliberately fast — stiction breaks or it does not
— and it collapses the moment the wheel turns, so breaking free does not also
mean lurching.

That boost is budgeted, because a stalled brushed motor on a bridge with no
current limit is a heater. After `STALL_BOOST_MAX_MS` (1.5 s) of continuous
stall the wheel coasts for `STALL_COOLDOWN_MS` (2 s), the integrator is dumped,
and the board prints `ERR,STALL,MOTOR_<n>`. Then it tries again. A robot wedged
against a wall pulses instead of cooking its gearbox, and the difference
between "driving" and "trying to drive" is visible from ROS:

```text
[WARN] motor 2 STALLED: commanded to move and not moving. The base is
       boosting it in bursts; check for an obstruction, a jammed gearbox
       or a flat battery.
```

`/diagnostics` carries the same thing as `stalled_motors`, and telemetry
`status` bits 0 and 1 are the raw signal.

### What that is worth

`python3 scripts/validate_motor_control.py --selftest` runs the firmware's
control law against the plant measured in `cal_result.md`, with no hardware
involved. Adding a friction step partway through a run — the model of a rug
edge or a sticky patch, expressed as the extra PWM the wheel now needs — gives
the numbers the retune was for. True wheel speed, at 0.05 s, 0.3 s, 0.6 s, 1 s
and 1.9 s after the load appears:

| commanded | drag | gains | recovery |
| --- | --- | --- | --- |
| 50 mm/s | +10 PWM | `Ki` 0.020 | 26 → 28 → 29 → 29 → **30 mm/s** |
| 50 mm/s | +10 PWM | `Ki` 0.60 | 28 → 36 → 40 → 44 → **48 mm/s** |
| 50 mm/s | +20 PWM | `Ki` 0.020 | 26 → 1 → **0 → 3 → 2 mm/s** (stalled, stick-slip) |
| 50 mm/s | +20 PWM | `Ki` 0.60 | 26 → 22 → 30 → 37 → **46 mm/s** |
| 100 mm/s | +15 PWM | `Ki` 0.020 | 66 → 68 → 68 → 69 → **70 mm/s** |
| 100 mm/s | +15 PWM | `Ki` 0.60 | 68 → 79 → 86 → 91 → **97 mm/s** |

The middle pair is the reported symptom exactly: at the old gains a modest load
at a slow commanded speed does not slow the robot down, it leaves the wheel
juddering between stopped and barely moving, and the loop corrects at about
0.4 PWM per second, which never arrives. The real
robot recovers faster still, because the model does not include the stall
boost.

Nothing is free: the step response overshoots by 2.7% at 150 mm/s and 5.5% at
40 mm/s, and steady-state ripple is under 0.9 mm/s. If your robot is heavier or
its gearboxes stiffer and you see hunting, lower `Ki` first — `PID,0.35,0.30,0.0015`
at runtime, no reflash — and only then reach for `Kp`.

### What this does not do

There is no current sensing on this robot, so none of the above is real torque
control — it is speed control given enough headroom to demand torque, plus a
time budget so the demand cannot go on forever. If you want more:

- **Lower the PWM frequency.** 31.4 kHz is easy on the ears and gentle on the
  windings, but switching losses in the TB6612 rise with frequency, so a lower
  carrier puts slightly more of the battery into the motor. Timer1 in
  phase-correct mode with prescaler 1 gives 15.6 kHz. **This invalidates the
  feed-forward calibration** — PWM frequency changes the torque produced at a
  given duty — so re-run `CAL` afterwards.
- **Slow-decay (brake-mode) PWM** holds current up through the off phase and is
  the standard answer to weak low-speed torque. It needs the PWM on the
  direction pins rather than the `PWM` pin, and on this board three of the four
  direction pins (`D4`, `D7`, `D8`) have no timer output at all. It would need
  rewiring, not a firmware change.
- **A shunt and a current loop** is the real answer, and it is a hardware
  project.

## 16. Mapping with RViz, SLAM and Nav2

### Pre-flight: is the robot actually ready to map?

With everything connected and `robot.launch.py` running, one command answers
whether a mapping run is worth starting:

```bash
python3 scripts/preflight_slam.py
```

It runs four phases, ordered so a failure is easy to place:

| phase | what it proves |
| --- | --- |
| `graph` | the four nodes are up, every topic is publishing at rate, the base reports healthy, no latched e-stop |
| `sensors` | the IMU reads 1 g at rest with little gyro drift, the LiDAR returns enough valid beams, frames are named right |
| `model` | the URDF parses, has every expected link and joint, and the TF tree reaches `odom → base_footprint → base_link → laser` |
| `motion` | **on blocks**: a `/cmd_vel` command reaches the wheels, both turn the right way, they match each other, and `/odom` agrees |

Everything before `motion` is read-only and safe on the floor; `motion` spins
the wheels and asks first. `--no-motion` stops after `model`, `--yes` skips the
prompt. A wheel that runs away aborts the run and stops the motors.

`motion` reports each wheel twice: `wheels matched` compares the two wheels
against *each other* on a forward command, and `wheel tracking` compares each
wheel against *its own target* in both directions. The second one is the
diagnostic. A wrong `MEASURED_COUNTS_PER_REV` is a pure scale error — the loop
closes on a mis-scaled encoder, so the wheel lands on the same fraction of
target whichever way it turns. A bad feed-forward fit or mechanical drag is
direction-dependent, because breakaway PWM differs per direction. Comparing the
two wheels to each other cannot tell those apart; comparing each wheel to its
own target in both directions can, and `wheel tracking` names which one it is.

Wheels are reported by motor number (`m1`, `m2`) because that is what the
firmware arrays, the telemetry line and a `PIDM` command all use. The report
also queries the bridge's `wheel_joint_names` and prints the wheel each number
drives, so a fault found in the numbers can be traced to a wheel you can look
at. Re-running `CAL` fits both motors in both directions anyway, so the
numbering matters for reading the report, not for the fix.

Two more results need reading in context:

- **The front stop gate blocks the forward checks.** `cmd_mux` zeroes any
  command with forward velocity while something sits inside
  `front_stop_distance`, so on a blocked run the forward command never reaches
  the motors and healthy wheels read as dead. The script detects this, offers
  to re-check once the space is clear, and marks the forward checks `skip`
  rather than failing them. Reverse and rotation still run — the gate only
  stops forward motion, which is why "reverse works, forward does nothing" is
  the signature of a blocked gate rather than a broken motor.
- **Yaw sign cannot be verified on blocks.** The wheels spin in the air, the
  chassis does not turn, and the gyro reads its own bias. The script reports
  `rotate: IMU agrees` as `skip` when the measured rate is inside the noise
  floor, and only fails it when there is real rotation with the wrong sign.
  Check the yaw sign on the floor instead, by hand — see *IMU direction* in
  section 19.

The `sensors` phase also prints a `LiDAR bearings` line: the range at 0, +90,
180 and -90 degrees. Nothing on the robot can tell which way the LiDAR is
bolted down, and a scan mounted 180 degrees out passes every other check in the
script. Compare those four numbers against the actual room — bearing 0 must be
the caster/LiDAR end, the direction a positive `linear.x` drives (section 2).

It ends with `READY FOR SLAM` or a list of what to fix and how. Mapping on top
of a broken TF tree or a mismatched wheel pair produces a map that cannot be
salvaged, so it is worth the thirty seconds.

For faults below the ROS layer — wheel signs, feed-forward, encoder scale —
use `scripts/validate_motor_control.py` on the Pi, which bypasses ROS and talks
to the Arduino directly (section 19).

### Starting SLAM

Keep `robot.launch.py` running on the Pi. On the workstation:

```bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=31
ros2 launch mini_robot desktop_slam.launch.py
```

The workstation runs:

- RViz2, with the **Goto** and **Teleop** panels from `mini_rviz_plugins`
- SLAM Toolbox
- Nav2 local and global costmaps
- NavFn A* global path planning
- Rotation Shim + Regulated Pure Pursuit path following
- Nav2 waypoint follower

Use RViz **2D Goal Pose** to test navigation while mapping.

Save the map:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/mini
```

## 17. Navigation using a saved map

```bash
ros2 launch mini_robot desktop_navigation.launch.py \
  map:=$HOME/maps/mini.yaml
```

Set the initial pose in RViz with **2D Pose Estimate**, then send a goal with
**2D Goal Pose**, or type a target into the Goto panel.

Single target from the command line (units match the Arduino: mm and degrees):

```bash
ros2 run mini_robot goto_cli 1000 1000 90
```

## 18. Predefined waypoints from the workstation

Edit:

```text
ros2_ws/src/mini_robot/config/waypoints.yaml
```

Then rebuild:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Start the interactive route selector:

```bash
ros2 run mini_robot waypoint_cli
```

The client sends the selected route to Nav2's `/follow_waypoints` action. Nav2
calculates the path and sends velocity commands through the Pi to Arduino.
Arduino does not store or solve map waypoints.

## 19. Required commissioning checks

Most of what follows is automated by `scripts/validate_motor_control.py`, which
talks the Arduino protocol directly -- not through `/cmd_vel` -- so a fault in
the bridge or the mux cannot be mistaken for a fault in the firmware. Run the
bench tests first, with the robot on blocks:

```bash
python3 scripts/validate_motor_control.py --port /dev/arduino_mini
```

That covers the link, both commissioning signs, closed-loop tracking across the
configured speed range, the command watchdog and the e-stop, and it re-fits the
feed-forward line from the PWM the loop actually settled on so a stale `FF_*` in
`Config.h` shows up as a number instead of a hunch. Then the two that need floor
space and a tape measure:

```bash
python3 scripts/validate_motor_control.py --tests floor --distance-mm 1000
```

Every failure prints the constant to change and its corrected value.

One thing the bench tests will flag that predates them: the feed-forward fit in
`Config.h` came from a sweep over PWM 30..210, but with the speed limit at
0.15 m/s the robot now drives at roughly PWM 14..31. The whole band is below the
measured range, and the fit's claim that the wheels break away at PWM 5.4 is an
extrapolation nothing has ever checked. Re-calibrate over the band that is
actually used -- in `arduino_mini_motor_calibration/`, at 500000 baud:

```text
CPR
ROT
DIR
RANGE,8,60
STEP,2
CAL
```

Then paste the four `FF_*` lines it prints into `Config.h`. Until that is done
the PID integrator is covering for a feed-forward term that under-drives at
every speed the robot is asked for, which shows up as a sluggish start and
stick-slip at the bottom of the range.

`--selftest` runs the same control law offline against a model of the motor,
with no hardware attached, and prints what each failure mode does to the robot's
real speed. Worth reading once: it shows that a wrong
`MEASURED_COUNTS_PER_REV` makes the robot outrun its command while telemetry,
`/odom` and RViz all still read exactly the commanded speed. That error is
invisible from inside the robot, which is why the distance check below is not
optional.

### Wheel direction

`ENCODER_SIGN` and `MOTOR_DIRECTION_SIGN` are not two independent checks, and
fixing them one at a time is how a robot ends up driving backwards while every
number reads correctly. Both sit between the encoder count and the robot frame:

```text
reported_speed = raw_ticks(polarity chosen by MOTOR_DIRECTION_SIGN) * ENCODER_SIGN
```

Flip either one alone and the reported speed changes sign. Flip **both** on the
same wheel and it does not — but that wheel now physically turns the other way.
Telemetry, `/odom`, RViz and the PID all agree it is fine. Nothing inside the
robot can see it.

So do not derive them from telemetry. Flash
`arduino_mini_motor_calibration/` and send `DIR`: it spins each wheel at a
fixed driver polarity with no signs applied, asks which way the robot tried to
roll, and derives both signs from that one answer, then confirms them. Paste
the `SIGN` block it prints into `MiniRobotGeometry.h`, then run
`bash scripts/sync_shared_sketch_files.sh` — it is `const` and shared across all
three sketch folders, so the copies must match.

Telemetry is still the right check *afterwards*: command slow forward motion
and confirm both wheel speeds read positive **and** that the top of both wheels
moves **toward the LiDAR end**. The second half is not optional; it is the half
the robot cannot do for you.

There is a third sign, and it is the one that cannot be caught this way at all:
`WHEEL_Y_SIGN` says which side of the centre line each motor is on. Get it
backwards and both wheels still drive forward correctly, both encoders still
read positive, straight-line motion is perfect — and every turn goes the wrong
way. The check is a single rotation command with the wheels off the ground:

```text
LEFT,30
```

A counter-clockwise turn must drive **motor 1 forward and motor 2 backward**
(motor 1 is at `-Y`). `scripts/preflight_slam.py` checks exactly this as
`rotate: wheel pattern`, and `scripts/validate_motor_control.py` compares the
board's reported `ysign` against the source.

But do **not** commission `WHEEL_Y_SIGN` by watching a rotation and judging
which way the robot tried to spin. Clockwise and counter-clockwise are easy to
call backwards, especially from behind a robot on blocks, and getting it wrong
puts the two motors on the wrong sides — which reverses every turn while every
straight-line check still passes.

Spin **one wheel** and look at it instead. A velocity command whose differential
term exactly cancels one wheel's target does that through the normal control
path, no special firmware mode required:

```text
VEL,25.00,-14.32      motor 1 at +50 mm/s, motor 2 stationary
VEL,25.00,14.32       motor 1 stationary, motor 2 at +50 mm/s
```

The zeroed wheel is below `MIN_WHEEL_SPEED_MM_S`, so it coasts and genuinely
does not turn. 50 mm/s is about 20 rpm on a 47 mm wheel: slow enough to read at
a glance. Stand behind the robot, look toward the LiDAR end, and note which
wheel moves. On this robot, the first command turns the **right** wheel, so
motor 1 is at `-Y`.

The other shortcut that does not work: reversing which end is the front does
**not** mean negating `ENCODER_SIGN` and `MOTOR_DIRECTION_SIGN`. Those are
measured against whichever end `DIR` was told was the front, so once they are
right they stay right — negating them "because the front moved" produces a robot
that drives backwards under a positive command while every reading still looks
healthy. Both mistakes have been made in this repo.

### IMU direction

Rotate the robot left by hand. `/imu/data` yaw and `/odom` yaw should increase.
If they decrease, change:

```cpp
const float IMU_YAW_SIGN = -1.0f;
```

`nav_monitor` will also report `REVERSED ANGULAR` if commanded and measured
rotation disagree in sign.

With no IMU fitted this check still applies, but it tests something else: the
heading then comes from the wheels, so a decreasing yaw on a left turn means
`WHEEL_Y_SIGN` is backwards, not `IMU_YAW_SIGN`. Push the robot forward by hand
instead of turning it, confirm `/odom` x increases, then turn it and check the
sign of the yaw.

### Straight-line PID

Drive 1 m at 0.10 m/s and compare both wheel-speed telemetry values. Increase
`Kp` slightly if a wheel reacts too slowly. Increase `Ki` slowly if it remains
below target. Keep `Kd` small because encoder speed is noisy.

Then check it under load, which is the case the gains are actually for: drive
across a rug edge, a carpet-to-floor transition or a taped-down cable at
50 mm/s. The wheel should slow briefly and recover, not stop. If it stops and
the board prints `ERR,STALL,MOTOR_<n>`, the boost budget ran out — see
[Torque at low speed](#torque-at-low-speed) before reaching for the gains.

### Distance calibration

Measure actual travel over 1 m. Correct `WHEEL_DIAMETER_MM` or
`MEASURED_COUNTS_PER_REV` if odometry distance is consistently scaled.

### Track width

Command an in-place 360-degree turn. Correct `TRACK_WIDTH_MM` if the
encoder-derived turning behaviour does not match the physical robot. With an
IMU fitted, the heading published to ROS comes from the IMU and `TRACK_WIDTH_MM`
only affects the wheel speeds the kinematics asks for; without one it also sets
the scale of the encoder heading, so it matters twice as much.

### LiDAR alignment

A wall directly in front — in front meaning the caster end, see section 2 —
must appear at angle zero in `/scan`. Adjust the `laser_joint` yaw in the URDF
if the scan is rotated in RViz.

If the whole scan looks exactly 180 degrees out, the LiDAR's zero and the
driving direction disagree. Check which end the robot drives towards on a
positive `linear.x` first — a `FORWARD,60` on the serial link, wheels off the
ground, watching which way the tops of the wheels turn — because a mount that
is 180 degrees off and a `MOTOR_DIRECTION_SIGN` pair that is inverted look
identical in RViz and need opposite fixes.

## 20. Safety notes

- Test with wheels raised before floor testing.
- Keep a physical battery disconnect or hardware emergency-stop switch.
- The software emergency stop is latched in Arduino and must be explicitly
  cleared.
- The LiDAR stop gate stops forward commands inside 300 mm, but it does not
  replace a certified safety system.
- Tune the robot footprint, inflation radius and velocity limits before
  autonomous operation near people.

## 21. Troubleshooting

### `apt update` fails with "Conflicting values set for option Signed-By"

Two apt source files describe the same `packages.ros.org` repo with two
different `Signed-By` values, which apt refuses to reconcile: the official
deb822 `/etc/apt/sources.list.d/ros2.sources` alongside a legacy one-line
`ros2.list`. `bootstrap_robot_stack.sh` installs ROS 2's apt source exclusively
via the official `ros2-apt-source` package and removes any leftover `ros2.list`
before it can conflict, so a plain re-run should self-heal this. If it still
fails, something outside this script wrote a conflicting source file:

```bash
grep -rl "packages.ros.org" /etc/apt/sources.list /etc/apt/sources.list.d/
```

Keep `/etc/apt/sources.list.d/ros2.sources` and delete anything else, then
re-run the bootstrap script.

### `colcon build` fails on `mini_rviz_plugins` on the Raspberry Pi

That package needs `rviz_common` and Qt5, which `ros-humble-ros-base` does not
install. Re-run `bash scripts/bootstrap_robot_stack.sh pi`, which removes the
symlink if a previous desktop-mode run created it.

### `ros2 launch mini_robot <file>.launch.py` says the file was not found

`~/ros2_ws/src/mini_robot` is supposed to be a symlink to
`ros2_ws/src/mini_robot` in this repo, so `colcon build` always installs
whatever is currently in the repo. If that path is a real directory instead,
`colcon build` keeps installing that stale copy and `git pull` never touches
it. Check:

```bash
ls -la ~/ros2_ws/src/mini_robot
```

Re-running the bootstrap script fixes this automatically: it moves the stale
directory aside to `mini_robot.stale-backup.<timestamp>`, symlinks the correct
path back to this repo, and rebuilds. Delete the `.stale-backup.*` directory
once you have confirmed the robot still works.

### `ros2: command not found`

The ROS 2 environment is added to `~/.bashrc` by `bootstrap_robot_stack.sh`, so
a shell that was already open when the script ran does not have it. Either open
a new shell or re-read it:

```bash
source ~/.bashrc
```

If a *new* shell still cannot find `ros2`, check that the block is actually
there (`grep -n "mini ROS 2 environment" ~/.bashrc`) and that
`~/ros2_ws/install/setup.bash` exists -- the bootstrap only writes the block
once, so a run that failed before the build leaves nothing to source.

Note that this only ever works for **interactive** shells. Ubuntu's stock
`~/.bashrc` starts with

```bash
case $- in
    *i*) ;;
      *) return;;
esac
```

and the ROS block is appended at the end, so it is never reached by anything
non-interactive: `ssh pi 'ros2 topic list'`, cron jobs, and systemd units all
land on that early `return`. Those must source ROS explicitly:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

which is exactly what `scripts/ros_network_env.sh` does. A systemd unit that
starts `robot.launch.py` at boot needs those two lines in a wrapper script, not
a dependency on `~/.bashrc`; that wrapper is `scripts/robot_bringup.sh`, which
the optional autostart unit runs (section 11).

### The workstation cannot see the robot's topics

Almost always a `ROS_DOMAIN_ID` mismatch. Check on both machines:

```bash
echo "$ROS_DOMAIN_ID / $ROS_LOCALHOST_ONLY"
```

Both must print the same domain (this project uses `31`) and `0`. Two traps:

- If `~/.bashrc` already exported a `ROS_DOMAIN_ID` for some *other* project,
  `bootstrap_robot_stack.sh` inherits it (`ROS_DOMAIN="${ROS_DOMAIN_ID:-31}"`)
  and writes that value into the block it appends. Force it:
  `ROS_DOMAIN_ID=31 bash scripts/bootstrap_robot_stack.sh desktop`.
- Whichever ROS block sits *last* in `~/.bashrc` wins.

To confirm the network path itself, publish from the Pi and watch from the
workstation:

```bash
# Pi
ros2 topic pub -r 5 /probe std_msgs/msg/String "{data: hello}"
# workstation
ros2 topic echo /probe --once
```

If that works, DDS and the firewall are fine and the problem is upstream.

### Motors hum but do not turn

The board should now say so itself: a wheel commanded to move and not moving
raises its `status` bit, shows up as `stalled_motors` in `/diagnostics`, and
after 1.5 s prints `ERR,STALL,MOTOR_<n>` and rests for 2 s before trying again.
`nav_monitor` reports it as `WHEELS STALLED`.

In order of likelihood:

1. **Something is holding the wheel.** An obstruction, a jammed gearbox, a
   cable wound into a wheel. Turn each wheel by hand with the power off.
2. **The battery is low.** Stall current sags the motor rail, which drops
   torque, which deepens the stall. Check it under load, not at rest.
3. **The feed-forward fit does not match the robot.** Re-run `CAL` (section 7).
   A fit measured on a different battery, surface or gearbox puts the whole
   drive band at the wrong PWM.
4. **`FF_BREAKAWAY_PWM` is too low for this robot.** It ships at `{10, 14}`,
   measured as the first ascending `SWEEP` step that produced motion on each
   wheel. A stiffer gearbox, a heavier robot or a colder day needs more. Re-run
   the calibration rather than guessing — `scripts/analyse_motor_calibration.py`
   prints the value — then re-check that slow turns are still smooth.

If the wheels hum and nothing appears in telemetry at all, that is a different
fault: the encoder is not counting, so the loop is blind. Check the encoder
wiring before touching the PID.

### A wheel runs away at full speed on a small command

A sign fault, not a tuning one. `ENCODER_SIGN` inverted on one wheel makes the
PID see an error roughly twice the target that never shrinks, so the integrator
winds to its cap and the wheel pins at `MAX_DRIVE_PWM`. Run
`python3 scripts/validate_motor_control.py --tests link,signs`, which reads the
board's compiled-in constants first and refuses to advise if the board is
running something other than the source on disk.

### The robot drives fine but every turn goes the wrong way

`WHEEL_Y_SIGN` in `MiniRobotGeometry.h` has the two motors on the wrong sides of
the centre line. Nothing else catches this: both wheels drive forward correctly,
both encoders read positive, straight-line motion is perfect. See
[Wheel direction](#wheel-direction).

If turns are correct but the *map* rotates the wrong way, that is
`IMU_YAW_SIGN` instead.

## License

MIT
# MiniRobot
