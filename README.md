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
  MPU9250 / MPU9255 / MPU6500, Mahony fusion (auto-detected)
  Two independent wheel-speed PID loops
  500 ms communication watchdog
  Latched emergency stop
```

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

## 3. Folder structure

```text
arduino_mini_base_controller/          the firmware to upload to the Arduino Uno
  arduino_mini_base_controller.ino     setup()/loop() and the protocol reference
  MiniRobotPins.h                      pin map and wheel indexing        \
  MiniRobotGeometry.h                  wheel size, counts/rev, signs      | shared
  MiniRobotMotors.h/.cpp               TB6612FNG channels, 31 kHz PWM     | layer
  MiniRobotEncoders.h/.cpp             quadrature ISRs, atomic reads     /
  Config.h                             timing, drive limits, feed-forward fit
  Types.h                              WheelPID (fractional-order PID)
  Imu.h/.cpp                           MPU/AK8963 driver + Mahony fusion
  MotorControl.h/.cpp                  kinematics, PID loop, odometry, watchdog
  Sweep.h/.cpp                         built-in SWEEP calibration command
  CommandParser.h/.cpp                 serial command set
  Telemetry.h/.cpp                     the 20 Hz 'T,...' line
arduino_mini_motor_calibration/        bench sketch: measures and fits the
  arduino_mini_motor_calibration.ino    feed-forward motor constants (section 7)
  MiniRobot*.h/.cpp                    byte-identical copies of the six
                                         shared-layer files above
  Signs.h/.cpp                         ENCODER_SIGN / MOTOR_DIRECTION_SIGN
                                         overrides for a wheel wired backwards
  Config.h                             baud, fit thresholds, sweep settings
  LinearFit.h                          Welford streaming least-squares
  FitResults.h/.cpp                    results table + paste-ready constants
  CalUtil.h/.cpp                       abort handling, one speed measurement
  Sweep.h/.cpp                         the PWM sweep and the fit
  Verify.h/.cpp                        open-loop check of the fitted constants
  Interactive.h/.cpp                   HOLD, TICKS, CONFIG, HELP
  CommandParser.h/.cpp                 serial command set
  README.md                            full reference for that sketch
  Calibration_results.md               log of a past CAL run
arduino_mini_imu_calibration/          bench sketch: measures the IMU constants
  arduino_mini_imu_calibration.ino      the firmware cannot measure itself
                                         (section 6)
  MiniRobot{Pins,Geometry,Motors}      four of the six shared-layer files; the
                                         encoders are not used here
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
```

## 4. Wiring

### TB6612FNG motor driver

| TB6612FNG | Arduino Uno | Notes |
|-----------|-------------|-------|
| `PWMA`    | `D10`       | left wheel speed  |
| `AIN1`    | `D5`        | left wheel direction |
| `AIN2`    | `D4`        | left wheel direction |
| `PWMB`    | `D9`        | right wheel speed |
| `BIN1`    | `D7`        | right wheel direction |
| `BIN2`    | `D8`        | right wheel direction |
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
| Left    | `D2` (INT0) | `D6`    |
| Right   | `D3` (INT1) | `D11`   |

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
`0x70` = MPU6500 (no magnetometer), `0x71` = MPU9250, `0x73` = MPU9255. Send
`IMU` over serial (or read `/diagnostics`) to see what it found.

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

Testing aliases are also supported:

```text
FORWARD,100
REVERSE,100
LEFT,45
RIGHT,45
```

Arduino publishes one telemetry line at 20 Hz, 22 comma-separated fields:

```text
T,time_ms,x_mm,y_mm,yaw_deg,vx_mm_s,wz_deg_s,
  tick_left,tick_right,
  speed_left,speed_right,
  pwm_left,pwm_right,
  roll_deg,pitch_deg,
  gx_deg_s,gy_deg_s,
  ax_g,ay_g,az_g,
  estop,watchdog
```

Motor numbering is `1` = left, `2` = right, everywhere in the protocol.

## 6. IMU behaviour and the magnetometer

The sketch runs a Mahony AHRS filter over the accelerometer and gyro, and
optionally the AK8963 magnetometer.

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
115200 baud and send:

```text
CAL
```

It sweeps both wheels in both directions (two passes each, about two and a
half minutes), fits the line on-board, and prints the four constant lines ready
to paste over the `FF_SLOPE_*` / `FF_INTERCEPT_*` definitions in
`arduino_mini_base_controller.ino`:

```text
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {1.8734f, 1.9102f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-71.20f, -76.85f};
...
```

Then send `VERIFY`: it drives a few target speeds through the fresh fit open
loop and reports the error at each. Under ~10% is good; PID absorbs the rest.
Send `HELP` for the rest of the commands (`TICKS` for counts-per-rev, `HOLD`
for a single-point spot check, `RANGE`/`STEP`/`DWELL`/`PASSES` to retune the
sweep). Any character aborts a running sweep.

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
| `MiniRobotEncoders.h/.cpp` | yes | yes | -- (unused) |

Every copy that exists **must stay byte-identical**. The motor calibration
sketch reports mm/s scaled by `WHEEL_DIAMETER_MM` and `MEASURED_COUNTS_PER_REV`
and the firmware consumes the fit scaled by the same constants, so a mismatch
silently bakes a constant error into every speed the robot drives. After
editing any copy, push it out and check:

```bash
cp arduino_mini_base_controller/MiniRobot{Pins,Geometry}.h \
   arduino_mini_base_controller/MiniRobotMotors.{h,cpp} \
   arduino_mini_imu_calibration/
cp arduino_mini_base_controller/MiniRobot*.h \
   arduino_mini_base_controller/MiniRobot*.cpp \
   arduino_mini_motor_calibration/

for f in MiniRobotPins.h MiniRobotGeometry.h MiniRobotMotors.h \
         MiniRobotMotors.cpp MiniRobotEncoders.h MiniRobotEncoders.cpp; do
  for d in arduino_mini_motor_calibration arduino_mini_imu_calibration; do
    [ -f "$d/$f" ] || continue
    cmp -s "arduino_mini_base_controller/$f" "$d/$f" \
      && echo "OK   $d/$f" || echo "DIFF $d/$f"
  done
done
```

Verified build size on `arduino:avr:uno` (AVR core 1.8.8, avr-gcc 7.3.0, LTO on
as the IDE defaults to):

| Sketch | Flash (of 32256) | SRAM (of 2048) |
| --- | --- | --- |
| `arduino_mini_base_controller` | 27554 B (85%) | 1102 B (54%) |
| `arduino_mini_motor_calibration` | 17292 B (54%) | 497 B (24%) |
| `arduino_mini_imu_calibration` | 30326 B (94%) | 791 B (39%) |

Splitting the firmware into modules cost nothing in SRAM. Duplicating the shared
layer per sketch costs nothing either: `const` file-scope data has internal
linkage, so the compiler folds the scalars and drops whatever a sketch does not
use.

Flash is the tight resource. The base controller has about 4.7 KB spare -- room
for small additions but not for another large subsystem; the cheapest savings
are dropping the `SWEEP` command once the feed-forward fit is measured, and the
nine-axis Mahony path if you never enable the magnetometer. The IMU calibration
sketch has under 2 KB spare and is essentially at its final size: if you need to
extend it, `RAW` and `MOTORS` are the two commands nothing else depends on.

Only one sketch is on the board at a time, so these do not compete -- but each
must fit on its own.

If no IMU is detected at boot the sketch latches the emergency stop and reports
`ERR,NO_IMU_DETECTED`. That is deliberate: without a heading source, odometry
would integrate every movement along a heading stuck at zero, and both the map
and Nav2 would be wrong in a way that is hard to see from the outside.

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

## 15. Motor PID behaviour

For straight motion, both wheels receive the same speed setpoint. Each wheel
has its own PID state and PWM output. This is intentional: equal speed does not
usually require equal PWM, because motor torque, gearbox friction and wheel
loading differ between the two sides.

Starting gains are:

```text
Kp = 0.30
Ki = 0.020
Kd = 0.0030
```

Change both wheels at runtime:

```text
PID,0.30,0.020,0.003
```

Change one wheel, where the motor number is 1 (left) or 2 (right):

```text
PIDM,2,0.28,0.020,0.003
```

The loops are fractional-order PID (PI^lambda D^mu). `lambda`/`mu` default to
`1.0`, which is exactly classic PID and takes a fast path that skips the
Grünwald–Letnikov shaping entirely. Both can be passed as optional 4th and 5th
arguments to `PID`/`PIDM` if you want to experiment.

## 16. Mapping with RViz, SLAM and Nav2

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
actually used -- in `arduino_mini_motor_calibration/`, at 115200 baud:

```text
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

### Encoder direction

Command slow forward motion. Both wheel speed values in Arduino telemetry must
be positive. If one is negative, flip that entry in:

```cpp
const int8_t ENCODER_SIGN[MOTOR_COUNT]
```

### Motor direction

If a wheel physically spins backwards when the firmware asks for forward, flip
that entry in:

```cpp
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT]
```

### IMU direction

Rotate the robot left by hand. `/imu/data` yaw and `/odom` yaw should increase.
If they decrease, change:

```cpp
const float IMU_YAW_SIGN = -1.0f;
```

`nav_monitor` will also report `REVERSED ANGULAR` if commanded and measured
rotation disagree in sign.

### Straight-line PID

Drive 1 m at 0.10 m/s and compare both wheel-speed telemetry values. Increase
`Kp` slightly if a wheel reacts too slowly. Increase `Ki` slowly if it remains
below target. Keep `Kd` small because encoder speed is noisy.

### Distance calibration

Measure actual travel over 1 m. Correct `WHEEL_DIAMETER_MM` or
`MEASURED_COUNTS_PER_REV` if odometry distance is consistently scaled.

### Track width

Command an in-place 360-degree turn. Correct `TRACK_WIDTH_MM` if the
encoder-derived turning behaviour does not match the physical robot. Heading
published to ROS still comes from the IMU.

### LiDAR alignment

A wall directly in front must appear at angle zero in `/scan`. Adjust the
`laser_joint` yaw in the URDF if the scan is rotated in RViz.

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
a dependency on `~/.bashrc`.

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

PWM is below the motors' breakaway threshold. `nav_monitor` reports this as
`WHEELS STALLED`. Either the feed-forward fit is still the shipped guess (see
section 7) or the battery is low.

## License

MIT
# MiniRobot
