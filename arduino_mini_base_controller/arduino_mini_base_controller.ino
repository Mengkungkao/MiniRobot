/*
  mini ROS2 low-level base controller

  Hardware:
  - Arduino Uno
  - TB6612FNG dual H-bridge (Zhangyue custom board) driving two DC gearmotors
  - Two quadrature encoders
  - MPU9250 / MPU9255 / MPU6500 on I2C (auto-detected)

  This is the two-wheel differential sibling of arduino_ros2_base_controller
  (four wheels, QGPMaker shield, MPU6050). The serial protocol, control
  structure and safety behaviour are deliberately the same so the ROS 2 side
  is a straight port; the differences are the motor driver, the wheel count
  and the IMU driver.

  Responsibility split:
  - Arduino: 100 Hz encoder/IMU acquisition, two independent wheel-speed PID
    loops, odometry integration, command watchdog and latched emergency stop.
  - Raspberry Pi 4B: ROS2 serial bridge, /cmd_vel, /odom, /imu/data,
    /joint_states.
  - Ubuntu workstation: RViz, SLAM Toolbox, Nav2, costmaps, planner and
    Regulated Pure Pursuit controller.

  Firmware layout (Arduino IDE/arduino-cli compile every .cpp/.h in this
  folder as one sketch, no separate library and no sketchbook setting needed):
    MiniRobotPins.h       pin map and motor indexing        } shared, kept
    MiniRobotGeometry.h   wheel size, counts/rev, body       } byte-identical
                            layout, commissioning signs      }
    MiniRobotMotors.h/.cpp  TB6612FNG channels, 31 kHz PWM  } with
    MiniRobotEncoders.h/.cpp  quadrature ISRs, atomic reads } the calibration
                                                           } sketch's copies
    Config.h          timing, drive limits, feed-forward fit
    Types.h           WheelPID fractional-order PID
    Imu.h/.cpp         MPU/AK8963 driver + Mahony 6/9-axis sensor fusion
    MotorControl.h/.cpp  drive kinematics, PID loop, odometry, watchdog/e-stop
    Sweep.h/.cpp       blocking open-loop PWM sweep (feed-forward calibration)
    CommandParser.h/.cpp  serial line protocol
    Telemetry.h/.cpp   the 20 Hz 'T,...' telemetry line
    arduino_mini_base_controller.ino  setup()/loop() only

  ROS coordinate convention (REP-103), shared with the Pi:
  - +X forward, and FORWARD IS THE LiDAR END
  - +Y to port
  - +yaw counter-clockwise

  The firmware used to call the opposite end forward and let the Pi rotate
  every x/y quantity by 180 degrees on the way past. It does not any more:
  one frame, defined once, in MiniRobotGeometry.h.

  Serial settings:
  - 115200 baud, 8-N-1
  - Commands are ASCII lines terminated by \n, \r or ':'

  Main command:
    VEL,<linear_mm_s>,<angular_deg_s>

  Test aliases and maintenance commands:
    FORWARD,<speed_mm_s>          drive towards the LiDAR end
    REVERSE,<speed_mm_s>
    LEFT,<angular_deg_s>          turn counter-clockwise in place
    RIGHT,<angular_deg_s>         turn clockwise in place
    STOP
    ESTOP
    CLEAR_ESTOP
    RESET_ODOM
    ZERO_YAW
    CAL_IMU
    MAG,<0|1>
    IMU
    CFG
    PID,<kp>,<ki>,<kd>[,<lambda>,<mu>]
    PIDM,<motor_1_to_2>,<kp>,<ki>,<kd>[,<lambda>,<mu>]
    SWEEP,<motor_1_to_2>,<pwm_step_1_to_50>,<dwell_ms_min_100>[,R]
    PING

  Motor numbering: 1 = TB6612 channel A, 2 = channel B. That is all a motor
  number means -- which side of the robot each one drives is stated once, in
  WHEEL_Y_SIGN (MiniRobotGeometry.h), and nowhere else.

  Wheel speed loops are fractional-order PID (PI^lambda D^mu). lambda/mu
  default to 1.0 (classic PID) when omitted from PID/PIDM.

  Telemetry at 20 Hz (23 comma-separated fields):
    T,time_ms,x_mm,y_mm,yaw_deg,vx_mm_s,wz_deg_s,
      tick_m1,tick_m2,
      speed_m1,speed_m2,
      pwm_m1,pwm_m2,
      roll_deg,pitch_deg,
      gx_deg_s,gy_deg_s,
      ax_g,ay_g,az_g,
      estop,watchdog,status

  yaw_deg/wz_deg_s are the heading the pose was integrated along: the IMU's
  when one is fitted, encoder-derived when one is not. 'status' is a bitmask:
  bit0 motor 1 stalled, bit1 motor 2 stalled, bit2 IMU present, bit3
  magnetometer fusion on.

  Calibration output from SWEEP (ignored by the ROS bridge):
    C,<motor>,<F|R>,<pwm>,<measured_mm_s>

  WIRING
  ------------------------------------------------------------
  TB6612FNG            Uno
    PWMA (motor 1)     D10
    AIN1 (motor 1)     D5
    AIN2 (motor 1)     D4
    PWMB (motor 2)     D9
    BIN1 (motor 2)     D7
    BIN2 (motor 2)     D8
    STBY               D12
    VM                 motor battery +
    VCC                5V
    GND                GND (common with battery ground)

  Encoders             Uno
    Motor 1 channel A  D2   (INT0)
    Motor 1 channel B  D6
    Motor 2 channel A  D3   (INT1)
    Motor 2 channel B  D11

  MPU9250/9255/6500    Uno
    VCC                3.3V or 5V, per your module
    GND                GND
    SDA                A4
    SCL                A5
*/

#include "Config.h"
#include "MotorControl.h"
#include "Imu.h"
#include "CommandParser.h"
#include "Telemetry.h"

static uint32_t lastControlMs = 0;
static uint32_t lastTelemetryMs = 0;

// One-time init: TB6612 outputs at ~31 kHz PWM, encoder interrupts, per-wheel
// PID gains, I2C at 400 kHz, and IMU startup calibration (the robot must be
// still).
void setup() {
  Serial.begin(SERIAL_BAUD);

  motorControlBegin();

  delay(IMU_STARTUP_SETTLE_MS);
  if (!imuBegin()) {
    // No IMU. The robot still has a heading source -- the difference between
    // the two wheel distances -- so it drives, maps and navigates on encoder
    // odometry alone, drifting faster wherever a wheel can slip. Telemetry
    // says so in every line (status bit 2 clear) rather than leaving a host to
    // wonder why yaw looks noisy.
    //
    // Set REQUIRE_IMU (Config.h) to make this fatal again: a latched e-stop
    // and a robot that will not move without its gyro.
    Serial.println(F("ERR,NO_IMU_DETECTED"));
    if (REQUIRE_IMU) {
      Serial.println(F("ERR,IMU_REQUIRED,ESTOP_LATCHED"));
      emergencyStopLatched = true;
    } else {
      Serial.println(F("INFO,HEADING_SOURCE,ENCODERS"));
    }
  }

  releaseAllMotors();
  resetEncoderDeltas();
  zeroYaw();
  armCommandWatchdog();

  lastControlMs = millis();
  lastTelemetryMs = millis();
  printReady();
}

// Main loop: poll serial every pass; run IMU acquisition + control at 100 Hz
// and telemetry at 20 Hz off millis() schedules.
void loop() {
  readSerialCommands();

  const uint32_t now = millis();
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    // Use the true elapsed time, clamped so a hiccup can't blow up the
    // integrator or derivative.
    float dt = (now - lastControlMs) * 0.001f;
    lastControlMs = now;
    dt = constrain(dt, CONTROL_DT_MIN_S, CONTROL_DT_MAX_S);

    if (mpuFound) updateIMU();
    updateControl(dt);
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    publishTelemetry();
  }
}
