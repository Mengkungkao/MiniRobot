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
    MiniRobotPins.h       pin map and wheel indexing        } shared, kept
    MiniRobotGeometry.h   wheel size, counts/rev, signs     } byte-identical
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

  ROS coordinate convention:
  - +X forward
  - +Y left
  - +yaw counter-clockwise

  Serial settings:
  - 500000 baud, 8-N-1
  - Commands are ASCII lines terminated by \n, \r or ':'

  Main command:
    VEL,<linear_mm_s>,<angular_deg_s>

  Test aliases and maintenance commands:
    FORWARD,<speed_mm_s>
    REVERSE,<speed_mm_s>
    LEFT,<angular_deg_s>
    RIGHT,<angular_deg_s>
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

  Motor numbering: 1 = LEFT (TB6612 channel A), 2 = RIGHT (channel B).

  Wheel speed loops are fractional-order PID (PI^lambda D^mu). lambda/mu
  default to 1.0 (classic PID) when omitted from PID/PIDM.

  Telemetry at 20 Hz (22 comma-separated fields):
    T,time_ms,x_mm,y_mm,yaw_deg,vx_mm_s,wz_deg_s,
      tick_left,tick_right,
      speed_left,speed_right,
      pwm_left,pwm_right,
      roll_deg,pitch_deg,
      gx_deg_s,gy_deg_s,
      ax_g,ay_g,az_g,
      estop,watchdog

  Calibration output from SWEEP (ignored by the ROS bridge):
    C,<motor>,<F|R>,<pwm>,<measured_mm_s>

  WIRING
  ------------------------------------------------------------
  TB6612FNG            Uno
    PWMA (left)        D10
    AIN1 (left)        D5
    AIN2 (left)        D4
    PWMB (right)       D9
    BIN1 (right)       D7
    BIN2 (right)       D8
    STBY               D12
    VM                 motor battery +
    VCC                5V
    GND                GND (common with battery ground)

  Encoders             Uno
    Left  channel A    D2   (INT0)
    Left  channel B    D6
    Right channel A    D3   (INT1)
    Right channel B    D11

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

  delay(200);
  if (!imuBegin()) {
    // Without a heading source the robot must not drive: odometry would
    // integrate every movement along a heading stuck at zero, so the map and
    // Nav2 would both be wrong in a way that is hard to see from the outside.
    Serial.println(F("ERR,NO_IMU_DETECTED"));
    emergencyStopLatched = true;
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
    dt = constrain(dt, 0.005f, 0.050f);

    if (mpuFound) updateIMU();
    updateControl(dt);
  }

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    publishTelemetry();
  }
}
