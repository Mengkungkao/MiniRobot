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
    PID,<kp>,<ki>,<kd>[,<lambda>,<mu>]
    PIDM,<motor_1_to_2>,<kp>,<ki>,<kd>[,<lambda>,<mu>]
    SWEEP,<motor_1_to_2>,<pwm_step>,<dwell_ms>[,R]
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

#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Pin map
// -----------------------------------------------------------------------------

// TB6612FNG channel A -> LEFT wheel
const uint8_t PIN_PWMA = 10;
const uint8_t PIN_AIN1 = 5;
const uint8_t PIN_AIN2 = 4;

// TB6612FNG channel B -> RIGHT wheel
const uint8_t PIN_PWMB = 9;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 8;

// Driver hardware enable. Held HIGH for the whole run; pulling it LOW puts
// both bridges into standby (outputs off) regardless of IN1/IN2/PWM.
const uint8_t PIN_STBY = 12;

// Encoders. Only D2/D3 are external-interrupt capable on the Uno, so channel A
// of each encoder goes there and channel B is read inside the ISR to recover
// the direction.
const uint8_t PIN_ENC_LEFT_A = 2;
const uint8_t PIN_ENC_LEFT_B = 6;
const uint8_t PIN_ENC_RIGHT_A = 3;
const uint8_t PIN_ENC_RIGHT_B = 11;

// -----------------------------------------------------------------------------
// Robot geometry and timing
// -----------------------------------------------------------------------------

const uint8_t MOTOR_COUNT = 2;
const uint8_t LEFT = 0;
const uint8_t RIGHT = 1;

const uint32_t SERIAL_BAUD = 500000UL;
const uint16_t CONTROL_PERIOD_MS = 10;      // 100 Hz motor control
const uint16_t TELEMETRY_PERIOD_MS = 50;    // 20 Hz telemetry
const uint16_t COMMAND_WATCHDOG_MS = 500;   // stop if Pi heartbeat disappears

const float WHEEL_DIAMETER_MM = 80.5f;

// COMMISSIONING VALUE. Encoder counts this firmware records per full turn of
// the *wheel* (i.e. after the gearbox), measured per wheel.
//
// This sketch attaches a CHANGE interrupt to channel A only, so it records two
// counts per encoder cycle (x2 decoding), not one. A "360 PPR" encoder on the
// output shaft therefore reads 720 counts per wheel revolution, which is the
// default below. If the encoder sits on the motor shaft instead, the real
// number is that times the gear ratio and will be in the thousands.
//
// Measure it, do not assume it:
//   1. Send RESET_ODOM.
//   2. Rotate one wheel by hand exactly one full turn.
//   3. Read that wheel's tick field from the telemetry line and put the
//      magnitude here.
// Per-wheel values (rather than one shared constant) matter because encoder
// undercount on one side makes its measured speed under-report, which makes
// PID drive that wheel physically faster than commanded.
const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {720.0f, 720.0f};

const float MM_PER_COUNT[MOTOR_COUNT] = {
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[0],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[1],
};

// COMMISSIONING VALUE. Distance between the two wheel contact patches.
// Command an in-place 360 degree turn and correct this if the encoder-derived
// rotation does not match the physical robot.
const float TRACK_WIDTH_MM = 200.0f;

// Limits should stay conservative until the robot has been tested on blocks.
const float MAX_LINEAR_MM_S = 250.0f;
const float MAX_ANGULAR_DEG_S = 120.0f;
const float MAX_WHEEL_MM_S = 250.0f;
const float LINEAR_ACCEL_MM_S2 = 150.0f;
const float ANGULAR_ACCEL_DEG_S2 = 120.0f;

const uint8_t MAX_DRIVE_PWM = 210;

// COMMISSIONING VALUE. Forward motion must produce a positive speed reading on
// both wheels. If one wheel reports negative while driving forward, flip its
// sign here rather than swapping the encoder wires.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {1, 1};

// COMMISSIONING VALUE. If a wheel physically spins backwards when the firmware
// asks for forward, flip its sign here rather than swapping the motor leads.
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {1, 1};

// COMMISSIONING VALUES. Per-wheel, per-direction open-loop PWM-to-speed linear
// fit (|measured_mm_s| = slope*pwm + intercept), inverted in
// speedFeedForwardPWM() so PID only has to correct the residual.
//
// The defaults below are a generic straight line through a PWM ~40 dead zone
// and ~400 mm/s at full PWM. They are a starting point, NOT a calibration:
// brushed gearmotors differ from each other by tens of percent and are
// direction-asymmetric, which shows up on the robot as driving straight
// forward but curving in reverse. Replace them with real numbers:
//
//   SWEEP,1,10,1000       left wheel, forward, PWM steps of 10, 1 s each
//   SWEEP,1,10,1000,R     left wheel, reverse
//   SWEEP,2,10,1000       right wheel, forward
//   SWEEP,2,10,1000,R     right wheel, reverse
//
// Each step prints "C,<motor>,<F|R>,<pwm>,<measured_mm_s>". Drop the stalled
// points at the bottom of the sweep, least-squares fit the rest, and paste the
// slope/intercept here.
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {1.860f, 1.860f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-74.4f, -74.4f};
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {1.860f, 1.860f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-74.4f, -74.4f};

// COMMISSIONING VALUE. A physical left turn must make ROS yaw increase
// (REP-103, CCW-positive). Rotate the robot left by hand and watch the yaw
// field in telemetry; if it decreases, set this to -1.0f.
const float IMU_YAW_SIGN = 1.0f;

// Lowest PWM the SWEEP calibration command starts from.
const uint8_t SWEEP_MIN_PWM = 30;

// -----------------------------------------------------------------------------
// IMU: MPU9250 / MPU9255 / MPU6500 with AK8963 magnetometer
// -----------------------------------------------------------------------------

const uint8_t MPU_ADDR_1 = 0x68;
const uint8_t MPU_ADDR_2 = 0x69;

const uint8_t REG_WHO_AM_I = 0x75;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_CONFIG = 0x1A;
const uint8_t REG_SMPLRT_DIV = 0x19;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
const uint8_t REG_INT_PIN_CFG = 0x37;
const uint8_t REG_INT_ENABLE = 0x38;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_GYRO_XOUT_H = 0x43;

// AK8963 magnetometer, exposed on the I2C bus once the MPU's bypass is enabled.
const uint8_t AK8963_ADDR = 0x0C;
const uint8_t AK8963_WHO_AM_I = 0x00;
const uint8_t AK8963_ST1 = 0x02;
const uint8_t AK8963_XOUT_L = 0x03;
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_ASAX = 0x10;

// Configured ranges: accel +/-2 g, gyro +/-250 deg/s, magnetometer 16-bit.
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE = 131.0f;
const float MAG_SCALE = 4912.0f / 32760.0f;

// Magnetic declination for true north. Only applied when magnetometer fusion
// is actually running, and even then it washes out of odometry because the
// heading is zeroed at startup.
const float MAG_DECLINATION_DEG = 12.0f;

// Mahony filter gains. Higher KP corrects faster but tracks accelerometer
// noise; KI removes gyro bias but destabilises the filter if pushed too high.
const float MAHONY_KP = 2.0f;
const float MAHONY_KI = 0.02f;

// Magnetometer fusion is OFF by default. On a small robot the AK8963 sits
// centimetres from two brushed motors and a battery, so the field it measures
// swings with motor current and turns yaw into a function of throttle. Gyro
// integration (6-axis fusion) has no absolute reference but is smooth and
// locally accurate, which is what odometry actually needs; SLAM supplies the
// absolute correction. Enable at runtime with MAG,1 only after checking that
// yaw stays put while the wheels spin.
bool useMagnetometer = false;

uint8_t mpuAddress = MPU_ADDR_1;
uint8_t mpuWhoAmI = 0x00;
bool mpuFound = false;
bool magnetometerPresent = false;
bool magnetometerDataValid = false;

// Converted sensor values: accel in g, gyro in deg/s, mag in uT.
float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
float magX = 0.0f, magY = 0.0f, magZ = 0.0f;

float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;

// Magnetometer hard-iron offsets (uT) and soft-iron scale correction. Leave at
// 0/1 until you have run a figure-of-eight calibration.
float magOffsetX = 0.0f, magOffsetY = 0.0f, magOffsetZ = 0.0f;
float magScaleX = 1.0f, magScaleY = 1.0f, magScaleZ = 1.0f;

// Factory sensitivity adjustment burned into the AK8963's fuse ROM.
float magAdjX = 1.0f, magAdjY = 1.0f, magAdjZ = 1.0f;

// Orientation quaternion and the Mahony integral feedback term.
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
bool fusionStarted = false;

// Fused attitude in degrees. fusedYaw is absolute (filter frame); the odometry
// heading below is this minus the zero reference.
float fusedRoll = 0.0f;
float fusedPitch = 0.0f;
float fusedYaw = 0.0f;

uint32_t lastFusionMicros = 0;

// -----------------------------------------------------------------------------
// TB6612FNG driver
// -----------------------------------------------------------------------------

// One H-bridge channel. Datasheet truth table, with STBY high:
//   IN1=H IN2=L  -> clockwise, speed set by PWM
//   IN1=L IN2=H  -> counter-clockwise, speed set by PWM
//   IN1=L IN2=L  -> stop, outputs high-impedance (the wheel freewheels)
//   IN1=H IN2=H  -> short brake, outputs shorted (the wheel resists rotation)
struct TB6612Channel {
  uint8_t pwmPin;
  uint8_t in1Pin;
  uint8_t in2Pin;

  void begin() const {
    pinMode(pwmPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    coast();
  }

  void coast() const {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

  void brake() const {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
  }

  void drive(bool forward, uint8_t duty) const {
    digitalWrite(in1Pin, forward ? HIGH : LOW);
    digitalWrite(in2Pin, forward ? LOW : HIGH);
    analogWrite(pwmPin, duty);
  }
};

const TB6612Channel motor[MOTOR_COUNT] = {
  {PIN_PWMA, PIN_AIN1, PIN_AIN2},   // LEFT
  {PIN_PWMB, PIN_BIN1, PIN_BIN2},   // RIGHT
};

// -----------------------------------------------------------------------------
// Encoders
// -----------------------------------------------------------------------------

// Written by the ISRs, so every read from normal code has to be atomic.
volatile int32_t encoderCount[MOTOR_COUNT] = {0, 0};

// Quadrature direction from one channel: on a CHANGE edge of A, A == B means
// one direction and A != B the other. Two counts per encoder cycle.
void encoderLeftISR() {
  if (digitalRead(PIN_ENC_LEFT_A) == digitalRead(PIN_ENC_LEFT_B)) encoderCount[LEFT]++;
  else encoderCount[LEFT]--;
}

void encoderRightISR() {
  if (digitalRead(PIN_ENC_RIGHT_A) == digitalRead(PIN_ENC_RIGHT_B)) encoderCount[RIGHT]++;
  else encoderCount[RIGHT]--;
}

// Read the tick delta since the previous call and clear the counter, with
// interrupts disabled so an ISR cannot update the count mid-read.
int32_t readEncoderAndResetAtomic(uint8_t index) {
  noInterrupts();
  const int32_t value = encoderCount[index];
  encoderCount[index] = 0;
  interrupts();
  return value;
}

// -----------------------------------------------------------------------------
// PID controller
// -----------------------------------------------------------------------------

// Fractional-order PID: PI^lambda D^mu, the same shape as the four-wheel
// controller so gains transfer between the two robots. The integral and
// derivative keep their classic form -- that is what gives a true pole at the
// origin, so a constant wheel-speed error is still integrated away to zero.
// The fractional order only reshapes what feeds them: the error passes through
// a Grunwald-Letnikov (GL) short-memory filter of the residual order
// (lambda-1) before the integral and (mu-1) before the derivative. At
// lambda == mu == 1.0 the GL filter is exactly the identity (order-0 binomial
// coefficients are 1,0,0,...), so the default configuration reduces to classic
// PID and takes the cheap path below.
struct WheelPID {
  static const uint8_t GL_MEMORY_LENGTH = 20;

  float kp;
  float ki;
  float kd;
  float lambda;
  float mu;
  float integral;
  float previousShapedError;
  float previousTarget;
  float errorHistory[GL_MEMORY_LENGTH];  // errorHistory[0] is the newest sample
  uint8_t historyLen;

  void configure(float p, float i, float d, float lam, float m) {
    kp = p;
    ki = i;
    kd = d;
    lambda = lam;
    mu = m;
    reset();
  }

  void reset() {
    integral = 0.0f;
    previousShapedError = 0.0f;
    previousTarget = 0.0f;
    for (uint8_t j = 0; j < GL_MEMORY_LENGTH; ++j) errorHistory[j] = 0.0f;
    historyLen = 0;
  }

  // GL binomial coefficient recursion: c0 = 1, cj = c(j-1) * (1 - (alpha+1)/j).
  // dtScale converts the unitless GL sum to physical units (dt^alpha).
  float glShape(float alpha, float dtScale) const {
    float coeff = 1.0f;
    float sum = errorHistory[0];
    for (uint8_t j = 1; j < historyLen; ++j) {
      coeff *= (1.0f - (alpha + 1.0f) / (float)j);
      sum += coeff * errorHistory[j];
    }
    return sum * dtScale;
  }

  // Order 1.0 means identity shaping, so skip both the 20-tap GL sum and the
  // pow() call. That is the default configuration and pow() is expensive on an
  // 8-bit AVR inside a 100 Hz loop.
  float shapedError(float order) const {
    if (fabs(order - 1.0f) < 1e-4f) return errorHistory[0];
    return glShape(order - 1.0f, pow(0.01f, 1.0f - order));
  }

  float update(float target, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    // Clear accumulated state when stopped or when wheel direction changes.
    if (fabs(target) < 1.0f || (target * previousTarget < 0.0f)) {
      reset();
    }
    previousTarget = target;

    const float error = target - measured;
    for (uint8_t j = GL_MEMORY_LENGTH - 1; j > 0; --j) errorHistory[j] = errorHistory[j - 1];
    errorHistory[0] = error;
    if (historyLen < GL_MEMORY_LENGTH) ++historyLen;

    integral += shapedError(lambda) * dt;
    // Anti-windup: cap the integral by its output contribution (ki*integral,
    // in PWM), not a fixed error-seconds bound. A fixed bound leaves only a
    // couple of PWM counts of trim at realistic ki values, so feed-forward
    // error can never be integrated away and wheels plateau off-target.
    const float MAX_INTEGRAL_PWM = 80.0f;
    if (ki > 0.0001f) {
      const float limit = MAX_INTEGRAL_PWM / ki;
      integral = constrain(integral, -limit, limit);
    } else {
      integral = constrain(integral, -500.0f, 500.0f);
    }

    const float shapedForDerivative = shapedError(mu);
    const float derivative = (shapedForDerivative - previousShapedError) / dt;
    previousShapedError = shapedForDerivative;

    return kp * error + ki * integral + kd * derivative;
  }
};

WheelPID wheelPID[MOTOR_COUNT];

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

float commandLinearMMs = 0.0f;
float commandAngularDegS = 0.0f;
float rampedLinearMMs = 0.0f;
float rampedAngularDegS = 0.0f;

float wheelTargetMMs[MOTOR_COUNT] = {0};
float wheelMeasuredMMs[MOTOR_COUNT] = {0};
float wheelFilteredMMs[MOTOR_COUNT] = {0};
float wheelPWM[MOTOR_COUNT] = {0};
int32_t cumulativeTicks[MOTOR_COUNT] = {0};

float poseXMM = 0.0f;
float poseYMM = 0.0f;
float yawDeg = 0.0f;
float yawRateDegS = 0.0f;
float yawZeroDeg = 0.0f;
float linearVelocityMMs = 0.0f;

bool emergencyStopLatched = false;
bool watchdogStopped = true;

uint32_t lastCommandMs = 0;
uint32_t lastControlMs = 0;
uint32_t lastTelemetryMs = 0;

char commandBuffer[96];
uint8_t commandLength = 0;

// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  if (Wire.endTransmission(false) != 0) return 0xFF;

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

// Burst-read 'count' registers. Any bytes the device did not return are zeroed
// so a truncated transfer cannot leave stale values in the caller's buffer.
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, count);

  uint8_t i = 0;
  while (Wire.available() && i < count) dest[i++] = Wire.read();
  while (i < count) dest[i++] = 0;
}

// -----------------------------------------------------------------------------
// IMU
// -----------------------------------------------------------------------------

// WHO_AM_I distinguishes the three supported parts: 0x70 MPU6500 (no
// magnetometer), 0x71 MPU9250, 0x73 MPU9255. The module may sit at either I2C
// address depending on how its AD0 pin is strapped.
void detectMPU() {
  const uint8_t id1 = readByte(MPU_ADDR_1, REG_WHO_AM_I);
  delay(10);
  const uint8_t id2 = readByte(MPU_ADDR_2, REG_WHO_AM_I);
  delay(10);

  if (id1 == 0x70 || id1 == 0x71 || id1 == 0x73) {
    mpuAddress = MPU_ADDR_1;
    mpuWhoAmI = id1;
    mpuFound = true;
  } else if (id2 == 0x70 || id2 == 0x71 || id2 == 0x73) {
    mpuAddress = MPU_ADDR_2;
    mpuWhoAmI = id2;
    mpuFound = true;
  } else {
    mpuFound = false;
  }
}

void initMPU() {
  writeByte(mpuAddress, REG_PWR_MGMT_1, 0x00);   // wake
  delay(100);
  writeByte(mpuAddress, REG_PWR_MGMT_1, 0x01);   // clock from gyro X PLL
  delay(100);
  writeByte(mpuAddress, REG_CONFIG, 0x03);       // gyro DLPF 41 Hz
  writeByte(mpuAddress, REG_SMPLRT_DIV, 0x04);   // 200 Hz sample rate
  writeByte(mpuAddress, REG_GYRO_CONFIG, 0x00);  // +/-250 deg/s
  writeByte(mpuAddress, REG_ACCEL_CONFIG, 0x00); // +/-2 g
  writeByte(mpuAddress, REG_ACCEL_CONFIG2, 0x03);// accel DLPF 41 Hz
  writeByte(mpuAddress, REG_INT_PIN_CFG, 0x02);  // I2C bypass: expose AK8963
  writeByte(mpuAddress, REG_INT_ENABLE, 0x01);   // data-ready interrupt
  delay(100);
}

void initMagnetometer() {
  magnetometerPresent = false;

  // The MPU6500 is the same die without the magnetometer.
  if (mpuWhoAmI == 0x70) return;
  if (readByte(AK8963_ADDR, AK8963_WHO_AM_I) != 0x48) return;

  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00);  // power down
  delay(10);
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x0F);  // fuse ROM access
  delay(10);

  uint8_t asa[3];
  readBytes(AK8963_ADDR, AK8963_ASAX, 3, asa);
  magAdjX = ((float)(asa[0] - 128) / 256.0f) + 1.0f;
  magAdjY = ((float)(asa[1] - 128) / 256.0f) + 1.0f;
  magAdjZ = ((float)(asa[2] - 128) / 256.0f) + 1.0f;

  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00);  // power down again
  delay(10);
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x16);  // continuous mode 2, 16-bit, 100 Hz
  delay(10);

  magnetometerPresent = true;
}

void readAccelGyro() {
  uint8_t data[14];
  readBytes(mpuAddress, REG_ACCEL_XOUT_H, 14, data);

  const int16_t rawAx = ((int16_t)data[0] << 8) | data[1];
  const int16_t rawAy = ((int16_t)data[2] << 8) | data[3];
  const int16_t rawAz = ((int16_t)data[4] << 8) | data[5];
  // data[6..7] is temperature, which this controller does not use.
  const int16_t rawGx = ((int16_t)data[8] << 8) | data[9];
  const int16_t rawGy = ((int16_t)data[10] << 8) | data[11];
  const int16_t rawGz = ((int16_t)data[12] << 8) | data[13];

  accelX = (float)rawAx / ACCEL_SCALE;
  accelY = (float)rawAy / ACCEL_SCALE;
  accelZ = (float)rawAz / ACCEL_SCALE;

  gyroX = ((float)rawGx / GYRO_SCALE) - gyroOffsetX;
  gyroY = ((float)rawGy / GYRO_SCALE) - gyroOffsetY;
  gyroZ = ((float)rawGz / GYRO_SCALE) - gyroOffsetZ;
}

// Returns false when no fresh sample is ready or the sensor reported magnetic
// overflow, in which case the caller falls back to 6-axis fusion for this step.
bool readMagnetometer() {
  if (!(readByte(AK8963_ADDR, AK8963_ST1) & 0x01)) return false;

  uint8_t data[7];
  readBytes(AK8963_ADDR, AK8963_XOUT_L, 7, data);
  if (data[6] & 0x08) return false;

  // The AK8963 is little-endian, unlike the MPU registers above.
  const int16_t rawMx = ((int16_t)data[1] << 8) | data[0];
  const int16_t rawMy = ((int16_t)data[3] << 8) | data[2];
  const int16_t rawMz = ((int16_t)data[5] << 8) | data[4];

  magX = ((float)rawMx * MAG_SCALE * magAdjX - magOffsetX) * magScaleX;
  magY = ((float)rawMy * MAG_SCALE * magAdjY - magOffsetY) * magScaleY;
  magZ = ((float)rawMz * MAG_SCALE * magAdjZ - magOffsetZ) * magScaleZ;
  return true;
}

// Average several hundred stationary samples to get the gyro's zero-rate
// offset. The robot must be completely still while this runs.
void calibrateGyro() {
  const int samples = 500;
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

  for (int i = 0; i < samples; ++i) {
    uint8_t data[6];
    readBytes(mpuAddress, REG_GYRO_XOUT_H, 6, data);
    sumX += (float)(int16_t)(((int16_t)data[0] << 8) | data[1]) / GYRO_SCALE;
    sumY += (float)(int16_t)(((int16_t)data[2] << 8) | data[3]) / GYRO_SCALE;
    sumZ += (float)(int16_t)(((int16_t)data[4] << 8) | data[5]) / GYRO_SCALE;
    delay(3);
  }

  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
}

float invSqrt(float x) {
  return 1.0f / sqrt(x);
}

// Wrap an angle into (-180, 180] degrees.
float normalizeDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// Shared tail of both Mahony variants: apply the proportional and integral
// correction to the gyro rates, integrate the quaternion, renormalise.
void applyMahonyFeedbackAndIntegrate(float gx, float gy, float gz,
                                     float ex, float ey, float ez, float dt) {
  if (MAHONY_KI > 0.0f) {
    integralFBx += MAHONY_KI * ex * dt;
    integralFBy += MAHONY_KI * ey * dt;
    integralFBz += MAHONY_KI * ez * dt;
    gx += integralFBx;
    gy += integralFBy;
    gz += integralFBz;
  }

  gx += MAHONY_KP * ex;
  gy += MAHONY_KP * ey;
  gz += MAHONY_KP * ez;

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  const float qa = q0;
  const float qb = q1;
  const float qc = q2;

  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  const float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// Gyro + accelerometer. Gravity corrects roll and pitch; yaw is pure gyro
// integration, which is exactly what wheel odometry wants.
void mahonyUpdate6Axis(float gx, float gy, float gz,
                       float ax, float ay, float az, float dt) {
  float norm = ax * ax + ay * ay + az * az;
  if (norm <= 0.0f) return;

  norm = invSqrt(norm);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Estimated direction of gravity from the current quaternion.
  const float vx = 2.0f * (q1 * q3 - q0 * q2);
  const float vy = 2.0f * (q0 * q1 + q2 * q3);
  const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // Error is the cross product between measured and estimated gravity.
  applyMahonyFeedbackAndIntegrate(gx, gy, gz,
                                  ay * vz - az * vy,
                                  az * vx - ax * vz,
                                  ax * vy - ay * vx, dt);
}

// Gyro + accelerometer + magnetometer. Adds an absolute yaw reference, at the
// cost of being pulled around by any nearby ferrous metal or motor current.
void mahonyUpdate9Axis(float gx, float gy, float gz,
                       float ax, float ay, float az,
                       float mx, float my, float mz, float dt) {
  float norm = ax * ax + ay * ay + az * az;
  if (norm <= 0.0f) return;
  norm = invSqrt(norm);
  ax *= norm;
  ay *= norm;
  az *= norm;

  norm = mx * mx + my * my + mz * mz;
  if (norm <= 0.0f) {
    mahonyUpdate6Axis(gx, gy, gz, ax, ay, az, dt);
    return;
  }
  norm = invSqrt(norm);
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Rotate the measured field into the earth frame, then rebuild it as a
  // purely horizontal-plus-vertical reference (bx, 0, bz) so magnetic
  // declination cannot contaminate roll and pitch.
  const float hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3) +
                   2.0f * my * (q1 * q2 - q0 * q3) +
                   2.0f * mz * (q1 * q3 + q0 * q2);
  const float hy = 2.0f * mx * (q1 * q2 + q0 * q3) +
                   2.0f * my * (0.5f - q1 * q1 - q3 * q3) +
                   2.0f * mz * (q2 * q3 - q0 * q1);
  const float bx = sqrt(hx * hx + hy * hy);
  const float bz = 2.0f * mx * (q1 * q3 - q0 * q2) +
                   2.0f * my * (q2 * q3 + q0 * q1) +
                   2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

  const float vx = 2.0f * (q1 * q3 - q0 * q2);
  const float vy = 2.0f * (q0 * q1 + q2 * q3);
  const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  const float wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) + 2.0f * bz * (q1 * q3 - q0 * q2);
  const float wy = 2.0f * bx * (q1 * q2 - q0 * q3) + 2.0f * bz * (q0 * q1 + q2 * q3);
  const float wz = 2.0f * bx * (q0 * q2 + q1 * q3) + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

  applyMahonyFeedbackAndIntegrate(gx, gy, gz,
                                  (ay * vz - az * vy) + (my * wz - mz * wy),
                                  (az * vx - ax * vz) + (mz * wx - mx * wz),
                                  (ax * vy - ay * vx) + (mx * wy - my * wx), dt);
}

void setQuaternionFromEuler(float rollRad, float pitchRad, float yawRad) {
  const float cy = cos(yawRad * 0.5f), sy = sin(yawRad * 0.5f);
  const float cp = cos(pitchRad * 0.5f), sp = sin(pitchRad * 0.5f);
  const float cr = cos(rollRad * 0.5f), sr = sin(rollRad * 0.5f);

  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;
}

// Tilt-compensated magnetic heading, used only to seed the filter on the first
// fusion step so yaw does not have to slew in from zero.
float tiltCompensatedMagYaw(float rollRad, float pitchRad) {
  const float cosRoll = cos(rollRad), sinRoll = sin(rollRad);
  const float cosPitch = cos(pitchRad), sinPitch = sin(pitchRad);

  const float xComp = magX * cosPitch + magZ * sinPitch;
  const float yComp = magX * sinRoll * sinPitch + magY * cosRoll - magZ * sinRoll * cosPitch;

  return normalizeDegrees(atan2(-yComp, xComp) * RAD_TO_DEG + MAG_DECLINATION_DEG);
}

// Yaw alone, recomputed every control cycle because odometry integrates along
// it. Roll and pitch are only needed by telemetry, so they are computed at the
// slower telemetry rate (two more transcendentals per call is real time on a
// 16 MHz AVR).
void updateFusedYaw() {
  fusedYaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                   1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
  if (useMagnetometer && magnetometerPresent) fusedYaw += MAG_DECLINATION_DEG;
  fusedYaw = normalizeDegrees(fusedYaw);
}

void updateFusedRollPitch() {
  fusedRoll = atan2(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  fusedPitch = asin(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * RAD_TO_DEG;
}

// One IMU acquisition + fusion step. Called at the control rate.
void updateIMU() {
  readAccelGyro();

  magnetometerDataValid = false;
  if (useMagnetometer && magnetometerPresent) {
    magnetometerDataValid = readMagnetometer();
  }

  const uint32_t now = micros();
  float dt = (now - lastFusionMicros) * 1e-6f;
  lastFusionMicros = now;
  if (dt <= 0.0f || dt > 0.5f) dt = CONTROL_PERIOD_MS * 0.001f;

  // Seed the filter from the accelerometer (and magnetometer if trusted) so it
  // starts at the real attitude instead of converging from identity.
  if (!fusionStarted) {
    const float rollRad = atan2(accelY, accelZ);
    const float pitchRad = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));
    const float yawRad = magnetometerDataValid
                       ? tiltCompensatedMagYaw(rollRad, pitchRad) * DEG_TO_RAD
                       : 0.0f;
    setQuaternionFromEuler(rollRad, pitchRad, yawRad);
    fusionStarted = true;
  }

  const float gxRad = gyroX * DEG_TO_RAD;
  const float gyRad = gyroY * DEG_TO_RAD;
  const float gzRad = gyroZ * DEG_TO_RAD;

  if (magnetometerDataValid) {
    mahonyUpdate9Axis(gxRad, gyRad, gzRad, accelX, accelY, accelZ, magX, magY, magZ, dt);
  } else {
    mahonyUpdate6Axis(gxRad, gyRad, gzRad, accelX, accelY, accelZ, dt);
  }

  updateFusedYaw();
  yawDeg = normalizeDegrees(IMU_YAW_SIGN * fusedYaw - yawZeroDeg);
  yawRateDegS = IMU_YAW_SIGN * gyroZ;
}

// Make the current heading the new zero by capturing the filter's absolute
// yaw as the reference offset.
void zeroYaw() {
  yawZeroDeg = IMU_YAW_SIGN * fusedYaw;
  yawDeg = 0.0f;
}

// -----------------------------------------------------------------------------
// Motor helpers
// -----------------------------------------------------------------------------

// Raise Timer1 (pins 9 and 10, i.e. both TB6612 PWM inputs) from the Arduino
// default 490 Hz to ~31.4 kHz by dropping its prescaler to 1. 490 Hz sits in
// the middle of the audible band and makes the motors sing at low duty; above
// ~20 kHz the whine disappears and the current ripple through the gearmotor
// windings is smaller. Timer0 is untouched, so millis()/micros()/delay() keep
// working normally.
void setPwmFrequency31kHz() {
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
}

// Discard any pending encoder ticks and clear the speed estimates, so stale
// counts don't produce a velocity spike on the next control cycle.
void resetEncoderDeltas() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    readEncoderAndResetAtomic(i);
    wheelMeasuredMMs[i] = 0.0f;
    wheelFilteredMMs[i] = 0.0f;
  }
}

// Step 'current' toward 'target' by at most 'maximumStep' -- the slew-rate
// limiter used for the acceleration ramps.
float moveToward(float current, float target, float maximumStep) {
  if (current < target) return min(current + maximumStep, target);
  if (current > target) return max(current - maximumStep, target);
  return current;
}

// Invert the per-wheel, per-direction PWM-to-speed linear fit to get the
// open-loop PWM expected to produce the target speed; PID then only has to
// correct the residual. Returns an unsigned PWM magnitude.
//
// There is deliberately no shared minimum-PWM floor: each wheel's fit already
// encodes its own dead zone (below -intercept/slope the fit says zero speed).
// Flooring both wheels at a common value forces the wheel with the smaller
// dead zone far past its target at low speeds, which shows up as one wheel
// visibly outrunning the other during slow turns.
float speedFeedForwardPWM(float targetMMs, uint8_t index) {
  const float magnitude = fabs(targetMMs);
  if (magnitude < 1.0f) return 0.0f;

  const bool reverse = targetMMs < 0.0f;
  const float slope = reverse ? FF_SLOPE_MM_S_PER_PWM_REV[index]
                              : FF_SLOPE_MM_S_PER_PWM_FWD[index];
  const float intercept = reverse ? FF_INTERCEPT_MM_S_REV[index]
                                  : FF_INTERCEPT_MM_S_FWD[index];
  return constrain((magnitude - intercept) / slope, 0.0f, (float)MAX_DRIVE_PWM);
}

// Stop with motors freewheeling (coast) and clear all controller state.
void releaseAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motor[i].coast();
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

// Stop with active braking (motor terminals shorted) -- used for e-stop.
void brakeAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motor[i].brake();
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

// Drive one wheel with a signed PWM value: sign selects direction (taken from
// the wheel target), magnitude sets duty. Near-zero target or output coasts the
// wheel instead of stalling it at a tiny PWM.
void applyMotorOutput(uint8_t index, float signedPWM) {
  if (fabs(wheelTargetMMs[index]) < 1.0f || fabs(signedPWM) < 1.0f) {
    motor[index].coast();
    wheelPWM[index] = 0.0f;
    return;
  }

  const uint8_t duty = (uint8_t)constrain((int)(fabs(signedPWM) + 0.5f), 0, 255);
  const bool wantForward = wheelTargetMMs[index] >= 0.0f;
  // MOTOR_DIRECTION_SIGN flips a miswired wheel in software.
  motor[index].drive(MOTOR_DIRECTION_SIGN[index] > 0 ? wantForward : !wantForward, duty);
  wheelPWM[index] = wantForward ? duty : -((float)duty);
}

// Accept a new body-velocity setpoint (clamped to limits) and feed the
// command watchdog.
void setVelocityCommand(float linearMMs, float angularDegS) {
  commandLinearMMs = constrain(linearMMs, -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S);
  commandAngularDegS = constrain(angularDegS, -MAX_ANGULAR_DEG_S, MAX_ANGULAR_DEG_S);
  lastCommandMs = millis();
  watchdogStopped = false;
}

// Zero the setpoints and ramps, then stop the motors (brake or coast).
void stopCommand(bool brake) {
  commandLinearMMs = 0.0f;
  commandAngularDegS = 0.0f;
  rampedLinearMMs = 0.0f;
  rampedAngularDegS = 0.0f;
  if (brake) brakeAllMotors();
  else releaseAllMotors();
}

// Zero the integrated pose, tick totals, and yaw reference.
void resetOdometry() {
  poseXMM = 0.0f;
  poseYMM = 0.0f;
  linearVelocityMMs = 0.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) cumulativeTicks[i] = 0;
  resetEncoderDeltas();
  zeroYaw();
}

// -----------------------------------------------------------------------------
// Open-loop PWM sweep (feed-forward calibration)
// -----------------------------------------------------------------------------

// Step one wheel through the PWM range and print the steady-state speed at
// each step, so the FF_SLOPE/FF_INTERCEPT constants above can be fitted from
// real data instead of guessed.
//
// This blocks: no commands are read and no telemetry is emitted while it runs
// (a full sweep takes step_count * dwell_ms). Run it from a plain serial
// terminal with the wheels off the floor, not with the ROS 2 stack connected.
void runSweep(uint8_t index, uint8_t step, uint16_t dwellMs, bool reverse) {
  stopCommand(false);
  emergencyStopLatched = false;

  Serial.print(F("INFO,SWEEP_START,MOTOR_"));
  Serial.print(index + 1);
  Serial.println(reverse ? F(",REVERSE") : F(",FORWARD"));

  const bool forwardPins = (MOTOR_DIRECTION_SIGN[index] > 0) ? !reverse : reverse;
  const float directionSign = reverse ? -1.0f : 1.0f;

  for (uint16_t pwm = SWEEP_MIN_PWM; pwm <= MAX_DRIVE_PWM; pwm += step) {
    motor[index].drive(forwardPins, (uint8_t)pwm);

    // Let the wheel reach steady state, then time a clean measurement window.
    delay(dwellMs / 2);
    readEncoderAndResetAtomic(index);
    const uint32_t windowStart = millis();
    delay(dwellMs / 2);
    const int32_t ticks = readEncoderAndResetAtomic(index) * ENCODER_SIGN[index];
    const float elapsedS = (millis() - windowStart) * 0.001f;

    const float speedMMs = (elapsedS > 0.0f)
                         ? (ticks * MM_PER_COUNT[index]) / elapsedS
                         : 0.0f;

    Serial.print(F("C,"));
    Serial.print(index + 1);
    Serial.print(reverse ? F(",R,") : F(",F,"));
    Serial.print(pwm);
    Serial.print(',');
    // Report the magnitude along the commanded direction, which is what the
    // linear fit is expressed in.
    Serial.println(speedMMs * directionSign, 2);
  }

  motor[index].coast();
  resetEncoderDeltas();
  Serial.println(F("ACK,SWEEP_DONE"));
}

// -----------------------------------------------------------------------------
// Command parser
// -----------------------------------------------------------------------------

void printImuInfo() {
  Serial.print(F("INFO,IMU,"));
  if (!mpuFound) {
    Serial.println(F("NONE"));
    return;
  }
  switch (mpuWhoAmI) {
    case 0x70: Serial.print(F("MPU6500")); break;
    case 0x71: Serial.print(F("MPU9250")); break;
    case 0x73: Serial.print(F("MPU9255")); break;
    default:   Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F(",ADDR_0x"));
  Serial.print(mpuAddress, HEX);
  Serial.print(F(",MAG_"));
  Serial.print(magnetometerPresent ? F("PRESENT") : F("ABSENT"));
  Serial.print(F(",MAG_FUSION_"));
  Serial.println(useMagnetometer ? F("ON") : F("OFF"));
}

void printReady() {
  Serial.println(F("READY,mini_ROS2_low_level_v1"));
  printImuInfo();
}

// Parse and execute one complete command line (see header comment for the
// command set). Tokenizes in place with strtok_r and replies with ACK/ERR.
void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  // Make command comparison case insensitive.
  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "VEL") == 0) {
    char *linearText = strtok_r(NULL, ",", &savePtr);
    char *angularText = strtok_r(NULL, ",", &savePtr);
    if (linearText && angularText && !emergencyStopLatched) {
      setVelocityCommand(atof(linearText), atof(angularText));
      Serial.println(F("ACK,VEL"));
    } else if (emergencyStopLatched) {
      Serial.println(F("ERR,ESTOP_LATCHED"));
    } else {
      Serial.println(F("ERR,VEL_FORMAT"));
    }
  }
  else if (strcmp(command, "FORWARD") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(value ? fabs(atof(value)) : 150.0f, 0.0f);
  }
  else if (strcmp(command, "REVERSE") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(-(value ? fabs(atof(value)) : 150.0f), 0.0f);
  }
  else if (strcmp(command, "LEFT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, value ? fabs(atof(value)) : 30.0f);
  }
  else if (strcmp(command, "RIGHT") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    if (!emergencyStopLatched) setVelocityCommand(0.0f, -(value ? fabs(atof(value)) : 30.0f));
  }
  else if (strcmp(command, "STOP") == 0) {
    stopCommand(false);
    lastCommandMs = millis();
    watchdogStopped = false;
    Serial.println(F("ACK,STOP"));
  }
  else if (strcmp(command, "ESTOP") == 0) {
    emergencyStopLatched = true;
    stopCommand(true);
    Serial.println(F("ACK,ESTOP_LATCHED"));
  }
  else if (strcmp(command, "CLEAR_ESTOP") == 0) {
    emergencyStopLatched = false;
    stopCommand(false);
    lastCommandMs = millis();
    watchdogStopped = false;
    Serial.println(F("ACK,ESTOP_CLEARED"));
  }
  else if (strcmp(command, "RESET_ODOM") == 0) {
    stopCommand(false);
    resetOdometry();
    Serial.println(F("ACK,ODOM_RESET"));
  }
  else if (strcmp(command, "ZERO_YAW") == 0) {
    zeroYaw();
    Serial.println(F("ACK,YAW_ZEROED"));
  }
  else if (strcmp(command, "CAL_IMU") == 0) {
    stopCommand(true);
    Serial.println(F("INFO,KEEP_ROBOT_STILL_CALIBRATING_IMU"));
    calibrateGyro();
    // The filter integrated the old bias while it was drifting, so restart it
    // from the accelerometer rather than keeping a skewed quaternion.
    fusionStarted = false;
    updateIMU();
    zeroYaw();
    Serial.println(F("ACK,IMU_CALIBRATED"));
  }
  else if (strcmp(command, "MAG") == 0) {
    char *value = strtok_r(NULL, ",", &savePtr);
    const bool wanted = value && atoi(value) != 0;
    if (wanted && !magnetometerPresent) {
      Serial.println(F("ERR,NO_MAGNETOMETER"));
    } else {
      useMagnetometer = wanted;
      // Yaw meaning changes (absolute vs relative), so rebuild the filter.
      fusionStarted = false;
      updateIMU();
      zeroYaw();
      printImuInfo();
      Serial.println(F("ACK,MAG"));
    }
  }
  else if (strcmp(command, "IMU") == 0) {
    printImuInfo();
  }
  else if (strcmp(command, "PID") == 0) {
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    char *lamText = strtok_r(NULL, ",", &savePtr);
    char *muText = strtok_r(NULL, ",", &savePtr);
    if (pText && iText && dText) {
      const float kp = atof(pText);
      const float ki = atof(iText);
      const float kd = atof(dText);
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) wheelPID[i].configure(kp, ki, kd, lambda, mu);
      Serial.println(F("ACK,PID_ALL"));
    } else {
      Serial.println(F("ERR,PID_FORMAT"));
    }
  }
  else if (strcmp(command, "PIDM") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *pText = strtok_r(NULL, ",", &savePtr);
    char *iText = strtok_r(NULL, ",", &savePtr);
    char *dText = strtok_r(NULL, ",", &savePtr);
    char *lamText = strtok_r(NULL, ",", &savePtr);
    char *muText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    if (index >= 0 && index < MOTOR_COUNT && pText && iText && dText) {
      const float lambda = lamText ? atof(lamText) : 1.0f;
      const float mu = muText ? atof(muText) : 1.0f;
      wheelPID[index].configure(atof(pText), atof(iText), atof(dText), lambda, mu);
      Serial.println(F("ACK,PID_MOTOR"));
    } else {
      Serial.println(F("ERR,PIDM_FORMAT"));
    }
  }
  else if (strcmp(command, "SWEEP") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *stepText = strtok_r(NULL, ",", &savePtr);
    char *dwellText = strtok_r(NULL, ",", &savePtr);
    char *dirText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    const int step = stepText ? atoi(stepText) : 10;
    const int dwell = dwellText ? atoi(dwellText) : 1000;
    if (index >= 0 && index < MOTOR_COUNT && step > 0 && dwell >= 100) {
      runSweep((uint8_t)index, (uint8_t)step, (uint16_t)dwell,
               dirText && (dirText[0] == 'R' || dirText[0] == 'r'));
    } else {
      Serial.println(F("ERR,SWEEP_FORMAT"));
    }
  }
  else if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
  }
  else {
    Serial.print(F("ERR,UNKNOWN_COMMAND,"));
    Serial.println(command);
  }
}

// Accumulate serial bytes into commandBuffer and dispatch a command whenever
// a terminator (\n, \r or ':') arrives. Oversized lines are discarded.
void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r' || c == ':') {
      if (commandLength > 0) {
        commandBuffer[commandLength] = '\0';
        processCommand(commandBuffer);
        commandLength = 0;
      }
    } else if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    } else {
      commandLength = 0;
      Serial.println(F("ERR,COMMAND_TOO_LONG"));
    }
  }
}

// -----------------------------------------------------------------------------
// Control and odometry
// -----------------------------------------------------------------------------

// 100 Hz control step: watchdog/e-stop handling, acceleration ramps,
// differential-drive kinematics, per-wheel feed-forward + PID, and odometry
// integration (encoder distance + IMU heading).
void updateControl(float dt) {
  // Communication loss is always handled locally on the Arduino.
  if (!emergencyStopLatched && millis() - lastCommandMs > COMMAND_WATCHDOG_MS) {
    watchdogStopped = true;
    stopCommand(false);
  }

  if (emergencyStopLatched) {
    brakeAllMotors();
    return;
  }

  // Slew-limit the commanded velocities so wheel targets change smoothly.
  rampedLinearMMs = moveToward(rampedLinearMMs, commandLinearMMs, LINEAR_ACCEL_MM_S2 * dt);
  rampedAngularDegS = moveToward(rampedAngularDegS, commandAngularDegS, ANGULAR_ACCEL_DEG_S2 * dt);

  // Differential-drive kinematics: body (v, w) -> left/right wheel rim speeds.
  const float angularRadS = rampedAngularDegS * DEG_TO_RAD;
  const float sideTarget[MOTOR_COUNT] = {
    rampedLinearMMs - angularRadS * TRACK_WIDTH_MM * 0.5f,   // LEFT
    rampedLinearMMs + angularRadS * TRACK_WIDTH_MM * 0.5f,   // RIGHT
  };

  float wheelDistanceMM[MOTOR_COUNT];

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    // Measure this cycle's travel from the encoder tick delta.
    const int32_t signedTicks = readEncoderAndResetAtomic(i) * ENCODER_SIGN[i];
    cumulativeTicks[i] += signedTicks;

    wheelDistanceMM[i] = signedTicks * MM_PER_COUNT[i];

    // Low-pass encoder speed while preserving signed direction.
    const float rawSpeedMMs = wheelDistanceMM[i] / dt;
    wheelFilteredMMs[i] += 0.35f * (rawSpeedMMs - wheelFilteredMMs[i]);
    wheelMeasuredMMs[i] = wheelFilteredMMs[i];

    wheelTargetMMs[i] = constrain(sideTarget[i], -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S);

    // Output = open-loop feed-forward + PID correction. The correction is
    // flipped for reverse targets because the PID works in signed speed while
    // the output here is a magnitude.
    const float feedForward = speedFeedForwardPWM(wheelTargetMMs[i], i);
    const float correction = wheelPID[i].update(wheelTargetMMs[i], wheelMeasuredMMs[i], dt);
    float outputMagnitude = feedForward + ((wheelTargetMMs[i] >= 0.0f) ? correction : -correction);
    outputMagnitude = constrain(outputMagnitude, 0.0f, 255.0f);
    applyMotorOutput(i, (wheelTargetMMs[i] >= 0.0f) ? outputMagnitude : -outputMagnitude);
  }

  // Odometry: body-centre distance is the mean of the two wheel distances,
  // integrated along the IMU heading (encoder distance + gyro yaw fusion).
  const float centreDistanceMM = 0.5f * (wheelDistanceMM[LEFT] + wheelDistanceMM[RIGHT]);
  const float yawRad = yawDeg * DEG_TO_RAD;
  poseXMM += centreDistanceMM * cos(yawRad);
  poseYMM += centreDistanceMM * sin(yawRad);
  linearVelocityMMs = centreDistanceMM / dt;
}

// Emit one 'T,...' CSV telemetry line (field layout in the header comment);
// consumed by the ROS 2 serial bridge.
void publishTelemetry() {
  updateFusedRollPitch();

  Serial.print(F("T,"));
  Serial.print(millis());
  Serial.print(','); Serial.print(poseXMM, 2);
  Serial.print(','); Serial.print(poseYMM, 2);
  Serial.print(','); Serial.print(yawDeg, 3);
  Serial.print(','); Serial.print(linearVelocityMMs, 2);
  Serial.print(','); Serial.print(yawRateDegS, 3);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(cumulativeTicks[i]);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelMeasuredMMs[i], 2);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelPWM[i], 1);
  }

  Serial.print(','); Serial.print(fusedRoll, 2);
  Serial.print(','); Serial.print(fusedPitch, 2);
  Serial.print(','); Serial.print(gyroX, 3);
  Serial.print(','); Serial.print(gyroY, 3);
  Serial.print(','); Serial.print(accelX, 4);
  Serial.print(','); Serial.print(accelY, 4);
  Serial.print(','); Serial.print(accelZ, 4);

  Serial.print(','); Serial.print(emergencyStopLatched ? 1 : 0);
  Serial.print(','); Serial.println(watchdogStopped ? 1 : 0);
}

// -----------------------------------------------------------------------------
// Arduino setup/loop
// -----------------------------------------------------------------------------

// One-time init: serial link, I2C at 400 kHz, TB6612 outputs at ~31 kHz PWM,
// encoder interrupts, per-wheel PID gains, and IMU startup calibration (the
// robot must be still).
void setup() {
  Serial.begin(SERIAL_BAUD);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) motor[i].begin();
  setPwmFrequency31kHz();

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);   // take both bridges out of standby

  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), encoderRightISR, CHANGE);

  Wire.begin();
  Wire.setClock(400000UL);

  // Starting gains, identical for both wheels: each wheel gets the same speed
  // target for straight motion, and its own PID output corrects the mismatch.
  // Retune after the SWEEP calibration has replaced the feed-forward fit.
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelPID[i].configure(0.30f, 0.020f, 0.0030f, 1.0f, 1.0f);
  }

  delay(200);
  detectMPU();
  if (mpuFound) {
    initMPU();
    initMagnetometer();
    Serial.println(F("INFO,KEEP_ROBOT_STILL_IMU_STARTUP_CALIBRATION"));
    calibrateGyro();
    lastFusionMicros = micros();
    updateIMU();
  } else {
    // Without a heading source the robot must not drive: odometry would
    // integrate every movement along a heading stuck at zero, so the map and
    // Nav2 would both be wrong in a way that is hard to see from the outside.
    Serial.println(F("ERR,NO_IMU_DETECTED"));
    emergencyStopLatched = true;
  }

  releaseAllMotors();
  resetEncoderDeltas();
  zeroYaw();

  lastCommandMs = millis();
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
