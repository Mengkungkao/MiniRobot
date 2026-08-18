#include "MotorControl.h"

#include <math.h>

#include <MiniRobotEncoders.h>

#include "Imu.h"

WheelPID wheelPID[MOTOR_COUNT];

// -----------------------------------------------------------------------------
// State
// -----------------------------------------------------------------------------

static float commandLinearMMs = 0.0f;
static float commandAngularDegS = 0.0f;
static float rampedLinearMMs = 0.0f;
static float rampedAngularDegS = 0.0f;

static float wheelTargetMMs[MOTOR_COUNT] = {0};
float wheelMeasuredMMs[MOTOR_COUNT] = {0};
static float wheelFilteredMMs[MOTOR_COUNT] = {0};
float wheelPWM[MOTOR_COUNT] = {0};
int32_t cumulativeTicks[MOTOR_COUNT] = {0};

float poseXMM = 0.0f;
float poseYMM = 0.0f;
float linearVelocityMMs = 0.0f;

bool emergencyStopLatched = false;
bool watchdogStopped = true;

static uint32_t lastCommandMs = 0;

// -----------------------------------------------------------------------------
// Motor helpers
// -----------------------------------------------------------------------------

// Step 'current' toward 'target' by at most 'maximumStep' -- the slew-rate
// limiter used for the acceleration ramps.
static float moveToward(float current, float target, float maximumStep) {
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
static float speedFeedForwardPWM(float targetMMs, uint8_t index) {
  const float magnitude = fabs(targetMMs);
  if (magnitude < 1.0f) return 0.0f;

  const bool reverse = targetMMs < 0.0f;
  const float slope = reverse ? FF_SLOPE_MM_S_PER_PWM_REV[index]
                              : FF_SLOPE_MM_S_PER_PWM_FWD[index];
  const float intercept = reverse ? FF_INTERCEPT_MM_S_REV[index]
                                  : FF_INTERCEPT_MM_S_FWD[index];
  return constrain((magnitude - intercept) / slope, 0.0f, (float)MAX_DRIVE_PWM);
}

// Drive one wheel with a signed PWM value: sign selects direction (taken from
// the wheel target), magnitude sets duty. Near-zero target or output coasts the
// wheel instead of stalling it at a tiny PWM.
static void applyMotorOutput(uint8_t index, float signedPWM) {
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

// Stop with active braking (motor terminals shorted) -- used for e-stop.
static void brakeAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motor[i].brake();
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

void resetEncoderDeltas() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    readEncoderAndResetAtomic(i);
    wheelMeasuredMMs[i] = 0.0f;
    wheelFilteredMMs[i] = 0.0f;
  }
}

void releaseAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motor[i].coast();
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
  }
}

void setVelocityCommand(float linearMMs, float angularDegS) {
  commandLinearMMs = constrain(linearMMs, -MAX_LINEAR_MM_S, MAX_LINEAR_MM_S);
  commandAngularDegS = constrain(angularDegS, -MAX_ANGULAR_DEG_S, MAX_ANGULAR_DEG_S);
  lastCommandMs = millis();
  watchdogStopped = false;
}

void refreshCommandWatchdog() {
  lastCommandMs = millis();
  watchdogStopped = false;
}

void armCommandWatchdog() {
  lastCommandMs = millis();
}

void stopCommand(bool brake) {
  commandLinearMMs = 0.0f;
  commandAngularDegS = 0.0f;
  rampedLinearMMs = 0.0f;
  rampedAngularDegS = 0.0f;
  if (brake) brakeAllMotors();
  else releaseAllMotors();
}

void resetOdometry() {
  poseXMM = 0.0f;
  poseYMM = 0.0f;
  linearVelocityMMs = 0.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) cumulativeTicks[i] = 0;
  resetEncoderDeltas();
  zeroYaw();
}

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

void motorControlBegin() {
  motorsBegin();   // outputs, ~31 kHz PWM, STBY high
  encodersBegin();

  // Starting gains, identical for both wheels: each wheel gets the same speed
  // target for straight motion, and its own PID output corrects the mismatch.
  // Retune after the SWEEP calibration has replaced the feed-forward fit.
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelPID[i].configure(0.30f, 0.020f, 0.0030f, 1.0f, 1.0f);
  }
}
