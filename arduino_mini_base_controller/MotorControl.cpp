#include "MotorControl.h"

#include <math.h>

#include "MiniRobotEncoders.h"

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

// Open measurement window per wheel: ticks and seconds accumulated since the
// speed estimate was last recomputed. See SPEED_WINDOW_MAX_MS in Config.h --
// one control period is too short to measure a slow wheel.
static int32_t windowTicks[MOTOR_COUNT] = {0};
static float windowSeconds[MOTOR_COUNT] = {0};

// Stall handling. boostPWM is the extra duty currently being applied to break
// stiction; stalledForMs is how long the wheel has been commanded-but-still;
// the cooldown pair is the forced rest after the boost budget runs out.
//
// The cooldown is timed as "now minus when it started", not as a deadline
// compared against millis(). Unsigned subtraction survives the 49.7-day
// millis() rollover; a stored deadline does not, and this is the same idiom
// the command watchdog uses.
static float boostPWM[MOTOR_COUNT] = {0};
static uint16_t stalledForMs[MOTOR_COUNT] = {0};
static uint32_t cooldownStartMs[MOTOR_COUNT] = {0};
static bool coolingDown[MOTOR_COUNT] = {false, false};
bool wheelStalled[MOTOR_COUNT] = {false, false};

float poseXMM = 0.0f;
float poseYMM = 0.0f;
float linearVelocityMMs = 0.0f;

float headingDeg = 0.0f;
float headingRateDegS = 0.0f;
// Heading integrated from the wheels alone, used when no IMU was detected.
static float encoderYawDeg = 0.0f;

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
// This is the fit and nothing but the fit -- no minimum. The breakaway floor
// lives in the control loop instead, where it can be applied only to a wheel
// that is actually standing still: a wheel already rolling holds speeds below
// its own breakaway perfectly well on momentum, and flooring it there would
// make every slow turn a series of lurches.
static float speedFeedForwardPWM(float targetMMs, uint8_t index) {
  const float magnitude = fabs(targetMMs);
  if (magnitude < MIN_WHEEL_SPEED_MM_S) return 0.0f;

  const bool reverse = targetMMs < 0.0f;
  const float slope = reverse ? FF_SLOPE_MM_S_PER_PWM_REV[index]
                              : FF_SLOPE_MM_S_PER_PWM_FWD[index];
  const float intercept = reverse ? FF_INTERCEPT_MM_S_REV[index]
                                  : FF_INTERCEPT_MM_S_FWD[index];

  // A non-positive slope is not a motor curve. The calibration sketch emits
  // "0.000f /*NOT_CALIBRATED*/" for a wheel whose fit failed, and dividing by
  // that yields +inf, which constrain() clamps to MAX_DRIVE_PWM -- the wheel
  // would slam to full duty at any nonzero target. Fall back to pure PID
  // instead, which is slow and visibly wrong rather than dangerous.
  if (slope <= 0.0f) return 0.0f;

  return constrain((magnitude - intercept) / slope, 0.0f, (float)MAX_DRIVE_PWM);
}

// Drive one wheel with a signed PWM value: sign selects direction (taken from
// the wheel target), magnitude sets duty. Near-zero target or output coasts the
// wheel instead of stalling it at a tiny PWM.
static void applyMotorOutput(uint8_t index, float signedPWM) {
  // The second test is in PWM counts, not mm/s: below one count of duty there
  // is nothing to apply, so coast rather than write a zero-ish PWM.
  if (fabs(wheelTargetMMs[index]) < MIN_WHEEL_SPEED_MM_S || fabs(signedPWM) < 1.0f) {
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
    boostPWM[i] = 0.0f;
    stalledForMs[i] = 0;
    coolingDown[i] = false;
  }
}

void resetEncoderDeltas() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    readEncoderAndResetAtomic(i);
    wheelMeasuredMMs[i] = 0.0f;
    wheelFilteredMMs[i] = 0.0f;
    windowTicks[i] = 0;
    windowSeconds[i] = 0.0f;
  }
}

void releaseAllMotors() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motor[i].coast();
    wheelPWM[i] = 0.0f;
    wheelTargetMMs[i] = 0.0f;
    wheelPID[i].reset();
    boostPWM[i] = 0.0f;
    stalledForMs[i] = 0;
    coolingDown[i] = false;
    wheelStalled[i] = false;
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
  encoderYawDeg = 0.0f;
  headingDeg = 0.0f;
  zeroYaw();
}

// Recompute one wheel's speed estimate. The window stays open until a count
// arrives (or SPEED_WINDOW_MAX_MS elapses) because a single 10 ms control
// period is shorter than the gap between encoder counts at low speed -- see
// SPEED_WINDOW_MAX_MS in Config.h. Returns this cycle's travel in mm, which is
// what odometry integrates and is exact regardless of when the window closes.
static float updateWheelSpeed(uint8_t index, float dt) {
  const int32_t signedTicks = readEncoderAndResetAtomic(index) * ENCODER_SIGN[index];
  cumulativeTicks[index] += signedTicks;

  windowTicks[index] += signedTicks;
  windowSeconds[index] += dt;

  const bool sawCount = (windowTicks[index] != 0);
  const bool windowExpired = (windowSeconds[index] >= SPEED_WINDOW_MAX_MS * 0.001f);
  if (sawCount || windowExpired) {
    const float rawSpeedMMs = (windowTicks[index] * MM_PER_COUNT[index]) / windowSeconds[index];
    wheelFilteredMMs[index] += WHEEL_SPEED_FILTER_ALPHA * (rawSpeedMMs - wheelFilteredMMs[index]);
    wheelMeasuredMMs[index] = wheelFilteredMMs[index];
    windowTicks[index] = 0;
    windowSeconds[index] = 0.0f;
  }

  return signedTicks * MM_PER_COUNT[index];
}

// Track one wheel's stall state, set 'resting' when it is inside a forced
// cooldown, and return the extra PWM to add on top of feed-forward and PID.
//
// A wheel is stalled when it is asked to move and is not moving. The boost
// ramps in fast because stiction either breaks or it does not, and it is
// budgeted in time because a locked-rotor brushed motor on a bridge with no
// current limit is a heater: STALL_BOOST_MAX_MS of pushing buys
// STALL_COOLDOWN_MS of coasting, and the cycle repeats for as long as the host
// keeps asking. See the STALL_* block in Config.h.
static float updateStallBoost(uint8_t index, float dt, bool &resting) {
  resting = false;
  const bool commanded = fabs(wheelTargetMMs[index]) >= MIN_WHEEL_SPEED_MM_S;
  const bool moving = fabs(wheelMeasuredMMs[index]) >= STALL_SPEED_MM_S;

  if (!commanded || moving) {
    // Moving again (or asked to stop): drop the boost immediately rather than
    // ramping it out, so breaking free does not also mean lurching. A wheel
    // that got going is not resting either, even mid-cooldown.
    boostPWM[index] = 0.0f;
    stalledForMs[index] = 0;
    coolingDown[index] = false;
    wheelStalled[index] = false;
    return 0.0f;
  }

  if (coolingDown[index]) {
    if (millis() - cooldownStartMs[index] < STALL_COOLDOWN_MS) {
      // Resting. Hold the wheel off entirely and keep the integrator empty, so
      // the cooldown is real and the loop does not simply resume where it left
      // off the instant the boost is allowed back.
      boostPWM[index] = 0.0f;
      wheelPID[index].reset();
      wheelStalled[index] = true;
      resting = true;
      return 0.0f;
    }
    coolingDown[index] = false;
  }

  const uint16_t elapsedMs = (uint16_t)(dt * 1000.0f + 0.5f);
  if (stalledForMs[index] < 60000) stalledForMs[index] += elapsedMs;

  if (stalledForMs[index] < STALL_DETECT_MS) return 0.0f;

  wheelStalled[index] = true;

  if (stalledForMs[index] >= STALL_DETECT_MS + STALL_BOOST_MAX_MS) {
    cooldownStartMs[index] = millis();
    coolingDown[index] = true;
    stalledForMs[index] = 0;
    boostPWM[index] = 0.0f;
    wheelPID[index].reset();
    Serial.print(F("ERR,STALL,MOTOR_"));
    Serial.println(index + 1);
    resting = true;
    return 0.0f;
  }

  boostPWM[index] = min(boostPWM[index] + STALL_BOOST_RATE_PWM_S * dt, STALL_BOOST_MAX_PWM);
  return boostPWM[index];
}

void updateControl(float dt) {
  // Communication loss is always handled locally on the Arduino.
  if (!emergencyStopLatched && millis() - lastCommandMs > COMMAND_WATCHDOG_MS) {
    watchdogStopped = true;
    stopCommand(false);
  }

  if (emergencyStopLatched) {
    brakeAllMotors();
    // Keep draining the encoders while latched. The return below skips the
    // normal per-wheel read, so without this the ISR counters accumulate every
    // tick the robot rolls to a halt or is pushed by hand, and the whole
    // backlog arrives as one 10 ms sample the instant the e-stop clears: a
    // pose jump the size of however far it moved, plus a huge false velocity
    // into the PID. Zero speed is also the honest telemetry while braked.
    resetEncoderDeltas();
    return;
  }

  // Slew-limit the commanded velocities so wheel targets change smoothly.
  rampedLinearMMs = moveToward(rampedLinearMMs, commandLinearMMs, LINEAR_ACCEL_MM_S2 * dt);
  rampedAngularDegS = moveToward(rampedAngularDegS, commandAngularDegS, ANGULAR_ACCEL_DEG_S2 * dt);

  // Differential-drive kinematics: body (v, w) -> wheel rim speeds. WHEEL_Y_SIGN
  // (MiniRobotGeometry.h) is where each wheel sits along +Y, so this one line
  // is the only place the body layout enters the control loop.
  const float angularRadS = rampedAngularDegS * DEG_TO_RAD;
  const float halfTrackMM = TRACK_WIDTH_MM * 0.5f;

  float wheelDistanceMM[MOTOR_COUNT];

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    // Measure this cycle's travel and refresh the speed estimate.
    wheelDistanceMM[i] = updateWheelSpeed(i, dt);

    const float sideTarget = rampedLinearMMs - WHEEL_Y_SIGN[i] * angularRadS * halfTrackMM;
    wheelTargetMMs[i] = constrain(sideTarget, -MAX_WHEEL_MM_S, MAX_WHEEL_MM_S);

    bool resting = false;
    const float boost = updateStallBoost(i, dt, resting);
    if (resting) {               // stall cooldown: this wheel rests
      motor[i].coast();
      wheelPWM[i] = 0.0f;
      continue;
    }

    // Output = open-loop feed-forward + PID correction + stall boost. The
    // correction is flipped for reverse targets because the PID works in
    // signed speed while the output here is a magnitude.
    const float feedForward = speedFeedForwardPWM(wheelTargetMMs[i], i);
    const float correction = wheelPID[i].update(wheelTargetMMs[i], wheelMeasuredMMs[i], dt);
    float outputMagnitude = feedForward + ((wheelTargetMMs[i] >= 0.0f) ? correction : -correction) + boost;

    // Breakaway floor, applied only to a wheel that is commanded and is not
    // turning. Below FF_BREAKAWAY_PWM the bridge is energised and the wheel is
    // not moving, which is the one output that can never be right: it is the
    // dead zone the fit does not describe. A wheel that is already rolling is
    // left alone -- it holds slower speeds than this on momentum, and flooring
    // it would turn every slow turn into a series of lurches.
    if (fabs(wheelTargetMMs[i]) >= MIN_WHEEL_SPEED_MM_S &&
        fabs(wheelMeasuredMMs[i]) < STALL_SPEED_MM_S) {
      outputMagnitude = max(outputMagnitude, FF_BREAKAWAY_PWM[i]);
    }
    // MAX_DRIVE_PWM, not 255. The feed-forward term is already clamped there,
    // but the PID correction and the boost are added on top, so the sum could
    // reach full duty -- past the top of the range the calibration sweep ever
    // measured, where no fit describes the motor and nothing has ever been
    // checked. Capping the sum keeps the whole loop inside the range it was
    // characterised over, and bounds how fast a wheel can run away if its
    // feedback is wrong (a flipped ENCODER_SIGN makes the error grow instead
    // of shrink, which drives the correction straight into this ceiling).
    outputMagnitude = constrain(outputMagnitude, 0.0f, (float)MAX_DRIVE_PWM);
    applyMotorOutput(i, (wheelTargetMMs[i] >= 0.0f) ? outputMagnitude : -outputMagnitude);
  }

  // Odometry. Body-centre distance is the mean of the two wheel distances; the
  // heading it is integrated along comes from the IMU when there is one.
  //
  // Without an IMU the wheels supply the heading instead: the two wheels differ
  // by exactly the arc the body swept, so d(yaw) = (d[-Y] - d[+Y]) / track. It
  // drifts with every slip and cannot see a shove, but it is a heading, and it
  // is what lets the robot run with the IMU unplugged (REQUIRE_IMU, Config.h).
  // Distances are integrated from the raw per-tick counts, which are exact:
  // filtering them would bias the pose, and every count has to be accounted
  // for exactly once or the position walks away.
  float centreDistanceMM = 0.0f;
  float yawDifferenceMM = 0.0f;
  // Velocities are reported from the filtered per-wheel estimates instead. A
  // velocity divided out of one 10 ms tick is the same 0-or-28 mm/s
  // quantisation the control loop stopped using, and it is what /odom's twist
  // and Nav2's feedback are built from.
  float centreSpeedMMs = 0.0f;
  float yawSpeedDifferenceMMs = 0.0f;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    centreDistanceMM += wheelDistanceMM[i] / MOTOR_COUNT;
    yawDifferenceMM -= WHEEL_Y_SIGN[i] * wheelDistanceMM[i];
    centreSpeedMMs += wheelMeasuredMMs[i] / MOTOR_COUNT;
    yawSpeedDifferenceMMs -= WHEEL_Y_SIGN[i] * wheelMeasuredMMs[i];
  }

  encoderYawDeg += (yawDifferenceMM / TRACK_WIDTH_MM) * RAD_TO_DEG;
  while (encoderYawDeg > 180.0f) encoderYawDeg -= 360.0f;
  while (encoderYawDeg < -180.0f) encoderYawDeg += 360.0f;

  if (mpuFound) {
    headingDeg = yawDeg;
    headingRateDegS = yawRateDegS;
  } else {
    headingDeg = encoderYawDeg;
    headingRateDegS = (yawSpeedDifferenceMMs / TRACK_WIDTH_MM) * RAD_TO_DEG;
  }

  const float headingRad = headingDeg * DEG_TO_RAD;
  poseXMM += centreDistanceMM * cos(headingRad);
  poseYMM += centreDistanceMM * sin(headingRad);
  linearVelocityMMs = centreSpeedMMs;
}

void motorControlBegin() {
  motorsBegin();   // outputs, ~31 kHz PWM, STBY high
  encodersBegin();

  // Boot gains from Config.h; PID / PIDM replace them at runtime.
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    wheelPID[i].configure(WHEEL_PID_KP, WHEEL_PID_KI, WHEEL_PID_KD,
                          WHEEL_PID_LAMBDA, WHEEL_PID_MU);
  }
}
