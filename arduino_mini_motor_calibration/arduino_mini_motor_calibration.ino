/*
  mini ROS2 base -- motor feed-forward calibration bench sketch

  WHAT THIS IS FOR
  ------------------------------------------------------------
  arduino_mini_base_controller.ino drives each wheel with a feed-forward term
  plus a PID correction. The feed-forward inverts a per-wheel, per-direction
  straight-line model of the motor:

      |speed_mm_s| = slope * pwm + intercept

  This sketch measures that line on the real hardware and least-squares fits
  it, so the four commissioning arrays in the main firmware

      FF_SLOPE_MM_S_PER_PWM_FWD[]   FF_INTERCEPT_MM_S_FWD[]
      FF_SLOPE_MM_S_PER_PWM_REV[]   FF_INTERCEPT_MM_S_REV[]

  come from data instead of a guess. Brushed gearmotors differ from each other
  by tens of percent and are direction-asymmetric; that asymmetry is what makes
  a robot drive straight forward and curve in reverse.

  It is a standalone bench tool: no IMU, no odometry, no PID, no ROS protocol.
  Flash it, calibrate, paste the printed constants into the main firmware, then
  flash the main firmware back.

  HOW TO USE IT
  ------------------------------------------------------------
  1. Put the robot on blocks so BOTH WHEELS SPIN FREE. Every measurement here
     is open loop and runs up to MAX_DRIVE_PWM.
  2. Use a charged battery, and keep it charged across the whole run. The fit
     is only valid at the bus voltage it was measured at, so a sagging battery
     bakes a wrong slope into the firmware.
  3. Flash this sketch and open a serial terminal at 115200 baud, line ending
     "Newline" (carriage return works too).
  4. Check MEASURED_COUNTS_PER_REV below is right for your encoders. Everything
     printed here is in mm/s derived from it, so if it is wrong the whole fit is
     wrong by the same factor. Verify with:  TICKS,20
     then turn one wheel by hand exactly one full turn and read the total.
  5. Run:  CAL
     A few minutes: both wheels, both directions, two passes each.
  6. Paste the printed "const float FF_..." block into
     arduino_mini_base_controller.ino, replacing the four existing lines.
  7. Optional but recommended:  VERIFY
     drives a few target speeds through the freshly fitted feed-forward and
     reports the open-loop error. Under ~10% is a good fit; PID cleans up the
     rest.

  Sending ANY character while a sweep is running aborts it and coasts the
  motors.

  COMMANDS
  ------------------------------------------------------------
    CAL                        calibrate both motors, both directions
    CAL,<motor>                one motor (1 = left, 2 = right), both directions
    SWEEP,<motor>[,R]          one motor, one direction (R = reverse)
    VERIFY                     open-loop check of the fitted constants
    VERIFY,<motor>             ... for one motor
    RESULT                     reprint the last fit table and constants block
    HOLD,<motor>,<pwm>[,R]     hold one PWM and stream the measured speed
    TICKS[,<seconds>]          stream raw encoder counts (counts-per-rev check)
    STEP,<pwm_step>            PWM step size             (default 10)
    RANGE,<min_pwm>,<max_pwm>  PWM sweep range           (default 30..210)
    DWELL,<settle_ms>,<win_ms> per-step timing           (default 400, 600)
    PASSES,<1_to_4>            sweep passes per direction (default 2: up, down)
    CONFIG                     print the current sweep settings
    STOP                       coast both motors
    HELP                       print this list

  OUTPUT LINES
  ------------------------------------------------------------
    C,<motor>,<F|R>,<pwm>,<mm_s>,<USED|STALL>   one measured point
    R,<motor>,<F|R>,<slope>,<intercept>,<r2>,<n>,<deadzone_pwm>,<mm_s_at_max>
    V,<motor>,<F|R>,<target_mm_s>,<measured_mm_s>,<error_pct>
    TICKS,<left>,<right>
    INFO,... / WARN,... / ERR,... / ACK,...

  The C rows are plain CSV, so you can also capture them and fit offline if you
  would rather look at the scatter yourself.

  WIRING -- identical to arduino_mini_base_controller.ino
  ------------------------------------------------------------
  TB6612FNG            Uno            Encoders          Uno
    PWMA (left)        D10              Left  A         D2  (INT0)
    AIN1 (left)        D5               Left  B         D6
    AIN2 (left)        D4               Right A         D3  (INT1)
    PWMB (right)       D9               Right B         D11
    BIN1 (right)       D7
    BIN2 (right)       D8             The IMU is not used by this sketch and
    STBY               D12            can stay connected or be left off.
    VM                 motor battery +
    VCC / GND          5V / GND (battery ground common with the Uno)
*/

#include <math.h>
#include <stdlib.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Pin map -- keep in sync with arduino_mini_base_controller.ino
// -----------------------------------------------------------------------------

const uint8_t PIN_PWMA = 10;    // TB6612 channel A -> LEFT wheel
const uint8_t PIN_AIN1 = 5;
const uint8_t PIN_AIN2 = 4;

const uint8_t PIN_PWMB = 9;     // TB6612 channel B -> RIGHT wheel
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 8;

const uint8_t PIN_STBY = 12;

const uint8_t PIN_ENC_LEFT_A = 2;
const uint8_t PIN_ENC_LEFT_B = 6;
const uint8_t PIN_ENC_RIGHT_A = 3;
const uint8_t PIN_ENC_RIGHT_B = 11;

// -----------------------------------------------------------------------------
// Geometry and limits -- keep in sync with arduino_mini_base_controller.ino
// -----------------------------------------------------------------------------

const uint8_t MOTOR_COUNT = 2;
const uint8_t LEFT = 0;
const uint8_t RIGHT = 1;

// A plain terminal baud rate, not the 500000 the ROS bridge uses. Nothing here
// is parsed by ROS, and 115200 works in every serial monitor.
const uint32_t SERIAL_BAUD = 115200UL;

const float WHEEL_DIAMETER_MM = 47.0f;

// THE measurement this whole sketch is scaled by. Channel A is decoded on
// CHANGE (x2), so a 360 PPR output-shaft encoder reads 720 counts per wheel
// revolution. Verify per wheel with the TICKS command before trusting any fit.
const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {720.0f, 720.0f};

const float MM_PER_COUNT[MOTOR_COUNT] = {
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[0],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[1],
};

// The fit must be measured over the same PWM range the firmware is allowed to
// use, so this has to match the main sketch.
const uint8_t MAX_DRIVE_PWM = 210;

// Forward motion must read positive on both wheels; a wheel that physically
// runs backwards on a forward command needs its direction sign flipped. Both
// arrays exist in the main firmware too -- set them there first, copy here, and
// this sketch will confirm them (it warns if a sweep measures the wrong sign).
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {1, 1};
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {1, 1};

// -----------------------------------------------------------------------------
// Calibration behaviour
// -----------------------------------------------------------------------------

// Points slower than this are stiction/dead-zone, where speed is not linear in
// PWM. They are printed as STALL and excluded from the fit -- including them
// would drag the line down and make the firmware under-drive at every speed.
const float MIN_VALID_SPEED_MM_S = 15.0f;

// Below this the fit is not trustworthy enough to paste (loose gearbox, a
// dragging wheel, or a battery that sagged during the sweep).
const float MIN_ACCEPTABLE_R2 = 0.97f;

// Target speeds VERIFY drives, in mm/s.
const float VERIFY_SPEEDS_MM_S[] = {60.0f, 120.0f, 180.0f, 240.0f};
const uint8_t VERIFY_SPEED_COUNT =
  sizeof(VERIFY_SPEEDS_MM_S) / sizeof(VERIFY_SPEEDS_MM_S[0]);

// Runtime-adjustable sweep settings (STEP / RANGE / DWELL / PASSES).
uint16_t sweepMinPWM = 30;
uint16_t sweepMaxPWM = MAX_DRIVE_PWM;
uint16_t sweepStepPWM = 10;
uint16_t settleMs = 400;      // let the wheel reach steady state
uint16_t windowMs = 600;      // then count ticks for this long
uint8_t sweepPasses = 2;      // 2 = ascending then descending

// Safety ceiling for HOLD, which otherwise runs a motor indefinitely.
const uint32_t HOLD_MAX_MS = 20000UL;

// -----------------------------------------------------------------------------
// TB6612FNG driver
// -----------------------------------------------------------------------------

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
// Encoders -- same x2 decode as the main firmware, so the counts mean the same
// -----------------------------------------------------------------------------

volatile int32_t encoderCount[MOTOR_COUNT] = {0, 0};

void encoderLeftISR() {
  if (digitalRead(PIN_ENC_LEFT_A) == digitalRead(PIN_ENC_LEFT_B)) encoderCount[LEFT]++;
  else encoderCount[LEFT]--;
}

void encoderRightISR() {
  if (digitalRead(PIN_ENC_RIGHT_A) == digitalRead(PIN_ENC_RIGHT_B)) encoderCount[RIGHT]++;
  else encoderCount[RIGHT]--;
}

int32_t readEncoderAndResetAtomic(uint8_t index) {
  noInterrupts();
  const int32_t value = encoderCount[index];
  encoderCount[index] = 0;
  interrupts();
  return value;
}

// Timer1 drives both PWM pins (D9/D10). Prescaler 1 puts it at ~31.4 kHz, out
// of the audible band -- the same setting the main firmware runs, which matters
// because PWM frequency changes the effective torque at a given duty.
void setPwmFrequency31kHz() {
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
}

// -----------------------------------------------------------------------------
// Streaming least-squares fit
// -----------------------------------------------------------------------------

// Welford-style accumulation of the means, variances and covariance. The
// textbook sum-of-squares form loses precision fast in 32-bit float once the
// sums grow; this stays well conditioned and needs no sample array, which
// matters on an Uno with 2 KB of SRAM.
struct LinearFit {
  uint16_t n;
  float meanX, meanY;
  float m2x, m2y, cxy;

  void reset() {
    n = 0;
    meanX = meanY = m2x = m2y = cxy = 0.0f;
  }

  void add(float x, float y) {
    ++n;
    const float dx = x - meanX;
    const float dy = y - meanY;
    meanX += dx / (float)n;
    meanY += dy / (float)n;
    m2x += dx * (x - meanX);
    m2y += dy * (y - meanY);
    cxy += dx * (y - meanY);
  }

  // y = slope*x + intercept, plus the coefficient of determination.
  bool solve(float &slope, float &intercept, float &r2) const {
    if (n < 3 || m2x <= 0.0f) return false;
    slope = cxy / m2x;
    intercept = meanY - slope * meanX;
    r2 = (m2y > 0.0f) ? (cxy * cxy) / (m2x * m2y) : 1.0f;
    return true;
  }
};

// -----------------------------------------------------------------------------
// Results
// -----------------------------------------------------------------------------

const uint8_t FWD = 0;
const uint8_t REV = 1;

struct FitResult {
  bool valid;
  float slope;          // mm/s per PWM count
  float intercept;      // mm/s at PWM 0 (negative: the dead zone)
  float r2;
  uint16_t samples;     // points that made it into the fit
  uint16_t stalled;     // points rejected as below MIN_VALID_SPEED_MM_S
  float maxSpeedMMs;    // fastest speed actually measured
  bool signWarning;     // wheel turned the opposite way to the command
};

FitResult results[MOTOR_COUNT][2];

char commandBuffer[48];
uint8_t commandLength = 0;

// Forward declarations: these call each other across the file.
void printFitRow(uint8_t index, uint8_t dirSlot);
void printConstantsBlock();
void printConstantLine(const __FlashStringHelper *name, uint8_t dirSlot, bool wantSlope);
void reportSlopeMismatch(uint8_t dirSlot, const __FlashStringHelper *label);

// -----------------------------------------------------------------------------
// Small helpers
// -----------------------------------------------------------------------------

void coastAll() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) motor[i].coast();
}

void printMotorLabel(uint8_t index) {
  Serial.print(index + 1);
  Serial.print(index == LEFT ? F("(LEFT)") : F("(RIGHT)"));
}

// True if the operator sent anything -- used as an abort request during the
// blocking sweeps. The bytes are swallowed so the abort character does not come
// back as a bogus command afterwards.
bool abortRequested() {
  if (Serial.available() <= 0) return false;
  while (Serial.available() > 0) Serial.read();
  return true;
}

// delay() that can be cut short by an abort request. Returns false if aborted.
bool sleepMs(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    if (abortRequested()) return false;
    delay(2);
  }
  return true;
}

// TB6612 pin polarity for "the wheel should turn this way", accounting for a
// motor wired backwards.
bool pinsForward(uint8_t index, bool reverse) {
  return (MOTOR_DIRECTION_SIGN[index] > 0) ? !reverse : reverse;
}

// Run one wheel at one PWM and return the steady-state speed in mm/s, signed in
// the robot frame (positive = this wheel is driving the robot forward). Returns
// NAN if the operator aborted mid-measurement.
float measureSpeedMMs(uint8_t index, uint16_t pwm, bool reverse) {
  motor[index].drive(pinsForward(index, reverse), (uint8_t)pwm);

  if (!sleepMs(settleMs)) return NAN;

  readEncoderAndResetAtomic(index);          // discard the spin-up ticks
  const uint32_t windowStart = millis();
  if (!sleepMs(windowMs)) return NAN;
  const int32_t ticks = readEncoderAndResetAtomic(index) * ENCODER_SIGN[index];
  const uint32_t elapsedMs = millis() - windowStart;

  if (elapsedMs == 0) return 0.0f;
  return (ticks * MM_PER_COUNT[index]) * 1000.0f / (float)elapsedMs;
}

// -----------------------------------------------------------------------------
// Sweep and fit
// -----------------------------------------------------------------------------

// Step one wheel across the PWM range in one direction, print every point, and
// fit the linear model to the points that were actually moving.
//
// Blocks for roughly passes * steps * (settle + window) milliseconds. Send any
// character to abort.
void sweepAndFit(uint8_t index, bool reverse) {
  const uint8_t dirSlot = reverse ? REV : FWD;
  FitResult &out = results[index][dirSlot];

  Serial.print(F("INFO,SWEEP_START,MOTOR_"));
  printMotorLabel(index);
  Serial.println(reverse ? F(",REVERSE") : F(",FORWARD"));

  LinearFit fit;
  fit.reset();

  // Reported speeds are magnitudes along the commanded direction, because that
  // is the form speedFeedForwardPWM() inverts: it feeds |target| into the fit
  // belonging to that direction.
  const float directionSign = reverse ? -1.0f : 1.0f;

  uint16_t stalled = 0;
  float maxSpeed = 0.0f;
  bool signWarning = false;
  bool aborted = false;

  for (uint8_t pass = 0; pass < sweepPasses && !aborted; ++pass) {
    // Even passes ramp up, odd passes ramp down. A gearmotor settles at a
    // slightly different speed at the same PWM depending on which side it
    // approached from; averaging both directions of travel fits the middle of
    // that hysteresis loop instead of one edge of it.
    const bool ascending = (pass % 2) == 0;
    const uint16_t steps = (sweepMaxPWM - sweepMinPWM) / sweepStepPWM + 1;

    for (uint16_t k = 0; k < steps; ++k) {
      const uint16_t pwm = ascending
                         ? (sweepMinPWM + k * sweepStepPWM)
                         : (sweepMinPWM + (steps - 1 - k) * sweepStepPWM);

      const float signedSpeed = measureSpeedMMs(index, pwm, reverse);
      if (isnan(signedSpeed)) {
        aborted = true;
        break;
      }

      const float speed = signedSpeed * directionSign;   // magnitude along command
      const bool used = speed >= MIN_VALID_SPEED_MM_S;

      Serial.print(F("C,"));
      Serial.print(index + 1);
      Serial.print(reverse ? F(",R,") : F(",F,"));
      Serial.print(pwm);
      Serial.print(',');
      Serial.print(speed, 2);
      Serial.println(used ? F(",USED") : F(",STALL"));

      if (used) {
        fit.add((float)pwm, speed);
        if (speed > maxSpeed) maxSpeed = speed;
      } else {
        ++stalled;
        // A wheel running backwards under a forward command is a wiring or sign
        // problem, not a slow motor, and no fit can repair it.
        if (speed <= -MIN_VALID_SPEED_MM_S) signWarning = true;
      }
    }

    motor[index].coast();
    if (!aborted && pass + 1 < sweepPasses) {
      if (!sleepMs(300)) aborted = true;    // let it stop before the next pass
    }
  }

  motor[index].coast();
  readEncoderAndResetAtomic(index);

  if (aborted) {
    Serial.println(F("WARN,SWEEP_ABORTED,NO_FIT_STORED"));
    return;
  }

  out.valid = fit.solve(out.slope, out.intercept, out.r2);
  out.samples = fit.n;
  out.stalled = stalled;
  out.maxSpeedMMs = maxSpeed;
  out.signWarning = signWarning;

  if (!out.valid) {
    Serial.println(F("ERR,FIT_FAILED,TOO_FEW_MOVING_POINTS"));
    Serial.println(F("INFO,CHECK_BATTERY_STBY_WIRING_AND_THAT_THE_WHEEL_IS_FREE"));
  } else if (out.slope <= 0.0f) {
    // More PWM produced less speed: whatever was measured, it is not a motor
    // curve, so refuse to hand it back as one.
    out.valid = false;
    Serial.println(F("ERR,FIT_REJECTED,NON_POSITIVE_SLOPE"));
  } else {
    printFitRow(index, dirSlot);
  }

  if (signWarning) {
    Serial.print(F("WARN,WRONG_DIRECTION,MOTOR_"));
    printMotorLabel(index);
    Serial.println(F(",FIX_MOTOR_DIRECTION_SIGN_OR_ENCODER_SIGN_FIRST"));
  }
  if (out.valid && out.r2 < MIN_ACCEPTABLE_R2) {
    Serial.print(F("WARN,LOW_R2,"));
    Serial.print(out.r2, 4);
    Serial.println(F(",RERUN_WITH_LONGER_DWELL_OR_A_CHARGED_BATTERY"));
  }
}

// One result row: R,motor,dir,slope,intercept,r2,samples,deadzone_pwm,mm_s_at_max_pwm
void printFitRow(uint8_t index, uint8_t dirSlot) {
  const FitResult &r = results[index][dirSlot];
  if (!r.valid) return;

  // Where the fitted line crosses zero speed, i.e. the modelled dead zone.
  // speedFeedForwardPWM() reproduces exactly this point by inverting the line.
  const float deadzonePWM = -r.intercept / r.slope;
  const float speedAtMax = r.slope * (float)MAX_DRIVE_PWM + r.intercept;

  Serial.print(F("R,"));
  Serial.print(index + 1);
  Serial.print(dirSlot == REV ? F(",R,") : F(",F,"));
  Serial.print(r.slope, 4);
  Serial.print(',');
  Serial.print(r.intercept, 2);
  Serial.print(',');
  Serial.print(r.r2, 4);
  Serial.print(',');
  Serial.print(r.samples);
  Serial.print(',');
  Serial.print(deadzonePWM, 1);
  Serial.print(',');
  Serial.println(speedAtMax, 1);
}

// The point of the whole sketch: four lines that replace the four commissioning
// lines in arduino_mini_base_controller.ino.
void printConstantsBlock() {
  Serial.println();
  Serial.println(F("INFO,PASTE_INTO_arduino_mini_base_controller.ino"));
  Serial.println(F("// ---------------------------------------------------------"));

  printConstantLine(F("FF_SLOPE_MM_S_PER_PWM_FWD"), FWD, true);
  printConstantLine(F("FF_INTERCEPT_MM_S_FWD"), FWD, false);
  printConstantLine(F("FF_SLOPE_MM_S_PER_PWM_REV"), REV, true);
  printConstantLine(F("FF_INTERCEPT_MM_S_REV"), REV, false);

  Serial.println(F("// ---------------------------------------------------------"));

  bool complete = true;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    for (uint8_t d = 0; d < 2; ++d) {
      if (!results[i][d].valid) complete = false;
    }
  }
  if (!complete) {
    Serial.println(F("WARN,INCOMPLETE,RUN_CAL_BEFORE_PASTING"));
    return;
  }

  // A left/right difference of a few percent is normal in these motors and is
  // exactly what the per-wheel fit exists to absorb -- but a large one usually
  // means a dragging wheel or a wrong counts-per-rev on one side, which the
  // firmware cannot fix because it would be correcting to a wrong measurement.
  reportSlopeMismatch(FWD, F("FORWARD"));
  reportSlopeMismatch(REV, F("REVERSE"));
}

void printConstantLine(const __FlashStringHelper *name, uint8_t dirSlot, bool wantSlope) {
  Serial.print(F("const float "));
  Serial.print(name);
  Serial.print(F("[MOTOR_COUNT] = {"));
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (i > 0) Serial.print(F(", "));
    const FitResult &r = results[i][dirSlot];
    if (!r.valid) {
      // Deliberately not a plausible number: an uncalibrated wheel should fail
      // loudly when pasted, not quietly drive wrong.
      Serial.print(F("0.000f /*NOT_CALIBRATED*/"));
    } else {
      Serial.print(wantSlope ? r.slope : r.intercept, wantSlope ? 4 : 2);
      Serial.print('f');
    }
  }
  Serial.println(F("};"));
}

void reportSlopeMismatch(uint8_t dirSlot, const __FlashStringHelper *label) {
  const float a = results[LEFT][dirSlot].slope;
  const float b = results[RIGHT][dirSlot].slope;
  const float larger = max(a, b);
  if (larger <= 0.0f) return;

  const float mismatchPct = 100.0f * fabs(a - b) / larger;
  Serial.print(F("INFO,SLOPE_MISMATCH_"));
  Serial.print(label);
  Serial.print(',');
  Serial.print(mismatchPct, 1);
  Serial.println('%');
  if (mismatchPct > 25.0f) {
    Serial.println(F("WARN,WHEELS_DIFFER_A_LOT,CHECK_FOR_DRAG_AND_RECHECK_COUNTS_PER_REV"));
  }
}

void printResultTable() {
  Serial.println();
  Serial.println(F("R,motor,dir,slope,intercept,r2,samples,deadzone_pwm,mm_s_at_max_pwm"));
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    for (uint8_t d = 0; d < 2; ++d) printFitRow(i, d);
  }
  printConstantsBlock();
}

// -----------------------------------------------------------------------------
// Verification
// -----------------------------------------------------------------------------

// The same inversion the main firmware performs, using the fit just measured.
// Takes indices rather than a FitResult reference because the Arduino build
// hoists auto-generated prototypes above the struct definitions in this file.
float feedForwardPWM(uint8_t index, uint8_t dirSlot, float targetMMs) {
  const FitResult &r = results[index][dirSlot];
  if (!r.valid || r.slope <= 0.0f) return 0.0f;
  return constrain((targetMMs - r.intercept) / r.slope, 0.0f, (float)MAX_DRIVE_PWM);
}

// Drive a few target speeds open loop through the fitted constants and report
// how far off the wheel actually lands. This is the check that the numbers are
// worth pasting: whatever residual is left here is what PID has to absorb.
void verifyMotor(uint8_t index) {
  for (uint8_t d = 0; d < 2; ++d) {
    const bool reverse = (d == REV);
    const FitResult &r = results[index][d];
    if (!r.valid) {
      Serial.print(F("WARN,NO_FIT,MOTOR_"));
      printMotorLabel(index);
      Serial.println(reverse ? F(",REVERSE") : F(",FORWARD"));
      continue;
    }

    for (uint8_t k = 0; k < VERIFY_SPEED_COUNT; ++k) {
      const float target = VERIFY_SPEEDS_MM_S[k];
      if (target > r.maxSpeedMMs) continue;    // beyond what this wheel reached

      const uint16_t pwm = (uint16_t)(feedForwardPWM(index, d, target) + 0.5f);
      const float signedSpeed = measureSpeedMMs(index, pwm, reverse);
      motor[index].coast();
      if (isnan(signedSpeed)) {
        Serial.println(F("WARN,VERIFY_ABORTED"));
        return;
      }

      const float measured = signedSpeed * (reverse ? -1.0f : 1.0f);
      Serial.print(F("V,"));
      Serial.print(index + 1);
      Serial.print(reverse ? F(",R,") : F(",F,"));
      Serial.print(target, 1);
      Serial.print(',');
      Serial.print(measured, 1);
      Serial.print(',');
      Serial.println(100.0f * (measured - target) / target, 1);

      if (!sleepMs(200)) return;
    }
  }
}

// -----------------------------------------------------------------------------
// Interactive helpers
// -----------------------------------------------------------------------------

// Hold one PWM and stream the measured speed, for spot-checking a single point
// or watching how much the speed sags as the battery drains.
void holdAndStream(uint8_t index, uint16_t pwm, bool reverse, uint32_t durationMs) {
  Serial.println(F("INFO,HOLD_START,SEND_ANY_CHARACTER_TO_STOP"));
  motor[index].drive(pinsForward(index, reverse), (uint8_t)pwm);
  readEncoderAndResetAtomic(index);

  const uint32_t start = millis();
  uint32_t lastSample = millis();
  while (millis() - start < durationMs) {
    if (abortRequested()) break;
    delay(5);
    if (millis() - lastSample < 200) continue;

    const int32_t ticks = readEncoderAndResetAtomic(index) * ENCODER_SIGN[index];
    const uint32_t elapsedMs = millis() - lastSample;
    lastSample = millis();

    const float speed = (ticks * MM_PER_COUNT[index]) * 1000.0f / (float)elapsedMs;
    Serial.print(F("C,"));
    Serial.print(index + 1);
    Serial.print(reverse ? F(",R,") : F(",F,"));
    Serial.print(pwm);
    Serial.print(',');
    Serial.print(speed * (reverse ? -1.0f : 1.0f), 2);
    Serial.println(F(",HOLD"));
  }

  motor[index].coast();
  readEncoderAndResetAtomic(index);
  Serial.println(F("ACK,HOLD_DONE"));
}

// Stream raw counts with the motors off, so counts-per-rev can be measured by
// turning a wheel exactly one turn by hand. Every mm/s in this sketch and in the
// firmware is scaled by that number, so it is the first thing to get right.
void streamTicks(uint32_t durationMs) {
  coastAll();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) readEncoderAndResetAtomic(i);

  int32_t total[MOTOR_COUNT] = {0, 0};
  Serial.println(F("INFO,TURN_ONE_WHEEL_EXACTLY_ONE_FULL_TURN_BY_HAND"));
  Serial.println(F("INFO,PUT_THE_MAGNITUDE_IN_MEASURED_COUNTS_PER_REV"));

  const uint32_t start = millis();
  uint32_t lastPrint = 0;
  while (millis() - start < durationMs) {
    if (abortRequested()) break;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      total[i] += readEncoderAndResetAtomic(i) * ENCODER_SIGN[i];
    }
    if (millis() - lastPrint >= 250) {
      lastPrint = millis();
      Serial.print(F("TICKS,"));
      Serial.print(total[LEFT]);
      Serial.print(',');
      Serial.println(total[RIGHT]);
    }
    delay(5);
  }
  Serial.println(F("ACK,TICKS_DONE"));
}

void printConfig() {
  Serial.print(F("INFO,CONFIG,range="));
  Serial.print(sweepMinPWM);
  Serial.print(F(".."));
  Serial.print(sweepMaxPWM);
  Serial.print(F(",step="));
  Serial.print(sweepStepPWM);
  Serial.print(F(",settle_ms="));
  Serial.print(settleMs);
  Serial.print(F(",window_ms="));
  Serial.print(windowMs);
  Serial.print(F(",passes="));
  Serial.println(sweepPasses);

  const uint16_t steps = (sweepMaxPWM - sweepMinPWM) / sweepStepPWM + 1;
  const uint32_t perDirectionS =
    ((uint32_t)steps * sweepPasses * (uint32_t)(settleMs + windowMs)) / 1000UL;
  Serial.print(F("INFO,ESTIMATED_SECONDS,per_direction="));
  Serial.print(perDirectionS);
  Serial.print(F(",full_CAL="));
  Serial.println(perDirectionS * 4);
}

void printHelp() {
  Serial.println(F("CMD,CAL[,motor]            calibrate (default: both motors, both directions)"));
  Serial.println(F("CMD,SWEEP,motor[,R]        one motor, one direction"));
  Serial.println(F("CMD,VERIFY[,motor]         open-loop check of the fitted constants"));
  Serial.println(F("CMD,RESULT                 reprint fits and the paste block"));
  Serial.println(F("CMD,HOLD,motor,pwm[,R]     hold one PWM, stream speed"));
  Serial.println(F("CMD,TICKS[,seconds]        stream raw encoder counts"));
  Serial.println(F("CMD,STEP,pwm_step          set sweep step"));
  Serial.println(F("CMD,RANGE,min_pwm,max_pwm  set sweep range"));
  Serial.println(F("CMD,DWELL,settle_ms,win_ms set per-step timing"));
  Serial.println(F("CMD,PASSES,1..4            set passes per direction"));
  Serial.println(F("CMD,CONFIG                 print settings and run-time estimate"));
  Serial.println(F("CMD,STOP                   coast both motors"));
  Serial.println(F("INFO,ANY_CHARACTER_ABORTS_A_RUNNING_SWEEP"));
}

// -----------------------------------------------------------------------------
// Command parser
// -----------------------------------------------------------------------------

void calibrateMotor(uint8_t index) {
  sweepAndFit(index, false);
  sleepMs(500);
  sweepAndFit(index, true);
  sleepMs(500);
}

void processCommand(char *line) {
  char *savePtr = NULL;
  char *command = strtok_r(line, ",", &savePtr);
  if (command == NULL) return;

  for (char *p = command; *p; ++p) {
    if (*p >= 'a' && *p <= 'z') *p = *p - ('a' - 'A');
  }

  if (strcmp(command, "CAL") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    if (motorText) {
      const int index = atoi(motorText) - 1;
      if (index < 0 || index >= MOTOR_COUNT) {
        Serial.println(F("ERR,CAL_MOTOR_RANGE"));
        return;
      }
      calibrateMotor((uint8_t)index);
    } else {
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) calibrateMotor(i);
    }
    printResultTable();
    Serial.println(F("ACK,CAL_DONE"));
  }
  else if (strcmp(command, "SWEEP") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *dirText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    if (index < 0 || index >= MOTOR_COUNT) {
      Serial.println(F("ERR,SWEEP_FORMAT"));
      return;
    }
    sweepAndFit((uint8_t)index, dirText && (dirText[0] == 'R' || dirText[0] == 'r'));
    Serial.println(F("ACK,SWEEP_DONE"));
  }
  else if (strcmp(command, "VERIFY") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    if (motorText) {
      const int index = atoi(motorText) - 1;
      if (index < 0 || index >= MOTOR_COUNT) {
        Serial.println(F("ERR,VERIFY_MOTOR_RANGE"));
        return;
      }
      verifyMotor((uint8_t)index);
    } else {
      for (uint8_t i = 0; i < MOTOR_COUNT; ++i) verifyMotor(i);
    }
    Serial.println(F("ACK,VERIFY_DONE"));
  }
  else if (strcmp(command, "RESULT") == 0) {
    printResultTable();
  }
  else if (strcmp(command, "HOLD") == 0) {
    char *motorText = strtok_r(NULL, ",", &savePtr);
    char *pwmText = strtok_r(NULL, ",", &savePtr);
    char *dirText = strtok_r(NULL, ",", &savePtr);
    const int index = motorText ? atoi(motorText) - 1 : -1;
    const int pwm = pwmText ? atoi(pwmText) : -1;
    if (index < 0 || index >= MOTOR_COUNT || pwm < 0 || pwm > MAX_DRIVE_PWM) {
      Serial.println(F("ERR,HOLD_FORMAT"));
      return;
    }
    holdAndStream((uint8_t)index, (uint16_t)pwm,
                  dirText && (dirText[0] == 'R' || dirText[0] == 'r'), HOLD_MAX_MS);
  }
  else if (strcmp(command, "TICKS") == 0) {
    char *secondsText = strtok_r(NULL, ",", &savePtr);
    const int seconds = secondsText ? atoi(secondsText) : 30;
    streamTicks((uint32_t)constrain(seconds, 1, 300) * 1000UL);
  }
  else if (strcmp(command, "STEP") == 0) {
    char *stepText = strtok_r(NULL, ",", &savePtr);
    const int step = stepText ? atoi(stepText) : 0;
    if (step < 1 || step > 50) {
      Serial.println(F("ERR,STEP_RANGE_1_50"));
      return;
    }
    sweepStepPWM = (uint16_t)step;
    printConfig();
  }
  else if (strcmp(command, "RANGE") == 0) {
    char *minText = strtok_r(NULL, ",", &savePtr);
    char *maxText = strtok_r(NULL, ",", &savePtr);
    const int lo = minText ? atoi(minText) : -1;
    const int hi = maxText ? atoi(maxText) : -1;
    // At least 30 PWM of span, or there is nothing to fit a line through.
    if (lo < 0 || hi > MAX_DRIVE_PWM || hi - lo < 30) {
      Serial.print(F("ERR,RANGE_MUST_BE_0.."));
      Serial.println(MAX_DRIVE_PWM);
      return;
    }
    sweepMinPWM = (uint16_t)lo;
    sweepMaxPWM = (uint16_t)hi;
    printConfig();
  }
  else if (strcmp(command, "DWELL") == 0) {
    char *settleText = strtok_r(NULL, ",", &savePtr);
    char *windowText = strtok_r(NULL, ",", &savePtr);
    const int settle = settleText ? atoi(settleText) : -1;
    const int window = windowText ? atoi(windowText) : -1;
    // A window shorter than ~200 ms quantises badly: at low PWM only a handful
    // of ticks land inside it, and the fit inherits that noise.
    if (settle < 50 || settle > 5000 || window < 200 || window > 5000) {
      Serial.println(F("ERR,DWELL_SETTLE_50_5000_WINDOW_200_5000"));
      return;
    }
    settleMs = (uint16_t)settle;
    windowMs = (uint16_t)window;
    printConfig();
  }
  else if (strcmp(command, "PASSES") == 0) {
    char *passText = strtok_r(NULL, ",", &savePtr);
    const int passes = passText ? atoi(passText) : 0;
    if (passes < 1 || passes > 4) {
      Serial.println(F("ERR,PASSES_RANGE_1_4"));
      return;
    }
    sweepPasses = (uint8_t)passes;
    printConfig();
  }
  else if (strcmp(command, "CONFIG") == 0) {
    printConfig();
  }
  else if (strcmp(command, "STOP") == 0) {
    coastAll();
    Serial.println(F("ACK,STOP"));
  }
  else if (strcmp(command, "HELP") == 0 || strcmp(command, "?") == 0) {
    printHelp();
  }
  else if (strcmp(command, "PING") == 0) {
    Serial.println(F("PONG"));
  }
  else {
    Serial.print(F("ERR,UNKNOWN_COMMAND,"));
    Serial.println(command);
  }
}

// Accumulate serial bytes and dispatch on \n, \r or ':' -- the same line
// discipline as the main firmware.
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
// Entry points
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    motor[i].begin();
    for (uint8_t d = 0; d < 2; ++d) {
      results[i][d].valid = false;
      results[i][d].signWarning = false;
      results[i][d].samples = 0;
      results[i][d].stalled = 0;
      results[i][d].maxSpeedMMs = 0.0f;
    }
  }
  setPwmFrequency31kHz();

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);   // take both bridges out of standby

  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), encoderRightISR, CHANGE);

  coastAll();
  delay(200);

  Serial.println();
  Serial.println(F("READY,mini_motor_ff_calibration_v1"));
  Serial.println(F("WARN,PUT_THE_ROBOT_ON_BLOCKS_BOTH_WHEELS_MUST_SPIN_FREE"));
  Serial.println(F("INFO,SEND_CAL_TO_START_OR_HELP_FOR_COMMANDS"));
  printConfig();
}

void loop() {
  readSerialCommands();
}
