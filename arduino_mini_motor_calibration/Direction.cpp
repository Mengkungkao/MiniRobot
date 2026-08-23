#include "Direction.h"

#include "MiniRobotEncoders.h"
#include "MiniRobotGeometry.h"
#include "MiniRobotMotors.h"

#include "Measure.h"

int8_t encoderSign[MOTOR_COUNT];
int8_t motorSign[MOTOR_COUNT];

void signsBegin() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    encoderSign[i] = ENCODER_SIGN[i];
    motorSign[i] = MOTOR_DIRECTION_SIGN[i];
  }
}

// motorSign > 0 means "IN1 high drives this wheel forward in the robot frame".
// So the pin polarity for a requested robot-frame direction is:
//
//   motorSign  reverse   pins
//      +1       false    high   (the polarity the operator saw go forward)
//      +1       true     low
//      -1       false    low
//      -1       true     high
static inline bool pinsFor(uint8_t index, bool reverse) {
  return (motorSign[index] > 0) != reverse;
}

void driveRawPolarity(uint8_t index, bool pinsHigh, uint16_t pwm) {
  motor[index].drive(pinsHigh, (uint8_t)pwm);
}

void driveWheel(uint8_t index, bool reverse, uint16_t pwm) {
  motor[index].drive(pinsFor(index, reverse), (uint8_t)pwm);
}

bool signsOverridden() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (encoderSign[i] != ENCODER_SIGN[i]) return true;
    if (motorSign[i] != MOTOR_DIRECTION_SIGN[i]) return true;
  }
  return false;
}

static void printSignArray(const __FlashStringHelper *name, const int8_t *values) {
  Serial.print(F("const int8_t "));
  Serial.print(name);
  Serial.print(F("[MOTOR_COUNT] = {"));
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (i > 0) Serial.print(F(", "));
    Serial.print(values[i]);
  }
  Serial.println(F("};"));
}

void printSigns() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F("SIGN,"));
    Serial.print(i + 1);
    Serial.print(F(",ENC,"));
    Serial.print(encoderSign[i]);
    Serial.print(F(",MOT,"));
    Serial.println(motorSign[i]);
  }

  if (!signsOverridden()) return;

  Serial.println(F("INFO,PASTE_INTO_MiniRobotGeometry.h,IN_BOTH_SKETCH_FOLDERS"));
  Serial.println(F("// ---------------------------------------------------------"));
  printSignArray(F("ENCODER_SIGN"), encoderSign);
  printSignArray(F("MOTOR_DIRECTION_SIGN"), motorSign);
  Serial.println(F("// ---------------------------------------------------------"));
}

// -----------------------------------------------------------------------------
// DIR
// -----------------------------------------------------------------------------
//
// WHY THIS COMMAND EXISTS
// ------------------------------------------------------------
// The encoder cannot tell you which way the robot moved. It can only tell you
// that its own count went up or down, and BOTH signs sit between that count and
// the robot frame:
//
//     reported_speed = raw_ticks(pin polarity chosen by motorSign) * encoderSign
//
// Flip either sign on its own and the reported speed changes sign. Flip BOTH
// and it does not -- but the wheel now physically turns the other way. That is
// the failure this command exists to make unreachable: a wheel driven backwards
// whose encoder also reads backwards reports exactly the same healthy positive
// speed as a correct wheel, so no amount of measuring will find it. The fit
// succeeds, the PID agrees with itself, and the robot drives away backwards.
//
// The only ground truth is a human looking at the wheel. So DIR asks, once per
// wheel, and derives both signs from the answer instead of asking the operator
// to work out which sign to flip -- the guess that produced the bug above.
//
// HOW IT WORKS
// ------------------------------------------------------------
// 1. Spin the wheel at a fixed RAW pin polarity (IN1 high), ignoring both
//    signs, and record the raw tick delta.
// 2. Ask the operator which way the robot tried to roll.
// 3. motorSign  = the operator said FORWARD ? +1 : -1
//       -- "IN1 high drove this wheel forward" is exactly what +1 means.
//    encoderSign = (raw > 0) == the operator said FORWARD ? +1 : -1
//       -- the physical direction that produced those counts is now known, so
//          the sign that makes forward count positive is fixed too.
// 4. Drive the wheel forward through the derived signs and confirm it now
//    measures positive. Both signs came from one observation, so this cannot
//    disagree unless something moved -- but it is one measurement window, and
//    it is the difference between "should be right" and "is right".
static char askForwardOrBack() {
  // Drop anything queued: the operator may have typed during the probe, and the
  // terminal's own line ending from the DIR command is often still in flight.
  while (Serial.available() > 0) Serial.read();

  Serial.println(F("ASK,WHICH_WAY_DID_THE_ROBOT_TRY_TO_ROLL,F=FORWARD,B=BACKWARD,X=ABORT"));

  const uint32_t start = millis();
  char first = 0;
  while (millis() - start < ANSWER_TIMEOUT_MS) {
    if (Serial.available() == 0) {
      delay(5);
      continue;
    }

    char c = (char)Serial.read();
    if (c >= 'a' && c <= 'z') c -= ('a' - 'A');
    if (c != '\n' && c != '\r') {
      if (first == 0) first = c;      // only the first character decides
      continue;
    }
    if (first == 0) continue;         // bare line ending, e.g. the \n of a \r\n

    const char answer = first;
    first = 0;
    if (answer == 'F' || answer == 'B') return answer;
    if (answer == 'X') return 0;
    Serial.println(F("ERR,ANSWER_F_OR_B_OR_X"));
  }

  Serial.println(F("WARN,DIR_TIMEOUT_NO_ANSWER"));
  return 0;
}

// Spin the wheel at IN1-high and return the raw counts, retrying once harder
// for a gearbox that did not break away. Returns 0 when nothing counted.
static int32_t probeWheel(uint8_t index) {
  const uint16_t pwms[2] = {DIR_PROBE_PWM, DIR_PROBE_PWM_RETRY};

  for (uint8_t k = 0; k < 2; ++k) {
    Serial.print(F("INFO,DIR_PROBE,MOTOR_"));
    printMotorLabel(index);
    Serial.print(F(",pwm="));
    Serial.print(pwms[k]);
    Serial.println(F(",WATCH_THIS_WHEEL_NOW"));

    const int32_t raw = measureRawTicks(index, true, pwms[k]);

    Serial.print(F("INFO,DIR_RAW_TICKS,"));
    Serial.println(raw);

    if (labs(raw) >= DIR_MIN_TICKS) return raw;

    if (k == 0) Serial.println(F("WARN,NOTHING_COUNTED,RETRYING_HARDER"));
  }
  return 0;
}

static bool commissionWheel(uint8_t index) {
  const int32_t raw = probeWheel(index);
  if (raw == 0) {
    Serial.print(F("ERR,DIR_NO_ENCODER_MOTION,MOTOR_"));
    printMotorLabel(index);
    Serial.println();
    // Net zero counts with the motor energised. The usual cause is a channel B
    // that is floating or on the wrong pin: the ISR then sees a constant B, so
    // it adds on one edge of A and subtracts on the next and nets out to zero
    // no matter how fast the wheel spins.
    Serial.println(F("INFO,IF_THE_WHEEL_SPUN,CHANNEL_B_IS_FLOATING_OR_MISWIRED,LEFT_B=D6_RIGHT_B=D11"));
    Serial.println(F("INFO,IF_IT_DID_NOT_SPIN,CHECK_VM_STBY_AND_THE_MOTOR_LEADS"));
    Serial.println(F("INFO,CONFIRM_BY_HAND_WITH,TICKS,20"));
    return false;
  }

  const char answer = askForwardOrBack();
  if (answer == 0) {
    Serial.println(F("WARN,DIR_ABORTED,SIGNS_UNCHANGED"));
    return false;
  }

  const bool rolledForward = (answer == 'F');
  motorSign[index] = rolledForward ? 1 : -1;
  encoderSign[index] = ((raw > 0) == rolledForward) ? 1 : -1;

  Serial.print(F("INFO,DIR_SOLVED,MOTOR_"));
  printMotorLabel(index);
  Serial.print(F(",ENC,"));
  Serial.print(encoderSign[index]);
  Serial.print(F(",MOT,"));
  Serial.println(motorSign[index]);

  // Now drive it forward the way every other command will, and confirm.
  const float speed = measureSpeedMMs(index, DIR_PROBE_PWM_RETRY, false);
  motor[index].coast();
  if (isnan(speed)) {
    Serial.println(F("WARN,DIR_ABORTED,SIGNS_KEPT_BUT_UNCONFIRMED"));
    return false;
  }

  Serial.print(F("DIR,"));
  Serial.print(index + 1);
  Serial.print(F(",F,"));
  Serial.print(DIR_PROBE_PWM_RETRY);
  Serial.print(',');
  Serial.print(speed, 2);
  Serial.print(',');
  if (speed >= MIN_VALID_SPEED_MM_S) {
    Serial.println(F("OK"));
    return true;
  }

  // Both signs came from one observation, so this should be unreachable. It
  // fires if the wheel stalled against something, or if the operator answered
  // about the other wheel.
  Serial.println(F("FAILED"));
  Serial.println(F("ERR,DIR_SELF_CHECK_FAILED,RERUN,DIR,AND_ANSWER_FOR_THIS_WHEEL"));
  return false;
}

void commissionDirections(int8_t onlyIndex) {
  Serial.println(F("INFO,DIR_START,ROBOT_ON_BLOCKS,BOTH_WHEELS_MUST_SPIN_FREE"));
  Serial.println(F("INFO,FORWARD_MEANS_THE_TOP_OF_THE_WHEEL_MOVES_TOWARD_THE_FRONT"));

  const uint8_t first = (onlyIndex < 0) ? 0 : (uint8_t)onlyIndex;
  const uint8_t last = (onlyIndex < 0) ? (MOTOR_COUNT - 1) : (uint8_t)onlyIndex;

  bool allOk = true;
  for (uint8_t i = first; i <= last; ++i) {
    if (!commissionWheel(i)) allOk = false;
    motorsCoastAll();
    if (!sleepMs(400)) {
      Serial.println(F("WARN,DIR_ABORTED"));
      return;
    }
  }

  // The whole-robot check. Each wheel was answered on its own, so a wheel
  // answered against the wrong end of the robot still passes its own self
  // check; only driving both together and watching the robot catches that.
  if (allOk && onlyIndex < 0) {
    Serial.println(F("INFO,DIR_BOTH_WHEELS_FORWARD,WATCH_THE_WHOLE_ROBOT"));
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) driveWheel(i, false, DIR_PROBE_PWM);
    delay(DIR_SETTLE_MS + DIR_WINDOW_MS);
    motorsCoastAll();

    const char answer = askForwardOrBack();
    if (answer == 'B') {
      allOk = false;
      Serial.println(F("ERR,DIR_INCONSISTENT,THE_TWO_ANSWERS_DISAGREE_ABOUT_WHICH_END_IS_THE_FRONT"));
      Serial.println(F("INFO,RERUN,DIR,ANSWERING_BOTH_WHEELS_AGAINST_THE_SAME_FRONT"));
    } else if (answer == 0) {
      allOk = false;
      Serial.println(F("WARN,DIR_NOT_CONFIRMED"));
    }
  }

  motorsCoastAll();
  printSigns();
  Serial.println(allOk ? F("ACK,DIR_DONE") : F("WARN,DIR_INCOMPLETE"));
}
