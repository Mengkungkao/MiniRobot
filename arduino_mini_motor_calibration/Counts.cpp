#include "Counts.h"

#include "MiniRobotEncoders.h"
#include "MiniRobotGeometry.h"
#include "MiniRobotMotors.h"

#include "Direction.h"
#include "Measure.h"

float countsPerRev[MOTOR_COUNT];
float mmPerCount[MOTOR_COUNT];

static void applyCounts(uint8_t index, float cpr) {
  countsPerRev[index] = cpr;
  mmPerCount[index] = (PI * WHEEL_DIAMETER_MM) / cpr;
}

void countsBegin() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) applyCounts(i, MEASURED_COUNTS_PER_REV[i]);
}

bool countsOverridden() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    // Measured values are never exactly the compiled-in float, so compare with
    // a tolerance far tighter than any real measurement error.
    if (fabs(countsPerRev[i] - MEASURED_COUNTS_PER_REV[i]) > 0.05f) return true;
  }
  return false;
}

void printCounts() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F("CPR,"));
    Serial.print(i + 1);
    Serial.print(F(",COUNTS_PER_REV,"));
    Serial.print(countsPerRev[i], 1);
    Serial.print(F(",MM_PER_COUNT,"));
    Serial.println(mmPerCount[i], 5);
  }
}

void printGeometryPasteBlock() {
  if (!countsOverridden() && !signsOverridden()) return;

  Serial.println();
  Serial.println(F("INFO,PASTE_INTO_MiniRobotGeometry.h,IN_BOTH_SKETCH_FOLDERS"));
  Serial.println(F("// ---------------------------------------------------------"));

  Serial.print(F("const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {"));
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (i > 0) Serial.print(F(", "));
    Serial.print(countsPerRev[i], 1);
    Serial.print('f');
  }
  Serial.println(F("};"));

  printSignArrays();
  Serial.println(F("// ---------------------------------------------------------"));
}

// -----------------------------------------------------------------------------
// CPR
// -----------------------------------------------------------------------------
//
// Turn the wheel by hand, count the ticks, divide by the number of turns.
//
// It has to be by hand and it has to be watched, because the number being
// measured is counts per turn of the WHEEL, after the gearbox -- and nothing
// on the board knows the gear ratio. Driving the motor instead would just
// count ticks against a clock, which is the same information the sweep already
// has and says nothing about how far the wheel went.
//
// More turns is strictly better. The error is dominated by how precisely the
// operator stops on the mark, which is a fixed number of counts however far the
// wheel went, so it divides down: one turn on this robot leaves a percent or
// two, five turns leaves a few tenths.

// Stream both wheels' running totals while the operator turns one of them.
// Returns false if they aborted or ran out of time.
static bool spinAndCount(uint8_t index, uint8_t turns,
                         int32_t &measured, int32_t &crosstalk) {
  const uint8_t otherIndex = (index == LEFT) ? RIGHT : LEFT;

  motorsCoastAll();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) readEncoderAndResetAtomic(i);
  while (Serial.available() > 0) Serial.read();

  Serial.print(F("INFO,CPR_SPIN,MOTOR_"));
  printMotorLabel(index);
  Serial.print(F(",TURN_THIS_WHEEL_EXACTLY_"));
  Serial.print(turns);
  Serial.println(F(",FULL_TURNS_BY_HAND"));
  Serial.println(F("INFO,MARK_THE_TYRE_AND_THE_CHASSIS_SO_ONE_TURN_IS_UNAMBIGUOUS"));
  Serial.println(F("INFO,DIRECTION_DOES_NOT_MATTER_HERE,ONLY_THE_MAGNITUDE_IS_USED"));
  Serial.println(F("ASK,PRESS_ENTER_WHEN_DONE,X_THEN_ENTER_TO_ABORT"));

  int32_t total[MOTOR_COUNT] = {0, 0};
  const uint32_t start = millis();
  uint32_t lastPrint = 0;
  bool finished = false;
  bool aborted = false;
  char firstChar = 0;

  while (!finished && (millis() - start) < CPR_TIMEOUT_MS) {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) total[i] += readEncoderAndResetAtomic(i);

    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c >= 'a' && c <= 'z') c -= ('a' - 'A');
      if (c == '\n' || c == '\r') {
        // Enter ends the count whether or not anything was typed first, so the
        // operator can keep both hands on the wheel and just tap it.
        finished = true;
        aborted = (firstChar == 'X');
        break;
      }
      if (firstChar == 0) firstChar = c;
    }

    if (millis() - lastPrint >= 250) {
      lastPrint = millis();
      Serial.print(F("CPR,LIVE,"));
      Serial.print(total[LEFT]);
      Serial.print(',');
      Serial.println(total[RIGHT]);
    }
    delay(5);
  }

  measured = total[index];
  crosstalk = total[otherIndex];

  if (aborted) {
    Serial.println(F("WARN,CPR_ABORTED,COUNTS_UNCHANGED"));
    return false;
  }
  if (!finished) {
    Serial.println(F("WARN,CPR_TIMEOUT,COUNTS_UNCHANGED"));
    return false;
  }
  return true;
}

static bool measureOneWheel(uint8_t index, uint8_t turns) {
  int32_t measured = 0;
  int32_t crosstalk = 0;
  if (!spinAndCount(index, turns, measured, crosstalk)) return false;

  if (labs(measured) < CPR_MIN_TICKS) {
    Serial.print(F("ERR,CPR_NO_COUNTS,MOTOR_"));
    printMotorLabel(index);
    Serial.print(F(",ticks="));
    Serial.println(measured);
    // Netting near zero while the wheel visibly turned is the channel-B
    // signature: the ISR reads a constant B, counts +1 then -1, and cancels.
    Serial.println(F("INFO,IF_THE_WHEEL_TURNED,CHANNEL_B_IS_FLOATING_OR_MISWIRED,LEFT_B=D6_RIGHT_B=D11"));
    Serial.println(F("INFO,IF_THE_OTHER_WHEEL_COUNTED_INSTEAD,THE_TWO_ENCODERS_ARE_SWAPPED"));
    return false;
  }

  // The other wheel should sit still. If it counted too, either the encoder
  // channels are cross-wired or the operator turned the wrong wheel -- both of
  // which produce a plausible-looking number that belongs to the other motor.
  if (labs(crosstalk) * 10 > labs(measured)) {
    Serial.print(F("WARN,CPR_CROSSTALK,THE_OTHER_WHEEL_ALSO_COUNTED,ticks="));
    Serial.println(crosstalk);
    Serial.println(F("INFO,CHECK_THE_ENCODERS_ARE_NOT_SWAPPED_AND_THAT_ONLY_ONE_WHEEL_MOVED"));
  }

  const float cpr = fabs((float)measured) / (float)turns;
  applyCounts(index, cpr);

  Serial.print(F("CPR,"));
  Serial.print(index + 1);
  Serial.print(F(",TICKS,"));
  Serial.print(measured);
  Serial.print(F(",TURNS,"));
  Serial.print(turns);
  Serial.print(F(",COUNTS_PER_REV,"));
  Serial.println(cpr, 1);
  return true;
}

void measureCountsPerRev(int8_t onlyIndex, uint8_t turns) {
  Serial.println(F("INFO,CPR_START,MOTORS_ARE_OFF,THIS_IS_MEASURED_BY_HAND"));

  const uint8_t first = (onlyIndex < 0) ? 0 : (uint8_t)onlyIndex;
  const uint8_t last = (onlyIndex < 0) ? (MOTOR_COUNT - 1) : (uint8_t)onlyIndex;

  bool allOk = true;
  for (uint8_t i = first; i <= last; ++i) {
    if (!measureOneWheel(i, turns)) allOk = false;
  }

  printCounts();

  // Both wheels are the same part with the same gearbox, so a real difference
  // between them is a miscount, not a fact about the robot. It matters because
  // the firmware trusts each wheel's own number: an undercounting encoder makes
  // that wheel report slow, which makes PID drive it physically fast.
  if (allOk && onlyIndex < 0) {
    const float larger = max(countsPerRev[LEFT], countsPerRev[RIGHT]);
    if (larger > 0.0f) {
      const float diffPct =
        100.0f * fabs(countsPerRev[LEFT] - countsPerRev[RIGHT]) / larger;
      Serial.print(F("INFO,CPR_MISMATCH,"));
      Serial.print(diffPct, 1);
      Serial.println('%');
      if (diffPct > 5.0f) {
        allOk = false;
        Serial.println(F("WARN,WHEELS_SHOULD_MATCH,ONE_COUNT_IS_WRONG,RERUN_CPR_WITH_MORE_TURNS"));
      }
    }
  }

  printGeometryPasteBlock();
  Serial.println(allOk ? F("ACK,CPR_DONE") : F("WARN,CPR_INCOMPLETE"));
}

// -----------------------------------------------------------------------------
// ROT
// -----------------------------------------------------------------------------
//
// CPR asks the operator to turn the wheel a whole number of turns and counts
// the ticks. ROT does the same sum backwards: it drives what the sketch now
// believes is a whole number of turns and stops, so the operator can see
// whether the mark comes back round.
//
// That is the only way to catch a counts-per-rev that is wrong by a clean
// factor -- a miscounted hand turn, an x2/x4 decoding mix-up, a gear ratio off
// by one stage. All of those produce a perfectly self-consistent number that
// CPR will happily report and every speed downstream will be scaled by.
//
// Note what the printed STOP_ERROR is and is not: it is how far past the target
// COUNT the wheel coasted before the brake caught it, which measures this
// command's own stopping accuracy. It says nothing about whether the count is
// right. That verdict is visual, and it has to be.

static bool waitForEnter(const __FlashStringHelper *prompt) {
  while (Serial.available() > 0) Serial.read();
  Serial.println(prompt);
  Serial.println(F("ASK,PRESS_ENTER_WHEN_READY,X_THEN_ENTER_TO_ABORT"));

  const uint32_t start = millis();
  char firstChar = 0;
  while ((millis() - start) < CPR_TIMEOUT_MS) {
    while (Serial.available() > 0) {
      char c = (char)Serial.read();
      if (c >= 'a' && c <= 'z') c -= ('a' - 'A');
      if (c == '\n' || c == '\r') return firstChar != 'X';
      if (firstChar == 0) firstChar = c;
    }
    delay(5);
  }
  Serial.println(F("WARN,ROT_TIMEOUT_NO_ANSWER"));
  return false;
}

static bool rotateOneWheel(uint8_t index, uint8_t turns) {
  const int32_t target = (int32_t)(countsPerRev[index] * (float)turns + 0.5f);
  if (target < 1) {
    Serial.println(F("ERR,ROT_TARGET_ZERO,RUN,CPR,FIRST"));
    return false;
  }
  const int32_t creep = (int32_t)(countsPerRev[index] * ROT_CREEP_FRACTION);

  Serial.print(F("INFO,ROT_RUN,MOTOR_"));
  printMotorLabel(index);
  Serial.print(F(",TURNS,"));
  Serial.print(turns);
  Serial.print(F(",TARGET_COUNTS,"));
  Serial.println(target);

  // Counted raw, with no signs applied, so ROT works before DIR has run and
  // whichever way the wheel happens to turn.
  readEncoderAndResetAtomic(index);
  int32_t total = 0;
  int32_t lastTotal = 0;
  uint16_t pwm = 0;
  uint32_t lastProgress = millis();
  bool stalled = false;
  bool aborted = false;

  while (labs(total) < target) {
    if (abortRequested()) {
      aborted = true;
      break;
    }

    const int32_t remaining = target - labs(total);
    const uint16_t want = (remaining > creep) ? ROT_PWM : ROT_CREEP_PWM;
    if (want != pwm) {
      pwm = want;
      driveRawPolarity(index, true, pwm);
    }

    delay(2);
    total += readEncoderAndResetAtomic(index);

    if (total != lastTotal) {
      lastTotal = total;
      lastProgress = millis();
    } else if ((millis() - lastProgress) > ROT_STALL_MS) {
      stalled = true;
      break;
    }
  }

  motor[index].brake();
  delay(ROT_SETTLE_MS);
  total += readEncoderAndResetAtomic(index);
  motor[index].coast();

  if (aborted) {
    Serial.println(F("WARN,ROT_ABORTED"));
    return false;
  }
  if (stalled) {
    Serial.print(F("ERR,ROT_NO_PROGRESS,MOTOR_"));
    printMotorLabel(index);
    Serial.print(F(",counts="));
    Serial.println(labs(total));
    Serial.println(F("INFO,THE_WHEEL_IS_JAMMED_OR_THE_ENCODER_STOPPED_REPORTING"));
    return false;
  }

  const int32_t actual = labs(total);
  const int32_t stopError = actual - target;

  Serial.print(F("ROT,"));
  Serial.print(index + 1);
  Serial.print(F(",TARGET,"));
  Serial.print(target);
  Serial.print(F(",ACTUAL,"));
  Serial.print(actual);
  Serial.print(F(",STOP_ERROR,"));
  Serial.print(stopError);
  Serial.print(F(",DEG,"));
  Serial.println(360.0f * (float)stopError / countsPerRev[index], 1);
  return true;
}

void rotateTurns(int8_t onlyIndex, uint8_t turns) {
  Serial.println(F("WARN,ROT_DRIVES_THE_MOTORS,ROBOT_ON_BLOCKS_WHEELS_SPINNING_FREE"));
  if (!waitForEnter(F("INFO,ROT,LINE_A_MARK_ON_EACH_TYRE_UP_WITH_A_MARK_ON_THE_CHASSIS"))) {
    motorsCoastAll();
    Serial.println(F("WARN,ROT_ABORTED"));
    return;
  }

  const uint8_t first = (onlyIndex < 0) ? 0 : (uint8_t)onlyIndex;
  const uint8_t last = (onlyIndex < 0) ? (MOTOR_COUNT - 1) : (uint8_t)onlyIndex;

  bool allOk = true;
  for (uint8_t i = first; i <= last; ++i) {
    if (!rotateOneWheel(i, turns)) allOk = false;
    motorsCoastAll();
    if (!sleepMs(600)) {
      Serial.println(F("WARN,ROT_ABORTED"));
      return;
    }
  }

  motorsCoastAll();
  if (allOk) {
    Serial.println(F("INFO,STOP_ERROR_IS_THIS_COMMANDS_OWN_BRAKING_OVERSHOOT,NOT_A_COUNT_ERROR"));
    Serial.println(F("INFO,THE_VERDICT_IS_VISUAL,DID_EACH_MARK_COME_BACK_ROUND"));
    Serial.println(F("INFO,STOPPED_SHORT,COUNTS_PER_REV_IS_TOO_LOW"));
    Serial.println(F("INFO,WENT_PAST,COUNTS_PER_REV_IS_TOO_HIGH"));
    Serial.println(F("INFO,EITHER_WAY_RERUN,CPR,WITH_MORE_TURNS"));
  }
  Serial.println(allOk ? F("ACK,ROT_DONE") : F("WARN,ROT_INCOMPLETE"));
}
