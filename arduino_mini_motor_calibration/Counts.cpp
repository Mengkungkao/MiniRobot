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
