#include "Fit.h"

#include "Counts.h"
#include "Direction.h"
#include "Measure.h"

FitResult results[MOTOR_COUNT][DIR_SLOT_COUNT];

void resetFitResults() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    for (uint8_t d = 0; d < DIR_SLOT_COUNT; ++d) {
      results[i][d].valid = false;
      results[i][d].samples = 0;
    }
  }
}

void printFitRow(uint8_t index, uint8_t dirSlot) {
  const FitResult &r = results[index][dirSlot];
  if (!r.valid) return;

  // Where the fitted line crosses zero speed, i.e. the modelled dead zone.
  // speedFeedForwardPWM() in the firmware reproduces exactly this point by
  // inverting the line.
  const float deadzonePWM = -r.intercept / r.slope;
  const float speedAtMax = r.slope * (float)MAX_DRIVE_PWM + r.intercept;

  Serial.print(F("R,"));
  Serial.print(index + 1);
  Serial.print(slotIsReverse(dirSlot) ? F(",R,") : F(",F,"));
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

static void printConstantLine(const __FlashStringHelper *name, uint8_t dirSlot, bool wantSlope) {
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

static void reportSlopeMismatch(uint8_t dirSlot, const __FlashStringHelper *label) {
  const float a = results[MOTOR_1][dirSlot].slope;
  const float b = results[MOTOR_2][dirSlot].slope;
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

// The point of the whole sketch: four lines that replace the four commissioning
// lines in arduino_mini_base_controller/Config.h.
static void printConstantsBlock() {
  Serial.println();
  Serial.println(F("INFO,PASTE_INTO_arduino_mini_base_controller/Config.h"));
  Serial.println(F("// ---------------------------------------------------------"));

  printConstantLine(F("FF_SLOPE_MM_S_PER_PWM_FWD"), FWD, true);
  printConstantLine(F("FF_INTERCEPT_MM_S_FWD"), FWD, false);
  printConstantLine(F("FF_SLOPE_MM_S_PER_PWM_REV"), REV, true);
  printConstantLine(F("FF_INTERCEPT_MM_S_REV"), REV, false);

  Serial.println(F("// ---------------------------------------------------------"));

  // Name every missing slot. "WARN,INCOMPLETE" on its own sends the operator
  // back to scroll through a few hundred lines of sweep output to work out
  // which wheel and which direction did not produce a line.
  bool complete = true;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    for (uint8_t d = 0; d < DIR_SLOT_COUNT; ++d) {
      if (results[i][d].valid) continue;
      complete = false;
      Serial.print(F("WARN,MISSING_FIT,MOTOR_"));
      printMotorLabel(i);
      Serial.println(slotIsReverse(d) ? F(",REVERSE") : F(",FORWARD"));
    }
  }
  if (!complete) {
    Serial.println(F("WARN,INCOMPLETE,DO_NOT_PASTE_A_0.000f_SLOPE"));
    Serial.println(F("INFO,RUN,DIR,TO_COMMISSION_THE_DIRECTION_SIGNS_FIRST"));
    return;
  }

  // A left/right difference of a few percent is normal in these motors and is
  // exactly what the per-wheel fit exists to absorb -- but a large one usually
  // means a dragging wheel or a wrong counts-per-rev on one side, which the
  // firmware cannot fix because it would be correcting to a wrong measurement.
  reportSlopeMismatch(FWD, F("FORWARD"));
  reportSlopeMismatch(REV, F("REVERSE"));

  // A fit measured under session overrides describes a robot that does not
  // exist yet. Pasting the slopes without the counts gives every speed scaled
  // by the wrong factor; without the signs, a wheel that runs backwards while
  // the PID reports success. Print them together or not at all.
  if (countsOverridden() || signsOverridden()) {
    Serial.println();
    Serial.println(F("WARN,MEASURED_WITH_OVERRIDDEN_GEOMETRY,PASTE_THIS_TOO"));
    printGeometryPasteBlock();
  }
}

void printResultTable() {
  Serial.println();
  Serial.println(F("R,motor,dir,slope,intercept,r2,samples,deadzone_pwm,mm_s_at_max_pwm"));
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    for (uint8_t d = 0; d < DIR_SLOT_COUNT; ++d) printFitRow(i, d);
  }
  printConstantsBlock();
}
