#include "Sweep.h"

#include "MiniRobotEncoders.h"
#include "MiniRobotMotors.h"

#include "Config.h"
#include "Fit.h"
#include "Measure.h"

// Runtime-adjustable sweep settings, declared extern in Config.h.
// 10, matching SWEEP_MIN_PWM in the base controller. With MAX_LINEAR_MM_S at
// 150 mm/s the firmware drives at roughly PWM 14..31, so a sweep starting at 30
// measures almost nothing the robot actually uses. For a fit aimed squarely at
// that band, send "RANGE,8,60" and "STEP,2" before CAL.
uint16_t sweepMinPWM = 10;
uint16_t sweepMaxPWM = MAX_DRIVE_PWM;
uint16_t sweepStepPWM = 10;
uint16_t settleMs = 400;
uint16_t windowMs = 600;
uint8_t sweepPasses = 2;

uint16_t sweepStepCount() {
  return (sweepMaxPWM - sweepMinPWM) / sweepStepPWM + 1;
}

// A failed fit is the one result an operator cannot act on: the pasted
// "0.000f /*NOT_CALIBRATED*/" looks identical whether the wheel never turned,
// turned the wrong way, or was merely too slow. Separate the three, and show
// the numbers the verdict came from.
static void explainFitFailure(uint8_t index, uint16_t used, uint16_t stalled,
                              float maxAbsSpeed, bool sawBackwards) {
  Serial.print(F("ERR,FIT_FAILED,MOTOR_"));
  printMotorLabel(index);
  Serial.print(F(",used="));
  Serial.print(used);
  Serial.print(F(",stalled="));
  Serial.print(stalled);
  Serial.print(F(",max_abs_mm_s="));
  Serial.println(maxAbsSpeed, 2);

  if (maxAbsSpeed < 1.0f) {
    // Net zero counts with the motor energised -- see commissionWheel() in
    // Direction.cpp for why channel B is the usual suspect.
    Serial.println(F("INFO,CAUSE,NO_ENCODER_MOTION"));
    Serial.println(F("INFO,FIX,CHECK_ENCODER_A_AND_B_WIRING_THEN_RUN,TICKS,20"));
  } else if (sawBackwards) {
    Serial.println(F("INFO,CAUSE,WHEEL_MEASURED_BACKWARDS"));
    Serial.println(F("INFO,FIX,RUN,DIR,IT_SETS_BOTH_SIGNS_FROM_ONE_LOOK_AT_THE_WHEEL"));
  } else {
    Serial.println(F("INFO,CAUSE,TOO_SLOW_OR_TOO_FEW_POINTS"));
    Serial.println(F("INFO,FIX,CHARGE_BATTERY_CHECK_STBY_AND_THAT_THE_WHEEL_SPINS_FREE"));
  }
}

void sweepAndFit(uint8_t index, bool reverse) {
  const uint16_t steps = sweepStepCount();
  if (steps < MIN_SWEEP_STEPS) {
    // Two points define a line exactly and say nothing about whether it is one.
    // Catch it here rather than after a minute of driving.
    Serial.print(F("ERR,TOO_FEW_STEPS,"));
    Serial.print(steps);
    Serial.println(F(",WIDEN_RANGE_OR_SHRINK_STEP"));
    return;
  }

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
  // Largest |speed| seen at ANY point, accepted or not. When every point is
  // rejected this is the only number that separates "the encoder never moved"
  // from "the wheel ran backwards" from "the wheel was just slow".
  float maxAbsSpeed = 0.0f;
  bool sawBackwards = false;
  bool aborted = false;

  for (uint8_t pass = 0; pass < sweepPasses && !aborted; ++pass) {
    // Even passes ramp up, odd passes ramp down. A gearmotor settles at a
    // slightly different speed at the same PWM depending on which side it
    // approached from; averaging both directions of travel fits the middle of
    // that hysteresis loop instead of one edge of it.
    const bool ascending = (pass % 2) == 0;

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

      if (fabs(speed) > maxAbsSpeed) maxAbsSpeed = fabs(speed);

      if (used) {
        fit.add((float)pwm, speed);
      } else {
        ++stalled;
        // A wheel running backwards under a forward command is a sign that was
        // never commissioned, not a slow motor, and no fit can repair it.
        if (speed <= -MIN_VALID_SPEED_MM_S) sawBackwards = true;
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

  if (!out.valid) {
    explainFitFailure(index, out.samples, stalled, maxAbsSpeed, sawBackwards);
    return;
  }
  if (out.slope <= 0.0f) {
    // More PWM produced less speed: whatever was measured, it is not a motor
    // curve, so refuse to hand it back as one.
    out.valid = false;
    Serial.println(F("ERR,FIT_REJECTED,NON_POSITIVE_SLOPE"));
    return;
  }

  printFitRow(index, dirSlot);

  if (sawBackwards) {
    // Some points fitted and some ran backwards: the wheel changed its mind
    // mid-sweep, which a single sign cannot describe.
    Serial.print(F("WARN,SOME_POINTS_RAN_BACKWARDS,MOTOR_"));
    printMotorLabel(index);
    Serial.println(F(",RUN,DIR,BEFORE_TRUSTING_THIS_FIT"));
  }
  if (out.r2 < MIN_ACCEPTABLE_R2) {
    Serial.print(F("WARN,LOW_R2,"));
    Serial.print(out.r2, 4);
    Serial.println(F(",RERUN_WITH_LONGER_DWELL_OR_A_CHARGED_BATTERY"));
  }
}

void calibrateMotor(uint8_t index) {
  sweepAndFit(index, false);
  sleepMs(500);
  sweepAndFit(index, true);
  sleepMs(500);
}
