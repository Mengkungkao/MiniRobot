#include "Sweep.h"

#include <MiniRobotEncoders.h>
#include <MiniRobotMotors.h>

#include "CalUtil.h"
#include "Config.h"
#include "FitResults.h"
#include "LinearFit.h"

// Runtime-adjustable sweep settings, declared extern in Config.h.
uint16_t sweepMinPWM = 30;
uint16_t sweepMaxPWM = MAX_DRIVE_PWM;
uint16_t sweepStepPWM = 10;
uint16_t settleMs = 400;
uint16_t windowMs = 600;
uint8_t sweepPasses = 2;

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

void calibrateMotor(uint8_t index) {
  sweepAndFit(index, false);
  sleepMs(500);
  sweepAndFit(index, true);
  sleepMs(500);
}
