#include "Verify.h"

#include <MiniRobotMotors.h>

#include "CalUtil.h"
#include "Config.h"
#include "FitResults.h"

// The same inversion the main firmware performs, using the fit just measured.
static float feedForwardPWM(const FitResult &r, float targetMMs) {
  if (!r.valid || r.slope <= 0.0f) return 0.0f;
  return constrain((targetMMs - r.intercept) / r.slope, 0.0f, (float)MAX_DRIVE_PWM);
}

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

      const uint16_t pwm = (uint16_t)(feedForwardPWM(r, target) + 0.5f);
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
