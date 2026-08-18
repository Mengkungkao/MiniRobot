#include "Sweep.h"

#include <MiniRobotEncoders.h>

#include "Config.h"
#include "MotorControl.h"

void runSweep(uint8_t index, uint8_t step, uint16_t dwellMs, bool reverse) {
  stopCommand(false);
  emergencyStopLatched = false;

  Serial.print(F("INFO,SWEEP_START,MOTOR_"));
  Serial.print(index + 1);
  Serial.println(reverse ? F(",REVERSE") : F(",FORWARD"));

  const bool forwardPins = motorPinsForward(index, reverse);
  const float directionSign = reverse ? -1.0f : 1.0f;

  for (uint16_t pwm = SWEEP_MIN_PWM; pwm <= MAX_DRIVE_PWM; pwm += step) {
    motor[index].drive(forwardPins, (uint8_t)pwm);

    // Let the wheel reach steady state, then time a clean measurement window.
    delay(dwellMs / 2);
    readEncoderAndResetAtomic(index);
    const uint32_t windowStart = millis();
    delay(dwellMs / 2);
    const int32_t ticks = readEncoderAndResetAtomic(index) * ENCODER_SIGN[index];
    const float elapsedS = (millis() - windowStart) * 0.001f;

    const float speedMMs = (elapsedS > 0.0f)
                         ? (ticks * MM_PER_COUNT[index]) / elapsedS
                         : 0.0f;

    Serial.print(F("C,"));
    Serial.print(index + 1);
    Serial.print(reverse ? F(",R,") : F(",F,"));
    Serial.print(pwm);
    Serial.print(',');
    // Report the magnitude along the commanded direction, which is what the
    // linear fit is expressed in.
    Serial.println(speedMMs * directionSign, 2);
  }

  motor[index].coast();
  resetEncoderDeltas();
  Serial.println(F("ACK,SWEEP_DONE"));
}
