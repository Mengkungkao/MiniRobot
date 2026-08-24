#include "Measure.h"

#include "MiniRobotEncoders.h"
#include "MiniRobotMotors.h"

#include "Config.h"
#include "Counts.h"
#include "Direction.h"

void printMotorLabel(uint8_t index) {
  // Motor 1 is TB6612 channel A and motor 2 is channel B; neither is named
  // after a side of the robot. Which side each drives is WHEEL_Y_SIGN's job
  // (MiniRobotGeometry.h) and does not belong in a bench sketch's prompts.
  Serial.print(index + 1);
  Serial.print(index == MOTOR_1 ? F("(CH_A)") : F("(CH_B)"));
}

bool abortRequested() {
  // Line endings do NOT count as an abort. A terminal set to "Both NL & CR"
  // sends "CAL\r\n": readSerialCommands() dispatches on the '\r' and leaves the
  // '\n' sitting in the UART buffer, so a naive "any pending byte aborts" test
  // kills the sweep before its first measurement. Drain the buffer either way
  // so the stray byte cannot come back as a bogus command afterwards.
  bool requested = false;
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c != '\n' && c != '\r') requested = true;
  }
  return requested;
}

bool sleepMs(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) {
    if (abortRequested()) return false;
    delay(2);
  }
  return true;
}

float measureSpeedMMs(uint8_t index, uint16_t pwm, bool reverse) {
  driveWheel(index, reverse, pwm);

  if (!sleepMs(settleMs)) return NAN;

  readEncoderAndResetAtomic(index);          // discard the spin-up ticks
  const uint32_t windowStart = millis();
  if (!sleepMs(windowMs)) return NAN;
  const int32_t ticks = readEncoderAndResetAtomic(index) * encoderSign[index];
  const uint32_t elapsedMs = millis() - windowStart;

  if (elapsedMs == 0) return 0.0f;
  return (ticks * mmPerCount[index]) * 1000.0f / (float)elapsedMs;
}

int32_t measureRawTicks(uint8_t index, bool pinsHigh, uint16_t pwm) {
  driveRawPolarity(index, pinsHigh, pwm);
  delay(DIR_SETTLE_MS);
  readEncoderAndResetAtomic(index);          // discard the spin-up ticks
  delay(DIR_WINDOW_MS);
  const int32_t ticks = readEncoderAndResetAtomic(index);
  motor[index].coast();
  return ticks;
}
