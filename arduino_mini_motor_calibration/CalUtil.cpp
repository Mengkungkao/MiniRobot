#include "CalUtil.h"

#include "MiniRobotEncoders.h"
#include "MiniRobotMotors.h"

#include "Config.h"
#include "Signs.h"

void printMotorLabel(uint8_t index) {
  Serial.print(index + 1);
  Serial.print(index == LEFT ? F("(LEFT)") : F("(RIGHT)"));
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
  motor[index].drive(calPinsForward(index, reverse), (uint8_t)pwm);

  if (!sleepMs(settleMs)) return NAN;

  readEncoderAndResetAtomic(index);          // discard the spin-up ticks
  const uint32_t windowStart = millis();
  if (!sleepMs(windowMs)) return NAN;
  const int32_t ticks = readEncoderAndResetAtomic(index) * encoderSign[index];
  const uint32_t elapsedMs = millis() - windowStart;

  if (elapsedMs == 0) return 0.0f;
  return (ticks * MM_PER_COUNT[index]) * 1000.0f / (float)elapsedMs;
}
