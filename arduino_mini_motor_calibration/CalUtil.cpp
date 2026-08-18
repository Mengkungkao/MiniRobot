#include "CalUtil.h"

#include <MiniRobotEncoders.h>
#include <MiniRobotMotors.h>

#include "Config.h"

void printMotorLabel(uint8_t index) {
  Serial.print(index + 1);
  Serial.print(index == LEFT ? F("(LEFT)") : F("(RIGHT)"));
}

bool abortRequested() {
  if (Serial.available() <= 0) return false;
  while (Serial.available() > 0) Serial.read();
  return true;
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
  motor[index].drive(motorPinsForward(index, reverse), (uint8_t)pwm);

  if (!sleepMs(settleMs)) return NAN;

  readEncoderAndResetAtomic(index);          // discard the spin-up ticks
  const uint32_t windowStart = millis();
  if (!sleepMs(windowMs)) return NAN;
  const int32_t ticks = readEncoderAndResetAtomic(index) * ENCODER_SIGN[index];
  const uint32_t elapsedMs = millis() - windowStart;

  if (elapsedMs == 0) return 0.0f;
  return (ticks * MM_PER_COUNT[index]) * 1000.0f / (float)elapsedMs;
}
