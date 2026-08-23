#include "Measure.h"

#include "MiniRobotEncoders.h"
#include "MiniRobotMotors.h"

#include "Config.h"
#include "Direction.h"

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
  driveWheel(index, reverse, pwm);

  if (!sleepMs(settleMs)) return NAN;

  readEncoderAndResetAtomic(index);          // discard the spin-up ticks
  const uint32_t windowStart = millis();
  if (!sleepMs(windowMs)) return NAN;
  const int32_t ticks = readEncoderAndResetAtomic(index) * encoderSign[index];
  const uint32_t elapsedMs = millis() - windowStart;

  if (elapsedMs == 0) return 0.0f;
  return (ticks * MM_PER_COUNT[index]) * 1000.0f / (float)elapsedMs;
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

void holdAndStream(uint8_t index, uint16_t pwm, bool reverse, uint32_t durationMs) {
  Serial.println(F("INFO,HOLD_START,SEND_ANY_CHARACTER_TO_STOP"));
  driveWheel(index, reverse, pwm);
  readEncoderAndResetAtomic(index);

  const uint32_t start = millis();
  uint32_t lastSample = millis();
  while (millis() - start < durationMs) {
    if (abortRequested()) break;
    delay(5);
    if (millis() - lastSample < 200) continue;

    const int32_t ticks = readEncoderAndResetAtomic(index) * encoderSign[index];
    const uint32_t elapsedMs = millis() - lastSample;
    lastSample = millis();

    const float speed = (ticks * MM_PER_COUNT[index]) * 1000.0f / (float)elapsedMs;
    Serial.print(F("C,"));
    Serial.print(index + 1);
    Serial.print(reverse ? F(",R,") : F(",F,"));
    Serial.print(pwm);
    Serial.print(',');
    Serial.print(speed * (reverse ? -1.0f : 1.0f), 2);
    Serial.println(F(",HOLD"));
  }

  motor[index].coast();
  readEncoderAndResetAtomic(index);
  Serial.println(F("ACK,HOLD_DONE"));
}

// Every mm/s in this sketch and in the firmware is scaled by counts-per-rev, so
// it is the first thing to get right.
void streamTicks(uint32_t durationMs) {
  motorsCoastAll();
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) readEncoderAndResetAtomic(i);

  int32_t total[MOTOR_COUNT] = {0, 0};
  Serial.println(F("INFO,TURN_ONE_WHEEL_EXACTLY_ONE_FULL_TURN_BY_HAND"));
  Serial.println(F("INFO,PUT_THE_MAGNITUDE_IN_MEASURED_COUNTS_PER_REV"));

  const uint32_t start = millis();
  uint32_t lastPrint = 0;
  while (millis() - start < durationMs) {
    if (abortRequested()) break;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      total[i] += readEncoderAndResetAtomic(i) * encoderSign[i];
    }
    if (millis() - lastPrint >= 250) {
      lastPrint = millis();
      Serial.print(F("TICKS,"));
      Serial.print(total[LEFT]);
      Serial.print(',');
      Serial.println(total[RIGHT]);
    }
    delay(5);
  }
  Serial.println(F("ACK,TICKS_DONE"));
}
