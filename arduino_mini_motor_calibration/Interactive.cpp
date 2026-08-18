#include "Interactive.h"

#include <MiniRobotEncoders.h>
#include <MiniRobotMotors.h>

#include "CalUtil.h"
#include "Config.h"

void holdAndStream(uint8_t index, uint16_t pwm, bool reverse, uint32_t durationMs) {
  Serial.println(F("INFO,HOLD_START,SEND_ANY_CHARACTER_TO_STOP"));
  motor[index].drive(motorPinsForward(index, reverse), (uint8_t)pwm);
  readEncoderAndResetAtomic(index);

  const uint32_t start = millis();
  uint32_t lastSample = millis();
  while (millis() - start < durationMs) {
    if (abortRequested()) break;
    delay(5);
    if (millis() - lastSample < 200) continue;

    const int32_t ticks = readEncoderAndResetAtomic(index) * ENCODER_SIGN[index];
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
      total[i] += readEncoderAndResetAtomic(i) * ENCODER_SIGN[i];
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

void printConfig() {
  Serial.print(F("INFO,CONFIG,range="));
  Serial.print(sweepMinPWM);
  Serial.print(F(".."));
  Serial.print(sweepMaxPWM);
  Serial.print(F(",step="));
  Serial.print(sweepStepPWM);
  Serial.print(F(",settle_ms="));
  Serial.print(settleMs);
  Serial.print(F(",window_ms="));
  Serial.print(windowMs);
  Serial.print(F(",passes="));
  Serial.println(sweepPasses);

  const uint16_t steps = (sweepMaxPWM - sweepMinPWM) / sweepStepPWM + 1;
  const uint32_t perDirectionS =
    ((uint32_t)steps * sweepPasses * (uint32_t)(settleMs + windowMs)) / 1000UL;
  Serial.print(F("INFO,ESTIMATED_SECONDS,per_direction="));
  Serial.print(perDirectionS);
  Serial.print(F(",full_CAL="));
  Serial.println(perDirectionS * 4);
}

void printHelp() {
  Serial.println(F("CMD,CAL[,motor]            calibrate (default: both motors, both directions)"));
  Serial.println(F("CMD,SWEEP,motor[,R]        one motor, one direction"));
  Serial.println(F("CMD,VERIFY[,motor]         open-loop check of the fitted constants"));
  Serial.println(F("CMD,RESULT                 reprint fits and the paste block"));
  Serial.println(F("CMD,HOLD,motor,pwm[,R]     hold one PWM, stream speed"));
  Serial.println(F("CMD,TICKS[,seconds]        stream raw encoder counts"));
  Serial.println(F("CMD,STEP,pwm_step          set sweep step"));
  Serial.println(F("CMD,RANGE,min_pwm,max_pwm  set sweep range"));
  Serial.println(F("CMD,DWELL,settle_ms,win_ms set per-step timing"));
  Serial.println(F("CMD,PASSES,1..4            set passes per direction"));
  Serial.println(F("CMD,CONFIG                 print settings and run-time estimate"));
  Serial.println(F("CMD,STOP                   coast both motors"));
  Serial.println(F("INFO,ANY_CHARACTER_ABORTS_A_RUNNING_SWEEP"));
}
