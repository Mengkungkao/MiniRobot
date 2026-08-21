#include "Interactive.h"

#include "MiniRobotEncoders.h"
#include "MiniRobotMotors.h"

#include "CalUtil.h"
#include "Config.h"
#include "Signs.h"

void holdAndStream(uint8_t index, uint16_t pwm, bool reverse, uint32_t durationMs) {
  Serial.println(F("INFO,HOLD_START,SEND_ANY_CHARACTER_TO_STOP"));
  motor[index].drive(calPinsForward(index, reverse), (uint8_t)pwm);
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

void runDirectionCheck() {
  // Halfway up the range: well clear of any dead zone, well short of the top,
  // and the same measurement path the sweep itself uses.
  const uint16_t probePWM = (uint16_t)constrain((int)MAX_DRIVE_PWM / 2, 40, (int)MAX_DRIVE_PWM);

  Serial.println(F("INFO,DIRECTION_CHECK,BOTH_WHEELS_MUST_SPIN_FREE"));
  Serial.println(F("CHECK,motor,dir,pwm,measured_mm_s,verdict"));

  bool sawNoMotion = false;
  bool sawBackwards = false;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    const float speed = measureSpeedMMs(i, probePWM, false);
    motor[i].coast();
    if (isnan(speed)) {
      motorsCoastAll();
      Serial.println(F("WARN,CHECK_ABORTED"));
      return;
    }

    Serial.print(F("CHECK,"));
    Serial.print(i + 1);
    Serial.print(F(",F,"));
    Serial.print(probePWM);
    Serial.print(',');
    Serial.print(speed, 2);
    Serial.print(',');
    if (fabs(speed) < MIN_VALID_SPEED_MM_S) {
      sawNoMotion = true;
      Serial.println(F("NO_ENCODER_MOTION"));
    } else if (speed < 0.0f) {
      sawBackwards = true;
      Serial.println(F("BACKWARDS"));
    } else {
      Serial.println(F("OK"));
    }

    if (!sleepMs(400)) {
      motorsCoastAll();
      Serial.println(F("WARN,CHECK_ABORTED"));
      return;
    }
  }

  motorsCoastAll();

  if (sawNoMotion) {
    Serial.println(F("INFO,NO_ENCODER_MOTION,THE_WHEEL_MAY_SPIN_BUT_NOTHING_COUNTS"));
    Serial.println(F("INFO,USUALLY_CHANNEL_B_FLOATING_OR_ON_THE_WRONG_PIN,LEFT_B=D6_RIGHT_B=D11"));
    Serial.println(F("INFO,CONFIRM_BY_HAND_WITH,TICKS,20"));
  }

  if (sawBackwards) {
    // BACKWARDS has two causes that look identical from here: the motor spins
    // the wrong way, or the encoder counts the wrong way. Only the operator can
    // tell them apart, by looking at the wheel -- and picking the wrong one is
    // worse than leaving it, because flipping ENCODER_SIGN on a motor that is
    // genuinely wired backwards makes the fit succeed and the PID agree with
    // itself while the robot turns instead of driving straight.
    Serial.println(F("INFO,BACKWARDS_HAS_TWO_CAUSES,LOOK_AT_THE_WHEEL_TO_TELL_THEM_APART"));
    Serial.println(F("INFO,RUN,HOLD,<motor>,105,AND_WATCH_WHICH_WAY_IT_ROLLS"));
    Serial.println(F("INFO,ROLLS_ROBOT_BACKWARD,THE_MOTOR_IS_REVERSED,RUN,SIGN,<motor>,MOT,-1"));
    Serial.println(F("INFO,ROLLS_ROBOT_FORWARD,THE_ENCODER_IS_REVERSED,RUN,SIGN,<motor>,ENC,-1"));
    Serial.println(F("INFO,THEN_RERUN,CHECK,AND_CAL,NO_RE_FLASH_NEEDED"));
  }

  Serial.println(F("ACK,CHECK_DONE"));
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

void printConfig() {
  // Geometry and signs first, because they decide what every number below
  // MEANS, and a bench sketch flashed from a stale checkout is otherwise
  // indistinguishable from a current one.
  //
  // This robot lost a calibration run to exactly that. The firmware carried the
  // corrected ENCODER_SIGN for the right wheel and this sketch, flashed
  // earlier, did not -- so a sweep labelled FORWARD drove the wheel one way
  // here and the other way there, and every motor 2 point came back rejected
  // with no hint that the two binaries disagreed. Printing them at boot costs
  // a few hundred bytes of a sketch that is half empty and turns a silent
  // wrong answer into a visible one.
  Serial.print(F("INFO,GEOMETRY,counts="));
  Serial.print(MEASURED_COUNTS_PER_REV[LEFT], 1);
  Serial.print('/');
  Serial.print(MEASURED_COUNTS_PER_REV[RIGHT], 1);
  Serial.print(F(",dia="));
  Serial.print(WHEEL_DIAMETER_MM, 2);
  Serial.print(F(",maxpwm="));
  Serial.println((int)MAX_DRIVE_PWM);
  printSigns();
  Serial.println(F("INFO,COMPARE_WITH,CFG,ON_arduino_mini_base_controller,THEY_MUST_AGREE"));

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
  Serial.println(F("CMD,CHECK                  pulse each wheel, report encoder sign"));
  Serial.println(F("CMD,SIGN                   print the commissioning signs"));
  Serial.println(F("CMD,SIGN,motor,ENC|MOT,1|-1  set one for this session"));
  Serial.println(F("CMD,TICKS[,seconds]        stream raw encoder counts"));
  Serial.println(F("CMD,STEP,pwm_step          set sweep step"));
  Serial.println(F("CMD,RANGE,min_pwm,max_pwm  set sweep range"));
  Serial.println(F("CMD,DWELL,settle_ms,win_ms set per-step timing"));
  Serial.println(F("CMD,PASSES,1..4            set passes per direction"));
  Serial.println(F("CMD,CONFIG                 print settings and run-time estimate"));
  Serial.println(F("CMD,STOP                   coast both motors"));
  Serial.println(F("INFO,ANY_CHARACTER_ABORTS_A_RUNNING_SWEEP"));
}
