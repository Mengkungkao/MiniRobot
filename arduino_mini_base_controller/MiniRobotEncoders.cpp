#include "MiniRobotEncoders.h"

// Written by the ISRs, so every read from normal code has to be atomic.
static volatile int32_t encoderCount[MOTOR_COUNT] = {0, 0};

// Quadrature direction from one channel: on a CHANGE edge of A, A == B means
// one direction and A != B the other. Two counts per encoder cycle.
static void encoderMotor1ISR() {
  if (digitalRead(PIN_ENC_M1_A) == digitalRead(PIN_ENC_M1_B)) encoderCount[MOTOR_1]++;
  else encoderCount[MOTOR_1]--;
}

static void encoderMotor2ISR() {
  if (digitalRead(PIN_ENC_M2_A) == digitalRead(PIN_ENC_M2_B)) encoderCount[MOTOR_2]++;
  else encoderCount[MOTOR_2]--;
}

void encodersBegin() {
  pinMode(PIN_ENC_M1_A, INPUT_PULLUP);
  pinMode(PIN_ENC_M1_B, INPUT_PULLUP);
  pinMode(PIN_ENC_M2_A, INPUT_PULLUP);
  pinMode(PIN_ENC_M2_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_M1_A), encoderMotor1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_M2_A), encoderMotor2ISR, CHANGE);
}

int32_t readEncoderAndResetAtomic(uint8_t index) {
  noInterrupts();
  const int32_t value = encoderCount[index];
  encoderCount[index] = 0;
  interrupts();
  return value;
}
