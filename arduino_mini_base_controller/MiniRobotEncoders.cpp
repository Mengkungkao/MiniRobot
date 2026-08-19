#include "MiniRobotEncoders.h"

// Written by the ISRs, so every read from normal code has to be atomic.
static volatile int32_t encoderCount[MOTOR_COUNT] = {0, 0};

// Quadrature direction from one channel: on a CHANGE edge of A, A == B means
// one direction and A != B the other. Two counts per encoder cycle.
static void encoderLeftISR() {
  if (digitalRead(PIN_ENC_LEFT_A) == digitalRead(PIN_ENC_LEFT_B)) encoderCount[LEFT]++;
  else encoderCount[LEFT]--;
}

static void encoderRightISR() {
  if (digitalRead(PIN_ENC_RIGHT_A) == digitalRead(PIN_ENC_RIGHT_B)) encoderCount[RIGHT]++;
  else encoderCount[RIGHT]--;
}

void encodersBegin() {
  pinMode(PIN_ENC_LEFT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_LEFT_B, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT_A), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT_A), encoderRightISR, CHANGE);
}

int32_t readEncoderAndResetAtomic(uint8_t index) {
  noInterrupts();
  const int32_t value = encoderCount[index];
  encoderCount[index] = 0;
  interrupts();
  return value;
}
