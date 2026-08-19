#include "Signs.h"

#include "MiniRobotGeometry.h"

int8_t encoderSign[MOTOR_COUNT];
int8_t motorSign[MOTOR_COUNT];

void signsBegin() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    encoderSign[i] = ENCODER_SIGN[i];
    motorSign[i] = MOTOR_DIRECTION_SIGN[i];
  }
}

bool calPinsForward(uint8_t index, bool reverse) {
  return (motorSign[index] > 0) ? !reverse : reverse;
}

bool signsOverridden() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (encoderSign[i] != ENCODER_SIGN[i]) return true;
    if (motorSign[i] != MOTOR_DIRECTION_SIGN[i]) return true;
  }
  return false;
}

static void printSignArray(const __FlashStringHelper *name, const int8_t *values) {
  Serial.print(F("const int8_t "));
  Serial.print(name);
  Serial.print(F("[MOTOR_COUNT] = {"));
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (i > 0) Serial.print(F(", "));
    Serial.print(values[i]);
  }
  Serial.println(F("};"));
}

void printSigns() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(F("SIGN,"));
    Serial.print(i + 1);
    Serial.print(F(",ENC,"));
    Serial.print(encoderSign[i]);
    Serial.print(F(",MOT,"));
    Serial.println(motorSign[i]);
  }

  if (!signsOverridden()) return;

  Serial.println(F("INFO,PASTE_INTO_MiniRobotGeometry.h,IN_BOTH_SKETCH_FOLDERS"));
  Serial.println(F("// ---------------------------------------------------------"));
  printSignArray(F("ENCODER_SIGN"), encoderSign);
  printSignArray(F("MOTOR_DIRECTION_SIGN"), motorSign);
  Serial.println(F("// ---------------------------------------------------------"));
}
