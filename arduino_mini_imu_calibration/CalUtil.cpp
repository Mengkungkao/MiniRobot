#include "CalUtil.h"

#include <math.h>

#include "ImuDriver.h"

uint16_t gyroSampleCount = 2000;    // 10 s at 5 ms
uint16_t accelSampleCount = 400;    // 2 s per face
uint32_t magDurationMs = 30000UL;

void printAxis(uint8_t axis) {
  Serial.print((char)('X' + axis));
}

void printFace(uint8_t face) {
  Serial.print((face & 1) ? '-' : '+');
  printAxis(face >> 1);
}

bool abortRequested() {
  // Line endings do NOT count as an abort. A terminal set to "Both NL & CR"
  // sends "GYRO\r\n": readSerialCommands() dispatches on the '\r' and leaves
  // the '\n' in the UART buffer, so a naive "any pending byte aborts" test
  // kills the run before its first sample. Drain either way so the stray byte
  // cannot come back as a bogus command afterwards.
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

bool waitForOperator(const __FlashStringHelper *prompt) {
  while (Serial.available() > 0) Serial.read();   // drop whatever is queued

  Serial.print(F("ASK,"));
  Serial.println(prompt);

  // Read one line. The first printable character decides: X cancels, anything
  // else continues, so "GO", "OK" or a bare Enter all mean the same thing.
  char first = '\0';
  for (;;) {
    while (Serial.available() == 0) delay(2);
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (first == '\0') continue;               // empty line: bare Enter
      break;
    }
    if (first == '\0') first = c;
  }

  if (first == 'x' || first == 'X') {
    Serial.println(F("WARN,CANCELLED"));
    return false;
  }
  return true;
}

bool waitUntilStill(uint32_t timeoutMs) {
  const uint32_t start = millis();
  uint32_t lastNudge = 0;
  Stats accel[AXIS_COUNT];
  Stats gyro[AXIS_COUNT];

  while (millis() - start < timeoutMs) {
    // Half a second of samples: long enough to catch a wobble, short enough
    // that the operator is not left waiting after the robot settles.
    if (!sampleAccelGyro(100, accel, gyro)) return false;
    if (!movedDuringRun(accel, gyro)) return true;

    if (lastNudge == 0 || millis() - lastNudge >= 2000) {
      lastNudge = millis();
      Serial.println(F("INFO,WAITING_FOR_THE_ROBOT_TO_BE_STILL"));
    }
  }

  Serial.println(F("ERR,NEVER_WENT_STILL,SET_IT_ON_A_SOLID_SURFACE_AND_LET_GO"));
  return false;
}

bool sampleAccelGyro(uint16_t count, Stats accel[AXIS_COUNT], Stats gyro[AXIS_COUNT]) {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    accel[i].reset();
    gyro[i].reset();
  }

  uint32_t nextSample = millis();
  for (uint16_t s = 0; s < count; ++s) {
    if (abortRequested()) return false;

    float a[AXIS_COUNT];
    float g[AXIS_COUNT];
    imuReadAccelGyro(a, g);
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
      accel[i].add(a[i]);
      gyro[i].add(g[i]);
    }

    // Pace off a running deadline rather than delay(): the I2C read itself
    // costs about a millisecond, which would otherwise stretch the run.
    nextSample += SAMPLE_PERIOD_MS;
    while ((int32_t)(millis() - nextSample) < 0) delay(1);
  }
  return true;
}

// Peak-to-peak is what catches a bump, but a fixed limit on it would also fail
// a perfectly stationary robot carrying a noisier-than-average sensor. Real
// motion pushes the span far outside what the axis's own noise can explain, so
// allow whichever is larger.
static bool axisMoved(const Stats &s, float fixedSpan) {
  const float noiseSpan = 8.0f * s.stddev();
  return s.span() > max(fixedSpan, noiseSpan);
}

bool movedDuringRun(const Stats accel[AXIS_COUNT], const Stats gyro[AXIS_COUNT]) {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (axisMoved(gyro[i], STILL_GYRO_SPAN_DEG_S)) return true;
    if (axisMoved(accel[i], STILL_ACCEL_SPAN_G)) return true;
  }
  return false;
}

float vectorNorm(const float v[AXIS_COUNT]) {
  return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

float angleBetweenDeg(const float a[AXIS_COUNT], const float b[AXIS_COUNT]) {
  const float na = vectorNorm(a);
  const float nb = vectorNorm(b);
  if (na <= 0.0f || nb <= 0.0f) return 0.0f;

  const float dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  return acos(constrain(dot / (na * nb), -1.0f, 1.0f)) * RAD_TO_DEG;
}
