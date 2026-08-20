#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Stats.h"

// "X" / "Y" / "Z", and "+X" / "-X" ... for the six accelerometer faces.
void printAxis(uint8_t axis);
void printFace(uint8_t face);

// True if the operator sent anything -- the abort request during blocking runs.
// The bytes are swallowed so the abort character does not come back as a bogus
// command afterwards.
bool abortRequested();

// delay() that can be cut short by an abort request. Returns false if aborted.
bool sleepMs(uint32_t ms);

// Print a prompt and block until the operator sends a line. Returns false if
// that line starts with X, which is how every prompt is cancelled.
bool waitForOperator(const __FlashStringHelper *prompt);

// Watch the gyro and accelerometer until they go quiet, so a collection window
// does not start while the robot is still rocking on its suspension. Returns
// false on timeout or abort.
bool waitUntilStill(uint32_t timeoutMs);

// Collect 'count' accel/gyro samples at SAMPLE_PERIOD_MS into six accumulators.
// Returns false if the operator aborted; the caller checks the spans to decide
// whether the robot held still.
bool sampleAccelGyro(uint16_t count, Stats accel[AXIS_COUNT], Stats gyro[AXIS_COUNT]);

// True if any accumulator moved more than the stillness thresholds allow.
bool movedDuringRun(const Stats accel[AXIS_COUNT], const Stats gyro[AXIS_COUNT]);

// sqrt(x^2 + y^2 + z^2), used on gravity and on the magnetic field.
float vectorNorm(const float v[AXIS_COUNT]);

// Angle between two vectors in degrees. How the MOTORS test reports field
// distortion: it is the heading error the distortion would cause.
float angleBetweenDeg(const float a[AXIS_COUNT], const float b[AXIS_COUNT]);
