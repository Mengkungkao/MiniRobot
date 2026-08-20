#pragma once

#include <Arduino.h>

// Measure the gyro's zero-rate offset, its noise, and how much the offset
// moved across the run. Blocking; the robot must be completely still.
// Returns false if the run was aborted or the robot moved.
bool calibrateGyroBias();
