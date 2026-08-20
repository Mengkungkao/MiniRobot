#pragma once

#include <Arduino.h>
#include <math.h>

// Welford-style running mean and variance, plus the min and max. Same reason as
// LinearFit.h in the motor calibration sketch: the textbook sum-of-squares form
// loses precision fast in 32-bit float, and there is no room on an Uno to keep
// the samples around -- a 30 second magnetometer sweep is 3000 of them.
struct Stats {
  uint16_t n;
  float mean;
  float m2;
  float minValue;
  float maxValue;

  void reset() {
    n = 0;
    mean = 0.0f;
    m2 = 0.0f;
    minValue = 0.0f;
    maxValue = 0.0f;
  }

  void add(float x) {
    if (n == 0 || x < minValue) minValue = x;
    if (n == 0 || x > maxValue) maxValue = x;
    ++n;
    const float delta = x - mean;
    mean += delta / (float)n;
    m2 += delta * (x - mean);
  }

  float stddev() const {
    return (n > 1) ? sqrt(m2 / (float)(n - 1)) : 0.0f;
  }

  // Peak to peak. What the stillness checks actually care about: a single bump
  // moves this immediately while it barely moves the standard deviation.
  float span() const {
    return (n > 0) ? (maxValue - minValue) : 0.0f;
  }

  // Midpoint of the extremes -- the hard-iron offset of a magnetometer axis
  // that has been rotated through the whole field.
  float centre() const {
    return (n > 0) ? ((maxValue + minValue) * 0.5f) : 0.0f;
  }
};
