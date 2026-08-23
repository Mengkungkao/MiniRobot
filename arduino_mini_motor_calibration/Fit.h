#pragma once

#include <Arduino.h>

#include "Config.h"

// Welford-style accumulation of the means, variances and covariance. The
// textbook sum-of-squares form loses precision fast in 32-bit float once the
// sums grow; this stays well conditioned and needs no sample array, which
// matters on an Uno with 2 KB of SRAM.
struct LinearFit {
  uint16_t n;
  float meanX, meanY;
  float m2x, m2y, cxy;

  void reset() {
    n = 0;
    meanX = meanY = m2x = m2y = cxy = 0.0f;
  }

  void add(float x, float y) {
    ++n;
    const float dx = x - meanX;
    const float dy = y - meanY;
    meanX += dx / (float)n;
    meanY += dy / (float)n;
    m2x += dx * (x - meanX);
    m2y += dy * (y - meanY);
    cxy += dx * (y - meanY);
  }

  // y = slope*x + intercept, plus the coefficient of determination.
  bool solve(float &slope, float &intercept, float &r2) const {
    if (n < 3 || m2x <= 0.0f) return false;
    slope = cxy / m2x;
    intercept = meanY - slope * meanX;
    r2 = (m2y > 0.0f) ? (cxy * cxy) / (m2x * m2y) : 1.0f;
    return true;
  }
};

struct FitResult {
  bool valid;
  float slope;          // mm/s per PWM count
  float intercept;      // mm/s at PWM 0 (negative: the dead zone)
  float r2;
  uint16_t samples;     // points that made it into the fit
};

// results[motor][FWD|REV]
extern FitResult results[MOTOR_COUNT][2];

// Clear every stored fit. Call once from setup().
void resetFitResults();

// One result row: R,motor,dir,slope,intercept,r2,samples,deadzone_pwm,mm_s_at_max
void printFitRow(uint8_t index, uint8_t dirSlot);

// The full table plus the paste-ready constants block.
void printResultTable();
