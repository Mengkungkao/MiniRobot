#pragma once

#include <Arduino.h>

#include "Config.h"

struct FitResult {
  bool valid;
  float slope;          // mm/s per PWM count
  float intercept;      // mm/s at PWM 0 (negative: the dead zone)
  float r2;
  uint16_t samples;     // points that made it into the fit
  uint16_t stalled;     // points rejected as below MIN_VALID_SPEED_MM_S
  float maxSpeedMMs;    // fastest speed actually measured
  bool signWarning;     // wheel turned the opposite way to the command
};

// results[motor][FWD|REV]
extern FitResult results[MOTOR_COUNT][2];

// Clear every stored fit. Call once from setup().
void resetFitResults();

// One result row: R,motor,dir,slope,intercept,r2,samples,deadzone_pwm,mm_s_at_max
void printFitRow(uint8_t index, uint8_t dirSlot);

// The full table plus the paste-ready constants block.
void printResultTable();
