#pragma once

#include <Arduino.h>

// Print "<n>(LEFT)" / "<n>(RIGHT)" for operator-facing messages.
void printMotorLabel(uint8_t index);

// True if the operator sent anything -- used as an abort request during the
// blocking sweeps. The bytes are swallowed so the abort character does not come
// back as a bogus command afterwards.
bool abortRequested();

// delay() that can be cut short by an abort request. Returns false if aborted.
bool sleepMs(uint32_t ms);

// Run one wheel at one PWM and return the steady-state speed in mm/s, signed in
// the robot frame (positive = this wheel is driving the robot forward). Returns
// NAN if the operator aborted mid-measurement.
float measureSpeedMMs(uint8_t index, uint16_t pwm, bool reverse);
