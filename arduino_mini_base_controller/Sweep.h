#pragma once

#include <Arduino.h>

// Step one wheel through the PWM range and print the steady-state speed at
// each step, so the FF_SLOPE/FF_INTERCEPT constants in Config.h can be fitted
// from real data instead of guessed.
//
// Refuses to run while the emergency stop is latched: clearing it is an
// explicit operator action, never a side effect of asking for a sweep.
//
// This blocks: no commands are read and no telemetry is emitted while it runs
// (a full sweep takes step_count * dwell_ms). Run it from a plain serial
// terminal with the wheels off the floor, not with the ROS 2 stack connected.
void runSweep(uint8_t index, uint8_t step, uint16_t dwellMs, bool reverse);
