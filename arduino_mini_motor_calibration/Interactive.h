#pragma once

#include <Arduino.h>

// Hold one PWM and stream the measured speed, for spot-checking a single point
// or watching how much the speed sags as the battery drains.
void holdAndStream(uint8_t index, uint16_t pwm, bool reverse, uint32_t durationMs);

// Stream raw counts with the motors off, so counts-per-rev can be measured by
// turning a wheel exactly one turn by hand.
void streamTicks(uint32_t durationMs);

// Print the current sweep settings and a run-time estimate.
void printConfig();

// Print the command list.
void printHelp();
