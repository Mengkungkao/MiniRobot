#pragma once

#include <Arduino.h>

// Hold one PWM and stream the measured speed, for spot-checking a single point
// or watching how much the speed sags as the battery drains.
void holdAndStream(uint8_t index, uint16_t pwm, bool reverse, uint32_t durationMs);

// Stream raw counts with the motors off, so counts-per-rev can be measured by
// turning a wheel exactly one turn by hand.
void streamTicks(uint32_t durationMs);

// Print the current sweep settings and a run-time estimate.
// Pulse every wheel forward for one measurement window and report the sign and
// magnitude the encoder actually returned. Four seconds instead of the 152 a
// full CAL takes, and it answers the only question a failed fit leaves open:
// did the wheel not move, move backwards, or move too slowly?
void runDirectionCheck();

void printConfig();

// Print the command list.
void printHelp();
