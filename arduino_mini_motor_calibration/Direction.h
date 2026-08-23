#pragma once

#include <Arduino.h>

#include "Config.h"

// -----------------------------------------------------------------------------
// The two commissioning signs, and the one command that can work them out
// -----------------------------------------------------------------------------
//
// MiniRobotGeometry.h declares ENCODER_SIGN and MOTOR_DIRECTION_SIGN as
// compile-time const, shared byte-for-byte with the firmware. That is right for
// a commissioned robot -- but working the values OUT is the commissioning step,
// and re-flashing between guesses turns a two-minute calibration into a
// ten-minute one.
//
// These start as copies of the compiled-in values and are changed for the
// session by DIR. Nothing is persisted: printSigns() prints whatever they ended
// up as, ready to paste into MiniRobotGeometry.h in BOTH sketch folders.
extern int8_t encoderSign[MOTOR_COUNT];
extern int8_t motorSign[MOTOR_COUNT];

// Load the compiled-in values. Call once from setup().
void signsBegin();

// Drive one wheel at a raw TB6612 pin polarity, ignoring both signs. This is
// the hardware as wired, with no interpretation layered on top, and it is what
// DIR probes with.
void driveRawPolarity(uint8_t index, bool pinsHigh, uint16_t pwm);

// Drive one wheel the requested way in the ROBOT frame, with
// MOTOR_DIRECTION_SIGN applied. Everything except DIR's probe uses this.
void driveWheel(uint8_t index, bool reverse, uint16_t pwm);

// Work out both signs for one wheel (or every wheel) from what the operator
// sees. This is the command that makes "the serial says forward" and "the wheel
// turns forward" the same statement.
void commissionDirections(int8_t onlyIndex);

// True when either sign differs from what MiniRobotGeometry.h was compiled with.
bool signsOverridden();

// Current values, plus the paste-ready lines when they differ from the header.
void printSigns();
