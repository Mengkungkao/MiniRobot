#pragma once

#include <Arduino.h>

#include "Config.h"

// -----------------------------------------------------------------------------
// Session-local overrides for the two commissioning signs
// -----------------------------------------------------------------------------
//
// MiniRobotGeometry.h declares ENCODER_SIGN and MOTOR_DIRECTION_SIGN as
// compile-time const, shared byte-for-byte with the firmware. That is right for
// a commissioned robot -- but working the values OUT is the commissioning step,
// and a wheel whose sign is wrong measures as negative, gets every point
// rejected as a stall, and comes back "0.000f /*NOT_CALIBRATED*/". Re-flashing
// between guesses turns a 150-second calibration into a ten-minute one.
//
// These start as copies of the compiled-in values and can be changed for the
// session with SIGN, so a single CAL run can measure a wheel whose sign was
// worked out thirty seconds earlier. Nothing is persisted: the result table
// prints whatever they ended up as, ready to paste into MiniRobotGeometry.h.
extern int8_t encoderSign[MOTOR_COUNT];
extern int8_t motorSign[MOTOR_COUNT];

// Load the compiled-in values. Call once from setup().
void signsBegin();

// Override-aware replacement for the shared motorPinsForward(), which reads the
// const MOTOR_DIRECTION_SIGN that SIGN cannot touch.
bool calPinsForward(uint8_t index, bool reverse);

// True when either sign differs from what MiniRobotGeometry.h was compiled with.
bool signsOverridden();

// Current values, plus the paste-ready lines when they differ from the header.
void printSigns();
