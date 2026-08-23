#pragma once

#include <Arduino.h>

#include "Config.h"

// -----------------------------------------------------------------------------
// Counts per wheel revolution -- the first thing to measure, and the scale
// factor under everything else
// -----------------------------------------------------------------------------
//
// Every speed this sketch prints, and every speed the firmware computes, is
// encoder counts scaled by this number. Get it wrong and the whole fit is wrong
// by exactly the same factor, silently: the robot outruns or under-runs its
// command while telemetry, odometry and the PID all agree it is on target.
// Nothing downstream can detect it, which is why CPR runs first.
//
// Like the direction signs, these start as copies of the compiled-in values and
// are replaced for the session by CPR, so one flash can measure the wheels and
// then calibrate them. Nothing is persisted -- printGeometryPasteBlock() prints
// what they ended up as, ready to paste into MiniRobotGeometry.h.
extern float countsPerRev[MOTOR_COUNT];
extern float mmPerCount[MOTOR_COUNT];

// Load the compiled-in values. Call once from setup(), before anything measures.
void countsBegin();

// Hand-spin measurement: ask the operator to turn each wheel a known number of
// full turns, count the ticks, and divide. onlyIndex < 0 does both wheels.
void measureCountsPerRev(int8_t onlyIndex, uint8_t turns);

// Drive each wheel countsPerRev * turns counts under power and brake, so the
// operator can see whether that many counts really is a whole revolution.
// onlyIndex < 0 does both wheels.
void rotateTurns(int8_t onlyIndex, uint8_t turns);

// True when either wheel differs from what MiniRobotGeometry.h was compiled with.
bool countsOverridden();

// One status row per wheel.
void printCounts();

// The paste-ready block for MiniRobotGeometry.h. It carries the counts AND the
// direction signs because they share one header file: pasting half of it leaves
// the robot describing itself with a mix of measured and stale numbers.
void printGeometryPasteBlock();
