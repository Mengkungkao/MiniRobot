#pragma once

#include <Arduino.h>

// Pin map, wheel geometry and the commissioning signs live in the shared
// MiniRobot* files, kept byte-identical here and in
// arduino_mini_base_controller/, so this bench tool measures the robot with
// exactly the same numbers the firmware drives it with. Everything below is
// specific to the calibration run itself.
#include "MiniRobotGeometry.h"
#include "MiniRobotPins.h"

// The same baud the base controller uses, so one serial-monitor profile covers
// both and a port probe does not have to guess. Nothing here is parsed by ROS.
// The CH340 on this robot handles 500000 fine; drop to 115200 if a terminal
// cannot open it. The IMU calibration sketch is still 115200.
const uint32_t SERIAL_BAUD = 500000UL;

// -----------------------------------------------------------------------------
// Fit acceptance
// -----------------------------------------------------------------------------

// Points slower than this are stiction/dead-zone, where speed is not linear in
// PWM. They are printed as STALL and excluded from the fit -- including them
// would drag the line down and make the firmware under-drive at every speed.
const float MIN_VALID_SPEED_MM_S = 15.0f;

// Below this the fit is not trustworthy enough to paste (loose gearbox, a
// dragging wheel, or a battery that sagged during the sweep).
const float MIN_ACCEPTABLE_R2 = 0.97f;

// -----------------------------------------------------------------------------
// DIR -- direction commissioning
// -----------------------------------------------------------------------------
//
// DIR spins one wheel at a fixed pin polarity, asks the operator which way the
// robot tried to roll, and derives BOTH signs from that one answer. See
// Direction.cpp for why that is the only way to get them right.

// Slow enough that the direction of rotation is obvious to the eye -- roughly
// two thirds of a wheel turn per second on this robot -- and still five times
// the measured break-away PWM. A probe fast enough to blur is a probe the
// operator answers by guessing.
const uint16_t DIR_PROBE_PWM = 40;

// Retry PWM for a wheel that did not break away at DIR_PROBE_PWM: a stiff
// gearbox and a dead encoder look identical at one PWM and quite different at
// two.
const uint16_t DIR_PROBE_PWM_RETRY = 90;

const uint16_t DIR_SETTLE_MS = 300;    // let it reach a steady speed
const uint16_t DIR_WINDOW_MS = 1200;   // then hold it long enough to be watched

// Counts below this over a whole DIR window mean the encoder is not reporting
// rotation, not that the wheel turned slowly: the probe above produces several
// hundred.
const int32_t DIR_MIN_TICKS = 20;

// How long DIR waits for the operator to answer before giving up and coasting.
const uint32_t ANSWER_TIMEOUT_MS = 120000UL;

// -----------------------------------------------------------------------------
// VERIFY
// -----------------------------------------------------------------------------

// Target speeds VERIFY drives, in mm/s.
const float VERIFY_SPEEDS_MM_S[] = {60.0f, 120.0f, 180.0f, 240.0f};
const uint8_t VERIFY_SPEED_COUNT =
  sizeof(VERIFY_SPEEDS_MM_S) / sizeof(VERIFY_SPEEDS_MM_S[0]);

// Safety ceiling for HOLD, which otherwise runs a motor indefinitely.
const uint32_t HOLD_MAX_MS = 20000UL;

// -----------------------------------------------------------------------------
// Runtime-adjustable sweep settings (STEP / RANGE / DWELL / PASSES)
// -----------------------------------------------------------------------------
// Defined in Sweep.cpp.

extern uint16_t sweepMinPWM;
extern uint16_t sweepMaxPWM;
extern uint16_t sweepStepPWM;
extern uint16_t settleMs;      // let the wheel reach steady state
extern uint16_t windowMs;      // then count ticks for this long
extern uint8_t sweepPasses;    // 2 = ascending then descending

// A line needs more than two points to mean anything, and both passes visit the
// same PWM values, so a sweep with fewer distinct steps than this is refused
// before it wastes a minute.
const uint16_t MIN_SWEEP_STEPS = 4;

// Direction slots used to index the results table.
const uint8_t FWD = 0;
const uint8_t REV = 1;
