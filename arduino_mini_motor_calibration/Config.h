#pragma once

#include <Arduino.h>

// Pin map, wheel geometry and the commissioning signs come from the shared
// MiniRobotBase library, so this bench tool measures the robot with exactly the
// same numbers the firmware drives it with. Everything below is specific to the
// calibration run itself.
#include <MiniRobotGeometry.h>
#include <MiniRobotPins.h>

// A plain terminal baud rate, not the 500000 the ROS bridge uses. Nothing here
// is parsed by ROS, and 115200 works in every serial monitor.
const uint32_t SERIAL_BAUD = 115200UL;

// -----------------------------------------------------------------------------
// Calibration behaviour
// -----------------------------------------------------------------------------

// Points slower than this are stiction/dead-zone, where speed is not linear in
// PWM. They are printed as STALL and excluded from the fit -- including them
// would drag the line down and make the firmware under-drive at every speed.
const float MIN_VALID_SPEED_MM_S = 15.0f;

// Below this the fit is not trustworthy enough to paste (loose gearbox, a
// dragging wheel, or a battery that sagged during the sweep).
const float MIN_ACCEPTABLE_R2 = 0.97f;

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

// Direction slots used to index the results table.
const uint8_t FWD = 0;
const uint8_t REV = 1;
