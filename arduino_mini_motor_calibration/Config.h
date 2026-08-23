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
// CPR -- counts per wheel revolution, measured by hand
// -----------------------------------------------------------------------------

// Full turns of the wheel the operator is asked for. The error is dominated by
// how precisely they stop on the mark, which is a fixed number of counts
// however far the wheel went, so more turns divides it down.
const uint8_t CPR_DEFAULT_TURNS = 5;
const uint8_t CPR_MAX_TURNS = 20;

// Fewer counts than this over several hand turns means the encoder is not
// reporting rotation at all -- a full turn on this robot is in the hundreds.
const int32_t CPR_MIN_TICKS = 50;

// Turning a wheel by hand and stopping on a mark is not a hurried job.
const uint32_t CPR_TIMEOUT_MS = 300000UL;

// -----------------------------------------------------------------------------
// ROT -- play the counts back under power
// -----------------------------------------------------------------------------
//
// Drives a wheel exactly countsPerRev * turns counts and brakes, so the
// operator can see whether that many counts really is a whole revolution.

// Main travel, then a crawl for the last stretch so the brake lands close to
// the target instead of sailing past it. The wheel is already turning by the
// time it drops to the crawl, so the low PWM only has to sustain motion, not
// break stiction.
const uint16_t ROT_PWM = 50;
const uint16_t ROT_CREEP_PWM = 15;

// Fraction of one revolution spent crawling.
const float ROT_CREEP_FRACTION = 0.15f;

// Let the brake finish before reading the final count.
const uint16_t ROT_SETTLE_MS = 400;

// No new counts for this long means the wheel is jammed or the encoder is dead.
// Far more useful than a fixed overall timeout, which would sit there for
// twenty seconds saying nothing.
const uint32_t ROT_STALL_MS = 2000UL;

const uint8_t ROT_MAX_TURNS = 20;

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

// -----------------------------------------------------------------------------
// Direction -- and the four different things that word means in this sketch
// -----------------------------------------------------------------------------
//
// Four encodings of "direction" coexist here. They are easy to confuse because
// three of them are numbers and two of those are +1/-1:
//
//   1. bool reverse        WHAT WAS ASKED FOR. false = forward, true = reverse.
//                          Travels down through driveWheel() and
//                          measureSpeedMMs() to the driver pins.
//
//   2. uint8_t dirSlot     WHICH RESULTS SLOT. FWD = 0, REV = 1. Never
//                          arithmetic, never a sign -- purely an index into
//                          results[motor][DIR_SLOT_COUNT].
//
//   3. sign of a speed     WHICH WAY THE ROBOT ACTUALLY WENT, in the robot
//                          frame: positive is forward no matter what was
//                          commanded. measureSpeedMMs() returns this, and
//                          sweepAndFit() multiplies it by its own +1/-1
//                          directionSign to get a magnitude along the command.
//
//   4. int8_t motorSign    NOT DIRECTIONS AT ALL. These are per-robot wiring
//      int8_t encoderSign  corrections, fixed for a given machine, and they are
//                          +1/-1 like (3) purely by coincidence. Multiplying a
//                          speed by one of these is meaningful; multiplying a
//                          dirSlot by one is not. See Direction.h.
//
// (2) is laid out so that dirSlot == (uint8_t)reverse, which makes converting
// between (1) and (2) free. That is an invariant the results array leans on,
// not a coincidence to rely on quietly, so it is asserted below and every
// conversion goes through the helpers rather than being written out by hand.
const uint8_t FWD = 0;
const uint8_t REV = 1;
const uint8_t DIR_SLOT_COUNT = 2;

static_assert(FWD == 0 && REV == 1,
              "dirSlot indexes results[][DIR_SLOT_COUNT] as {FWD, REV}");
static_assert(DIR_SLOT_COUNT == REV + 1,
              "one results slot per direction, no more and no fewer");
static_assert(FWD == (uint8_t)false && REV == (uint8_t)true,
              "dirSlotFor() and the results array rely on the slots lining up "
              "with bool reverse");

// bool reverse -> results slot.
inline uint8_t dirSlotFor(bool reverse) { return reverse ? REV : FWD; }

// results slot -> was that the reverse sweep? The single predicate behind every
// F/R letter this sketch prints, so a row cannot be labelled one way and stored
// the other.
inline bool slotIsReverse(uint8_t dirSlot) { return dirSlot == REV; }
