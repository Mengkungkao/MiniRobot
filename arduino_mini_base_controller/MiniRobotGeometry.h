#pragma once

#include <Arduino.h>

#include "MiniRobotPins.h"

// -----------------------------------------------------------------------------
// Wheel geometry and commissioning signs -- THE single copy
// -----------------------------------------------------------------------------
//
// Everything here is measured from the real robot and is shared by the base
// controller and the calibration bench sketch. They MUST agree: the calibration
// sketch reports mm/s scaled by these numbers and the firmware consumes the fit
// scaled by the same numbers, so a mismatch silently bakes a constant error
// into every speed the robot drives.

const float WHEEL_DIAMETER_MM = 47.0f;

// COMMISSIONING VALUE. Encoder counts recorded per full turn of the *wheel*
// (i.e. after the gearbox), measured per wheel.
//
// The sketches attach a CHANGE interrupt to channel A only, so they record two
// counts per encoder cycle (x2 decoding), not one.
//
// THIS ROBOT: a 13 PPR magnetic encoder on the *motor* shaft, ahead of a 1:20
// gearbox, so one wheel revolution is
//     13 PPR x 2 (x2 decoding) x 20 (gear ratio) = 520 counts.
// Note that an encoder on the motor shaft does not automatically land "in the
// thousands": a low-PPR magnetic encoder behind a modest reduction can easily
// read less than an output-shaft optical encoder would.
//
// Measure it, do not assume it. Easiest is the calibration sketch: send CPR
// and it walks you through both wheels -- turn each one a known number of full
// turns by hand, it counts the ticks, divides, and prints this line ready to
// paste back. Or from the base controller: send RESET_ODOM, turn the wheel one
// full turn, and read that wheel's tick field out of the telemetry line.
//
// Per-wheel values (rather than one shared constant) matter because encoder
// undercount on one side makes its measured speed under-report, which makes
// PID drive that wheel physically faster than commanded.
const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {520.0f, 520.0f};

const float MM_PER_COUNT[MOTOR_COUNT] = {
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[0],
  (PI * WHEEL_DIAMETER_MM) / MEASURED_COUNTS_PER_REV[1],
};

// The PWM ceiling both sketches are allowed to use. The feed-forward fit has to
// be measured over the same range the firmware drives over, so this is shared.
const uint8_t MAX_DRIVE_PWM = 210;

// COMMISSIONING VALUE. Forward motion must produce a positive speed reading on
// both wheels. If one wheel reports negative while driving forward, flip its
// sign here rather than swapping the encoder wires.
//
// THIS ROBOT: {1, -1}, and that -1 is independent of the -1 in
// MOTOR_DIRECTION_SIGN below. Two separate hardware facts are both inverted on
// the right channel: the motor is mounted mirrored (corrected below) and its
// encoder counts the opposite way to the wheel it is bolted to (corrected
// here). Setting both is not double-correcting a single fault -- this is the
// pair the robot actually runs straight on, confirmed on blocks with both
// wheels turning together.
//
// Do not "simplify" this to one -1. The two constants answer different
// questions and either can be inverted on its own; on this robot both are.
//
// What telemetry cannot check is whether the pair matches the hardware at all:
// a wheel driven backwards whose encoder also reads backwards reports exactly
// the same healthy positive speed as a correct wheel. That check is visual --
// the top of both wheels must move toward the FRONT of the robot on a forward
// command. A wheel that measures positive while turning the wrong way is a
// MOTOR_DIRECTION_SIGN problem, not an ENCODER_SIGN one.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, 1};

// COMMISSIONING VALUE. If a wheel physically spins backwards when the firmware
// asks for forward, flip its sign here rather than swapping the motor leads.
//
// THIS ROBOT: the two gearmotors are mounted mirrored about the centre line, so
// identical wiring drives them in opposite directions in the robot frame. The
// right motor is the mirrored one, hence -1: the polarity that sends the left
// wheel forward would send the right wheel backwards.
//
// The calibration sketch saw the combined effect of this and the inverted
// encoder above, and reported motor 2 as WHEEL_MEASURED_BACKWARDS through both
// sweeps (arduino_mini_motor_calibration/Calibration_results.md). That report
// says a sign is wrong somewhere in the channel; it cannot say which.
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {-1, -1};
