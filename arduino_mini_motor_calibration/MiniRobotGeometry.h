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
// Measure it, do not assume it. Easiest is the calibration sketch:
//   TICKS,20    then turn one wheel by hand exactly one full turn.
// Or from the base controller: send RESET_ODOM, turn the wheel one full turn,
// and read that wheel's tick field out of the telemetry line.
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
// Left at {1, 1} deliberately. The right wheel's backwards reading is a motor
// direction problem, fixed below, and only ONE of the two signs is ever the fix
// for a given wheel. Flipping both cancels out and leaves the worst possible
// state: the wheel drives one way and measures the other, so the PID error
// grows instead of shrinking, the integrator winds to its cap, and that wheel
// sits at full duty no matter how small the command.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {1, 1};

// COMMISSIONING VALUE. If a wheel physically spins backwards when the firmware
// asks for forward, flip its sign here rather than swapping the motor leads.
//
// THIS ROBOT: the two gearmotors are mounted mirrored about the centre line, so
// identical wiring drives them in opposite directions in the robot frame. The
// right motor is the mirrored one, hence -1: the polarity that sends the left
// wheel forward sends the right wheel backwards. The calibration sketch saw the
// consequence of this and reported motor 2 as WHEEL_MEASURED_BACKWARDS through
// both sweeps (arduino_mini_motor_calibration/Calibration_results.md) -- that
// encoder was reporting the truth about a wheel driven the wrong way.
//
// To confirm on the bench: drive slowly forward and LOOK at the wheels. Both
// must carry the robot forward. Telemetry alone cannot tell this apart from a
// flipped encoder, because both faults produce the same negative speed.
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {1, -1};
