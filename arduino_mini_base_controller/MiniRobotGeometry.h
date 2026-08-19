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
// Both sketches attach a CHANGE interrupt to channel A only, so they record two
// counts per encoder cycle (x2 decoding), not one. A "360 PPR" encoder on the
// output shaft therefore reads 720 counts per wheel revolution, which is the
// default below. If the encoder sits on the motor shaft instead, the real
// number is that times the gear ratio and will be in the thousands.
//
// Measure it, do not assume it. Easiest is the calibration sketch:
//   TICKS,20    then turn one wheel by hand exactly one full turn.
// Or from the base controller: send RESET_ODOM, turn the wheel one full turn,
// and read that wheel's tick field out of the telemetry line.
//
// Per-wheel values (rather than one shared constant) matter because encoder
// undercount on one side makes its measured speed under-report, which makes
// PID drive that wheel physically faster than commanded.
const float MEASURED_COUNTS_PER_REV[MOTOR_COUNT] = {720.0f, 720.0f};

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
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {1, 1};

// COMMISSIONING VALUE. If a wheel physically spins backwards when the firmware
// asks for forward, flip its sign here rather than swapping the motor leads.
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {1, 1};
