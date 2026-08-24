#pragma once

#include <Arduino.h>

#include "MiniRobotPins.h"

// -----------------------------------------------------------------------------
// Wheel geometry, body layout and commissioning signs -- THE single copy
// -----------------------------------------------------------------------------
//
// Everything here is measured from the real robot and is shared by the base
// controller and the two bench sketches. They MUST agree: the calibration
// sketch reports mm/s scaled by these numbers and the firmware consumes the fit
// scaled by the same numbers, so a mismatch silently bakes a constant error
// into every speed the robot drives.

const float WHEEL_DIAMETER_MM = 47.0f;

// -----------------------------------------------------------------------------
// Which way the robot faces, and which side each motor is on
// -----------------------------------------------------------------------------
//
// FRONT = THE LiDAR END. The COIN-D6 sits above the caster with its own zero
// mark pointing that way, so base_link +X, LiDAR angle zero, "drive forward"
// and the firmware's positive VEL all mean the same physical direction. The
// drive axle trails. ROS conventions apply throughout (REP-103): +X forward,
// +Y to the port side, +Z up, yaw positive counter-clockwise.
//
// The firmware used to call the opposite end forward and the Pi rotated
// everything by 180 degrees on the way past (reverse_base_direction). That
// indirection is gone: the Arduino and ROS now share one frame, and this file
// is the only place the body layout is stated.
//
// WHEEL_Y_SIGN is that statement. It is the wheel's position along +Y in units
// of half the track, so it is exactly what differential-drive kinematics needs:
//     wheel speed = v - WHEEL_Y_SIGN[i] * omega * TRACK_WIDTH_MM/2
// A wheel at +Y travels slower than the body centre in a counter-clockwise
// turn, which is what the minus sign says.
//
// THIS ROBOT: motor 1 (TB6612 channel A) sits at -Y and motor 2 (channel B) at
// +Y. Nothing else in the firmware needs to know that, and no motor is named
// after a side anywhere -- change these two numbers and the whole stack turns
// the right way.
//
// MEASURED, and measured in the one way that is hard to get wrong. There is no
// way to derive this from telemetry: both assignments produce identical numbers
// on every straight-line test, and differ only in which way the chassis turns.
//
// The reliable procedure is to spin ONE wheel and look at it. A velocity
// command whose differential term exactly cancels one wheel's target does that
// through the normal control path:
//     VEL,25.00,-14.32   ->  motor 1 at +50 mm/s, motor 2 at 0
//     VEL,25.00,+14.32   ->  motor 1 at 0, motor 2 at +50 mm/s
// (the zeroed wheel is below MIN_WHEEL_SPEED_MM_S, so it coasts and genuinely
// does not move). On blocks, the wheel that turned under the first command was
// the one on the right with the LiDAR end pointing away -- so motor 1 is at -Y.
//
// Do NOT settle this by judging which way the robot "tried to spin". That
// judgement was made once on this robot and was wrong, which put the two motors
// on the wrong sides and reversed every turn while every other check passed.
const int8_t WHEEL_Y_SIGN[MOTOR_COUNT] = {-1, +1};

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

// COMMISSIONING VALUE. Driving towards the FRONT (the LiDAR end) must produce a
// positive speed reading on both wheels. If one wheel reports negative while
// the robot drives forward, flip its sign here rather than swapping the encoder
// wires.
//
// THIS ROBOT: {-1, +1}. Motor 1's encoder counts the opposite way to the wheel
// it is bolted to, and that -1 is independent of MOTOR_DIRECTION_SIGN below:
// two separate hardware facts, either of which can be inverted on its own.
//
// These came from a DIR run on the calibration sketch (cal_result.md) and were
// confirmed on blocks: driving forward moves both wheel rims towards the LiDAR
// end and both wheels report positive.
//
// Do NOT "flip both because the front moved". That reasoning is what put this
// pair back to {+1, -1} once, and the robot then drove backwards under a
// positive command while every number still read healthy. The signs are
// measured against whichever end DIR was told was the front, so once they are
// right they stay right.
//
// What telemetry cannot check is whether the pair matches the hardware at all:
// a wheel driven backwards whose encoder also reads backwards reports exactly
// the same healthy positive speed as a correct wheel. That check is visual --
// the top of both wheels must move toward the LiDAR end on a forward command.
// A wheel that measures positive while turning the wrong way is a
// MOTOR_DIRECTION_SIGN problem, not an ENCODER_SIGN one.
const int8_t ENCODER_SIGN[MOTOR_COUNT] = {-1, +1};

// COMMISSIONING VALUE. If a wheel physically spins backwards when the firmware
// asks for forward, flip its sign here rather than swapping the motor leads.
//
// THIS ROBOT: {-1, -1}. The two gearmotors are mounted mirrored about the
// centre line, but their leads are wired mirrored to match, so one TB6612
// polarity drives both wheels the same way in the body frame -- and that
// polarity is the inverted one.
//
// From the same DIR run as ENCODER_SIGN above (cal_result.md), confirmed on
// blocks: a positive VEL moves the top of both wheels towards the LiDAR end.
const int8_t MOTOR_DIRECTION_SIGN[MOTOR_COUNT] = {-1, -1};
