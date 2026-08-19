#pragma once

#include <Arduino.h>

// Pin map, wheel geometry and the commissioning signs live in the shared
// MiniRobot* files, which are kept byte-identical in this folder and in
// arduino_mini_motor_calibration/. Edit one, copy it to the other -- the two
// sketches MUST agree about what an encoder tick is worth. Everything below is
// specific to the base controller.
#include "MiniRobotGeometry.h"
#include "MiniRobotPins.h"

// -----------------------------------------------------------------------------
// Timing
// -----------------------------------------------------------------------------

const uint32_t SERIAL_BAUD = 500000UL;
const uint16_t CONTROL_PERIOD_MS = 10;      // 100 Hz motor control
const uint16_t TELEMETRY_PERIOD_MS = 50;    // 20 Hz telemetry
const uint16_t COMMAND_WATCHDOG_MS = 500;   // stop if Pi heartbeat disappears

// -----------------------------------------------------------------------------
// Robot geometry specific to driving (the shared library holds the wheel/encoder
// geometry that the calibration sketch also needs)
// -----------------------------------------------------------------------------

// COMMISSIONING VALUE. Distance between the two wheel contact patches.
// Command an in-place 360 degree turn and correct this if the encoder-derived
// rotation does not match the physical robot.
const float TRACK_WIDTH_MM = 200.0f;

// Limits should stay conservative until the robot has been tested on blocks.
const float MAX_LINEAR_MM_S = 250.0f;
const float MAX_ANGULAR_DEG_S = 120.0f;
const float MAX_WHEEL_MM_S = 250.0f;
const float LINEAR_ACCEL_MM_S2 = 150.0f;
const float ANGULAR_ACCEL_DEG_S2 = 120.0f;

// -----------------------------------------------------------------------------
// Feed-forward calibration
// -----------------------------------------------------------------------------

// COMMISSIONING VALUES. Per-wheel, per-direction open-loop PWM-to-speed linear
// fit (|measured_mm_s| = slope*pwm + intercept), inverted in
// speedFeedForwardPWM() so PID only has to correct the residual.
//
// The defaults below are a generic straight line through a PWM ~40 dead zone
// and ~400 mm/s at full PWM. They are a starting point, NOT a calibration:
// brushed gearmotors differ from each other by tens of percent and are
// direction-asymmetric, which shows up on the robot as driving straight
// forward but curving in reverse. Replace them with real numbers.
//
// Easiest: upload arduino_mini_motor_calibration/, send CAL, and paste the four
// lines it prints back over the four below -- it sweeps both wheels in both
// directions, fits the lines on-board and checks them with VERIFY.
//
// Or sweep from here and fit the points yourself:
//
//   SWEEP,1,10,1000       left wheel, forward, PWM steps of 10, 1 s each
//   SWEEP,1,10,1000,R     left wheel, reverse
//   SWEEP,2,10,1000       right wheel, forward
//   SWEEP,2,10,1000,R     right wheel, reverse
//
// Each step prints "C,<motor>,<F|R>,<pwm>,<measured_mm_s>". Drop the stalled
// points at the bottom of the sweep, least-squares fit the rest, and paste the
// slope/intercept here.
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {1.860f, 1.860f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-74.4f, -74.4f};
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {1.860f, 1.860f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-74.4f, -74.4f};

// Lowest PWM the SWEEP calibration command starts from.
const uint8_t SWEEP_MIN_PWM = 30;

// -----------------------------------------------------------------------------
// IMU
// -----------------------------------------------------------------------------

// COMMISSIONING VALUE. A physical left turn must make ROS yaw increase
// (REP-103, CCW-positive). Rotate the robot left by hand and watch the yaw
// field in telemetry; if it decreases, set this to -1.0f.
const float IMU_YAW_SIGN = 1.0f;
