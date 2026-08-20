#pragma once

#include <Arduino.h>

// The pin map, wheel geometry and motor helpers live in the shared MiniRobot*
// files, kept byte-identical here, in arduino_mini_base_controller/ and in
// arduino_mini_motor_calibration/. This sketch only needs them for MOTORS,
// which spins the wheels to measure what motor current does to the
// magnetometer. Everything below is specific to the IMU calibration run.
#include "MiniRobotGeometry.h"
#include "MiniRobotPins.h"

// A plain terminal baud rate, not the 500000 the ROS bridge uses. Nothing here
// is parsed by ROS, and 115200 works in every serial monitor.
const uint32_t SERIAL_BAUD = 115200UL;

// -----------------------------------------------------------------------------
// Sampling
// -----------------------------------------------------------------------------

// The MPU is configured for a 200 Hz output rate, so a 5 ms loop takes every
// sample once and never reads the same one twice.
const uint16_t SAMPLE_PERIOD_MS = 5;

// The AK8963 runs at 100 Hz in continuous mode 2.
const uint16_t MAG_SAMPLE_PERIOD_MS = 10;

// Runtime-adjustable (SAMPLES / MAGTIME), defined in CalUtil.cpp.
extern uint16_t gyroSampleCount;    // total, split into two halves
extern uint16_t accelSampleCount;   // per face
extern uint32_t magDurationMs;

// How long waitUntilStill() waits for the robot to stop moving before giving up.
const uint32_t STILL_TIMEOUT_MS = 20000UL;

// -----------------------------------------------------------------------------
// What counts as "still"
// -----------------------------------------------------------------------------

// A hand resting on the robot shows up here long before it shows up in the
// bias. Anything above these spans during a collection window invalidates it.
const float STILL_GYRO_SPAN_DEG_S = 2.0f;
const float STILL_ACCEL_SPAN_G = 0.05f;

// -----------------------------------------------------------------------------
// Accelerometer six-face calibration
// -----------------------------------------------------------------------------

// The axis facing up must read at least this much of a g, and the other two
// must read less than this, or the robot is resting between two faces rather
// than flat on one.
const float ACCEL_FACE_MIN_G = 0.80f;
const float ACCEL_FACE_CROSS_MAX_G = 0.35f;

// A consumer MPU is factory-trimmed to a few percent. Anything past these is
// more likely a bad face measurement than a real sensor error.
const float ACCEL_BIAS_WARN_G = 0.15f;
const float ACCEL_SCALE_WARN = 0.10f;   // |scale - 1|

// -----------------------------------------------------------------------------
// Gyro
// -----------------------------------------------------------------------------

// Zero-rate offset larger than this is out of family for an MPU6500-class part
// and usually means it was moving during the run.
const float GYRO_BIAS_WARN_DEG_S = 8.0f;

// Per-sample standard deviation with the 41 Hz DLPF. Roughly 0.1 deg/s is
// normal; past this something is vibrating or the supply is noisy.
const float GYRO_NOISE_WARN_DEG_S = 0.50f;

// Difference between the first and second half of the run. This is the part of
// the bias that a single startup calibration cannot remove, so it is what
// actually drifts yaw.
const float GYRO_DRIFT_WARN_DEG_S = 0.20f;

// The gyro bias moves while the die warms up. A run started from cold measures
// a bias the sensor has already left behind.
const float GYRO_WARMUP_TEMP_WARN_C = 1.5f;

// -----------------------------------------------------------------------------
// Magnetometer
// -----------------------------------------------------------------------------

// Earth's total field is 25-65 uT depending on where you are. Outside this the
// measurement is dominated by something on the robot, not by the planet.
const float EARTH_FIELD_MIN_UT = 20.0f;
const float EARTH_FIELD_MAX_UT = 75.0f;

// Minimum min-to-max span an axis must see before its hard-iron offset means
// anything. An axis that was never rotated through the field stays flat.
const float MAG_MIN_SPAN_UT = 25.0f;
const uint16_t MAG_MIN_SAMPLES = 300;

// Soft-iron correction is the ratio between the axis spans. Real soft iron on
// a small robot is a few percent; a large ratio is usually incomplete rotation.
const float MAG_SOFT_IRON_WARN = 1.30f;

// After correction every sample should land on a sphere of constant radius.
// The spread of that radius, as a percentage, is the calibration's quality.
const float MAG_RESIDUAL_WARN_PCT = 8.0f;

// Length of the check pass that measures that spread.
const uint32_t MAG_CHECK_MS = 10000UL;

// -----------------------------------------------------------------------------
// MOTORS interference test
// -----------------------------------------------------------------------------

// PWM levels the test drives, and how long it holds each one.
const uint8_t MOTOR_TEST_PWM[] = {80, 140, 200};
const uint8_t MOTOR_TEST_PWM_COUNT = sizeof(MOTOR_TEST_PWM) / sizeof(MOTOR_TEST_PWM[0]);
const uint16_t MOTOR_TEST_HOLD_MS = 2500;

// Angle between the field measured with the motors off and with them running.
// Past this, nine-axis fusion turns heading into a function of throttle.
const float MAG_DEVIATION_WARN_DEG = 5.0f;

// -----------------------------------------------------------------------------
// YAWSIGN
// -----------------------------------------------------------------------------

// How long the operator gets to turn the robot, and how far it must go before
// the integrated angle is big enough to trust the sign of.
const uint32_t YAWSIGN_WINDOW_MS = 10000UL;
const float YAWSIGN_MIN_DEG = 30.0f;

// Axis indices. The whole sketch works in {X, Y, Z} arrays rather than the
// firmware's named scalars, because every stage here does the same thing three
// times.
const uint8_t AXIS_X = 0;
const uint8_t AXIS_Y = 1;
const uint8_t AXIS_Z = 2;
const uint8_t AXIS_COUNT = 3;
