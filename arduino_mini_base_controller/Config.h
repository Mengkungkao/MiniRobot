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
//
// Capped for mapping. The LiDAR sweeps at 10 Hz, so whatever the robot does in
// 100 ms is smeared across a single scan: at 150 mm/s that is 15 mm of
// translation, small against the map resolution, and slam_toolbox can match
// consecutive scans instead of guessing. The band the robot is driven in is
// 50..150 mm/s -- 50 mm/s is the slow end Nav2 approaches goals at, and below
// that the wheels are into their breakaway zone where the feed-forward fit
// stops describing them.
//
// These three are one set and have to move together. In-place rotation puts
// TRACK_WIDTH_MM/2 * omega on each wheel, so the angular limit is whatever
// keeps that inside MAX_WHEEL_MM_S:
//     150 mm/s / 100 mm = 1.5 rad/s = 86 deg/s
// Raising MAX_ANGULAR_DEG_S past that does not turn faster, it just clips at
// MAX_WHEEL_MM_S and quietly makes the turn slower than commanded.
//
// The ROS side must agree: max_linear_m_s / max_angular_rad_s in
// ros2_ws/src/mini_robot/config/base.yaml, and kMaxLinear / kMaxAngular in
// mini_rviz_plugins/src/teleop_panel.cpp. This clamp is the authoritative one;
// the others exist so the operator is not offered speeds the base will refuse.
const float MAX_LINEAR_MM_S = 150.0f;
const float MAX_ANGULAR_DEG_S = 86.0f;    // 1.5 rad/s
const float MAX_WHEEL_MM_S = 150.0f;
const float LINEAR_ACCEL_MM_S2 = 150.0f;  // 0 -> full speed in 1.0 s
const float ANGULAR_ACCEL_DEG_S2 = 120.0f;

// -----------------------------------------------------------------------------
// Feed-forward calibration
// -----------------------------------------------------------------------------

// COMMISSIONING VALUES. Per-wheel, per-direction open-loop PWM-to-speed linear
// fit (|measured_mm_s| = slope*pwm + intercept), inverted in
// speedFeedForwardPWM() so PID only has to correct the residual.
//
// PROVENANCE: these came from a sweep taken while MEASURED_COUNTS_PER_REV was
// still the wrong 720 (the real figure is 520 -- 13 PPR motor-shaft encoder
// x2 decoding x 1:20 gearbox). Every mm/s in that sweep was therefore scaled
// by pi*D/720 instead of pi*D/520, so the slopes and intercepts below have
// been multiplied through by 720/520 = 1.384615 to put them back in true
// mm/s. That is arithmetic, not measurement: the fit is only as good as the
// run it came from, and the logged run in
// arduino_mini_motor_calibration/Calibration_results.md never produced a
// valid motor 2 fit at all. Re-run CAL and paste a fresh, self-consistent
// result over these when the bench is set up.
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
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {6.4448f, 6.1370f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-43.43f, -45.41f};
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {6.2414f, 6.3233f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-43.24f, -48.20f};

// Lowest PWM the SWEEP calibration command starts from.
//
// 10, not 30. Once MAX_LINEAR_MM_S came down to 150 mm/s the whole drive band
// moved to roughly PWM 14..31, so a sweep starting at 30 measures only speeds
// the robot will now never be asked for and says nothing about the ones it
// will. The fit above still comes from a 30..210 sweep and is extrapolated
// below 30: it puts breakaway at PWM 5.4, which no measurement supports and no
// gearmotor on a TB6612 achieves. Re-run the calibration over the band that is
// actually driven -- in arduino_mini_motor_calibration/, "RANGE,8,60" then
// "STEP,2" then "CAL" -- and paste the result over the four lines above.
const uint8_t SWEEP_MIN_PWM = 10;

// -----------------------------------------------------------------------------
// IMU
// -----------------------------------------------------------------------------

// COMMISSIONING VALUE. A physical left turn must make ROS yaw increase
// (REP-103, CCW-positive). Rotate the robot left by hand and watch the yaw
// field in telemetry; if it decreases, set this to -1.0f.
const float IMU_YAW_SIGN = 1.0f;

// Mahony complementary-filter gains. Higher KP corrects the gyro against
// gravity faster but tracks accelerometer noise with it; KI removes gyro bias
// but destabilises the filter if pushed too high.
const float MAHONY_KP = 2.0f;
const float MAHONY_KI = 0.02f;

// COMMISSIONING VALUE. Magnetic declination for true north at your location.
// Only applied when magnetometer fusion is actually running, and even then it
// washes out of odometry because the heading is zeroed at startup.
const float MAG_DECLINATION_DEG = 12.0f;

// Samples averaged by CAL_IMU to find the gyro zero-rate offset. The robot must
// be completely still for all of them; more samples is a longer stillness
// requirement, not a free improvement.
const uint16_t GYRO_CAL_SAMPLES = 500;

// COMMISSIONING VALUES. Magnetometer hard-iron offsets (uT) and soft-iron scale
// correction, in X, Y, Z order. 0/1 means uncalibrated, which is what ships.
// arduino_mini_imu_calibration/ measures these and prints them ready to paste;
// they only matter once magnetometer fusion is switched on with MAG,1.
const float MAG_OFFSET_UT[3] = {0.0f, 0.0f, 0.0f};
const float MAG_SCALE_CORRECTION[3] = {1.0f, 1.0f, 1.0f};

// -----------------------------------------------------------------------------
// Wheel speed loop
// -----------------------------------------------------------------------------
//
// Starting gains, identical for both wheels: each wheel gets the same speed
// target for straight motion and its own PID corrects the mismatch. Retune
// after a SWEEP calibration has replaced the feed-forward fit above -- the PID
// only has to handle the residual, so a good fit means small gains.
//
// Changeable at runtime with PID / PIDM without reflashing; these are only the
// values the board boots with. lambda and mu are the fractional orders of the
// integral and derivative (see Types.h); at 1.0 the loop is classic PID and
// takes a fast path that skips the Grunwald-Letnikov shaping entirely.
const float WHEEL_PID_KP = 0.30f;
const float WHEEL_PID_KI = 0.020f;
const float WHEEL_PID_KD = 0.0030f;
const float WHEEL_PID_LAMBDA = 1.0f;
const float WHEEL_PID_MU = 1.0f;

// Anti-windup, expressed as the largest PWM the integral term is allowed to
// contribute rather than as a bound on error-seconds. A fixed error-seconds
// bound leaves only a couple of PWM counts of trim at realistic ki values, so
// feed-forward error can never be integrated away and the wheels plateau
// off-target. This is the headroom the loop has to cover a wrong feed-forward:
// if a wheel needs more than this much correction, fix the fit, not this.
const float MAX_INTEGRAL_PWM = 80.0f;

// Fallback bound on the raw integral when ki is effectively zero and the PWM
// limit above cannot be divided out.
const float INTEGRAL_FALLBACK_LIMIT = 500.0f;

// Single-pole low-pass on the encoder speed estimate, applied per control tick:
// filtered += alpha * (raw - filtered). Encoder speed over a 10 ms window is
// quantised and noisy, and the derivative term amplifies exactly that. Higher
// alpha tracks faster and passes more noise; 0.35 settles in roughly three
// ticks (~30 ms).
const float WHEEL_SPEED_FILTER_ALPHA = 0.35f;

// Speeds below this count as stopped. A wheel commanded slower than this coasts
// rather than being held at a tiny PWM that only heats the bridge, and the
// feed-forward returns zero rather than inverting its fit near the origin where
// no fit is valid.
const float MIN_WHEEL_SPEED_MM_S = 1.0f;

// -----------------------------------------------------------------------------
// Control loop timing guards
// -----------------------------------------------------------------------------

// The measured tick interval is clamped to this range before it reaches the
// PID. A scheduling hiccup that stretches one cycle would otherwise divide a
// normal tick count by a large dt (a false speed drop) and multiply the
// derivative by a small one (a spike straight into the output).
const float CONTROL_DT_MIN_S = 0.005f;
const float CONTROL_DT_MAX_S = 0.050f;

// Settling time after the motor driver is powered up and before the IMU is
// probed, so the I2C bus is quiet for detection.
const uint16_t IMU_STARTUP_SETTLE_MS = 200;

// -----------------------------------------------------------------------------
// Serial command defaults
// -----------------------------------------------------------------------------

// Speeds used by the bare FORWARD / REVERSE / LEFT / RIGHT test aliases when no
// value is given. Deliberately below the limits above: an alias typed at a
// terminal should move the robot noticeably, not run it at full speed.
const float DEFAULT_TEST_LINEAR_MM_S = 100.0f;
const float DEFAULT_TEST_ANGULAR_DEG_S = 30.0f;

// Longest command line accepted. Anything longer is discarded with
// ERR,COMMAND_TOO_LONG rather than silently truncated into a different command.
const uint8_t COMMAND_BUFFER_SIZE = 96;

// Defaults and accepted bounds for the SWEEP command. The step ceiling matters:
// atoi() yields an int and a value like 256 truncates to 0 in the uint8_t
// parameter, which makes the sweep loop never advance -- an infinite loop with
// the wheel energised.
const uint8_t SWEEP_DEFAULT_STEP = 10;
const uint16_t SWEEP_DEFAULT_DWELL_MS = 1000;
const uint8_t SWEEP_MAX_STEP = 50;
const uint16_t SWEEP_MIN_DWELL_MS = 100;
