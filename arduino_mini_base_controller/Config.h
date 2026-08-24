#pragma once

#include <Arduino.h>

// Pin map, wheel geometry, the body layout (WHEEL_Y_SIGN) and the
// commissioning signs live in the shared MiniRobot* files, which are kept
// byte-identical here, in arduino_mini_motor_calibration/ and in
// arduino_mini_imu_calibration/. Edit one, copy it to the others -- the
// sketches MUST agree about what an encoder tick is worth and which way is
// forward. `bash scripts/sync_shared_sketch_files.sh --check` proves they do.
// Everything below is specific to the base controller.
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
// PROVENANCE: arduino_mini_motor_calibration/cal_result.md, a two-pass CAL over
// PWM 4..72 in steps of 2 with the robot on blocks -- 280 samples, 252 of them
// above the stall threshold, every fit r2 >= 0.9999 over 61..64 points. The
// four lines below are that run's paste block, unmodified. The raw samples are
// kept in cal_raw.log next to the report, so the fit can be re-derived or
// re-plotted without re-running the robot.
//
// The sweep covers PWM 4..72 = 0..420 mm/s, and the robot drives at most
// MAX_LINEAR_MM_S (150 mm/s, about PWM 30), so the operating band sits well
// inside the measured range with no extrapolation. RMS residual is under
// 1.2 mm/s on every wheel and direction, so the straight line is a fair model
// and not just an assumption r2 flattered.
//
// What the fit does NOT describe is the bottom. Extrapolated to zero speed it
// puts breakaway at PWM 6.9..7.6, while the sweep measured the wheels actually
// starting at PWM 10, 12 and 14 depending on wheel and direction.
// FF_BREAKAWAY_PWM below is that measurement.
//
// Easiest: upload arduino_mini_motor_calibration/, send CAL, and paste the four
// lines it prints back over the four below -- it sweeps both wheels in both
// directions, fits the lines on-board and checks them with VERIFY.
//
// Or sweep from here and fit the points yourself:
//
//   SWEEP,1,10,1000       motor 1, forward, PWM steps of 10, 1 s each
//   SWEEP,1,10,1000,R     motor 1, reverse
//   SWEEP,2,10,1000       motor 2, forward
//   SWEEP,2,10,1000,R     motor 2, reverse
//
// Each step prints "C,<motor>,<F|R>,<pwm>,<measured_mm_s>". Drop the stalled
// points at the bottom of the sweep, least-squares fit the rest, and paste the
// slope/intercept here.
const float FF_SLOPE_MM_S_PER_PWM_FWD[MOTOR_COUNT] = {6.4639f, 6.2343f};
const float FF_INTERCEPT_MM_S_FWD[MOTOR_COUNT] = {-44.31f, -44.78f};
const float FF_SLOPE_MM_S_PER_PWM_REV[MOTOR_COUNT] = {6.2624f, 6.4385f};
const float FF_INTERCEPT_MM_S_REV[MOTOR_COUNT] = {-44.38f, -49.07f};

// Lowest PWM the built-in SWEEP command starts from.
//
// 10, not 30. The whole drive band is roughly PWM 12..30, so a sweep starting
// at 30 measures only speeds the robot will never be asked for and says
// nothing about the ones it will. Starting below the stall point also makes
// the breakaway visible: the first few steps report 0.00 mm/s, and the first
// step that does not is FF_BREAKAWAY_PWM.
const uint8_t SWEEP_MIN_PWM = 10;

// -----------------------------------------------------------------------------
// IMU
// -----------------------------------------------------------------------------

// Whether a missing IMU is fatal.
//
// false (the default) means the robot degrades instead of refusing to move: if
// no MPU answers on I2C, the heading fed to odometry comes from the encoders
// instead -- yaw integrated from the difference between the two wheel
// distances. That heading drifts faster than a gyro's on any surface where a
// wheel can slip, and it cannot see the robot being shoved sideways, but it is
// a real heading and the robot still drives, still maps and still navigates.
// The IMU-vs-encoder choice is reported in the boot banner, in INFO,IMU and in
// the telemetry status field, so nothing about it is silent.
//
// true restores the old behaviour: no IMU means a latched emergency stop and a
// robot that will not move at all.
const bool REQUIRE_IMU = false;

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
// target for straight motion and its own PID corrects the mismatch.
//
// Changeable at runtime with PID / PIDM without reflashing; these are only the
// values the board boots with. lambda and mu are the fractional orders of the
// integral and derivative (see Types.h); at 1.0 the loop is classic PID and
// takes a fast path that skips the Grunwald-Letnikov shaping entirely.
//
// TUNED FOR TORQUE AT LOW SPEED, which is what these gains are really for. The
// feed-forward already puts the wheel at roughly the right PWM on a clean
// surface, so on a bench the loop looks fine at any ki. The job of the loop is
// the other case: carpet, a rug edge, a sticky patch, a small step -- load the
// wheel does not shed by itself, where holding the commanded speed means
// pushing PWM (and therefore current, and therefore torque) well above what
// the fit asks for.
//
// The plant is 6.4 mm/s of wheel speed per PWM count (FF_SLOPE above), so a
// gain of 1/6.4 = 0.156 is unity loop gain:
//   kp 0.35 -> proportional loop gain 2.2, i.e. a disturbance settles to about
//              a third of its open-loop size immediately.
//   ki 0.60 -> integral loop gain 3.8/s, so the remaining third is integrated
//              out with a time constant near 0.26 s. The old 0.020 gave 0.13/s
//              -- an 8 second time constant, which is why a wheel that hit a
//              sticky patch used to just sit there slowed down: the loop was
//              correcting, roughly 0.4 PWM per second.
//   kd 0.0015 stays small. Encoder speed at 30 mm/s is under one count per
//              control tick, so the derivative is mostly quantisation noise
//              however it is filtered.
const float WHEEL_PID_KP = 0.35f;
const float WHEEL_PID_KI = 0.60f;
const float WHEEL_PID_KD = 0.0015f;
const float WHEEL_PID_LAMBDA = 1.0f;
const float WHEEL_PID_MU = 1.0f;

// Anti-windup, expressed as the largest PWM the integral term is allowed to
// contribute rather than as a bound on error-seconds. A fixed error-seconds
// bound leaves only a couple of PWM counts of trim at realistic ki values, so
// feed-forward error can never be integrated away and the wheels plateau
// off-target. This is the headroom the loop has to cover load: 120 PWM on top
// of a feed-forward of ~20 is most of the bridge, which is the point -- a
// robot that can only apply the torque its fit predicts cannot climb over a
// cable. STALL_* below is what stops that headroom being held indefinitely.
const float MAX_INTEGRAL_PWM = 120.0f;

// Fallback bound on the raw integral when ki is effectively zero and the PWM
// limit above cannot be divided out.
const float INTEGRAL_FALLBACK_LIMIT = 500.0f;

// Single-pole low-pass on the encoder speed estimate: filtered += alpha *
// (raw - filtered), applied once per *measurement*, not once per control tick
// (see SPEED_WINDOW_MAX_MS). Higher alpha tracks faster and passes more noise.
const float WHEEL_SPEED_FILTER_ALPHA = 0.35f;

// Speeds below this count as stopped. A wheel commanded slower than this coasts
// rather than being held at a tiny PWM that only heats the bridge, and the
// feed-forward returns zero rather than inverting its fit near the origin where
// no fit is valid.
const float MIN_WHEEL_SPEED_MM_S = 1.0f;

// -----------------------------------------------------------------------------
// Low speed: measurement, breakaway and stall
// -----------------------------------------------------------------------------
//
// Everything in this block exists because slow is the hard case for a brushed
// gearmotor driven open-loop-ish from a PWM bridge. Three separate problems,
// three separate fixes.

// PROBLEM 1: at low speed the encoder barely ticks, so speed measured over one
// 10 ms control period is quantisation noise rather than a measurement.
//
// One count is pi*47/520 = 0.284 mm. At 30 mm/s a wheel produces 1.06 counts
// per 10 ms tick, so the per-tick estimate alternates between 0 and 28.4 mm/s
// -- and a PID cannot hold a speed it cannot see. Worse, the zero samples look
// exactly like a stall, so the integrator winds up and the wheel surges.
//
// So the speed estimate is measured over a window that closes when a tick
// actually arrives, rather than on a fixed schedule: ticks and elapsed time
// accumulate until at least one count has been seen, and the estimate is only
// then recomputed and refiltered. The window is capped so that a genuinely
// stopped wheel is reported as stopped promptly instead of holding its last
// speed forever -- 120 ms is 12 control ticks, and corresponds to a wheel
// slower than 2.4 mm/s, which is stopped for this robot's purposes.
const uint16_t SPEED_WINDOW_MAX_MS = 120;

// PROBLEM 2: the linear fit does not describe the bottom of the range.
//
// COMMISSIONING VALUES, per wheel: the lowest PWM at which the wheel actually
// starts turning from rest. Extrapolating the fit to zero speed says PWM
// 6.9..7.6; the calibration sweep measured a dead stall right up to PWM 8, and
// the wheels starting at 10, 12 and 14 depending on wheel and direction.
// Between those the bridge is energised, the wheel is not moving, and nothing
// but heat comes of it.
//
// Read it off the ASCENDING leg of a sweep, not the descending one. Coming
// down, a turning wheel keeps turning at PWM values it could never have
// started from -- motor 2 was still running at PWM 10 on the way down and
// needed 14 to start on the way up. A wheel commanded from rest is always on
// the ascending side, so that is the number that belongs here.
// scripts/analyse_motor_calibration.py derives it and prints this line ready
// to paste.
//
// THIS ROBOT: {10, 14}. The two wheels genuinely differ, which is the argument
// against one shared floor: at 14 the easier wheel lurches on every start, and
// at 10 the stiffer one sits dead until the stall boost rescues it.
//
// This is a floor on the output of a wheel that is commanded and is NOT
// turning, not a floor on the feed-forward. The distinction matters: a wheel
// already rolling coasts through speeds far below its breakaway, and flooring
// it there would make every slow turn a series of lurches. A wheel standing
// still has no momentum to coast on, so the only useful output is one big
// enough to move it.
const float FF_BREAKAWAY_PWM[MOTOR_COUNT] = {10.0f, 14.0f};

// PROBLEM 3: a wheel that is loaded to a standstill needs more torque than any
// fit predicts, but must not be held there indefinitely.
//
// While a wheel is commanded to move and is not moving, an extra PWM term
// ramps in on top of feed-forward and PID. It is deliberately fast -- the
// point is to break stiction, and stiction breaks or it does not -- and it
// decays as soon as the wheel turns.
//
// A stalled brushed motor draws locked-rotor current through a bridge with no
// current limit, so this cannot be open-ended. Boost time is budgeted: after
// STALL_BOOST_MAX_MS of continuous stall the wheel is coasted for
// STALL_COOLDOWN_MS, the integrator is dumped, and telemetry raises that
// wheel's stall bit for the Pi to see. It then tries again. A robot wedged
// against a wall therefore pulses instead of cooking its gearbox, and the host
// can tell the difference between "driving" and "trying to drive".
const float STALL_SPEED_MM_S = 8.0f;         // below this, with a target, = stalled
const uint16_t STALL_DETECT_MS = 150;        // ...for this long before boosting
const float STALL_BOOST_RATE_PWM_S = 400.0f; // ramp rate of the extra term
const float STALL_BOOST_MAX_PWM = 90.0f;     // ceiling on the extra term alone
const uint16_t STALL_BOOST_MAX_MS = 1500;    // longest continuous boost
const uint16_t STALL_COOLDOWN_MS = 2000;     // forced rest after that

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
