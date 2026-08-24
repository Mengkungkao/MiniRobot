#pragma once

#include <Arduino.h>
#include "MiniRobotMotors.h"

#include "Config.h"
#include "Types.h"

extern WheelPID wheelPID[MOTOR_COUNT];

// Telemetry-visible wheel/odometry state.
extern float wheelMeasuredMMs[MOTOR_COUNT];
extern float wheelPWM[MOTOR_COUNT];
extern int32_t cumulativeTicks[MOTOR_COUNT];
extern float poseXMM;
extern float poseYMM;
extern float linearVelocityMMs;

// Heading and turn rate actually used for odometry: the IMU's when one was
// detected, encoder-derived otherwise (see REQUIRE_IMU in Config.h). Telemetry
// reports these rather than the raw IMU fields, so the yaw a host reads is
// always the yaw the pose was integrated along.
extern float headingDeg;
extern float headingRateDegS;

// True while a wheel is commanded to move but is not moving. Bit i of the
// telemetry status field; cleared as soon as the wheel turns again.
extern bool wheelStalled[MOTOR_COUNT];

extern bool emergencyStopLatched;
extern bool watchdogStopped;

// One-time init: motor outputs, ~31 kHz PWM, STBY, encoders and default PID
// gains. Call once from setup().
void motorControlBegin();

// Start the command watchdog timer fresh. Call once at the end of setup(),
// after the robot is actually ready to receive commands.
void armCommandWatchdog();

// Discard any pending encoder ticks and clear the speed estimates, so stale
// counts don't produce a velocity spike on the next control cycle.
void resetEncoderDeltas();

// Stop with motors freewheeling (coast) and clear all controller state.
void releaseAllMotors();

// Accept a new body-velocity setpoint (clamped to limits) and feed the
// command watchdog.
void setVelocityCommand(float linearMMs, float angularDegS);

// Pet the command watchdog without changing the velocity setpoint.
void refreshCommandWatchdog();

// Zero the setpoints and ramps, then stop the motors (brake or coast).
void stopCommand(bool brake);

// Zero the integrated pose, tick totals, and heading reference.
void resetOdometry();

// 100 Hz control step: watchdog/e-stop handling, acceleration ramps,
// differential-drive kinematics, per-wheel feed-forward + PID + stall boost,
// and odometry integration (encoder distance + heading).
void updateControl(float dt);
