#pragma once

#include <Arduino.h>

extern bool mpuFound;
extern bool useMagnetometer;
extern bool magnetometerPresent;

// Converted sensor values: accel in g, gyro in deg/s.
extern float accelX, accelY, accelZ;
extern float gyroX, gyroY, gyroZ;

// Fused attitude in degrees, refreshed by updateFusedRollPitch() at the
// telemetry rate.
extern float fusedRoll;
extern float fusedPitch;

// Odometry heading: yawDeg is fusedYaw with the zero reference subtracted,
// refreshed by updateIMU() at the control rate.
extern float yawDeg;
extern float yawRateDegS;

// Detect the MPU, initialise it and the magnetometer if present, and run the
// blocking startup gyro calibration. Returns mpuFound. The robot must be still
// while this runs.
bool imuBegin();

// One IMU acquisition + fusion step. Call at the control rate.
void updateIMU();

// Recompute fusedRoll/fusedPitch. Call at the (slower) telemetry rate.
void updateFusedRollPitch();

// Make the current heading the new zero.
void zeroYaw();

// Average several hundred stationary samples for the gyro's zero-rate offset.
// Blocking; the robot must be completely still while this runs.
void calibrateGyro();

// Restart the Mahony filter from the accelerometer instead of keeping a
// quaternion that integrated through a bias change. Call after calibrateGyro()
// or after toggling useMagnetometer.
void reinitFusion();

void printImuInfo();
