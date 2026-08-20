#pragma once

#include <Arduino.h>

#include "Config.h"

// -----------------------------------------------------------------------------
// MPU9250 / MPU9255 / MPU6500 + AK8963, register level only
// -----------------------------------------------------------------------------
//
// This is the acquisition half of arduino_mini_base_controller/Imu.cpp with the
// Mahony filter left out: detection, the same register configuration, and
// scaled reads. No bias or hard-iron correction is applied anywhere in this
// file -- measuring those is the point of the sketch, and a driver that
// silently corrected its own output could not measure them.
//
// THE REGISTER CONFIGURATION MUST MATCH THE FIRMWARE. Full-scale range sets
// what one LSB is worth and the DLPF sets how much noise reaches the average;
// a bias measured at one setting does not describe a sensor running at
// another. If you change initMPU() in Imu.cpp, change imuBegin() here too.

extern bool imuFound;
extern bool magPresent;
extern uint8_t imuAddress;
extern uint8_t imuWhoAmI;

// Factory sensitivity adjustment from the AK8963's fuse ROM, already applied to
// everything imuReadMag() returns. Printed by INFO because a module reading
// wildly different values per axis is worth knowing about.
extern float magAdj[AXIS_COUNT];

// Wire.begin(), detect the part, configure it, bring up the magnetometer.
// Returns imuFound.
bool imuBegin();

void imuPrintInfo();

// accel in g, gyro in deg/s. Both raw in the sense that matters: no offsets.
void imuReadAccelGyro(float accel[AXIS_COUNT], float gyro[AXIS_COUNT]);

// Raw LSB counts, for the RAW command and for wiring faults that scaling hides.
void imuReadRawAccelGyro(int16_t accel[AXIS_COUNT], int16_t gyro[AXIS_COUNT]);

// Magnetometer in uT, factory-adjusted only. False when no fresh sample was
// ready or the sensor reported magnetic overflow.
bool imuReadMag(float mag[AXIS_COUNT]);

// Die temperature. The gyro bias moves with it, so every gyro run reports it.
float imuReadTempC();
