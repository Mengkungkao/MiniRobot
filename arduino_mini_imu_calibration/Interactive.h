#pragma once

#include <Arduino.h>

// Quick health check: what the part is, how fast it reads, whether gravity and
// the local field come out the right size, and how quiet the gyro is. A few
// seconds, and the right first command on a robot that has never been checked.
void runHealthCheck();

// Work out IMU_YAW_SIGN by asking the operator to turn the robot left and
// watching which way the gyro integrates.
void runYawSignCheck();

// Stream corrected values (and the tilt-compensated heading, if there is a
// magnetometer). This is where a calibration proves itself: the numbers should
// sit still when the robot does.
void streamLive(uint32_t durationMs);

// Stream raw LSB counts. For wiring and byte-order faults that scaling hides.
void streamRaw(uint32_t durationMs);

void printConfig();
void printHelp();
