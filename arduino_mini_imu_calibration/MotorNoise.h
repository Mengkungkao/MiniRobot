#pragma once

#include <Arduino.h>

// Spin both wheels at a few PWM levels and measure what that does to the
// magnetometer and to the gyro. This is the test that decides whether nine-axis
// fusion is usable on this robot at all: the AK8963 sits centimetres from two
// brushed motors, and a field that swings with motor current turns heading into
// a function of throttle.
//
// The robot must be on blocks. Blocking; any character aborts and coasts.
void runMotorInterferenceTest();
