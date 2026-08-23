#pragma once

#include <Arduino.h>

// Step one wheel across the PWM range in one direction, print every point, and
// fit the linear model to the points that were actually moving. Stores the
// result in results[index][FWD|REV].
//
// Blocks for roughly passes * steps * (settle + window) milliseconds. Send any
// character to abort.
void sweepAndFit(uint8_t index, bool reverse);

// Both directions of one wheel, with a pause between them.
void calibrateMotor(uint8_t index);

// Drive a few target speeds open loop through the fitted constants and report
// how far off the wheel actually lands. This is the check that the numbers are
// worth pasting: whatever residual is left here is what PID has to absorb.
void verifyMotor(uint8_t index);

// How many distinct PWM points one pass of the current settings visits.
uint16_t sweepStepCount();
