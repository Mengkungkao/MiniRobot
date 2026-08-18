#pragma once

#include <Arduino.h>

// Drive a few target speeds open loop through the fitted constants and report
// how far off the wheel actually lands. This is the check that the numbers are
// worth pasting: whatever residual is left here is what PID has to absorb.
void verifyMotor(uint8_t index);
