#pragma once

#include <Arduino.h>

#include "MiniRobotPins.h"

// Configure the encoder pins and attach the quadrature ISRs. Call once from
// setup().
//
// Decoding is x2: a CHANGE interrupt on channel A only, with channel B read
// inside the ISR to recover the direction. MEASURED_COUNTS_PER_REV must be
// expressed in these same x2 counts.
void encodersBegin();

// Read the tick delta since the previous call and clear the counter, with
// interrupts disabled so an ISR cannot update the count mid-read.
int32_t readEncoderAndResetAtomic(uint8_t index);
