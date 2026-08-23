#pragma once

#include <Arduino.h>

// Print "<n>(LEFT)" / "<n>(RIGHT)" for operator-facing messages.
void printMotorLabel(uint8_t index);

// True if the operator sent anything -- used as an abort request during the
// blocking sweeps. The bytes are swallowed so the abort character does not come
// back as a bogus command afterwards.
bool abortRequested();

// delay() that can be cut short by an abort request. Returns false if aborted.
bool sleepMs(uint32_t ms);

// Run one wheel at one PWM and return the steady-state speed in mm/s, signed in
// the ROBOT frame (positive = this wheel is driving the robot forward), with
// both commissioning signs applied. Returns NAN if the operator aborted
// mid-measurement. Leaves the motor running, so a sweep does not pay a spin-up
// transient at every step.
float measureSpeedMMs(uint8_t index, uint16_t pwm, bool reverse);

// Raw encoder delta over one DIR probe at a given PIN polarity, with NO signs
// applied -- the naked hardware fact DIR needs to reason from.
//
// Deliberately not abortable: the probe lasts under two seconds and the
// operator has just been told to watch the wheel, so a keystroke arriving now
// is far more likely to be an early answer than an abort. Direction.cpp drains
// the buffer before it asks the question.
int32_t measureRawTicks(uint8_t index, bool pinsHigh, uint16_t pwm);

