#include "MiniRobotMotors.h"

const TB6612Channel motor[MOTOR_COUNT] = {
  {PIN_PWMA, PIN_AIN1, PIN_AIN2},   // motor 1, channel A
  {PIN_PWMB, PIN_BIN1, PIN_BIN2},   // motor 2, channel B
};

// Raise Timer1 (pins 9 and 10, i.e. both TB6612 PWM inputs) from the Arduino
// default 490 Hz to ~31.4 kHz by dropping its prescaler to 1. 490 Hz sits in
// the middle of the audible band and makes the motors sing at low duty; above
// ~20 kHz the whine disappears and the current ripple through the gearmotor
// windings is smaller. Timer0 is untouched, so millis()/micros()/delay() keep
// working normally.
//
// Both sketches must run the same frequency: PWM frequency changes the
// effective torque at a given duty, so a calibration measured at one frequency
// does not describe a robot driving at another.
static void setPwmFrequency31kHz() {
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
}

void motorsBegin() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) motor[i].begin();
  setPwmFrequency31kHz();

  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);   // take both bridges out of standby
}

void motorsCoastAll() {
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) motor[i].coast();
}

bool motorPinsForward(uint8_t index, bool reverse) {
  return (MOTOR_DIRECTION_SIGN[index] > 0) ? !reverse : reverse;
}
