#pragma once

#include <Arduino.h>

#include "MiniRobotGeometry.h"
#include "MiniRobotPins.h"

// One H-bridge channel. Datasheet truth table, with STBY high:
//   IN1=H IN2=L  -> clockwise, speed set by PWM
//   IN1=L IN2=H  -> counter-clockwise, speed set by PWM
//   IN1=L IN2=L  -> stop, outputs high-impedance (the wheel freewheels)
//   IN1=H IN2=H  -> short brake, outputs shorted (the wheel resists rotation)
struct TB6612Channel {
  uint8_t pwmPin;
  uint8_t in1Pin;
  uint8_t in2Pin;

  void begin() const {
    pinMode(pwmPin, OUTPUT);
    pinMode(in1Pin, OUTPUT);
    pinMode(in2Pin, OUTPUT);
    coast();
  }

  void coast() const {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

  void brake() const {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, HIGH);
  }

  void drive(bool forward, uint8_t duty) const {
    digitalWrite(in1Pin, forward ? HIGH : LOW);
    digitalWrite(in2Pin, forward ? LOW : HIGH);
    analogWrite(pwmPin, duty);
  }
};

// The two wheels, indexed by LEFT / RIGHT.
extern const TB6612Channel motor[MOTOR_COUNT];

// Configure both channels as outputs, take the driver out of standby and set
// the PWM frequency. Call once from setup(), before driving anything.
void motorsBegin();

// Coast both wheels.
void motorsCoastAll();

// TB6612 pin polarity for "this wheel should turn this way", accounting for a
// motor that is wired backwards (MOTOR_DIRECTION_SIGN).
bool motorPinsForward(uint8_t index, bool reverse);
