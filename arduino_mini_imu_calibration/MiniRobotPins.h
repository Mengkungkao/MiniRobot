#pragma once

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Pin map and wheel indexing -- THE single copy, shared by every sketch
// -----------------------------------------------------------------------------
//
// Motors are identified by their TB6612 channel and nothing else: motor 1 is
// channel A, motor 2 is channel B. Which side of the robot each one drives is
// NOT part of its name -- that is one fact, stated once, in WHEEL_Y_SIGN
// (MiniRobotGeometry.h). Naming a channel "left" bakes a body orientation into
// the wiring layer, and this robot has already changed which end is the front
// once; the wiring did not change with it.
//
// WIRING
// ------------------------------------------------------------
// TB6612FNG            Uno
//   PWMA (motor 1)     D10
//   AIN1 (motor 1)     D5
//   AIN2 (motor 1)     D4
//   PWMB (motor 2)     D9
//   BIN1 (motor 2)     D7
//   BIN2 (motor 2)     D8
//   STBY               D12
//   VM                 motor battery +
//   VCC                5V
//   GND                GND (common with battery ground)
//
// Encoders             Uno
//   Motor 1 channel A  D2   (INT0)
//   Motor 1 channel B  D6
//   Motor 2 channel A  D3   (INT1)
//   Motor 2 channel B  D11

// TB6612FNG channel A -> motor 1
const uint8_t PIN_PWMA = 10;
const uint8_t PIN_AIN1 = 5;
const uint8_t PIN_AIN2 = 4;

// TB6612FNG channel B -> motor 2
const uint8_t PIN_PWMB = 9;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 8;

// Driver hardware enable. Held HIGH for the whole run; pulling it LOW puts
// both bridges into standby (outputs off) regardless of IN1/IN2/PWM.
const uint8_t PIN_STBY = 12;

// Encoders. Only D2/D3 are external-interrupt capable on the Uno, so channel A
// of each encoder goes there and channel B is read inside the ISR to recover
// the direction.
const uint8_t PIN_ENC_M1_A = 2;
const uint8_t PIN_ENC_M1_B = 6;
const uint8_t PIN_ENC_M2_A = 3;
const uint8_t PIN_ENC_M2_B = 11;

// Motor numbering used everywhere, on the wire and in every per-wheel array:
// motor 1 = TB6612 channel A = index 0, motor 2 = channel B = index 1.
const uint8_t MOTOR_COUNT = 2;
const uint8_t MOTOR_1 = 0;
const uint8_t MOTOR_2 = 1;
