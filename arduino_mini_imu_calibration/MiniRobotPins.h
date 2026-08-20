#pragma once

#include <Arduino.h>

// -----------------------------------------------------------------------------
// Pin map and wheel indexing -- THE single copy, shared by every sketch
// -----------------------------------------------------------------------------
//
// WIRING
// ------------------------------------------------------------
// TB6612FNG            Uno
//   PWMA (left)        D10
//   AIN1 (left)        D5
//   AIN2 (left)        D4
//   PWMB (right)       D9
//   BIN1 (right)       D7
//   BIN2 (right)       D8
//   STBY               D12
//   VM                 motor battery +
//   VCC                5V
//   GND                GND (common with battery ground)
//
// Encoders             Uno
//   Left  channel A    D2   (INT0)
//   Left  channel B    D6
//   Right channel A    D3   (INT1)
//   Right channel B    D11

// TB6612FNG channel A -> LEFT wheel
const uint8_t PIN_PWMA = 10;
const uint8_t PIN_AIN1 = 5;
const uint8_t PIN_AIN2 = 4;

// TB6612FNG channel B -> RIGHT wheel
const uint8_t PIN_PWMB = 9;
const uint8_t PIN_BIN1 = 7;
const uint8_t PIN_BIN2 = 8;

// Driver hardware enable. Held HIGH for the whole run; pulling it LOW puts
// both bridges into standby (outputs off) regardless of IN1/IN2/PWM.
const uint8_t PIN_STBY = 12;

// Encoders. Only D2/D3 are external-interrupt capable on the Uno, so channel A
// of each encoder goes there and channel B is read inside the ISR to recover
// the direction.
const uint8_t PIN_ENC_LEFT_A = 2;
const uint8_t PIN_ENC_LEFT_B = 6;
const uint8_t PIN_ENC_RIGHT_A = 3;
const uint8_t PIN_ENC_RIGHT_B = 11;

// Motor numbering used everywhere: 1 = LEFT (channel A), 2 = RIGHT (channel B).
const uint8_t MOTOR_COUNT = 2;
const uint8_t LEFT = 0;
const uint8_t RIGHT = 1;
