#pragma once

#include <Arduino.h>

// Six-face accelerometer calibration: bias and scale per axis.
//
// The faces are not named in advance. The operator rests the robot on any face
// they like and the sketch works out which one it is from gravity, so nothing
// here depends on knowing how the IMU is mounted or which way its silkscreen
// points. Run it until all six are collected. Returns false if it was
// cancelled before every face was measured.
bool calibrateAccelAllFaces();

// One face, measured wherever the robot is sitting right now.
bool measureAccelFace();

// Forget the faces collected so far (a re-run after moving the IMU).
void resetAccelFaces();

// Print which of the six are still missing.
void printMissingFaces();
