#pragma once

#include <Arduino.h>

#include "Config.h"

// Everything the sketch has measured so far. One instance, so RESULT can
// reprint it and CAL can fill it in stage by stage; nothing is persisted, so a
// reset or a re-upload starts from empty.
struct ImuCalResults {
  bool gyroValid;
  float gyroBias[AXIS_COUNT];    // deg/s, the zero-rate offset
  float gyroNoise[AXIS_COUNT];   // deg/s, per-sample standard deviation
  float gyroDrift[AXIS_COUNT];   // deg/s, second half minus first half
  float gyroTempC;

  bool accelValid;
  float accelBias[AXIS_COUNT];   // g
  float accelScale[AXIS_COUNT];  // dimensionless, 1.0 = perfect

  bool magValid;
  float magOffset[AXIS_COUNT];   // uT, hard iron
  float magScale[AXIS_COUNT];    // dimensionless, soft iron
  float magFieldUT;              // corrected field magnitude
  float magResidualPct;          // spread of that magnitude, the quality number

  bool yawSignValid;
  float yawSign;

  bool motorTestRun;
  float motorDeviationDeg;       // worst field distortion with the wheels spinning
};

extern ImuCalResults cal;

void resetResults();

// One row per axis. The stages print these as soon as they have them and
// RESULT prints them again later, so they live here rather than three times
// over in the calibration files.
void printGyroRows();
void printAccelRows();
void printMagRows();

// Everything measured so far, with the headers.
void printResultTable();

// The lines to paste into the firmware, and where each one goes.
void printPasteBlock();
