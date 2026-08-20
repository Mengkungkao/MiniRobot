#include "Results.h"

#include "CalUtil.h"
#include "ImuDriver.h"

ImuCalResults cal;

void resetResults() {
  cal.gyroValid = false;
  cal.accelValid = false;
  cal.magValid = false;
  cal.yawSignValid = false;
  cal.motorTestRun = false;
  cal.gyroTempC = 0.0f;
  cal.magFieldUT = 0.0f;
  cal.magResidualPct = 0.0f;
  cal.motorDeviationDeg = 0.0f;
  cal.yawSign = 1.0f;

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    cal.gyroBias[i] = 0.0f;
    cal.gyroNoise[i] = 0.0f;
    cal.gyroDrift[i] = 0.0f;
    cal.accelBias[i] = 0.0f;
    cal.accelScale[i] = 1.0f;
    cal.magOffset[i] = 0.0f;
    cal.magScale[i] = 1.0f;
  }
}

void printGyroRows() {
  Serial.println(F("HDR,G,axis,bias_deg_s,noise_deg_s,drift_deg_s"));
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    Serial.print(F("G,"));
    printAxis(i);
    Serial.print(',');
    Serial.print(cal.gyroBias[i], 4);
    Serial.print(',');
    Serial.print(cal.gyroNoise[i], 4);
    Serial.print(',');
    Serial.println(cal.gyroDrift[i], 4);
  }
}

void printAccelRows() {
  Serial.println(F("HDR,A,axis,bias_g,scale"));
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    Serial.print(F("A,"));
    printAxis(i);
    Serial.print(',');
    Serial.print(cal.accelBias[i], 5);
    Serial.print(',');
    Serial.println(cal.accelScale[i], 5);
  }
}

void printMagRows() {
  Serial.println(F("HDR,M,axis,offset_uT,scale"));
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    Serial.print(F("M,"));
    printAxis(i);
    Serial.print(',');
    Serial.print(cal.magOffset[i], 2);
    Serial.print(',');
    Serial.println(cal.magScale[i], 4);
  }
}

void printResultTable() {
  Serial.println(F("INFO,RESULTS"));

  if (cal.gyroValid) {
    printGyroRows();
    Serial.print(F("INFO,GYRO_TEMP_C,"));
    Serial.println(cal.gyroTempC, 1);
  } else {
    Serial.println(F("WARN,NO_GYRO_RESULT,RUN,GYRO"));
  }

  if (cal.accelValid) {
    printAccelRows();
  } else {
    Serial.println(F("WARN,NO_ACCEL_RESULT,RUN,ACCEL"));
  }

  if (cal.magValid) {
    printMagRows();
    Serial.print(F("INFO,MAG_FIELD_UT,"));
    Serial.print(cal.magFieldUT, 1);
    Serial.print(F(",RESIDUAL_PCT,"));
    // Zero means the check pass never ran, not a flawless calibration.
    if (cal.magResidualPct > 0.0f) Serial.println(cal.magResidualPct, 1);
    else Serial.println(F("NOT_MEASURED"));
  } else if (magPresent) {
    Serial.println(F("WARN,NO_MAG_RESULT,RUN,MAG"));
  }

  if (cal.motorTestRun) {
    Serial.print(F("INFO,MOTOR_FIELD_DEVIATION_DEG,"));
    Serial.print(cal.motorDeviationDeg, 1);
    Serial.println(cal.motorDeviationDeg > MAG_DEVIATION_WARN_DEG
                   ? F(",KEEP_MAGNETOMETER_FUSION_OFF")
                   : F(",MAGNETOMETER_FUSION_IS_SAFE_TO_TRY"));
  }
}

// Three floats on one line, in the same shape as the firmware's own statics.
static void printTriple(const __FlashStringHelper *prefix, const char *suffix,
                        const float value[AXIS_COUNT], uint8_t digits) {
  Serial.print(prefix);
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (i > 0) Serial.print(F(", "));
    Serial.print(suffix);
    Serial.print((char)('X' + i));
    Serial.print(F(" = "));
    Serial.print(value[i], digits);
    Serial.print('f');
  }
  Serial.println(';');
}

void printPasteBlock() {
  Serial.println();
  Serial.println(F("INFO,PASTE_INTO_arduino_mini_base_controller/Imu.cpp"));
  Serial.println(F("// ---------------------------------------------------------"));

  // These two lines already exist in Imu.cpp, at 0/1. Replace them in place.
  if (!cal.magValid) {
    Serial.println(F("// magnetometer NOT calibrated -- these are the shipped defaults"));
  }
  printTriple(F("static float "), "magOffset", cal.magOffset, 2);
  printTriple(F("static float "), "magScale", cal.magScale, 4);
  Serial.println(F("// ---------------------------------------------------------"));

  if (cal.accelValid) {
    // The firmware has no accelerometer correction yet: readAccelGyro() divides
    // by ACCEL_SCALE and stops there. Adding it takes two new statics and three
    // edited lines, all printed here so there is nothing to work out.
    Serial.println();
    Serial.println(F("INFO,ACCEL_CORRECTION_IS_NEW_CODE,ADD_IT_NEXT_TO_THE_OTHER_STATICS"));
    Serial.println(F("// ---------------------------------------------------------"));
    printTriple(F("static const float "), "accelOffset", cal.accelBias, 5);
    printTriple(F("static const float "), "accelScale", cal.accelScale, 5);
    Serial.println(F("// then in readAccelGyro(), replace the three accel lines with:"));
    Serial.println(F("//   accelX = ((float)rawAx / ACCEL_SCALE - accelOffsetX) * accelScaleX;"));
    Serial.println(F("//   accelY = ((float)rawAy / ACCEL_SCALE - accelOffsetY) * accelScaleY;"));
    Serial.println(F("//   accelZ = ((float)rawAz / ACCEL_SCALE - accelOffsetZ) * accelScaleZ;"));
    Serial.println(F("// ---------------------------------------------------------"));
  }

  if (cal.yawSignValid) {
    Serial.println();
    Serial.println(F("INFO,PASTE_INTO_arduino_mini_base_controller/Config.h"));
    Serial.println(F("// ---------------------------------------------------------"));
    Serial.print(F("const float IMU_YAW_SIGN = "));
    Serial.print(cal.yawSign, 1);
    Serial.println(F("f;"));
    Serial.println(F("// ---------------------------------------------------------"));
  }

  // The firmware runs calibrateGyro() on every boot, so the measured zero-rate
  // offset is a health check, not a constant to carry across.
  if (cal.gyroValid) {
    Serial.println(F("INFO,GYRO_BIAS_IS_REMEASURED_AT_EVERY_BOOT_BY_THE_FIRMWARE,NOTHING_TO_PASTE"));
  }
}
