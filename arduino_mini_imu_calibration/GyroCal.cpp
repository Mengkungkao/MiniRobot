#include "GyroCal.h"

#include <math.h>

#include "CalUtil.h"
#include "Config.h"
#include "ImuDriver.h"
#include "Results.h"
#include "Stats.h"

// The run is split in two halves so the difference between them can be
// reported. That difference is the number that matters on this robot: the
// firmware re-measures the zero-rate offset at every boot, so a constant bias
// costs nothing, while a bias that keeps moving after the boot calibration is
// exactly what walks the heading away.
static bool collectHalf(uint16_t count, float mean[AXIS_COUNT], float noise[AXIS_COUNT]) {
  Stats accel[AXIS_COUNT];
  Stats gyro[AXIS_COUNT];

  if (!sampleAccelGyro(count, accel, gyro)) {
    Serial.println(F("WARN,GYRO_ABORTED"));
    return false;
  }
  if (movedDuringRun(accel, gyro)) {
    Serial.println(F("ERR,MOVED_DURING_RUN,THE_MEASUREMENT_IS_NOT_A_BIAS"));
    return false;
  }

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    mean[i] = gyro[i].mean;
    noise[i] = gyro[i].stddev();
  }
  return true;
}

bool calibrateGyroBias() {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return false;
  }

  const uint16_t half = gyroSampleCount / 2;
  Serial.print(F("INFO,GYRO_CAL,samples="));
  Serial.print((uint16_t)(half * 2));
  Serial.print(F(",seconds="));
  Serial.println((uint16_t)((uint32_t)half * 2 * SAMPLE_PERIOD_MS / 1000UL));
  Serial.println(F("INFO,KEEP_THE_ROBOT_STILL,HANDS_OFF,ON_A_SOLID_SURFACE"));

  if (!waitUntilStill(STILL_TIMEOUT_MS)) return false;

  const float tempStart = imuReadTempC();

  float mean1[AXIS_COUNT], noise1[AXIS_COUNT];
  float mean2[AXIS_COUNT], noise2[AXIS_COUNT];
  if (!collectHalf(half, mean1, noise1)) return false;
  if (!collectHalf(half, mean2, noise2)) return false;

  const float tempEnd = imuReadTempC();

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    cal.gyroBias[i] = (mean1[i] + mean2[i]) * 0.5f;
    cal.gyroDrift[i] = mean2[i] - mean1[i];
    // Each half is averaged separately, so this is noise around a steady mean
    // and does not include the drift between the halves reported above.
    cal.gyroNoise[i] = (noise1[i] + noise2[i]) * 0.5f;
  }
  cal.gyroTempC = tempEnd;
  cal.gyroValid = true;

  printGyroRows();

  bool largeBias = false;
  bool noisy = false;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (fabs(cal.gyroBias[i]) > GYRO_BIAS_WARN_DEG_S) largeBias = true;
    if (cal.gyroNoise[i] > GYRO_NOISE_WARN_DEG_S) noisy = true;
  }

  Serial.print(F("INFO,TEMP_C,start="));
  Serial.print(tempStart, 1);
  Serial.print(F(",end="));
  Serial.println(tempEnd, 1);

  // What the operator actually wants to know: with the bias removed at boot,
  // how fast does the heading walk away from the truth.
  Serial.print(F("INFO,EXPECTED_YAW_DRIFT_DEG_PER_MIN,"));
  Serial.println(fabs(cal.gyroDrift[AXIS_Z]) * 60.0f, 2);

  if (largeBias) {
    Serial.println(F("WARN,LARGE_BIAS,OUT_OF_FAMILY_FOR_THIS_PART,WAS_IT_REALLY_STILL"));
  }
  if (noisy) {
    Serial.println(F("WARN,NOISY,CHECK_FOR_VIBRATION_AND_A_CLEAN_3V3_SUPPLY"));
  }
  if (fabs(cal.gyroDrift[AXIS_Z]) > GYRO_DRIFT_WARN_DEG_S) {
    Serial.println(F("WARN,BIAS_STILL_MOVING,YAW_WILL_DRIFT,LET_THE_IMU_WARM_UP_AND_RERUN"));
  }
  if (fabs(tempEnd - tempStart) > GYRO_WARMUP_TEMP_WARN_C) {
    Serial.println(F("WARN,STILL_WARMING_UP,THE_BIAS_MEASURED_HERE_IS_ALREADY_OUT_OF_DATE"));
    Serial.println(F("INFO,LEAVE_IT_POWERED_FOR_A_FEW_MINUTES_AND_RUN,GYRO,AGAIN"));
  }

  Serial.println(F("ACK,GYRO_DONE"));
  return true;
}
