#include "Interactive.h"

#include <math.h>

#include "CalUtil.h"
#include "Config.h"
#include "ImuDriver.h"
#include "MagCal.h"
#include "Results.h"
#include "Stats.h"

// Apply whatever has been measured so far. Before the first GYRO/ACCEL run
// these are zeros and ones, so LIVE still works -- it just shows the sensor
// uncorrected, which is a fair picture of the robot before commissioning.
static void correctedAccelGyro(float accel[AXIS_COUNT], float gyro[AXIS_COUNT]) {
  imuReadAccelGyro(accel, gyro);
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    accel[i] = (accel[i] - cal.accelBias[i]) * cal.accelScale[i];
    gyro[i] -= cal.gyroBias[i];
  }
}

// Tilt-compensated heading, the same construction the firmware seeds its filter
// with, minus the declination -- this is for watching the calibration behave,
// not for navigating.
static float magHeadingDeg(const float mag[AXIS_COUNT], float rollRad, float pitchRad) {
  const float cosRoll = cos(rollRad), sinRoll = sin(rollRad);
  const float cosPitch = cos(pitchRad), sinPitch = sin(pitchRad);

  const float xComp = mag[0] * cosPitch + mag[2] * sinPitch;
  const float yComp = mag[0] * sinRoll * sinPitch + mag[1] * cosRoll - mag[2] * sinRoll * cosPitch;

  float heading = atan2(-yComp, xComp) * RAD_TO_DEG;
  if (heading < 0.0f) heading += 360.0f;
  return heading;
}

void runHealthCheck() {
  imuPrintInfo();
  if (!imuFound) return;

  // How fast the bus actually delivers samples. The calibration loops assume
  // 200 Hz is available; a module on long unshielded wires often is not.
  const uint32_t rateStart = micros();
  for (uint16_t i = 0; i < 200; ++i) {
    int16_t a[AXIS_COUNT], g[AXIS_COUNT];
    imuReadRawAccelGyro(a, g);
  }
  const uint32_t elapsedUs = micros() - rateStart;
  Serial.print(F("CHECK,READ_RATE_HZ,"));
  Serial.println(200.0f * 1e6f / (float)elapsedUs, 0);

  Stats accel[AXIS_COUNT];
  Stats gyro[AXIS_COUNT];
  if (!sampleAccelGyro(200, accel, gyro)) return;   // 1 s

  float mean[AXIS_COUNT];
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) mean[i] = accel[i].mean;
  const float gravity = vectorNorm(mean);

  Serial.print(F("CHECK,GRAVITY_G,"));
  Serial.print(gravity, 3);
  // The magnitude of the accelerometer vector is 1 g at rest whatever the
  // orientation, so this needs no assumption about how the IMU is mounted.
  Serial.println((gravity > 0.90f && gravity < 1.10f) ? F(",OK") : F(",BAD_SCALE_OR_MOVING"));

  Serial.print(F("CHECK,GYRO_NOISE_DEG_S"));
  bool noisy = false;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    Serial.print(',');
    Serial.print(gyro[i].stddev(), 3);
    if (gyro[i].stddev() > GYRO_NOISE_WARN_DEG_S) noisy = true;
  }
  Serial.println(noisy ? F(",NOISY") : F(",OK"));

  Serial.print(F("CHECK,GYRO_BIAS_DEG_S"));
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    Serial.print(',');
    Serial.print(gyro[i].mean, 3);
  }
  Serial.println(F(",QUICK_LOOK_ONLY,RUN,GYRO,FOR_THE_REAL_ONE"));

  if (magPresent) {
    Stats field;
    field.reset();
    const uint32_t start = millis();
    while (millis() - start < 1000UL) {
      float m[AXIS_COUNT];
      if (imuReadMag(m)) field.add(vectorNorm(m));
      delay(MAG_SAMPLE_PERIOD_MS);
    }
    Serial.print(F("CHECK,FIELD_UT,"));
    if (field.n == 0) {
      Serial.println(F("NO_DATA,AK8963_NOT_RESPONDING"));
    } else {
      Serial.print(field.mean, 1);
      // Uncorrected, so hard iron is still in it -- this only catches a dead
      // or saturated magnetometer, not a bad calibration.
      Serial.println((field.mean > EARTH_FIELD_MIN_UT && field.mean < EARTH_FIELD_MAX_UT * 2.0f)
                     ? F(",PLAUSIBLE") : F(",OUT_OF_RANGE_STRONG_HARD_IRON"));
    }
  }

  Serial.print(F("CHECK,TEMP_C,"));
  Serial.println(imuReadTempC(), 1);
  Serial.println(F("ACK,CHECK_DONE"));
}

void runYawSignCheck() {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return;
  }

  Serial.println(F("INFO,YAW_SIGN,ROS_REQUIRES_A_LEFT_TURN_TO_INCREASE_YAW"));
  if (!waitForOperator(F("SET_THE_ROBOT_LEVEL_AND_STILL_THEN_PRESS_ENTER"))) return;
  if (!waitUntilStill(STILL_TIMEOUT_MS)) return;

  Stats accel[AXIS_COUNT];
  Stats gyro[AXIS_COUNT];
  if (!sampleAccelGyro(200, accel, gyro)) return;

  // A yaw sign only means anything if yaw is what the Z gyro measures. On a
  // side-mounted IMU it is not, and no sign flip in the firmware can fix that.
  if (fabs(accel[AXIS_Z].mean) < 0.9f) {
    Serial.println(F("WARN,IMU_IS_NOT_MOUNTED_FLAT,Z_IS_NOT_VERTICAL"));
    Serial.println(F("INFO,IMU_YAW_SIGN_CANNOT_FIX_A_SIDE_MOUNTED_IMU,REMOUNT_IT_FLAT"));
  }

  const float biasZ = gyro[AXIS_Z].mean;

  Serial.print(F("INFO,YOU_HAVE_SECONDS,"));
  Serial.println(YAWSIGN_WINDOW_MS / 1000UL);
  if (!waitForOperator(F("PRESS_ENTER_THEN_TURN_THE_ROBOT_LEFT_CCW_SEEN_FROM_ABOVE_ABOUT_90_DEG"))) return;

  float angle = 0.0f;
  const uint32_t start = millis();
  uint32_t last = micros();
  while (millis() - start < YAWSIGN_WINDOW_MS) {
    float a[AXIS_COUNT], g[AXIS_COUNT];
    imuReadAccelGyro(a, g);

    const uint32_t now = micros();
    angle += (g[AXIS_Z] - biasZ) * ((now - last) * 1e-6f);
    last = now;

    delay(SAMPLE_PERIOD_MS);
  }

  Serial.print(F("INFO,INTEGRATED_DEG,"));
  Serial.println(angle, 1);

  if (fabs(angle) < YAWSIGN_MIN_DEG) {
    Serial.println(F("ERR,NOT_ENOUGH_ROTATION,TURN_FURTHER_AND_RERUN,YAWSIGN"));
    return;
  }

  cal.yawSign = (angle > 0.0f) ? 1.0f : -1.0f;
  cal.yawSignValid = true;

  Serial.print(F("INFO,IMU_YAW_SIGN,"));
  Serial.println(cal.yawSign, 1);
  Serial.println(cal.yawSign > 0.0f
                 ? F("INFO,THE_GYRO_ALREADY_AGREES_WITH_ROS,NO_CHANGE_NEEDED")
                 : F("INFO,THE_GYRO_IS_INVERTED_RELATIVE_TO_ROS,PASTE_THE_MINUS_ONE"));
  Serial.println(F("ACK,YAWSIGN_DONE"));
}

void streamLive(uint32_t durationMs) {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return;
  }

  Serial.println(F("INFO,LIVE,CORRECTED_VALUES,SEND_ANY_CHARACTER_TO_STOP"));
  Serial.print(F("HDR,L,ax_g,ay_g,az_g,gx,gy,gz,roll_deg,pitch_deg,yaw_deg"));
  Serial.println(magPresent ? F(",field_uT,heading_deg") : F(""));

  float yaw = 0.0f;
  const uint32_t start = millis();
  uint32_t lastMicros = micros();
  uint32_t lastPrint = 0;

  while (millis() - start < durationMs) {
    if (abortRequested()) break;

    float accel[AXIS_COUNT], gyro[AXIS_COUNT];
    correctedAccelGyro(accel, gyro);

    // Integrated at the sample rate, printed at a readable one -- integrating
    // only what gets printed would throw most of the rotation away.
    const uint32_t now = micros();
    yaw += gyro[AXIS_Z] * ((now - lastMicros) * 1e-6f);
    lastMicros = now;

    if (millis() - lastPrint >= 100) {
      lastPrint = millis();

      const float rollRad = atan2(accel[1], accel[2]);
      const float pitchRad = atan2(-accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

      Serial.print(F("L"));
      for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
        Serial.print(',');
        Serial.print(accel[i], 3);
      }
      for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
        Serial.print(',');
        Serial.print(gyro[i], 2);
      }
      Serial.print(',');
      Serial.print(rollRad * RAD_TO_DEG, 1);
      Serial.print(',');
      Serial.print(pitchRad * RAD_TO_DEG, 1);
      Serial.print(',');
      Serial.print(yaw, 1);

      if (magPresent) {
        float raw[AXIS_COUNT];
        if (imuReadMag(raw)) {
          float mag[AXIS_COUNT];
          magApplyCorrection(raw, mag);
          Serial.print(',');
          Serial.print(vectorNorm(mag), 1);
          Serial.print(',');
          Serial.print(magHeadingDeg(mag, rollRad, pitchRad), 1);
        } else {
          Serial.print(F(",,"));
        }
      }
      Serial.println();
    }

    delay(SAMPLE_PERIOD_MS);
  }

  Serial.println(F("ACK,LIVE_DONE"));
}

void streamRaw(uint32_t durationMs) {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return;
  }

  Serial.println(F("INFO,RAW,LSB_COUNTS,NO_CORRECTION,SEND_ANY_CHARACTER_TO_STOP"));
  Serial.println(magPresent ? F("HDR,RAW,ax,ay,az,gx,gy,gz,mx,my,mz")
                            : F("HDR,RAW,ax,ay,az,gx,gy,gz"));

  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (abortRequested()) break;

    int16_t accel[AXIS_COUNT], gyro[AXIS_COUNT];
    imuReadRawAccelGyro(accel, gyro);

    Serial.print(F("RAW"));
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
      Serial.print(',');
      Serial.print(accel[i]);
    }
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
      Serial.print(',');
      Serial.print(gyro[i]);
    }

    if (magPresent) {
      float mag[AXIS_COUNT];
      if (imuReadMag(mag)) {
        for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
          Serial.print(',');
          Serial.print(mag[i], 1);
        }
      } else {
        Serial.print(F(",,,"));
      }
    }
    Serial.println();

    delay(200);
  }

  Serial.println(F("ACK,RAW_DONE"));
}

void printConfig() {
  Serial.print(F("INFO,CONFIG,gyro_samples="));
  Serial.print(gyroSampleCount);
  Serial.print(F(",accel_samples_per_face="));
  Serial.print(accelSampleCount);
  Serial.print(F(",mag_seconds="));
  Serial.println(magDurationMs / 1000UL);

  Serial.print(F("INFO,ESTIMATED_SECONDS,gyro="));
  Serial.print((uint16_t)((uint32_t)gyroSampleCount * SAMPLE_PERIOD_MS / 1000UL));
  Serial.print(F(",accel_per_face="));
  Serial.print((uint16_t)((uint32_t)accelSampleCount * SAMPLE_PERIOD_MS / 1000UL));
  Serial.print(F(",mag="));
  Serial.println((magDurationMs + MAG_CHECK_MS) / 1000UL);
}

void printHelp() {
  Serial.println(F("CMD,CAL                    guided full run: CHECK, GYRO, ACCEL, YAWSIGN, MAG"));
  Serial.println(F("CMD,CHECK                  quick health check of the sensor"));
  Serial.println(F("CMD,GYRO                   zero-rate offset, noise and drift"));
  Serial.println(F("CMD,ACCEL                  six-face bias and scale"));
  Serial.println(F("CMD,FACE                   measure one face where it sits now"));
  Serial.println(F("CMD,MAG[,seconds]          hard and soft iron, figure-of-eight"));
  Serial.println(F("CMD,MOTORS                 what the wheels do to the magnetometer"));
  Serial.println(F("CMD,YAWSIGN                work out IMU_YAW_SIGN"));
  Serial.println(F("CMD,LIVE[,seconds]         stream corrected values"));
  Serial.println(F("CMD,RAW[,seconds]          stream raw LSB counts"));
  Serial.println(F("CMD,RESULT                 reprint the table and the paste block"));
  Serial.println(F("CMD,SAMPLES,gyro,accel     set sample counts"));
  Serial.println(F("CMD,MAGTIME,seconds        set the magnetometer sweep length"));
  Serial.println(F("CMD,CONFIG                 print settings and time estimates"));
  Serial.println(F("CMD,STOP                   coast both motors"));
  Serial.println(F("INFO,ANY_CHARACTER_ABORTS_A_RUNNING_MEASUREMENT,X_CANCELS_AT_A_PROMPT"));
}
