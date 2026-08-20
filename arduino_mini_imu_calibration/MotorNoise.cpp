#include "MotorNoise.h"

#include <math.h>

#include "MiniRobotMotors.h"

#include "CalUtil.h"
#include "Config.h"
#include "ImuDriver.h"
#include "MagCal.h"
#include "Results.h"
#include "Stats.h"

// Average the corrected field and the gyro noise over one window. Returns false
// if the operator aborted.
static bool sampleWindow(uint16_t durationMs, float field[AXIS_COUNT], float &gyroNoiseZ) {
  Stats mag[AXIS_COUNT];
  Stats gyroZ;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) mag[i].reset();
  gyroZ.reset();

  const uint32_t start = millis();
  while (millis() - start < durationMs) {
    if (abortRequested()) return false;

    float a[AXIS_COUNT], g[AXIS_COUNT];
    imuReadAccelGyro(a, g);
    gyroZ.add(g[AXIS_Z]);

    float raw[AXIS_COUNT];
    if (imuReadMag(raw)) {
      float corrected[AXIS_COUNT];
      magApplyCorrection(raw, corrected);
      for (uint8_t i = 0; i < AXIS_COUNT; ++i) mag[i].add(corrected[i]);
    }

    delay(MAG_SAMPLE_PERIOD_MS);
  }

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) field[i] = mag[i].mean;
  gyroNoiseZ = gyroZ.stddev();
  return true;
}

void runMotorInterferenceTest() {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return;
  }

  Serial.println(F("WARN,PUT_THE_ROBOT_ON_BLOCKS_BOTH_WHEELS_MUST_SPIN_FREE"));
  if (!magPresent) {
    Serial.println(F("INFO,NO_MAGNETOMETER,MEASURING_GYRO_VIBRATION_ONLY"));
  } else if (!cal.magValid) {
    Serial.println(F("INFO,MAGNETOMETER_NOT_CALIBRATED_YET,ANGLES_ARE_APPROXIMATE"));
  }
  if (!waitForOperator(F("ROBOT_ON_BLOCKS_AND_NOT_TOUCHING_ANYTHING_THEN_PRESS_ENTER"))) return;

  float baseline[AXIS_COUNT];
  float baselineNoise;
  Serial.println(F("INFO,BASELINE,MOTORS_OFF"));
  if (!sampleWindow(2000, baseline, baselineNoise)) {
    Serial.println(F("WARN,MOTORS_ABORTED"));
    return;
  }

  Serial.print(F("INFO,BASELINE_FIELD_UT,"));
  Serial.print(vectorNorm(baseline), 1);
  Serial.print(F(",GYRO_Z_NOISE,"));
  Serial.println(baselineNoise, 3);

  Serial.println(F("HDR,MI,pwm,field_deg,field_uT,gyro_z_noise_deg_s"));

  float worstDeviation = 0.0f;
  bool aborted = false;

  for (uint8_t p = 0; p < MOTOR_TEST_PWM_COUNT; ++p) {
    const uint8_t pwm = MOTOR_TEST_PWM[p];
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
      motor[i].drive(motorPinsForward(i, false), pwm);
    }

    // Let the current settle before measuring, the same reason the motor
    // calibration sketch has a settle window before its counting window.
    if (!sleepMs(500)) { aborted = true; break; }

    float running[AXIS_COUNT];
    float runningNoise;
    if (!sampleWindow(MOTOR_TEST_HOLD_MS, running, runningNoise)) { aborted = true; break; }

    const float deviation = magPresent ? angleBetweenDeg(baseline, running) : 0.0f;
    if (deviation > worstDeviation) worstDeviation = deviation;

    Serial.print(F("MI,"));
    Serial.print(pwm);
    Serial.print(',');
    Serial.print(deviation, 1);
    Serial.print(',');
    Serial.print(vectorNorm(running) - vectorNorm(baseline), 1);
    Serial.print(',');
    Serial.println(runningNoise, 3);
  }

  motorsCoastAll();

  if (aborted) {
    Serial.println(F("WARN,MOTORS_ABORTED"));
    return;
  }

  // Vibration rectifies into gyro bias, which is a yaw drift the firmware only
  // sees while driving -- and it re-measures its bias when parked, so it never
  // catches this one.
  float settledNoise;
  float settledField[AXIS_COUNT];
  if (sampleWindow(1000, settledField, settledNoise)) {
    Serial.print(F("INFO,GYRO_Z_NOISE_AFTER,"));
    Serial.println(settledNoise, 3);
  }

  if (magPresent) {
    cal.motorTestRun = true;
    cal.motorDeviationDeg = worstDeviation;

    Serial.print(F("INFO,WORST_FIELD_DEVIATION_DEG,"));
    Serial.println(worstDeviation, 1);

    if (worstDeviation > MAG_DEVIATION_WARN_DEG) {
      Serial.println(F("WARN,MOTORS_MOVE_THE_FIELD,KEEP_MAGNETOMETER_FUSION_OFF"));
      Serial.println(F("INFO,THE_HEADING_WOULD_SWING_BY_ABOUT_THAT_MUCH_WITH_THROTTLE"));
      Serial.println(F("INFO,MOVE_THE_IMU_FURTHER_FROM_THE_MOTORS_AND_BATTERY_TO_IMPROVE_IT"));
    } else {
      Serial.println(F("INFO,FIELD_IS_STABLE,NINE_AXIS_FUSION_IS_WORTH_TRYING"));
      Serial.println(F("INFO,ENABLE_IT_WITH,MAG,1,IN_THE_FIRMWARE_AND_WATCH_YAW_WHILE_DRIVING"));
    }
  }

  Serial.println(F("ACK,MOTORS_DONE"));
}
