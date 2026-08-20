#include "MagCal.h"

#include <math.h>

#include "CalUtil.h"
#include "ImuDriver.h"
#include "Results.h"
#include "Stats.h"

void magApplyCorrection(const float raw[AXIS_COUNT], float out[AXIS_COUNT]) {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    out[i] = (raw[i] - cal.magOffset[i]) * cal.magScale[i];
  }
}

// Sweep the field for durationMs, tracking the extremes of each axis. Returns
// false only if the operator aborted -- a sweep that collected nothing because
// the AK8963 stopped answering is a different failure, and the caller tells
// them apart by looking at the sample count.
static bool sweepField(uint32_t durationMs, Stats mag[AXIS_COUNT]) {
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) mag[i].reset();

  const uint32_t start = millis();
  uint32_t lastPrint = start;
  uint32_t nextSample = start;

  while (millis() - start < durationMs) {
    if (abortRequested()) return false;

    float m[AXIS_COUNT];
    if (imuReadMag(m)) {
      for (uint8_t i = 0; i < AXIS_COUNT; ++i) mag[i].add(m[i]);
    }

    // Progress is the point of these lines: the operator is watching the spans
    // grow to know which way still needs turning.
    if (millis() - lastPrint >= 2000) {
      lastPrint = millis();
      Serial.print(F("M,"));
      Serial.print((millis() - start) / 1000UL);
      Serial.print(',');
      Serial.print(mag[0].n);
      for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
        Serial.print(',');
        Serial.print(mag[i].span(), 1);
      }
      Serial.println();
    }

    nextSample += MAG_SAMPLE_PERIOD_MS;
    while ((int32_t)(millis() - nextSample) < 0) delay(1);
  }

  return true;
}

// Rotate for a while longer with the correction applied. Every sample should
// now land on a sphere of one radius however the robot is held, so the spread
// of that radius is the honest quality number -- min/max hard iron cannot check
// itself, because the extremes it used are exactly the samples it fitted.
static bool runCheckPass() {
  if (!waitForOperator(F("KEEP_ROTATING_FOR_THE_CHECK_PASS_THEN_PRESS_ENTER"))) return false;

  Stats field;
  field.reset();

  const uint32_t start = millis();
  uint32_t nextSample = start;
  while (millis() - start < MAG_CHECK_MS) {
    if (abortRequested()) {
      Serial.println(F("WARN,CHECK_PASS_ABORTED"));
      return false;
    }

    float raw[AXIS_COUNT];
    if (imuReadMag(raw)) {
      float corrected[AXIS_COUNT];
      magApplyCorrection(raw, corrected);
      field.add(vectorNorm(corrected));
    }

    nextSample += MAG_SAMPLE_PERIOD_MS;
    while ((int32_t)(millis() - nextSample) < 0) delay(1);
  }

  if (field.n < MAG_MIN_SAMPLES / 4 || field.mean <= 0.0f) {
    Serial.println(F("WARN,CHECK_PASS_TOO_SHORT,QUALITY_UNKNOWN"));
    return false;
  }

  cal.magFieldUT = field.mean;
  cal.magResidualPct = 100.0f * field.stddev() / field.mean;

  Serial.print(F("INFO,CORRECTED_FIELD_UT,"));
  Serial.print(cal.magFieldUT, 1);
  Serial.print(F(",SPREAD_PCT,"));
  Serial.println(cal.magResidualPct, 1);
  return true;
}

bool calibrateMagnetometer(uint32_t durationMs) {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return false;
  }
  if (!magPresent) {
    Serial.println(F("ERR,NO_MAGNETOMETER,MPU6500_HAS_NO_AK8963"));
    return false;
  }

  Serial.print(F("INFO,MAG_CAL,seconds="));
  Serial.println(durationMs / 1000UL);
  Serial.println(F("INFO,TURN_THE_ROBOT_SLOWLY_THROUGH_FIGURE_OF_EIGHTS"));
  Serial.println(F("INFO,EVERY_SIDE_MUST_POINT_EVERY_WAY,ROLL_IT_AND_PITCH_IT_TOO"));
  Serial.println(F("INFO,AWAY_FROM_STEEL_DESKS_LAPTOPS_AND_SPEAKERS,MOTORS_OFF"));
  Serial.println(F("INFO,THE_M_LINES_SHOW_EACH_AXIS_SPAN_GROWING,KEEP_GOING_UNTIL_ALL_THREE_STOP"));

  if (!waitForOperator(F("PICK_THE_ROBOT_UP_THEN_PRESS_ENTER_AND_START_TURNING"))) return false;

  Stats mag[AXIS_COUNT];
  if (!sweepField(durationMs, mag)) {
    Serial.println(F("WARN,MAG_ABORTED"));
    return false;
  }

  const uint16_t samples = mag[0].n;
  if (samples < MAG_MIN_SAMPLES) {
    Serial.print(F("ERR,TOO_FEW_SAMPLES,"));
    Serial.println(samples);
    Serial.println(F("INFO,THE_AK8963_WAS_NOT_RETURNING_DATA,CHECK_WIRING_AND_RERUN"));
    return false;
  }

  // An axis that was never rotated through the field has a span near zero, and
  // the midpoint of nothing is not an offset. Refuse rather than store it.
  bool thinCoverage = false;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (mag[i].span() < MAG_MIN_SPAN_UT) {
      thinCoverage = true;
      Serial.print(F("ERR,AXIS_NOT_ROTATED_ENOUGH,"));
      printAxis(i);
      Serial.print(F(",span_uT,"));
      Serial.println(mag[i].span(), 1);
    }
  }
  if (thinCoverage) {
    Serial.println(F("INFO,RERUN_MAG_AND_TURN_THE_ROBOT_THROUGH_ALL_THREE_AXES"));
    return false;
  }

  float spanSum = 0.0f;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) spanSum += mag[i].span();
  const float spanAverage = spanSum / (float)AXIS_COUNT;

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    cal.magOffset[i] = mag[i].centre();            // hard iron
    cal.magScale[i] = spanAverage / mag[i].span(); // soft iron
  }
  // Half the average span is the radius of the fitted sphere, i.e. the local
  // field strength whatever the hard iron was. The check pass replaces it with
  // a measured mean; a zero residual means that pass never ran.
  cal.magFieldUT = spanAverage * 0.5f;
  cal.magResidualPct = 0.0f;
  cal.magValid = true;

  printMagRows();

  // The spans are what the offsets were derived from, so they are worth seeing
  // next to them: three similar numbers mean the rotation covered the sphere.
  Serial.print(F("INFO,SPANS_UT"));
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    Serial.print(',');
    Serial.print(mag[i].span(), 1);
  }
  Serial.println();

  const float fieldEstimate = cal.magFieldUT;
  Serial.print(F("INFO,FIELD_ESTIMATE_UT,"));
  Serial.println(fieldEstimate, 1);

  runCheckPass();

  if (fieldEstimate < EARTH_FIELD_MIN_UT || fieldEstimate > EARTH_FIELD_MAX_UT) {
    Serial.println(F("WARN,FIELD_OUTSIDE_25_TO_65_uT,SOMETHING_ON_THE_ROBOT_IS_MAGNETIC"));
    Serial.println(F("INFO,USUALLY_THE_MOTORS_OR_A_STEEL_CHASSIS_PART_NEAR_THE_IMU"));
  }
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (cal.magScale[i] > MAG_SOFT_IRON_WARN || cal.magScale[i] < 1.0f / MAG_SOFT_IRON_WARN) {
      Serial.println(F("WARN,LARGE_SOFT_IRON,USUALLY_INCOMPLETE_ROTATION_NOT_REAL_IRON"));
      break;
    }
  }
  if (cal.magResidualPct > MAG_RESIDUAL_WARN_PCT) {
    Serial.println(F("WARN,POOR_FIT,THE_CORRECTED_FIELD_STILL_CHANGES_WITH_ORIENTATION"));
    Serial.println(F("INFO,RERUN_MAG_WITH_SLOWER_MORE_COMPLETE_ROTATION"));
  }

  Serial.println(F("INFO,MAGNETOMETER_FUSION_IS_OFF_IN_THE_FIRMWARE,RUN,MOTORS,BEFORE_ENABLING_IT"));
  Serial.println(F("ACK,MAG_DONE"));
  return true;
}
