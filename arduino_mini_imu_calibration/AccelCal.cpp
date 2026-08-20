#include "AccelCal.h"

#include <math.h>

#include "CalUtil.h"
#include "Config.h"
#include "ImuDriver.h"
#include "Results.h"
#include "Stats.h"

// Face index = axis * 2 + (pointing up ? 0 : 1), i.e. +X, -X, +Y, -Y, +Z, -Z.
static const uint8_t FACE_COUNT = AXIS_COUNT * 2;

static bool faceDone[FACE_COUNT];
static float faceValue[FACE_COUNT];   // what the up axis read, in g
static float faceCross[FACE_COUNT];   // worst of the other two, in g

void resetAccelFaces() {
  for (uint8_t f = 0; f < FACE_COUNT; ++f) {
    faceDone[f] = false;
    faceValue[f] = 0.0f;
    faceCross[f] = 0.0f;
  }
}

void printMissingFaces() {
  Serial.print(F("INFO,FACES_REMAINING"));
  uint8_t remaining = 0;
  for (uint8_t f = 0; f < FACE_COUNT; ++f) {
    if (faceDone[f]) continue;
    ++remaining;
    Serial.print(',');
    printFace(f);
  }
  if (remaining == 0) Serial.print(F(",NONE"));
  Serial.println();
}

// Which face is the robot resting on? The dominant axis names it, provided it
// is carrying most of a g and the other two are close to nothing -- otherwise
// the robot is propped between two faces and the reading is a mix of both.
static bool classifyFace(const float a[AXIS_COUNT], uint8_t &face,
                         float &onAxis, float &cross) {
  uint8_t axis = 0;
  for (uint8_t i = 1; i < AXIS_COUNT; ++i) {
    if (fabs(a[i]) > fabs(a[axis])) axis = i;
  }

  onAxis = a[axis];
  cross = 0.0f;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (i != axis && fabs(a[i]) > cross) cross = fabs(a[i]);
  }

  face = axis * 2 + (onAxis > 0.0f ? 0 : 1);
  return fabs(onAxis) >= ACCEL_FACE_MIN_G && cross <= ACCEL_FACE_CROSS_MAX_G;
}

// Bias is the midpoint of the two opposite faces, scale is the 2 g between
// them divided by what the sensor actually reported for it.
static bool computeAccelFit() {
  for (uint8_t axis = 0; axis < AXIS_COUNT; ++axis) {
    const uint8_t up = axis * 2;
    const uint8_t down = axis * 2 + 1;
    if (!faceDone[up] || !faceDone[down]) return false;

    const float span = faceValue[up] - faceValue[down];
    if (span < 1.0f) {
      // Two faces that disagree by less than 1 g cannot be opposite ends of
      // gravity; something was measured wrong.
      Serial.print(F("ERR,IMPLAUSIBLE_SPAN,AXIS_"));
      printAxis(axis);
      Serial.print(',');
      Serial.println(span, 4);
      return false;
    }

    cal.accelBias[axis] = (faceValue[up] + faceValue[down]) * 0.5f;
    cal.accelScale[axis] = 2.0f / span;
  }
  return true;
}

bool measureAccelFace() {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return false;
  }
  if (!waitUntilStill(STILL_TIMEOUT_MS)) return false;

  Stats accel[AXIS_COUNT];
  Stats gyro[AXIS_COUNT];
  if (!sampleAccelGyro(accelSampleCount, accel, gyro)) {
    Serial.println(F("WARN,ACCEL_FACE_ABORTED"));
    return false;
  }
  if (movedDuringRun(accel, gyro)) {
    Serial.println(F("ERR,MOVED_DURING_RUN,HOLD_IT_STILL_AND_TRY_THAT_FACE_AGAIN"));
    return false;
  }

  float mean[AXIS_COUNT];
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) mean[i] = accel[i].mean;

  uint8_t face;
  float onAxis, cross;
  const bool flat = classifyFace(mean, face, onAxis, cross);

  Serial.print(F("AF,"));
  printFace(face);
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    Serial.print(',');
    Serial.print(mean[i], 4);
  }
  Serial.println(flat ? F(",OK") : F(",NOT_FLAT"));

  if (!flat) {
    Serial.println(F("ERR,NOT_RESTING_ON_ONE_FACE,ONE_AXIS_MUST_CARRY_GRAVITY_ALONE"));
    Serial.println(F("INFO,LIE_IT_FLAT_ON_A_LEVEL_SURFACE,NOT_PROPPED_AT_AN_ANGLE"));
    return false;
  }

  if (faceDone[face]) Serial.println(F("INFO,FACE_ALREADY_MEASURED,OVERWRITING"));

  faceDone[face] = true;
  faceValue[face] = onAxis;
  faceCross[face] = cross;

  // Refresh the fit as soon as both ends of every axis exist, so a re-measured
  // face takes effect without repeating the whole sequence.
  cal.accelValid = computeAccelFit();
  return true;
}

static void printAccelSummary() {
  printAccelRows();

  bool largeBias = false;
  bool oddScale = false;
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    if (fabs(cal.accelBias[i]) > ACCEL_BIAS_WARN_G) largeBias = true;
    if (fabs(cal.accelScale[i] - 1.0f) > ACCEL_SCALE_WARN) oddScale = true;
  }

  // How level the surfaces were. It does not enter the fit -- it says how much
  // to trust it.
  float worstCross = 0.0f;
  for (uint8_t f = 0; f < FACE_COUNT; ++f) {
    if (faceCross[f] > worstCross) worstCross = faceCross[f];
  }
  Serial.print(F("INFO,WORST_OFF_AXIS_G,"));
  Serial.print(worstCross, 3);
  Serial.print(F(",APPROX_TILT_DEG,"));
  Serial.println(asin(constrain(worstCross, -1.0f, 1.0f)) * RAD_TO_DEG, 1);

  if (largeBias) {
    Serial.println(F("WARN,LARGE_BIAS,OUT_OF_FAMILY_FOR_THIS_PART,RECHECK_THE_FACES"));
  }
  if (oddScale) {
    Serial.println(F("WARN,ODD_SCALE,A_FACE_WAS_PROBABLY_NOT_FLAT"));
  }
}

bool calibrateAccelAllFaces() {
  if (!imuFound) {
    Serial.println(F("ERR,NO_IMU"));
    return false;
  }

  resetAccelFaces();
  cal.accelValid = false;

  Serial.println(F("INFO,ACCEL_CAL,SIX_FACES,ANY_ORDER"));
  Serial.println(F("INFO,REST_THE_ROBOT_ON_EACH_FACE_IN_TURN,FLAT_AND_LET_GO_OF_IT"));
  Serial.println(F("INFO,THE_SKETCH_WORKS_OUT_WHICH_FACE_IT_IS,SO_MOUNTING_DOES_NOT_MATTER"));
  Serial.println(F("INFO,SEND_X_AT_ANY_PROMPT_TO_STOP"));

  // No attempt limit: a face that comes back NOT_FLAT is just asked for again.
  while (!cal.accelValid) {
    printMissingFaces();
    if (!waitForOperator(F("PLACE_THE_ROBOT_ON_AN_UNMEASURED_FACE_THEN_PRESS_ENTER"))) {
      Serial.println(F("WARN,ACCEL_INCOMPLETE,NO_FIT_STORED"));
      return false;
    }
    measureAccelFace();
  }

  printAccelSummary();
  Serial.println(F("ACK,ACCEL_DONE"));
  return true;
}
