#include "Telemetry.h"

#include "Config.h"
#include "Imu.h"
#include "MotorControl.h"

// One bit per condition a host cannot infer from the numeric fields. Kept as a
// single integer so adding a condition later does not change the field count
// again; a host that does not know a bit ignores it.
//   bit 0  motor 1 commanded to move but not moving
//   bit 1  motor 2 commanded to move but not moving
//   bit 2  an IMU was detected (clear = heading comes from the encoders)
//   bit 3  magnetometer fusion is active
static uint8_t statusBits() {
  uint8_t bits = 0;
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    if (wheelStalled[i]) bits |= (uint8_t)(1 << i);
  }
  if (mpuFound) bits |= 0x04;
  if (useMagnetometer && magnetometerPresent) bits |= 0x08;
  return bits;
}

void publishTelemetry() {
  updateFusedRollPitch();

  Serial.print(F("T,"));
  Serial.print(millis());
  Serial.print(','); Serial.print(poseXMM, 2);
  Serial.print(','); Serial.print(poseYMM, 2);
  // The heading the pose above was actually integrated along, whichever sensor
  // produced it -- not the raw IMU field, which is meaningless with no IMU.
  Serial.print(','); Serial.print(headingDeg, 3);
  Serial.print(','); Serial.print(linearVelocityMMs, 2);
  Serial.print(','); Serial.print(headingRateDegS, 3);

  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(cumulativeTicks[i]);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelMeasuredMMs[i], 2);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
    Serial.print(','); Serial.print(wheelPWM[i], 1);
  }

  Serial.print(','); Serial.print(fusedRoll, 2);
  Serial.print(','); Serial.print(fusedPitch, 2);
  Serial.print(','); Serial.print(gyroX, 3);
  Serial.print(','); Serial.print(gyroY, 3);
  Serial.print(','); Serial.print(accelX, 4);
  Serial.print(','); Serial.print(accelY, 4);
  Serial.print(','); Serial.print(accelZ, 4);

  Serial.print(','); Serial.print(emergencyStopLatched ? 1 : 0);
  Serial.print(','); Serial.print(watchdogStopped ? 1 : 0);
  Serial.print(','); Serial.println(statusBits());
}
