#include "Telemetry.h"

#include "Config.h"
#include "Imu.h"
#include "MotorControl.h"

void publishTelemetry() {
  updateFusedRollPitch();

  Serial.print(F("T,"));
  Serial.print(millis());
  Serial.print(','); Serial.print(poseXMM, 2);
  Serial.print(','); Serial.print(poseYMM, 2);
  Serial.print(','); Serial.print(yawDeg, 3);
  Serial.print(','); Serial.print(linearVelocityMMs, 2);
  Serial.print(','); Serial.print(yawRateDegS, 3);

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
  Serial.print(','); Serial.println(watchdogStopped ? 1 : 0);
}
