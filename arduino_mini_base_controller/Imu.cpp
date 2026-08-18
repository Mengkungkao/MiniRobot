#include "Imu.h"

#include <Wire.h>
#include <math.h>

#include "Config.h"

// -----------------------------------------------------------------------------
// IMU: MPU9250 / MPU9255 / MPU6500 with AK8963 magnetometer
// -----------------------------------------------------------------------------

static const uint8_t MPU_ADDR_1 = 0x68;
static const uint8_t MPU_ADDR_2 = 0x69;

static const uint8_t REG_WHO_AM_I = 0x75;
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_CONFIG2 = 0x1D;
static const uint8_t REG_INT_PIN_CFG = 0x37;
static const uint8_t REG_INT_ENABLE = 0x38;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_GYRO_XOUT_H = 0x43;

// AK8963 magnetometer, exposed on the I2C bus once the MPU's bypass is enabled.
static const uint8_t AK8963_ADDR = 0x0C;
static const uint8_t AK8963_WHO_AM_I = 0x00;
static const uint8_t AK8963_ST1 = 0x02;
static const uint8_t AK8963_XOUT_L = 0x03;
static const uint8_t AK8963_CNTL1 = 0x0A;
static const uint8_t AK8963_ASAX = 0x10;

// Configured ranges: accel +/-2 g, gyro +/-250 deg/s, magnetometer 16-bit.
static const float ACCEL_SCALE = 16384.0f;
static const float GYRO_SCALE = 131.0f;
static const float MAG_SCALE = 4912.0f / 32760.0f;

// Magnetic declination for true north. Only applied when magnetometer fusion
// is actually running, and even then it washes out of odometry because the
// heading is zeroed at startup.
static const float MAG_DECLINATION_DEG = 12.0f;

// Mahony filter gains. Higher KP corrects faster but tracks accelerometer
// noise; KI removes gyro bias but destabilises the filter if pushed too high.
static const float MAHONY_KP = 2.0f;
static const float MAHONY_KI = 0.02f;

// Magnetometer fusion is OFF by default. On a small robot the AK8963 sits
// centimetres from two brushed motors and a battery, so the field it measures
// swings with motor current and turns yaw into a function of throttle. Gyro
// integration (6-axis fusion) has no absolute reference but is smooth and
// locally accurate, which is what odometry actually needs; SLAM supplies the
// absolute correction. Enable at runtime with MAG,1 only after checking that
// yaw stays put while the wheels spin.
bool useMagnetometer = false;

static uint8_t mpuAddress = MPU_ADDR_1;
static uint8_t mpuWhoAmI = 0x00;
bool mpuFound = false;
bool magnetometerPresent = false;
static bool magnetometerDataValid = false;

float accelX = 0.0f, accelY = 0.0f, accelZ = 0.0f;
float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
static float magX = 0.0f, magY = 0.0f, magZ = 0.0f;

static float gyroOffsetX = 0.0f, gyroOffsetY = 0.0f, gyroOffsetZ = 0.0f;

// Magnetometer hard-iron offsets (uT) and soft-iron scale correction. Leave at
// 0/1 until you have run a figure-of-eight calibration.
static float magOffsetX = 0.0f, magOffsetY = 0.0f, magOffsetZ = 0.0f;
static float magScaleX = 1.0f, magScaleY = 1.0f, magScaleZ = 1.0f;

// Factory sensitivity adjustment burned into the AK8963's fuse ROM.
static float magAdjX = 1.0f, magAdjY = 1.0f, magAdjZ = 1.0f;

// Orientation quaternion and the Mahony integral feedback term.
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
static bool fusionStarted = false;

// Fused attitude in degrees. fusedYaw is absolute (filter frame); the odometry
// heading below is this minus the zero reference.
float fusedRoll = 0.0f;
float fusedPitch = 0.0f;
static float fusedYaw = 0.0f;

static uint32_t lastFusionMicros = 0;

float yawDeg = 0.0f;
float yawRateDegS = 0.0f;
static float yawZeroDeg = 0.0f;

// -----------------------------------------------------------------------------
// I2C helpers
// -----------------------------------------------------------------------------

static void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

static uint8_t readByte(uint8_t address, uint8_t subAddress) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  if (Wire.endTransmission(false) != 0) return 0xFF;

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0xFF;
}

// Burst-read 'count' registers. Any bytes the device did not return are zeroed
// so a truncated transfer cannot leave stale values in the caller's buffer.
static void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, count);

  uint8_t i = 0;
  while (Wire.available() && i < count) dest[i++] = Wire.read();
  while (i < count) dest[i++] = 0;
}

// WHO_AM_I distinguishes the three supported parts: 0x70 MPU6500 (no
// magnetometer), 0x71 MPU9250, 0x73 MPU9255. The module may sit at either I2C
// address depending on how its AD0 pin is strapped.
static void detectMPU() {
  const uint8_t id1 = readByte(MPU_ADDR_1, REG_WHO_AM_I);
  delay(10);
  const uint8_t id2 = readByte(MPU_ADDR_2, REG_WHO_AM_I);
  delay(10);

  if (id1 == 0x70 || id1 == 0x71 || id1 == 0x73) {
    mpuAddress = MPU_ADDR_1;
    mpuWhoAmI = id1;
    mpuFound = true;
  } else if (id2 == 0x70 || id2 == 0x71 || id2 == 0x73) {
    mpuAddress = MPU_ADDR_2;
    mpuWhoAmI = id2;
    mpuFound = true;
  } else {
    mpuFound = false;
  }
}

static void initMPU() {
  writeByte(mpuAddress, REG_PWR_MGMT_1, 0x00);   // wake
  delay(100);
  writeByte(mpuAddress, REG_PWR_MGMT_1, 0x01);   // clock from gyro X PLL
  delay(100);
  writeByte(mpuAddress, REG_CONFIG, 0x03);       // gyro DLPF 41 Hz
  writeByte(mpuAddress, REG_SMPLRT_DIV, 0x04);   // 200 Hz sample rate
  writeByte(mpuAddress, REG_GYRO_CONFIG, 0x00);  // +/-250 deg/s
  writeByte(mpuAddress, REG_ACCEL_CONFIG, 0x00); // +/-2 g
  writeByte(mpuAddress, REG_ACCEL_CONFIG2, 0x03);// accel DLPF 41 Hz
  writeByte(mpuAddress, REG_INT_PIN_CFG, 0x02);  // I2C bypass: expose AK8963
  writeByte(mpuAddress, REG_INT_ENABLE, 0x01);   // data-ready interrupt
  delay(100);
}

static void initMagnetometer() {
  magnetometerPresent = false;

  // The MPU6500 is the same die without the magnetometer.
  if (mpuWhoAmI == 0x70) return;
  if (readByte(AK8963_ADDR, AK8963_WHO_AM_I) != 0x48) return;

  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00);  // power down
  delay(10);
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x0F);  // fuse ROM access
  delay(10);

  uint8_t asa[3];
  readBytes(AK8963_ADDR, AK8963_ASAX, 3, asa);
  magAdjX = ((float)(asa[0] - 128) / 256.0f) + 1.0f;
  magAdjY = ((float)(asa[1] - 128) / 256.0f) + 1.0f;
  magAdjZ = ((float)(asa[2] - 128) / 256.0f) + 1.0f;

  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00);  // power down again
  delay(10);
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x16);  // continuous mode 2, 16-bit, 100 Hz
  delay(10);

  magnetometerPresent = true;
}

static void readAccelGyro() {
  uint8_t data[14];
  readBytes(mpuAddress, REG_ACCEL_XOUT_H, 14, data);

  const int16_t rawAx = ((int16_t)data[0] << 8) | data[1];
  const int16_t rawAy = ((int16_t)data[2] << 8) | data[3];
  const int16_t rawAz = ((int16_t)data[4] << 8) | data[5];
  // data[6..7] is temperature, which this controller does not use.
  const int16_t rawGx = ((int16_t)data[8] << 8) | data[9];
  const int16_t rawGy = ((int16_t)data[10] << 8) | data[11];
  const int16_t rawGz = ((int16_t)data[12] << 8) | data[13];

  accelX = (float)rawAx / ACCEL_SCALE;
  accelY = (float)rawAy / ACCEL_SCALE;
  accelZ = (float)rawAz / ACCEL_SCALE;

  gyroX = ((float)rawGx / GYRO_SCALE) - gyroOffsetX;
  gyroY = ((float)rawGy / GYRO_SCALE) - gyroOffsetY;
  gyroZ = ((float)rawGz / GYRO_SCALE) - gyroOffsetZ;
}

// Returns false when no fresh sample is ready or the sensor reported magnetic
// overflow, in which case the caller falls back to 6-axis fusion for this step.
static bool readMagnetometer() {
  if (!(readByte(AK8963_ADDR, AK8963_ST1) & 0x01)) return false;

  uint8_t data[7];
  readBytes(AK8963_ADDR, AK8963_XOUT_L, 7, data);
  if (data[6] & 0x08) return false;

  // The AK8963 is little-endian, unlike the MPU registers above.
  const int16_t rawMx = ((int16_t)data[1] << 8) | data[0];
  const int16_t rawMy = ((int16_t)data[3] << 8) | data[2];
  const int16_t rawMz = ((int16_t)data[5] << 8) | data[4];

  magX = ((float)rawMx * MAG_SCALE * magAdjX - magOffsetX) * magScaleX;
  magY = ((float)rawMy * MAG_SCALE * magAdjY - magOffsetY) * magScaleY;
  magZ = ((float)rawMz * MAG_SCALE * magAdjZ - magOffsetZ) * magScaleZ;
  return true;
}

// Average several hundred stationary samples to get the gyro's zero-rate
// offset. The robot must be completely still while this runs.
void calibrateGyro() {
  const int samples = 500;
  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;

  for (int i = 0; i < samples; ++i) {
    uint8_t data[6];
    readBytes(mpuAddress, REG_GYRO_XOUT_H, 6, data);
    sumX += (float)(int16_t)(((int16_t)data[0] << 8) | data[1]) / GYRO_SCALE;
    sumY += (float)(int16_t)(((int16_t)data[2] << 8) | data[3]) / GYRO_SCALE;
    sumZ += (float)(int16_t)(((int16_t)data[4] << 8) | data[5]) / GYRO_SCALE;
    delay(3);
  }

  gyroOffsetX = sumX / samples;
  gyroOffsetY = sumY / samples;
  gyroOffsetZ = sumZ / samples;
}

static float invSqrt(float x) {
  return 1.0f / sqrt(x);
}

// Wrap an angle into (-180, 180] degrees.
static float normalizeDegrees(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

// Shared tail of both Mahony variants: apply the proportional and integral
// correction to the gyro rates, integrate the quaternion, renormalise.
static void applyMahonyFeedbackAndIntegrate(float gx, float gy, float gz,
                                            float ex, float ey, float ez, float dt) {
  if (MAHONY_KI > 0.0f) {
    integralFBx += MAHONY_KI * ex * dt;
    integralFBy += MAHONY_KI * ey * dt;
    integralFBz += MAHONY_KI * ez * dt;
    gx += integralFBx;
    gy += integralFBy;
    gz += integralFBz;
  }

  gx += MAHONY_KP * ex;
  gy += MAHONY_KP * ey;
  gz += MAHONY_KP * ez;

  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;

  const float qa = q0;
  const float qb = q1;
  const float qc = q2;

  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);

  const float recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

// Gyro + accelerometer. Gravity corrects roll and pitch; yaw is pure gyro
// integration, which is exactly what wheel odometry wants.
static void mahonyUpdate6Axis(float gx, float gy, float gz,
                              float ax, float ay, float az, float dt) {
  float norm = ax * ax + ay * ay + az * az;
  if (norm <= 0.0f) return;

  norm = invSqrt(norm);
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Estimated direction of gravity from the current quaternion.
  const float vx = 2.0f * (q1 * q3 - q0 * q2);
  const float vy = 2.0f * (q0 * q1 + q2 * q3);
  const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  // Error is the cross product between measured and estimated gravity.
  applyMahonyFeedbackAndIntegrate(gx, gy, gz,
                                  ay * vz - az * vy,
                                  az * vx - ax * vz,
                                  ax * vy - ay * vx, dt);
}

// Gyro + accelerometer + magnetometer. Adds an absolute yaw reference, at the
// cost of being pulled around by any nearby ferrous metal or motor current.
static void mahonyUpdate9Axis(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float mx, float my, float mz, float dt) {
  float norm = ax * ax + ay * ay + az * az;
  if (norm <= 0.0f) return;
  norm = invSqrt(norm);
  ax *= norm;
  ay *= norm;
  az *= norm;

  norm = mx * mx + my * my + mz * mz;
  if (norm <= 0.0f) {
    mahonyUpdate6Axis(gx, gy, gz, ax, ay, az, dt);
    return;
  }
  norm = invSqrt(norm);
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Rotate the measured field into the earth frame, then rebuild it as a
  // purely horizontal-plus-vertical reference (bx, 0, bz) so magnetic
  // declination cannot contaminate roll and pitch.
  const float hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3) +
                   2.0f * my * (q1 * q2 - q0 * q3) +
                   2.0f * mz * (q1 * q3 + q0 * q2);
  const float hy = 2.0f * mx * (q1 * q2 + q0 * q3) +
                   2.0f * my * (0.5f - q1 * q1 - q3 * q3) +
                   2.0f * mz * (q2 * q3 - q0 * q1);
  const float bx = sqrt(hx * hx + hy * hy);
  const float bz = 2.0f * mx * (q1 * q3 - q0 * q2) +
                   2.0f * my * (q2 * q3 + q0 * q1) +
                   2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

  const float vx = 2.0f * (q1 * q3 - q0 * q2);
  const float vy = 2.0f * (q0 * q1 + q2 * q3);
  const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  const float wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) + 2.0f * bz * (q1 * q3 - q0 * q2);
  const float wy = 2.0f * bx * (q1 * q2 - q0 * q3) + 2.0f * bz * (q0 * q1 + q2 * q3);
  const float wz = 2.0f * bx * (q0 * q2 + q1 * q3) + 2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

  applyMahonyFeedbackAndIntegrate(gx, gy, gz,
                                  (ay * vz - az * vy) + (my * wz - mz * wy),
                                  (az * vx - ax * vz) + (mz * wx - mx * wz),
                                  (ax * vy - ay * vx) + (mx * wy - my * wx), dt);
}

static void setQuaternionFromEuler(float rollRad, float pitchRad, float yawRad) {
  const float cy = cos(yawRad * 0.5f), sy = sin(yawRad * 0.5f);
  const float cp = cos(pitchRad * 0.5f), sp = sin(pitchRad * 0.5f);
  const float cr = cos(rollRad * 0.5f), sr = sin(rollRad * 0.5f);

  q0 = cr * cp * cy + sr * sp * sy;
  q1 = sr * cp * cy - cr * sp * sy;
  q2 = cr * sp * cy + sr * cp * sy;
  q3 = cr * cp * sy - sr * sp * cy;
}

// Tilt-compensated magnetic heading, used only to seed the filter on the first
// fusion step so yaw does not have to slew in from zero.
static float tiltCompensatedMagYaw(float rollRad, float pitchRad) {
  const float cosRoll = cos(rollRad), sinRoll = sin(rollRad);
  const float cosPitch = cos(pitchRad), sinPitch = sin(pitchRad);

  const float xComp = magX * cosPitch + magZ * sinPitch;
  const float yComp = magX * sinRoll * sinPitch + magY * cosRoll - magZ * sinRoll * cosPitch;

  return normalizeDegrees(atan2(-yComp, xComp) * RAD_TO_DEG + MAG_DECLINATION_DEG);
}

// Yaw alone, recomputed every control cycle because odometry integrates along
// it. Roll and pitch are only needed by telemetry, so they are computed at the
// slower telemetry rate (two more transcendentals per call is real time on a
// 16 MHz AVR).
static void updateFusedYaw() {
  fusedYaw = atan2(2.0f * (q0 * q3 + q1 * q2),
                   1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG;
  if (useMagnetometer && magnetometerPresent) fusedYaw += MAG_DECLINATION_DEG;
  fusedYaw = normalizeDegrees(fusedYaw);
}

void updateFusedRollPitch() {
  fusedRoll = atan2(2.0f * (q0 * q1 + q2 * q3),
                    1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG;
  fusedPitch = asin(constrain(2.0f * (q0 * q2 - q3 * q1), -1.0f, 1.0f)) * RAD_TO_DEG;
}

// One IMU acquisition + fusion step. Called at the control rate.
void updateIMU() {
  readAccelGyro();

  magnetometerDataValid = false;
  if (useMagnetometer && magnetometerPresent) {
    magnetometerDataValid = readMagnetometer();
  }

  const uint32_t now = micros();
  float dt = (now - lastFusionMicros) * 1e-6f;
  lastFusionMicros = now;
  if (dt <= 0.0f || dt > 0.5f) dt = CONTROL_PERIOD_MS * 0.001f;

  // Seed the filter from the accelerometer (and magnetometer if trusted) so it
  // starts at the real attitude instead of converging from identity.
  if (!fusionStarted) {
    const float rollRad = atan2(accelY, accelZ);
    const float pitchRad = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));
    const float yawRad = magnetometerDataValid
                       ? tiltCompensatedMagYaw(rollRad, pitchRad) * DEG_TO_RAD
                       : 0.0f;
    setQuaternionFromEuler(rollRad, pitchRad, yawRad);
    fusionStarted = true;
  }

  const float gxRad = gyroX * DEG_TO_RAD;
  const float gyRad = gyroY * DEG_TO_RAD;
  const float gzRad = gyroZ * DEG_TO_RAD;

  if (magnetometerDataValid) {
    mahonyUpdate9Axis(gxRad, gyRad, gzRad, accelX, accelY, accelZ, magX, magY, magZ, dt);
  } else {
    mahonyUpdate6Axis(gxRad, gyRad, gzRad, accelX, accelY, accelZ, dt);
  }

  updateFusedYaw();
  yawDeg = normalizeDegrees(IMU_YAW_SIGN * fusedYaw - yawZeroDeg);
  yawRateDegS = IMU_YAW_SIGN * gyroZ;
}

// Make the current heading the new zero by capturing the filter's absolute
// yaw as the reference offset.
void zeroYaw() {
  yawZeroDeg = IMU_YAW_SIGN * fusedYaw;
  yawDeg = 0.0f;
}

void reinitFusion() {
  // The filter integrated the old bias/reference while it was drifting, so
  // restart it from the accelerometer rather than keeping a skewed quaternion.
  fusionStarted = false;
  updateIMU();
}

void printImuInfo() {
  Serial.print(F("INFO,IMU,"));
  if (!mpuFound) {
    Serial.println(F("NONE"));
    return;
  }
  switch (mpuWhoAmI) {
    case 0x70: Serial.print(F("MPU6500")); break;
    case 0x71: Serial.print(F("MPU9250")); break;
    case 0x73: Serial.print(F("MPU9255")); break;
    default:   Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F(",ADDR_0x"));
  Serial.print(mpuAddress, HEX);
  Serial.print(F(",MAG_"));
  Serial.print(magnetometerPresent ? F("PRESENT") : F("ABSENT"));
  Serial.print(F(",MAG_FUSION_"));
  Serial.println(useMagnetometer ? F("ON") : F("OFF"));
}

bool imuBegin() {
  Wire.begin();
  Wire.setClock(400000UL);

  detectMPU();
  if (mpuFound) {
    initMPU();
    initMagnetometer();
    Serial.println(F("INFO,KEEP_ROBOT_STILL_IMU_STARTUP_CALIBRATION"));
    calibrateGyro();
    lastFusionMicros = micros();
    updateIMU();
  }
  return mpuFound;
}
