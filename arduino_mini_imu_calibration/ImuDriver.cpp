#include "ImuDriver.h"

#include <Wire.h>
#include <math.h>

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
static const uint8_t REG_TEMP_OUT_H = 0x41;

// AK8963 magnetometer, on the I2C bus once the MPU's bypass is enabled.
static const uint8_t AK8963_ADDR = 0x0C;
static const uint8_t AK8963_WHO_AM_I = 0x00;
static const uint8_t AK8963_ST1 = 0x02;
static const uint8_t AK8963_XOUT_L = 0x03;
static const uint8_t AK8963_CNTL1 = 0x0A;
static const uint8_t AK8963_ASAX = 0x10;

// Configured ranges: accel +/-2 g, gyro +/-250 deg/s, magnetometer 16-bit.
// Identical to the firmware, and the numbers this sketch prints only mean
// anything to the firmware because they are.
static const float ACCEL_SCALE = 16384.0f;
static const float GYRO_SCALE = 131.0f;
static const float MAG_SCALE = 4912.0f / 32760.0f;

// Datasheet temperature transfer function, room offset zero.
static const float TEMP_SENSITIVITY = 333.87f;
static const float TEMP_ROOM_C = 21.0f;

bool imuFound = false;
bool magPresent = false;
uint8_t imuAddress = MPU_ADDR_1;
uint8_t imuWhoAmI = 0x00;

float magAdj[AXIS_COUNT] = {1.0f, 1.0f, 1.0f};

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

// -----------------------------------------------------------------------------
// Bring-up
// -----------------------------------------------------------------------------

// WHO_AM_I distinguishes the three supported parts: 0x70 MPU6500 (no
// magnetometer), 0x71 MPU9250, 0x73 MPU9255. The module may sit at either I2C
// address depending on how its AD0 pin is strapped.
static void detectMPU() {
  const uint8_t id1 = readByte(MPU_ADDR_1, REG_WHO_AM_I);
  delay(10);
  const uint8_t id2 = readByte(MPU_ADDR_2, REG_WHO_AM_I);
  delay(10);

  if (id1 == 0x70 || id1 == 0x71 || id1 == 0x73) {
    imuAddress = MPU_ADDR_1;
    imuWhoAmI = id1;
    imuFound = true;
  } else if (id2 == 0x70 || id2 == 0x71 || id2 == 0x73) {
    imuAddress = MPU_ADDR_2;
    imuWhoAmI = id2;
    imuFound = true;
  } else {
    imuFound = false;
  }
}

static void initMPU() {
  writeByte(imuAddress, REG_PWR_MGMT_1, 0x00);   // wake
  delay(100);
  writeByte(imuAddress, REG_PWR_MGMT_1, 0x01);   // clock from gyro X PLL
  delay(100);
  writeByte(imuAddress, REG_CONFIG, 0x03);       // gyro DLPF 41 Hz
  writeByte(imuAddress, REG_SMPLRT_DIV, 0x04);   // 200 Hz sample rate
  writeByte(imuAddress, REG_GYRO_CONFIG, 0x00);  // +/-250 deg/s
  writeByte(imuAddress, REG_ACCEL_CONFIG, 0x00); // +/-2 g
  writeByte(imuAddress, REG_ACCEL_CONFIG2, 0x03);// accel DLPF 41 Hz
  writeByte(imuAddress, REG_INT_PIN_CFG, 0x02);  // I2C bypass: expose AK8963
  writeByte(imuAddress, REG_INT_ENABLE, 0x01);   // data-ready interrupt
  delay(100);
}

static void initMagnetometer() {
  magPresent = false;

  // The MPU6500 is the same die without the magnetometer.
  if (imuWhoAmI == 0x70) return;
  if (readByte(AK8963_ADDR, AK8963_WHO_AM_I) != 0x48) return;

  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00);  // power down
  delay(10);
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x0F);  // fuse ROM access
  delay(10);

  uint8_t asa[AXIS_COUNT];
  readBytes(AK8963_ADDR, AK8963_ASAX, AXIS_COUNT, asa);
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    magAdj[i] = ((float)(asa[i] - 128) / 256.0f) + 1.0f;
  }

  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x00);  // power down again
  delay(10);
  writeByte(AK8963_ADDR, AK8963_CNTL1, 0x16);  // continuous mode 2, 16-bit, 100 Hz
  delay(10);

  magPresent = true;
}

bool imuBegin() {
  Wire.begin();
  Wire.setClock(400000UL);

  detectMPU();
  if (!imuFound) return false;

  initMPU();
  initMagnetometer();
  return true;
}

// -----------------------------------------------------------------------------
// Reads
// -----------------------------------------------------------------------------

void imuReadRawAccelGyro(int16_t accel[AXIS_COUNT], int16_t gyro[AXIS_COUNT]) {
  uint8_t data[14];
  readBytes(imuAddress, REG_ACCEL_XOUT_H, 14, data);

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    accel[i] = ((int16_t)data[i * 2] << 8) | data[i * 2 + 1];
    // data[6..7] is temperature, so the gyro block starts at 8.
    gyro[i] = ((int16_t)data[8 + i * 2] << 8) | data[9 + i * 2];
  }
}

void imuReadAccelGyro(float accel[AXIS_COUNT], float gyro[AXIS_COUNT]) {
  int16_t rawAccel[AXIS_COUNT];
  int16_t rawGyro[AXIS_COUNT];
  imuReadRawAccelGyro(rawAccel, rawGyro);

  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    accel[i] = (float)rawAccel[i] / ACCEL_SCALE;
    gyro[i] = (float)rawGyro[i] / GYRO_SCALE;
  }
}

bool imuReadMag(float mag[AXIS_COUNT]) {
  if (!magPresent) return false;
  if (!(readByte(AK8963_ADDR, AK8963_ST1) & 0x01)) return false;

  uint8_t data[7];
  readBytes(AK8963_ADDR, AK8963_XOUT_L, 7, data);
  if (data[6] & 0x08) return false;   // magnetic overflow, sample is meaningless

  // The AK8963 is little-endian, unlike the MPU registers above.
  for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
    const int16_t raw = ((int16_t)data[i * 2 + 1] << 8) | data[i * 2];
    mag[i] = (float)raw * MAG_SCALE * magAdj[i];
  }
  return true;
}

float imuReadTempC() {
  uint8_t data[2];
  readBytes(imuAddress, REG_TEMP_OUT_H, 2, data);
  const int16_t raw = ((int16_t)data[0] << 8) | data[1];
  return (float)raw / TEMP_SENSITIVITY + TEMP_ROOM_C;
}

void imuPrintInfo() {
  Serial.print(F("INFO,IMU,"));
  if (!imuFound) {
    Serial.println(F("NONE,CHECK_SDA_A4_SCL_A5_AND_POWER"));
    return;
  }
  switch (imuWhoAmI) {
    case 0x70: Serial.print(F("MPU6500")); break;
    case 0x71: Serial.print(F("MPU9250")); break;
    case 0x73: Serial.print(F("MPU9255")); break;
    default:   Serial.print(F("UNKNOWN")); break;
  }
  Serial.print(F(",ADDR_0x"));
  Serial.print(imuAddress, HEX);
  Serial.print(F(",MAG_"));
  Serial.print(magPresent ? F("PRESENT") : F("ABSENT"));
  if (magPresent) {
    Serial.print(F(",MAG_ASA,"));
    for (uint8_t i = 0; i < AXIS_COUNT; ++i) {
      if (i > 0) Serial.print(',');
      Serial.print(magAdj[i], 3);
    }
  }
  Serial.println();
}
