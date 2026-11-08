#pragma once
// ======================================================================
// IMU driver: MPU6050 / MPU6500 / MPU9250 (accelerometer + gyroscope)
// ======================================================================
// All three chips share the same accel/gyro register map, so one driver
// covers them. The chip is identified from its WHO_AM_I register.
// The MPU9250's magnetometer (a separate AK8963 die) is NOT used here.
//
// Wiring: shares the I2C bus with the OLED (OLED = 0x3C, IMU = 0x68).
//   VCC -> 3V3, GND -> GND, SDA -> I2C_SDA, SCL -> I2C_SCL
//   AD0 low (or floating) = 0x68, AD0 high = 0x69. Both are tried.
//
// The IMU is optional: if it is not found the robot still boots and runs.
//
// Pitch/roll axis directions depend on how the board is mounted on the
// robot. Check them with the "imu" serial command and adjust if needed.
// ======================================================================

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// --- Registers (common to MPU6050 / 6500 / 9250) ---
#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B   // 14 bytes: accel XYZ, temp, gyro XYZ
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

// Full-scale ranges used: +-2 g and +-250 deg/s
#define MPU_ACCEL_LSB_PER_G   16384.0f
#define MPU_GYRO_LSB_PER_DPS  131.0f

#define IMU_UPDATE_INTERVAL_MS 10      // 100 Hz
#define IMU_COMPLEMENTARY_ALPHA 0.98f  // gyro weight in the pitch/roll filter
#define IMU_GYRO_CAL_SAMPLES 200       // ~0.4 s, robot must be still

struct ImuData {
  float ax, ay, az;      // acceleration, g
  float gx, gy, gz;      // angular rate, deg/s (bias removed)
  float tempC;           // die temperature, deg C
  float pitch, roll;     // filtered orientation, degrees
  unsigned long lastUpdateMs;
};

bool imuPresent = false;
uint8_t imuAddr = 0;
uint8_t imuWhoAmI = 0;
const char* imuChipName = "none";
bool imuIsMpu6050 = true;            // selects the temperature formula
ImuData imu = {};
float imuGyroBias[3] = {0, 0, 0};
uint32_t imuReadErrors = 0;

static bool imuWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(imuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool imuReadRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static const char* imuNameForWhoAmI(uint8_t id) {
  switch (id) {
    case 0x68: return "MPU6050";
    case 0x70: return "MPU6500";
    case 0x71: return "MPU9250";
    case 0x73: return "MPU9255";
    default:   return nullptr;
  }
}

// Raw read into scaled units (no bias removal, no filtering).
static bool imuReadRaw(float& ax, float& ay, float& az,
                       float& gx, float& gy, float& gz, float& tempC) {
  uint8_t b[14];
  if (!imuReadRegs(imuAddr, MPU_REG_ACCEL_XOUT_H, b, 14)) {
    imuReadErrors++;
    return false;
  }
  int16_t rax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ray = (int16_t)((b[2] << 8) | b[3]);
  int16_t raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rt  = (int16_t)((b[6] << 8) | b[7]);
  int16_t rgx = (int16_t)((b[8] << 8) | b[9]);
  int16_t rgy = (int16_t)((b[10] << 8) | b[11]);
  int16_t rgz = (int16_t)((b[12] << 8) | b[13]);

  ax = rax / MPU_ACCEL_LSB_PER_G;
  ay = ray / MPU_ACCEL_LSB_PER_G;
  az = raz / MPU_ACCEL_LSB_PER_G;
  gx = rgx / MPU_GYRO_LSB_PER_DPS;
  gy = rgy / MPU_GYRO_LSB_PER_DPS;
  gz = rgz / MPU_GYRO_LSB_PER_DPS;
  tempC = imuIsMpu6050 ? (rt / 340.0f + 36.53f) : (rt / 333.87f + 21.0f);
  return true;
}

static void imuAccelAngles(float ax, float ay, float az, float& pitch, float& roll) {
  roll  = atan2f(ay, az) * 180.0f / PI;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
}

// Average the gyro while the robot is still to remove its zero offset.
bool imuCalibrateGyro() {
  if (!imuPresent) return false;
  float sum[3] = {0, 0, 0};
  int good = 0;
  for (int i = 0; i < IMU_GYRO_CAL_SAMPLES; i++) {
    float ax, ay, az, gx, gy, gz, t;
    if (imuReadRaw(ax, ay, az, gx, gy, gz, t)) {
      sum[0] += gx; sum[1] += gy; sum[2] += gz;
      good++;
    }
    delay(2);
  }
  if (good < IMU_GYRO_CAL_SAMPLES / 2) return false;
  for (int i = 0; i < 3; i++) imuGyroBias[i] = sum[i] / good;
  return true;
}

// Probe 0x68 then 0x69, configure the chip, calibrate the gyro.
// Call after Wire.begin(). Returns false (and leaves imuPresent = false)
// if no supported IMU answers.
bool imuBegin() {
  const uint8_t candidates[] = {0x68, 0x69};
  for (uint8_t addr : candidates) {
    uint8_t id;
    if (!imuReadRegs(addr, MPU_REG_WHO_AM_I, &id, 1)) continue;
    imuAddr = addr;
    imuWhoAmI = id;
    const char* name = imuNameForWhoAmI(id);
    if (name == nullptr) {
      // Clone chips sometimes report odd IDs. Try it anyway, but say so.
      Serial.printf("IMU: unknown WHO_AM_I 0x%02X at 0x%02X, treating as MPU-compatible\n", id, addr);
      name = "MPU-compatible (unknown ID)";
    }
    imuChipName = name;
    imuIsMpu6050 = (id == 0x68);
    break;
  }
  if (imuAddr == 0) {
    Serial.println("IMU: not found at 0x68 or 0x69. Continuing without IMU.");
    return false;
  }

  bool ok = true;
  ok &= imuWriteReg(MPU_REG_PWR_MGMT_1, 0x80);   // device reset
  delay(100);
  ok &= imuWriteReg(MPU_REG_PWR_MGMT_1, 0x01);   // wake, PLL clock from gyro
  delay(10);
  ok &= imuWriteReg(MPU_REG_SMPLRT_DIV, 0x09);   // 1 kHz / (1 + 9) = 100 Hz
  ok &= imuWriteReg(MPU_REG_CONFIG, 0x03);       // digital low-pass ~44 Hz
  ok &= imuWriteReg(MPU_REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
  ok &= imuWriteReg(MPU_REG_ACCEL_CONFIG, 0x00); // +-2 g
  if (!ok) {
    Serial.println("IMU: configuration write failed. Continuing without IMU.");
    imuAddr = 0;
    return false;
  }

  imuPresent = true;
  if (!imuCalibrateGyro()) {
    Serial.println("IMU: gyro calibration failed, using zero bias.");
  }

  // Seed the filter from the accelerometer so it starts at the right angle.
  float ax, ay, az, gx, gy, gz, t;
  if (imuReadRaw(ax, ay, az, gx, gy, gz, t)) {
    imuAccelAngles(ax, ay, az, imu.pitch, imu.roll);
  }
  imu.lastUpdateMs = millis();

  Serial.printf("IMU: %s (WHO_AM_I 0x%02X) at 0x%02X, gyro bias %.2f %.2f %.2f deg/s\n",
                imuChipName, imuWhoAmI, imuAddr,
                imuGyroBias[0], imuGyroBias[1], imuGyroBias[2]);
  return true;
}

// Non-blocking. Call often (loop and long waits); reads at IMU_UPDATE_INTERVAL_MS.
void imuUpdate() {
  if (!imuPresent) return;
  unsigned long now = millis();
  unsigned long elapsed = now - imu.lastUpdateMs;
  if (elapsed < IMU_UPDATE_INTERVAL_MS) return;

  float ax, ay, az, gx, gy, gz, t;
  if (!imuReadRaw(ax, ay, az, gx, gy, gz, t)) return;

  imu.ax = ax; imu.ay = ay; imu.az = az;
  imu.gx = gx - imuGyroBias[0];
  imu.gy = gy - imuGyroBias[1];
  imu.gz = gz - imuGyroBias[2];
  imu.tempC = t;

  float accPitch, accRoll;
  imuAccelAngles(ax, ay, az, accPitch, accRoll);

  float dt = elapsed / 1000.0f;
  if (dt > 0.1f) {
    // The loop was blocked for a while: gyro integration would be stale,
    // so restart the filter from the accelerometer.
    imu.pitch = accPitch;
    imu.roll = accRoll;
  } else {
    imu.roll  = IMU_COMPLEMENTARY_ALPHA * (imu.roll  + imu.gx * dt) + (1.0f - IMU_COMPLEMENTARY_ALPHA) * accRoll;
    imu.pitch = IMU_COMPLEMENTARY_ALPHA * (imu.pitch + imu.gy * dt) + (1.0f - IMU_COMPLEMENTARY_ALPHA) * accPitch;
  }
  imu.lastUpdateMs = now;
}

void imuPrint() {
  if (!imuPresent) {
    Serial.println("IMU: not present");
    return;
  }
  Serial.printf("IMU %s | acc g: %.2f %.2f %.2f | gyro dps: %.1f %.1f %.1f | pitch %.1f roll %.1f | %.1f C | errors %lu\n",
                imuChipName, imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz,
                imu.pitch, imu.roll, imu.tempC, (unsigned long)imuReadErrors);
}

String imuJson() {
  String json = "{\"present\":" + String(imuPresent ? "true" : "false");
  if (imuPresent) {
    json += ",\"chip\":\"" + String(imuChipName) + "\"";
    json += ",\"accel\":[" + String(imu.ax, 3) + "," + String(imu.ay, 3) + "," + String(imu.az, 3) + "]";
    json += ",\"gyro\":[" + String(imu.gx, 2) + "," + String(imu.gy, 2) + "," + String(imu.gz, 2) + "]";
    json += ",\"pitch\":" + String(imu.pitch, 1);
    json += ",\"roll\":" + String(imu.roll, 1);
    json += ",\"tempC\":" + String(imu.tempC, 1);
    json += ",\"errors\":" + String(imuReadErrors);
  }
  json += "}";
  return json;
}
