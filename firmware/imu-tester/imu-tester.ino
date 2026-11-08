/*
 * imu-tester.ino: standalone MPU6050 test for the ESP32-S3 SuperMini
 *
 * Checks the wiring and shows live accelerometer / gyroscope data.
 * No libraries needed beyond Wire. Does not use the servos or the OLED.
 *
 * Wiring (same I2C pins as the main Sesame firmware):
 *   MPU6050 VCC -> 3V3      MPU6050 SDA -> GPIO6
 *   MPU6050 GND -> GND      MPU6050 SCL -> GPIO11
 *   AD0 low or unconnected = address 0x68, AD0 high = 0x69 (both tried)
 *
 * Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot = Enabled,
 * Serial Monitor at 115200 baud, line ending "Newline".
 *
 * Serial commands (type a letter and press Enter):
 *   s  scan the I2C bus and list every device found
 *   c  calibrate the gyro (keep the sensor still for ~1 s)
 *   p  pause / resume the live readout
 *   m  switch output: readable text <-> Serial Plotter format
 *   r  re-initialise the MPU6050
 *   h  show this help
 *
 * What "good" looks like, sensor lying flat and still:
 *   accel about 0, 0, +1 g      gyro about 0, 0, 0 deg/s (after calibration)
 *   pitch and roll about 0      temperature roughly room temperature
 */

#include <Wire.h>
#include <math.h>

#define I2C_SDA 6   // found with i2c-pin-finder
#define I2C_SCL 11
#define I2C_FREQ 400000

#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75

const float ACCEL_LSB_PER_G  = 16384.0f;  // +-2 g range
const float GYRO_LSB_PER_DPS = 131.0f;    // +-250 deg/s range
const float FILTER_ALPHA     = 0.98f;     // complementary filter gyro weight

const unsigned long SAMPLE_INTERVAL_MS = 10;   // 100 Hz sampling
const unsigned long PRINT_INTERVAL_MS  = 100;  // 10 Hz output

uint8_t mpuAddr = 0;
bool mpuOk = false;
bool paused = false;
bool plotterMode = false;

float gyroBias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, tempC;
float pitch = 0, roll = 0;
unsigned long lastSampleMs = 0, lastPrintMs = 0;
uint32_t readErrors = 0;

// ---------- I2C helpers ----------
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

void scanBus() {
  Serial.println("Scanning I2C bus (SDA=GPIO" + String(I2C_SDA) + ", SCL=GPIO" + String(I2C_SCL) + ")...");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found++;
      Serial.printf("  0x%02X", addr);
      if (addr == 0x68 || addr == 0x69) Serial.print("  <- MPU6050 / MPU6500 / MPU9250");
      else if (addr == 0x3C || addr == 0x3D) Serial.print("  <- SSD1306 OLED");
      else if (addr == 0x0C) Serial.print("  <- AK8963 magnetometer (MPU9250 bypass mode)");
      Serial.println();
    }
  }
  if (found == 0) {
    Serial.println("  No devices found. Check: VCC on 3V3, GND, SDA/SCL not swapped, solder joints.");
  } else {
    Serial.printf("  %d device(s) found.\n", found);
  }
}

// ---------- MPU6050 ----------
bool readRaw() {
  uint8_t b[14];
  if (!readRegs(mpuAddr, REG_ACCEL_XOUT_H, b, 14)) {
    readErrors++;
    return false;
  }
  ax = (int16_t)((b[0] << 8) | b[1]) / ACCEL_LSB_PER_G;
  ay = (int16_t)((b[2] << 8) | b[3]) / ACCEL_LSB_PER_G;
  az = (int16_t)((b[4] << 8) | b[5]) / ACCEL_LSB_PER_G;
  tempC = (int16_t)((b[6] << 8) | b[7]) / 340.0f + 36.53f;   // MPU6050 formula
  gx = (int16_t)((b[8] << 8) | b[9]) / GYRO_LSB_PER_DPS;
  gy = (int16_t)((b[10] << 8) | b[11]) / GYRO_LSB_PER_DPS;
  gz = (int16_t)((b[12] << 8) | b[13]) / GYRO_LSB_PER_DPS;
  return true;
}

void accelAngles(float& p, float& r) {
  r = atan2f(ay, az) * 180.0f / PI;
  p = atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI;
}

bool calibrateGyro() {
  Serial.println("Calibrating gyro: keep the sensor still...");
  float sum[3] = {0, 0, 0};
  int good = 0;
  for (int i = 0; i < 500; i++) {
    if (readRaw()) { sum[0] += gx; sum[1] += gy; sum[2] += gz; good++; }
    delay(2);
  }
  if (good < 250) {
    Serial.println("Calibration failed: too many read errors.");
    return false;
  }
  for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / good;
  Serial.printf("Gyro bias: %.2f  %.2f  %.2f deg/s\n", gyroBias[0], gyroBias[1], gyroBias[2]);
  if (fabsf(gyroBias[0]) > 10 || fabsf(gyroBias[1]) > 10 || fabsf(gyroBias[2]) > 10) {
    Serial.println("  Large bias: the sensor probably moved during calibration. Type 'c' to retry.");
  }
  return true;
}

bool initMpu() {
  mpuOk = false;
  mpuAddr = 0;
  const uint8_t candidates[] = {0x68, 0x69};
  uint8_t id = 0;
  for (uint8_t addr : candidates) {
    if (readRegs(addr, REG_WHO_AM_I, &id, 1)) { mpuAddr = addr; break; }
  }
  if (mpuAddr == 0) {
    Serial.println("MPU6050 NOT FOUND at 0x68 or 0x69.");
    scanBus();
    return false;
  }

  Serial.printf("Found device at 0x%02X, WHO_AM_I = 0x%02X ", mpuAddr, id);
  switch (id) {
    case 0x68: Serial.println("(MPU6050, as expected)"); break;
    case 0x70: Serial.println("(MPU6500, not an MPU6050: temperature reading will be off)"); break;
    case 0x71: Serial.println("(MPU9250, not an MPU6050: temperature reading will be off)"); break;
    case 0x73: Serial.println("(MPU9255, not an MPU6050: temperature reading will be off)"); break;
    default:   Serial.println("(unknown ID, possibly a clone chip: trying anyway)"); break;
  }

  bool ok = true;
  ok &= writeReg(REG_PWR_MGMT_1, 0x80);   // reset
  delay(100);
  ok &= writeReg(REG_PWR_MGMT_1, 0x01);   // wake, gyro PLL clock
  delay(10);
  ok &= writeReg(REG_SMPLRT_DIV, 0x09);   // 100 Hz
  ok &= writeReg(REG_CONFIG, 0x03);       // low-pass ~44 Hz
  ok &= writeReg(REG_GYRO_CONFIG, 0x00);  // +-250 deg/s
  ok &= writeReg(REG_ACCEL_CONFIG, 0x00); // +-2 g
  if (!ok) {
    Serial.println("Configuration writes failed.");
    return false;
  }

  // Sanity check: a still sensor should measure about 1 g in total.
  delay(50);
  if (readRaw()) {
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    Serial.printf("Accel magnitude: %.2f g ", mag);
    Serial.println((mag > 0.8f && mag < 1.2f) ? "(OK)" : "(expected about 1.0 g: check the sensor)");
  }

  mpuOk = true;
  calibrateGyro();
  if (readRaw()) accelAngles(pitch, roll);
  lastSampleMs = millis();
  return true;
}

void printHelp() {
  Serial.println("Commands: s=scan  c=calibrate  p=pause/resume  m=text/plotter  r=re-init  h=help");
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);  // wait for USB serial, max 3 s
  delay(200);

  Serial.println();
  Serial.println("===================================");
  Serial.println("   Sesame MPU6050 IMU Tester");
  Serial.println("===================================");
  printHelp();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  initMpu();
}

void loop() {
  // Short pause every pass. Without it, a loop that has nothing to do (e.g.
  // no MPU found) polls USB serial so fast that the USB interrupt is starved
  // and the interrupt watchdog resets the board (seen on ESP32 core 3.3.12).
  delay(1);

  // Serial commands
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 's': scanBus(); break;
      case 'c': if (mpuOk) calibrateGyro(); else Serial.println("No MPU6050. Type 'r' to retry."); break;
      case 'p': paused = !paused; Serial.println(paused ? "Paused." : "Resumed."); break;
      case 'm': plotterMode = !plotterMode; Serial.println(plotterMode ? "Plotter mode." : "Text mode."); break;
      case 'r': initMpu(); break;
      case 'h': printHelp(); break;
      default: break;  // ignore newlines and unknown keys
    }
  }

  if (!mpuOk) return;

  // Sample + filter at 100 Hz
  unsigned long now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {
    float dt = (now - lastSampleMs) / 1000.0f;
    lastSampleMs = now;
    if (readRaw()) {
      gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];
      float accP, accR;
      accelAngles(accP, accR);
      roll  = FILTER_ALPHA * (roll  + gx * dt) + (1 - FILTER_ALPHA) * accR;
      pitch = FILTER_ALPHA * (pitch + gy * dt) + (1 - FILTER_ALPHA) * accP;
    }
  }

  // Print at 10 Hz
  if (!paused && now - lastPrintMs >= PRINT_INTERVAL_MS) {
    lastPrintMs = now;
    if (plotterMode) {
      // Arduino Serial Plotter: label:value pairs
      Serial.printf("ax:%.2f,ay:%.2f,az:%.2f,pitch:%.1f,roll:%.1f\n", ax, ay, az, pitch, roll);
    } else {
      Serial.printf("acc g %6.2f %6.2f %6.2f | gyro dps %7.1f %7.1f %7.1f | pitch %6.1f roll %6.1f | %5.1f C | err %lu\n",
                    ax, ay, az, gx, gy, gz, pitch, roll, tempC, (unsigned long)readErrors);
    }
  }
}
