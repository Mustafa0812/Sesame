/*
 * imu-servo-tester.ino: MPU6050 + servos working together (ESP32-S3 SuperMini)
 *
 * Checks that the IMU keeps working while servos run (no brown-outs or I2C
 * errors), gives a simple "servo follows tilt" demo, and measures the servo
 * supply voltage / current / power with an INA260, shown on the OLED and serial.
 * Libraries: ESP32Servo, Adafruit SSD1306, Adafruit GFX. (IMU and INA260 are
 * read directly over Wire, no library.) The IMU, OLED and INA260 are each
 * optional: the sketch runs without them and tells you what it didn't find.
 *
 * WIRING (same as the main Sesame firmware)
 *   All three I2C devices share one bus: SDA -> GPIO6, SCL -> GPIO11,
 *   VCC -> 3V3, GND -> GND.
 *     MPU6050 IMU  address 0x68
 *     SSD1306 OLED address 0x3C
 *     INA260       address 0x40 (A0/A1 pins left at default)
 *   INA260 power path: put it IN SERIES with the servo supply's positive wire.
 *     external supply + -> INA260 IN+,  INA260 IN- -> servo V+ rail.
 *     Supply GND, servo GND and board GND all connected together.
 *     (INA260 limits: 36 V, 15 A. Terminal labels vary between breakouts.)
 *   Servo signal wires: servo 0..7 -> GPIO 4, 5, 1, 7, 10, 2, 12, 13
 *   POWER THE SERVOS FROM AN EXTERNAL 5-6 V SUPPLY, not the board's 5V pin, and
 *   connect that supply's GND to the board's GND.
 *
 * SAFETY
 *   - Servos start OFF (limp). Nothing moves until you send a command.
 *   - Start with ONE servo, not attached to the robot's legs, then add more.
 *   - The pulse range 732-2929 us is wider than many servos accept. If a servo
 *     buzzes or stalls at the ends, narrow it with MIN_PULSE / MAX_PULSE below.
 *   - Type "stop" to cut power to all servos at any time.
 *
 * Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot = Enabled,
 * Serial Monitor 115200 baud, line ending "Newline".
 *
 * COMMANDS
 *   <id> <angle>    move servo id (0-7) to angle (0-180), e.g.  "0 90"
 *   all <angle>     move all 8 servos, e.g.  "all 90"
 *   sweep <id>      slowly sweep one servo between 60 and 120 degrees
 *   tilt <a> <b>    servo a follows ROLL, servo b follows PITCH (centre = 90)
 *   gain <x>        tilt gain: servo degrees per degree of tilt (default 1.0)
 *   flip            reverse the direction of both tilt servos
 *   stop            stop any mode and power down (detach) all servos
 *   imu cal         recalibrate the gyro (keep still)
 *   peak            reset the peak-current reading
 *   p               pause / resume the status printout
 *   h               help
 */

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ---------- Hardware ----------
#define I2C_SDA 6
#define I2C_SCL 11
const int servoPins[8] = {4, 5, 1, 7, 10, 2, 12, 13};
const int MIN_PULSE = 732;    // us at 0 degrees (same as the main firmware)
const int MAX_PULSE = 2929;   // us at 180 degrees

// OLED (SSD1306 128x64)
#define OLED_I2C_ADDR 0x3C
const unsigned long OLED_INTERVAL_MS = 250;     // each refresh blocks I2C ~25 ms, so keep it slow

// INA260 power monitor
const unsigned long INA_INTERVAL_MS = 20;       // 50 Hz
const float LOW_VOLTAGE_WARN = 4.5f;            // volts: flags a sagging servo supply
const int   CURRENT_BAR_MAX_MA = 3000;          // full-scale of the bar on the OLED

// ---------- Behaviour ----------
const int SWEEP_MIN = 60, SWEEP_MAX = 120;      // sweep range, degrees
const unsigned long SWEEP_STEP_MS = 20;         // 1 degree per step
const int TILT_MIN = 30, TILT_MAX = 150;        // tilt servos stay inside this range
const unsigned long CONTROL_INTERVAL_MS = 20;   // tilt update, 50 Hz
const float TILT_SMOOTHING = 0.2f;              // 0..1, higher = faster response
const unsigned long PRINT_INTERVAL_MS = 250;

// ---------- IMU registers (MPU6050) ----------
#define REG_SMPLRT_DIV   0x19
#define REG_CONFIG       0x1A
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1   0x6B
#define REG_WHO_AM_I     0x75
const float ACCEL_LSB_PER_G  = 16384.0f;
const float GYRO_LSB_PER_DPS = 131.0f;
const float FILTER_ALPHA     = 0.98f;
const unsigned long IMU_INTERVAL_MS = 10;       // 100 Hz

enum Mode { MODE_IDLE, MODE_SWEEP, MODE_TILT };

Servo servos[8];
bool attached[8] = {false};
int lastAngle[8] = {-1, -1, -1, -1, -1, -1, -1, -1};

Mode mode = MODE_IDLE;
int sweepId = 0, sweepAngle = SWEEP_MIN, sweepDir = 1;
int tiltRollId = 0, tiltPitchId = 1;
float tiltGain = 1.0f;
int tiltSign = 1;
float tiltRollCmd = 90, tiltPitchCmd = 90;

uint8_t mpuAddr = 0;
bool mpuOk = false;
float gyroBias[3] = {0, 0, 0};
float ax, ay, az, gx, gy, gz, pitch = 0, roll = 0;
uint32_t readErrors = 0, readOk = 0;
unsigned long lastImuMs = 0, lastControlMs = 0, lastPrintMs = 0, lastSweepMs = 0;
bool printPaused = false;

// ---------- I2C / IMU ----------
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

bool readRaw() {
  uint8_t b[14];
  if (!readRegs(mpuAddr, REG_ACCEL_XOUT_H, b, 14)) { readErrors++; return false; }
  readOk++;
  ax = (int16_t)((b[0] << 8) | b[1]) / ACCEL_LSB_PER_G;
  ay = (int16_t)((b[2] << 8) | b[3]) / ACCEL_LSB_PER_G;
  az = (int16_t)((b[4] << 8) | b[5]) / ACCEL_LSB_PER_G;
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
  for (int i = 0; i < 400; i++) {
    if (readRaw()) { sum[0] += gx; sum[1] += gy; sum[2] += gz; good++; }
    delay(2);
  }
  if (good < 200) { Serial.println("Calibration failed (too many read errors)."); return false; }
  for (int i = 0; i < 3; i++) gyroBias[i] = sum[i] / good;
  Serial.printf("Gyro bias: %.2f %.2f %.2f deg/s\n", gyroBias[0], gyroBias[1], gyroBias[2]);
  return true;
}

bool initImu() {
  const uint8_t candidates[] = {0x68, 0x69};
  uint8_t id = 0;
  for (uint8_t addr : candidates) {
    if (readRegs(addr, REG_WHO_AM_I, &id, 1)) { mpuAddr = addr; break; }
  }
  if (mpuAddr == 0) {
    Serial.println("IMU NOT FOUND at 0x68/0x69. Servo commands still work.");
    return false;
  }
  Serial.printf("IMU found at 0x%02X, WHO_AM_I 0x%02X\n", mpuAddr, id);
  bool ok = true;
  ok &= writeReg(REG_PWR_MGMT_1, 0x80);
  delay(100);
  ok &= writeReg(REG_PWR_MGMT_1, 0x01);
  delay(10);
  ok &= writeReg(REG_SMPLRT_DIV, 0x09);
  ok &= writeReg(REG_CONFIG, 0x03);
  ok &= writeReg(REG_GYRO_CONFIG, 0x00);
  ok &= writeReg(REG_ACCEL_CONFIG, 0x00);
  if (!ok) { Serial.println("IMU configuration failed."); return false; }
  mpuOk = true;
  calibrateGyro();
  if (readRaw()) accelAngles(pitch, roll);
  lastImuMs = millis();
  return true;
}

void updateImu() {
  if (!mpuOk) return;
  unsigned long now = millis();
  unsigned long elapsed = now - lastImuMs;
  if (elapsed < IMU_INTERVAL_MS) return;
  if (!readRaw()) return;   // counts the error, keeps the previous angles
  gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];
  float accP, accR;
  accelAngles(accP, accR);
  float dt = elapsed / 1000.0f;
  if (dt > 0.1f) {          // we were blocked for a while: restart from the accelerometer
    pitch = accP; roll = accR;
  } else {
    roll  = FILTER_ALPHA * (roll  + gx * dt) + (1 - FILTER_ALPHA) * accR;
    pitch = FILTER_ALPHA * (pitch + gy * dt) + (1 - FILTER_ALPHA) * accP;
  }
  lastImuMs = now;
}

// ---------- INA260 (servo supply voltage / current / power) ----------
#define INA_REG_CONFIG  0x00
#define INA_REG_CURRENT 0x01   // signed, 1.25 mA per LSB
#define INA_REG_VOLTAGE 0x02   // unsigned, 1.25 mV per LSB
#define INA_REG_POWER   0x03   // unsigned, 10 mW per LSB
#define INA_REG_MFR_ID  0xFE   // reads 0x5449 ("TI")
#define INA_REG_DIE_ID  0xFF   // upper 12 bits read 0x227 for the INA260

uint8_t inaAddr = 0;
bool inaOk = false;
float busV = 0;
int currentMa = 0, peakMa = 0, powerMw = 0;
uint32_t inaErrors = 0;
unsigned long lastInaMs = 0;

bool inaRead16(uint8_t addr, uint8_t reg, uint16_t& val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = 2;
  if (Wire.requestFrom(addr, n) != n) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  val = ((uint16_t)hi << 8) | lo;
  return true;
}

bool inaWrite16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

bool initIna() {
  // Look for an INA260 at 0x40-0x4F by checking its ID registers.
  for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
    uint16_t mfr = 0, die = 0;
    if (!inaRead16(addr, INA_REG_MFR_ID, mfr)) continue;
    if (!inaRead16(addr, INA_REG_DIE_ID, die)) continue;
    if (mfr == 0x5449 && (die >> 4) == 0x227) { inaAddr = addr; break; }
    Serial.printf("Device at 0x%02X is not an INA260 (MFR 0x%04X, DIE 0x%04X).\n", addr, mfr, die);
  }
  if (inaAddr == 0) {
    Serial.println("INA260 NOT FOUND at 0x40-0x4F. Power readings disabled.");
    return false;
  }
  inaWrite16(inaAddr, INA_REG_CONFIG, 0x8000);   // reset
  delay(5);
  // Config: 4-sample averaging, 1.1 ms conversions, continuous (update about every 9 ms)
  if (!inaWrite16(inaAddr, INA_REG_CONFIG, 0x6327)) {
    Serial.println("INA260 configuration failed. Power readings disabled.");
    return false;
  }
  Serial.printf("INA260 found at 0x%02X\n", inaAddr);
  inaOk = true;
  return true;
}

void updateIna() {
  if (!inaOk) return;
  unsigned long now = millis();
  if (now - lastInaMs < INA_INTERVAL_MS) return;
  lastInaMs = now;
  uint16_t rawI, rawV, rawP;
  if (!inaRead16(inaAddr, INA_REG_CURRENT, rawI) ||
      !inaRead16(inaAddr, INA_REG_VOLTAGE, rawV) ||
      !inaRead16(inaAddr, INA_REG_POWER, rawP)) {
    inaErrors++;
    return;
  }
  currentMa = (int)lroundf((int16_t)rawI * 1.25f);
  busV = rawV * 0.00125f;
  powerMw = (int)rawP * 10;
  if (abs(currentMa) > peakMa) peakMa = abs(currentMa);
}

// ---------- OLED ----------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool oledOk = false;
unsigned long lastOledMs = 0;

int servosOn() {
  int n = 0;
  for (int i = 0; i < 8; i++) if (attached[i]) n++;
  return n;
}

const char* modeName() {
  return (mode == MODE_IDLE) ? "idle" : (mode == MODE_SWEEP ? "sweep" : "tilt");
}

bool initOled() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("OLED NOT FOUND at 0x3C. Display disabled.");
    return false;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("IMU + servo tester");
  display.display();
  oledOk = true;
  Serial.println("OLED found at 0x3C");
  return true;
}

void updateOled() {
  if (!oledOk) return;
  unsigned long now = millis();
  if (now - lastOledMs < OLED_INTERVAL_MS) return;
  lastOledMs = now;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Top line: mode, servos on, tilt angles
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (mpuOk) display.printf("%s on%d P%.0f R%.0f", modeName(), servosOn(), pitch, roll);
  else       display.printf("%s on%d  no IMU", modeName(), servosOn());

  if (inaOk) {
    bool lowV = busV < LOW_VOLTAGE_WARN;
    display.setTextSize(2);
    display.setCursor(0, 10);
    display.printf("%.2f V", busV);
    display.setCursor(0, 28);
    display.printf("%d mA", currentMa);
    if (lowV) {
      display.setTextSize(1);
      display.setCursor(92, 14);
      display.print("LOW V");
    }
    // Current bar
    int w = constrain((int)((long)abs(currentMa) * 126 / CURRENT_BAR_MAX_MA), 0, 126);
    display.drawRect(0, 46, 128, 6, SSD1306_WHITE);
    display.fillRect(1, 47, w, 4, SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 55);
    display.printf("pk %d mA  %.1f W", peakMa, powerMw / 1000.0f);
  } else {
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("INA260 not found");
  }
  display.display();
}

// ---------- Servos ----------
void attachServo(int id) {
  if (attached[id]) return;
  servos[id].setPeriodHertz(50);
  servos[id].attach(servoPins[id], MIN_PULSE, MAX_PULSE);
  attached[id] = true;
}

void moveServo(int id, int angle) {
  angle = constrain(angle, 0, 180);
  attachServo(id);
  servos[id].write(angle);
  lastAngle[id] = angle;
}

void stopAll() {
  mode = MODE_IDLE;
  for (int i = 0; i < 8; i++) {
    if (attached[i]) { servos[i].detach(); attached[i] = false; }
    lastAngle[i] = -1;
  }
}

// ---------- Modes ----------
void updateSweep() {
  if (mode != MODE_SWEEP) return;
  unsigned long now = millis();
  if (now - lastSweepMs < SWEEP_STEP_MS) return;
  lastSweepMs = now;
  sweepAngle += sweepDir;
  if (sweepAngle >= SWEEP_MAX) { sweepAngle = SWEEP_MAX; sweepDir = -1; }
  if (sweepAngle <= SWEEP_MIN) { sweepAngle = SWEEP_MIN; sweepDir = 1; }
  moveServo(sweepId, sweepAngle);
}

void updateTilt() {
  if (mode != MODE_TILT || !mpuOk) return;
  unsigned long now = millis();
  if (now - lastControlMs < CONTROL_INTERVAL_MS) return;
  lastControlMs = now;
  float targetRoll  = constrain(90 + tiltSign * tiltGain * roll,  TILT_MIN, TILT_MAX);
  float targetPitch = constrain(90 + tiltSign * tiltGain * pitch, TILT_MIN, TILT_MAX);
  tiltRollCmd  += (targetRoll  - tiltRollCmd)  * TILT_SMOOTHING;
  tiltPitchCmd += (targetPitch - tiltPitchCmd) * TILT_SMOOTHING;
  moveServo(tiltRollId,  (int)lroundf(tiltRollCmd));
  moveServo(tiltPitchId, (int)lroundf(tiltPitchCmd));
}

// ---------- Commands ----------
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  <id> <angle>   move one servo (id 0-7, angle 0-180)");
  Serial.println("  all <angle>    move all servos");
  Serial.println("  sweep <id>     sweep one servo 60-120");
  Serial.println("  tilt <a> <b>   servo a follows roll, servo b follows pitch");
  Serial.println("  gain <x>       tilt gain      flip   reverse tilt direction");
  Serial.println("  stop           stop and power down all servos");
  Serial.println("  imu cal        recalibrate gyro (keep still)");
  Serial.println("  peak           reset the peak-current reading");
  Serial.println("  p              pause/resume printout      h   help");
}

void handleCommand(char* cmd) {
  int a, b;
  float f;
  if (strcmp(cmd, "h") == 0) printHelp();
  else if (strcmp(cmd, "p") == 0) { printPaused = !printPaused; Serial.println(printPaused ? "Printout paused." : "Printout resumed."); }
  else if (strcmp(cmd, "stop") == 0) { stopAll(); Serial.println("All servos OFF (limp)."); }
  else if (strcmp(cmd, "imu cal") == 0) { if (mpuOk) calibrateGyro(); else Serial.println("No IMU."); }
  else if (strcmp(cmd, "peak") == 0) { peakMa = 0; Serial.println("Peak current reset."); }
  else if (strcmp(cmd, "flip") == 0) { tiltSign = -tiltSign; Serial.printf("Tilt direction %s.\n", tiltSign > 0 ? "normal" : "reversed"); }
  else if (sscanf(cmd, "gain %f", &f) == 1) { tiltGain = constrain(f, 0.1f, 3.0f); Serial.printf("Tilt gain %.2f\n", tiltGain); }
  else if (sscanf(cmd, "all %d", &a) == 1) {
    mode = MODE_IDLE;
    for (int i = 0; i < 8; i++) moveServo(i, a);
    Serial.printf("All servos -> %d\n", constrain(a, 0, 180));
  }
  else if (sscanf(cmd, "sweep %d", &a) == 1) {
    if (a < 0 || a > 7) { Serial.println("Servo id must be 0-7."); return; }
    sweepId = a; sweepAngle = SWEEP_MIN; sweepDir = 1; mode = MODE_SWEEP;
    Serial.printf("Sweeping servo %d between %d and %d. 'stop' to end.\n", a, SWEEP_MIN, SWEEP_MAX);
  }
  else if (sscanf(cmd, "tilt %d %d", &a, &b) == 2) {
    if (!mpuOk) { Serial.println("No IMU: tilt mode needs it."); return; }
    if (a < 0 || a > 7 || b < 0 || b > 7 || a == b) { Serial.println("Need two different servo ids 0-7."); return; }
    tiltRollId = a; tiltPitchId = b; tiltRollCmd = 90; tiltPitchCmd = 90; mode = MODE_TILT;
    Serial.printf("Tilt mode: servo %d follows roll, servo %d follows pitch. 'flip' reverses, 'stop' ends.\n", a, b);
  }
  else if (sscanf(cmd, "%d %d", &a, &b) == 2) {
    if (a < 0 || a > 7) { Serial.println("Servo id must be 0-7."); return; }
    mode = MODE_IDLE;
    moveServo(a, b);
    Serial.printf("Servo %d -> %d\n", a, constrain(b, 0, 180));
  }
  else Serial.println("Unknown command. Type h for help.");
}

void readSerial() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = 0; handleCommand(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

void printStatus() {
  if (printPaused || millis() - lastPrintMs < PRINT_INTERVAL_MS) return;
  lastPrintMs = millis();
  const char* m = (mode == MODE_IDLE) ? "idle " : (mode == MODE_SWEEP ? "sweep" : "tilt ");
  int active = 0;
  for (int i = 0; i < 8; i++) if (attached[i]) active++;
  if (mpuOk) {
    Serial.printf("[%s] servos on %d | pitch %6.1f roll %6.1f | gyro %6.1f %6.1f %6.1f | imu ok %lu err %lu",
                  m, active, pitch, roll, gx, gy, gz, (unsigned long)readOk, (unsigned long)readErrors);
  } else {
    Serial.printf("[%s] servos on %d | no IMU", m, active);
  }
  if (inaOk) {
    Serial.printf(" | %.2f V %d mA (pk %d) %.2f W err %lu%s", busV, currentMa, peakMa,
                  powerMw / 1000.0f, (unsigned long)inaErrors, busV < LOW_VOLTAGE_WARN ? " LOW V!" : "");
  }
  if (mode == MODE_TILT) Serial.printf(" | servo%d=%d servo%d=%d", tiltRollId, lastAngle[tiltRollId], tiltPitchId, lastAngle[tiltPitchId]);
  Serial.println();
}

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  delay(200);

  Serial.println();
  Serial.println("===================================");
  Serial.println("   Sesame IMU + Servo Tester");
  Serial.println("===================================");
  Serial.println("Servos are OFF (limp). Power them from an external 5-6 V supply.");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  initImu();
  initIna();
  initOled();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  printHelp();
}

void loop() {
  delay(1);   // keep the USB interrupt serviced (avoids a watchdog reset on core 3.x)
  readSerial();
  updateImu();
  updateIna();
  updateSweep();
  updateTilt();
  updateOled();
  printStatus();
}
