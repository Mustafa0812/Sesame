/*
 * PCA9685 servo tester (ESP32-S3 SuperMini)
 *
 * No external libraries: talks to the PCA9685 (16 channel, 12-bit PWM) over
 * plain Wire. Shares the I2C bus with the MPU6050 (0x68) / OLED (0x3C) fine,
 * the PCA9685 sits at 0x40 by default.
 *
 * Wiring:
 *   PCA9685 VCC -> 3V3        (logic power only)
 *   PCA9685 GND -> GND        (common with ESP32 AND servo supply)
 *   PCA9685 SDA -> GPIO6
 *   PCA9685 SCL -> GPIO11
 *   PCA9685 V+  -> 5-6V servo supply (NOT from the ESP32 3V3 pin)
 *   Servo on channel 0..15 (signal / V+ / GND)
 *
 * Serial commands (115200, newline terminated):
 *   c<N>    select channel 0..15            e.g. c3
 *   a<deg>  stop sweep, move to angle 0..180 e.g. a90
 *   s       toggle sweep 0 <-> 180 on selected channel
 *   t       sweep ALL channels together (toggle)
 *   x       all channels off (servos go limp)
 *   ?       I2C scan
 */

#include <Wire.h>

#define I2C_SDA 6
#define I2C_SCL 11
#define PCA_ADDR 0x40

// PCA9685 registers
#define REG_MODE1     0x00
#define REG_MODE2     0x01
#define REG_LED0_ON_L 0x06
#define REG_ALL_LED_OFF_H 0xFD
#define REG_PRESCALE  0xFE

#define SERVO_HZ      50
#define OSC_HZ        25000000UL
#define SERVO_MIN_US  500     // pulse at 0 deg   (narrow to ~600 if servo buzzes at end)
#define SERVO_MAX_US  2500    // pulse at 180 deg (narrow to ~2400 if servo buzzes at end)

#define SWEEP_STEP_DEG 1
#define SWEEP_DELAY_MS 15

uint8_t channel = 0;
bool sweepOne = false;
bool sweepAll = false;
int sweepAngle = 0;
int sweepDir = 1;

void pcaWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t pcaRead(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool pcaInit() {
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() != 0) return false;

  pcaWrite(REG_MODE1, 0x10);                       // sleep (required to set prescale)
  uint8_t prescale = (uint8_t)(OSC_HZ / (4096UL * SERVO_HZ) - 1 + 0.5);  // ~121 for 50 Hz
  pcaWrite(REG_PRESCALE, prescale);
  pcaWrite(REG_MODE2, 0x04);                       // totem-pole outputs
  pcaWrite(REG_MODE1, 0x20);                       // wake, auto-increment
  delay(5);                                        // oscillator settle
  return true;
}

// Raw 12-bit PWM: on at tick 0, off at tick `off` (0..4095)
void setPwm(uint8_t ch, uint16_t off) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(REG_LED0_ON_L + 4 * ch);
  Wire.write(0);
  Wire.write(0);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

void servoOff(uint8_t ch) {
  // full-off bit (OFF_H bit 4) => no pulse, servo limp
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(REG_LED0_ON_L + 4 * ch);
  Wire.write(0); Wire.write(0); Wire.write(0); Wire.write(0x10);
  Wire.endTransmission();
}

void servoAngle(uint8_t ch, int deg) {
  deg = constrain(deg, 0, 180);
  uint32_t us = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * deg / 180;
  uint16_t ticks = (uint16_t)(us * 4096UL * SERVO_HZ / 1000000UL);   // us -> 12-bit ticks
  setPwm(ch, ticks);
}

void i2cScan() {
  Serial.println("I2C scan:");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("  found 0x%02X\n", a);
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line[0];
  int val = line.substring(1).toInt();

  switch (cmd) {
    case 'c':
      if (val >= 0 && val <= 15) {
        channel = val;
        Serial.printf("channel %d\n", channel);
      } else Serial.println("channel must be 0..15");
      break;
    case 'a':
      sweepOne = sweepAll = false;
      servoAngle(channel, val);
      Serial.printf("ch %d -> %d deg\n", channel, constrain(val, 0, 180));
      break;
    case 's':
      sweepOne = !sweepOne; sweepAll = false;
      Serial.println(sweepOne ? "sweep ON" : "sweep OFF");
      break;
    case 't':
      sweepAll = !sweepAll; sweepOne = false;
      Serial.println(sweepAll ? "sweep ALL ON" : "sweep ALL OFF");
      break;
    case 'x':
      sweepOne = sweepAll = false;
      for (uint8_t i = 0; i < 16; i++) servoOff(i);
      Serial.println("all off");
      break;
    case '?':
      i2cScan();
      break;
    default:
      Serial.println("cmds: c<N> a<deg> s t x ?");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!pcaInit()) {
    Serial.println("PCA9685 NOT found at 0x40 - check wiring (SDA 6, SCL 11) / address jumpers");
    i2cScan();
  } else {
    Serial.println("PCA9685 ready.");
    Serial.println("cmds: c<N> select channel, a<deg> angle, s sweep, t sweep all, x off, ? scan");
    servoAngle(channel, 90);   // start on center
  }
}

void loop() {
  handleSerial();

  if (sweepOne || sweepAll) {
    sweepAngle += sweepDir * SWEEP_STEP_DEG;
    if (sweepAngle >= 180) { sweepAngle = 180; sweepDir = -1; }
    if (sweepAngle <= 0)   { sweepAngle = 0;   sweepDir = 1; }

    if (sweepAll) for (uint8_t i = 0; i < 16; i++) servoAngle(i, sweepAngle);
    else          servoAngle(channel, sweepAngle);
    delay(SWEEP_DELAY_MS);
  }
}
