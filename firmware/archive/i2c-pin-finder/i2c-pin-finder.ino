/*
 * i2c-pin-finder.ino: find which GPIOs an I2C device (MPU6050, OLED...) is wired to
 *
 * Use this when the I2C scan finds nothing and you don't trust the pinout.
 * It does not need to know which pins you used.
 *
 * HOW IT WORKS
 *  Phase 1, pin state test: every candidate GPIO is read with the internal
 *    pull-down on, then with the pull-up on.
 *      - HIGH even with the pull-down  -> something external holds it high.
 *        An MPU6050 module (GY-521) has 4.7k pull-ups on SDA and SCL, so the two
 *        wired I2C pins show up this way. It also proves the module has power.
 *      - LOW even with the pull-up     -> the pin is tied to GND (or shorted).
 *      - follows the pull              -> floating / not connected. Normal.
 *  Phase 2, pair scan: tries every (SDA, SCL) combination among the HIGH pins
 *    and reports any I2C device that answers.
 *
 * BEFORE YOU RUN IT
 *  - Disconnect the servos (the scan toggles pins and could twitch them).
 *  - Wire ONLY the MPU6050 to the board: VCC -> 3V3, GND -> GND, SDA and SCL to
 *    two GPIO pads. Do not connect VCC to a GPIO.
 *
 * Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot = Enabled,
 * Serial Monitor 115200 baud, line ending "Newline".
 *
 * Commands:  r = run again    f = force a pair scan over ALL candidate pins
 *            (slow; the pins are driven, so keep servos disconnected)
 */

#include <Wire.h>

// GPIOs worth testing on an ESP32-S3. Left out on purpose: 0 (BOOT), 19/20 (USB),
// 26-37 (internal flash/PSRAM), 43/44 (UART0), 45/46 (strapping).
const int CANDIDATES[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13,
                          14, 15, 16, 17, 18, 21, 38, 39, 40, 41, 42, 47, 48};
const int NUM_CAND = sizeof(CANDIDATES) / sizeof(CANDIDATES[0]);

// Addresses worth probing: MPU6050/6500/9250, OLED, AK8963, common others.
const uint8_t PROBE_ADDRS[] = {0x68, 0x69, 0x3C, 0x3D, 0x0C, 0x1E, 0x40, 0x76, 0x77};
const int NUM_PROBE = sizeof(PROBE_ADDRS) / sizeof(PROBE_ADDRS[0]);

enum PinState { PIN_FLOATING, PIN_HIGH, PIN_LOW, PIN_ODD };
PinState pinStates[NUM_CAND];

const char* nameOf(uint8_t addr) {
  switch (addr) {
    case 0x68: case 0x69: return "MPU6050 / MPU6500 / MPU9250";
    case 0x3C: case 0x3D: return "SSD1306 OLED";
    case 0x0C: return "AK8963 magnetometer";
    case 0x1E: return "HMC5883L / LIS3MDL";
    case 0x40: return "PCA9685 / INA219 / similar";
    case 0x76: case 0x77: return "BMP280 / BME280";
    default: return "unknown";
  }
}

void testPins() {
  Serial.println("---- Phase 1: pin state test ----");
  Serial.println("GPIO  pulldown  pullup   verdict");
  for (int i = 0; i < NUM_CAND; i++) {
    int p = CANDIDATES[i];
    pinMode(p, INPUT_PULLDOWN); delay(3);
    int withDown = digitalRead(p);
    pinMode(p, INPUT_PULLUP);   delay(3);
    int withUp = digitalRead(p);
    pinMode(p, INPUT);

    const char* verdict;
    if (withDown == 1 && withUp == 1)      { pinStates[i] = PIN_HIGH;     verdict = "HIGH even with pulldown  <-- held high externally (module pull-up?)"; }
    else if (withDown == 0 && withUp == 0) { pinStates[i] = PIN_LOW;      verdict = "LOW even with pullup     <-- tied to GND / held low"; }
    else if (withDown == 0 && withUp == 1) { pinStates[i] = PIN_FLOATING; verdict = "floating (not connected)"; }
    else                                   { pinStates[i] = PIN_ODD;      verdict = "odd (high with pulldown, low with pullup)"; }
    Serial.printf("%4d  %8d  %6d   %s\n", p, withDown, withUp, verdict);
  }
}

// Returns number of devices found for this (sda, scl) pair; prints each one.
int probePair(int sda, int scl) {
  Wire.end();
  if (!Wire.begin(sda, scl, 100000)) return 0;
  Wire.setTimeOut(10);
  int found = 0;
  for (int k = 0; k < NUM_PROBE; k++) {
    Wire.beginTransmission(PROBE_ADDRS[k]);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  >>> FOUND 0x%02X (%s) with SDA=GPIO%d, SCL=GPIO%d\n",
                    PROBE_ADDRS[k], nameOf(PROBE_ADDRS[k]), sda, scl);
      found++;
    }
  }
  Wire.end();
  return found;
}

void pairScan(bool allPins) {
  int pool[NUM_CAND];
  int n = 0;
  for (int i = 0; i < NUM_CAND; i++) {
    if (allPins || pinStates[i] == PIN_HIGH) pool[n++] = CANDIDATES[i];
  }
  Serial.printf("---- Phase 2: pair scan over %d pin(s)%s ----\n", n, allPins ? " (ALL candidates)" : " (HIGH pins only)");
  int total = 0;
  for (int a = 0; a < n; a++) {
    for (int b = 0; b < n; b++) {
      if (a == b) continue;
      total += probePair(pool[a], pool[b]);
    }
  }
  Serial.printf("Pair scan finished: %d match(es).\n", total);
  if (total == 0) {
    Serial.println("No device answered on any pair. See the diagnosis below.");
  }
}

void diagnose() {
  int highCount = 0, lowCount = 0;
  for (int i = 0; i < NUM_CAND; i++) {
    if (pinStates[i] == PIN_HIGH) highCount++;
    if (pinStates[i] == PIN_LOW) lowCount++;
  }
  Serial.println("---- Diagnosis ----");
  if (highCount == 0) {
    Serial.println("No pin is held high by an external pull-up. So either:");
    Serial.println("  1. The SDA/SCL wires don't reach the module (broken wire, unsoldered header,");
    Serial.println("     jumper not seated, breadboard contact).");
    Serial.println("  2. The module has no power (VCC not on 3V3, GND missing, module dead).");
    Serial.println("  3. The wires go to GPIOs not tested here (0, 19, 20, 26-37, 43-46).");
    Serial.println("  4. Your module has no pull-up resistors (some bare MPU chips don't).");
    Serial.println("     In that case type 'f' to force a full pair scan.");
  } else if (highCount == 1) {
    Serial.println("Only ONE pin is held high. SDA or SCL isn't connected, or one of the");
    Serial.println("two wires is on a pin that isn't tested here.");
  } else {
    Serial.printf("%d pins are held high. SDA and SCL should be two of them.\n", highCount);
    Serial.println("If Phase 2 found nothing: check the MPU6050's own wiring, e.g. AD0, or a dead chip.");
  }
  if (lowCount > 0) {
    Serial.println("Pins reading LOW with pullup are tied to GND. If one of them is a pin you");
    Serial.println("meant to use for SDA/SCL, that wire may be touching GND.");
  }
}

void runAll(bool forceAll) {
  Serial.println();
  Serial.println("===================================");
  Serial.println("   Sesame I2C Pin Finder");
  Serial.println("===================================");
  testPins();
  int highCount = 0;
  for (int i = 0; i < NUM_CAND; i++) if (pinStates[i] == PIN_HIGH) highCount++;
  if (forceAll || highCount >= 2) {
    pairScan(forceAll);
  } else {
    Serial.println("---- Phase 2 skipped: fewer than 2 pins held high ----");
  }
  diagnose();
  Serial.println();
  Serial.println("Type 'r' to run again, 'f' to force a scan of all pins.");
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  delay(300);
  runAll(false);
}

void loop() {
  delay(1);   // keep the USB interrupt serviced (avoids a watchdog reset on core 3.x)
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') runAll(false);
    else if (c == 'f') {
      Serial.println("Forcing full scan. Servos must be disconnected.");
      runAll(true);
    }
  }
}
