/*
 * ESP-NOW sequence tester - ESP32-S3 SuperMini, 8 servos per board
 *
 * Flash the SAME sketch to both boards (both bots use the main firmware's
 * 8-servo pin map). Then pick roles with serial commands:
 *
 *   leader     this board runs the real sequences from movement-sequences.h
 *              and streams all 8 servo angles over ESP-NOW (50 Hz while a
 *              sequence runs, 10 Hz keep-alive otherwise)
 *   follower   (default) mirrors the leader's 8 servos, with speed-limited
 *              (smoothed) motion so a lost packet does not cause a jerk
 *
 * Serial 115200, newline. Commands:
 *   help | status | leader | follower
 *   seq <name>   leader only. Names: rest stand wave dance swim point pushup
 *                bow cute freaky worm shake shrug dead crab
 *                forward backward left right   (walks repeat; 'stop' ends them)
 *   stop         ends a walking sequence (the only command accepted mid-sequence)
 *   power <dBm>  TX power 2-20 (default 14)
 *   slew <deg>   follower max movement per 10 ms (default 5 = 500 deg/s)
 *
 * Follower safety: angles clamped 0-180, stale/out-of-order poses dropped,
 * servos go to 90 if the leader is silent for FAILSAFE_MS.
 *
 * Power: all servos from the external 5-6 V supply, common ground with the
 * ESP32. No face display here: face calls are stubs.
 * Libraries: none beyond the ESP32 core (own LEDC servo driver, no ESP32Servo).
 * Enable "USB CDC On Boot".
 * NOTE: movement-sequences.h here is a copy of the one in sesame-firmware-main.
 */

#define ESPNOW_USE_STA   // station-mode ESP-NOW, no soft AP

#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "espnow-comms.h"
#include "movement-sequences.h"
#include "battery.h"

// ---------------------------------------------------------------- OLED + battery (INA260)
// I2C: SDA GPIO6, SCL GPIO11 (shared by OLED 0x3C and INA260 0x40). Power both from 3V3.
// INA260 goes on the BATTERY side, before the buck: battery + -> IN+, IN- -> buck input +.
// See battery.h for the method. On a bench supply type 'batt r 0' (no internal resistance).
enum Role { ROLE_LEADER, ROLE_FOLLOWER };
extern Role role;                            // defined with the other globals below
const int I2C_SDA_PIN = 6;
const int I2C_SCL_PIN = 11;
const unsigned long OLED_PERIOD_MS = 500;   // each refresh blocks I2C for ~25 ms at 400 kHz

Adafruit_SSD1306 oled(128, 64, &Wire, -1);
bool oledOk = false;
unsigned long lastOledMs = 0;
float battPeakMa = 0;
String boardId = "";                         // last 2 bytes of this chip's MAC, set in setup()

void drawBatteryScreen() {
  oled.clearDisplay();
  oled.setTextColor(1);
  oled.setTextWrap(false);

  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("ID ");
  oled.print(boardId);
  oled.setCursor(128 - 6 * 8, 0);
  oled.print(role == ROLE_LEADER ? "LEADER  " : "FOLLOWER");

  if (!battInaOk) {
    oled.setCursor(0, 24);
    oled.print("INA260 not found");
    oled.setCursor(0, 36);
    oled.print("(0x40, SDA6 SCL11)");
    oled.display();
    return;
  }

  bool unknown = battStale || !battHaveFilter;

  oled.setTextSize(3);
  oled.setCursor(0, 12);
  if (unknown) oled.print("--");
  else { oled.print(battPercent); oled.print("%"); }

  oled.setTextSize(1);
  oled.setCursor(0, 38);
  oled.printf("%.2fV %4.0fmA pk%4.0f", battBusV, battCurrentMa, battPeakMa);
  oled.setCursor(0, 48);
  oled.printf("cell %.2fV R%dmOhm", battOcvFiltered / BATT_CELLS, battInternalMohm);
  if (!unknown && battLow) { oled.setCursor(90, 14); oled.print("LOW!"); }

  // level bar
  oled.drawRect(0, 56, 128, 8, 1);
  if (!unknown) {
    int fw = (battPercent * 124 + 50) / 100;
    if (fw > 0) oled.fillRect(2, 58, fw, 4, 1);
  }
  oled.display();
}

void updateOled() {
  if (!oledOk) return;
  unsigned long now = millis();
  if (now - lastOledMs < OLED_PERIOD_MS) return;
  lastOledMs = now;
  if (battCurrentMa > battPeakMa) battPeakMa = battCurrentMa;
  drawBatteryScreen();
}

// ---------------------------------------------------------------- servo driver
// Minimal LEDC servo driver (Arduino-ESP32 core 3.x API). Replaces ESP32Servo, whose
// ESP32-S3 channel allocation made several servos move together (sweep R1 moved R1, L1
// and R4). Each servo gets its own LEDC channel: the S3 has exactly 8.
// Per-servo calibration, in servo-index order (R1 R2 L1 L2 R4 R3 L3 L4, same as servoPins).
// The sequences think in "logical" angles where 90 = rest. The angle sent to the servo is
//     physical = CAL_REST[i] + CAL_DIR[i] * (logical - 90)      (clamped to 0..180)
// CAL_REST = the measured physical angle where the robot stands straight (rest pose).
// CAL_DIR  = +1, or -1 to flip a joint whose real travel goes the other way (also what you
//            need when rest sits near 0 or 180 and the pose has to move toward the end stop).
// Calibrated 2026-11-08 (hips). Legs R4 R3 L3 L4 are still being calibrated: 90 / +1.
//   R1 = 7, R2 = 11 (flipped: stand = 56), L1 = 87 (GPIO 7), L2 = 0 (GPIO 1).
// L1 and L2 follow the GPIO swap: the servo that measured 87 is on GPIO 7 (now L1), the one
// that measured 0 is on GPIO 1 (now L2). Re-check with 'move L1 90' / 'move L2 90' if unsure.
const int CAL_REST[8] = {  7, 56, 87,  0, 90, 90, 90, 90 };   // R2 horn remounted: straight is now 56
const int CAL_DIR[8]  = {  1,  1,  1,  1,  1,  1,  1,  1 };
// R2: horn remounted so straight = 56, leaving 45 degrees of travel downward (stand = 11).

int calibratedAngle(int idx, int logical) {
  if (idx < 0 || idx > 7) return constrain(logical, 0, 180);
  return constrain(CAL_REST[idx] + CAL_DIR[idx] * (constrain(logical, 0, 180) - 90), 0, 180);
}

class LedcServo {
 public:
  void attach(int p, int minUs, int maxUs, int index = -1) {
    idx = index;
    pin = p;
    minPulse = minUs;
    maxPulse = maxUs;
    attached = ledcAttach(pin, 50, 14);   // 50 Hz, 14-bit (16384 counts per 20 ms)
    if (!attached) Serial.printf("LEDC attach FAILED on GPIO %d\n", pin);
  }
  void write(int angle) { writeRaw(calibratedAngle(idx, angle)); }   // logical (90 = rest) -> physical
  void writeRaw(int angle) {             // physical angle, no calibration
    if (!attached) return;
    angle = constrain(angle, 0, 180);
    long us = minPulse + (long)(maxPulse - minPulse) * angle / 180;
    uint32_t duty = (uint32_t)((uint64_t)us * 50ULL * 16384ULL / 1000000ULL);
    ledcWrite(pin, duty);
  }
 private:
  int pin = -1;
  int idx = -1;
  int minPulse = 732, maxPulse = 2929;
  bool attached = false;
};

// ---------------------------------------------------------------- config
const int NUM_SERVOS = 8;
// Servo map, identical on the leader and the follower (and on the finished bot).
// Index -> name (from movement-sequences.h) -> signal GPIO:
//   0 R1 -> GPIO 4    right hip        4 R4 -> GPIO 10   right leg
//   1 R2 -> GPIO 5    right hip        5 R3 -> GPIO 2    right leg
//   2 L1 -> GPIO 7    left hip         6 L3 -> GPIO 12   left leg
//   3 L2 -> GPIO 1    left hip         7 L4 -> GPIO 13   left leg
// (L1 and L2 were swapped after assembly: the original S3 map had L1 -> GPIO 1, L2 -> GPIO 7.)
// Which hip is front/back is not documented in the repo; label your servos by name.
const int servoPins[NUM_SERVOS] = {4, 5, 7, 1, 10, 2, 12, 13};   // same as main firmware
const int MIN_US = 732;
const int MAX_US = 2929;

// 8 dBm fixed ~97% loss at 30 cm (overload); 14 worked at 1.5-3 m. See memory notes.
int txPowerDbm = 14;

const unsigned long STREAM_BUSY_MS = 20;     // pose period while a sequence runs
const unsigned long STREAM_IDLE_MS = 100;    // keep-alive period otherwise
const unsigned long FAILSAFE_MS = 500;
const unsigned long RESYNC_MS = 1000;

// ---------------------------------------------------------------- globals required by movement-sequences.h
int frameDelay = 100;
int walkCycles = 10;
String currentCommand = "";
const int motorCurrentDelay = 20;   // ms between servo writes, same as main firmware

struct __attribute__((packed)) Pose8Payload { uint8_t angles[NUM_SERVOS]; };

LedcServo servos[NUM_SERVOS];

Role role = ROLE_FOLLOWER;
bool sequenceRunning = false;

// leader state
uint8_t cmdAngle[NUM_SERVOS];
unsigned long lastPoseSendMs = 0;

// follower state
float curAngle[NUM_SERVOS];
uint8_t tgtAngle[NUM_SERVOS];
float slewStep = 5.0f;                       // degrees per 10 ms tick
unsigned long lastSlewMs = 0;
bool gotAnyPose = false, failsafeTripped = false;
unsigned long lastPoseRxMs = 0;
uint16_t lastPoseSeq = 0;
uint32_t poseStale = 0, intervalPoses = 0, intervalMaxGap = 0;
int8_t lastPoseRssi = 0;
unsigned long lastStatsMs = 0;

// ---------------------------------------------------------------- forward declarations
void serviceAll();
void handleCommand(String cmd);

// ---------------------------------------------------------------- stubs / helpers required by movement-sequences.h
void setFace(const String& faceName) {}
void setFaceMode(FaceAnimMode mode) {}
void setFaceWithMode(const String& faceName, FaceAnimMode mode) {}
void enterIdle() {}

void delayWithFace(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    serviceAll();
    delay(5);
  }
}

void setServoAngle(uint8_t channel, int angle) {
  if (channel >= NUM_SERVOS) return;
  angle = constrain(angle, 0, 180);
  cmdAngle[channel] = (uint8_t)angle;
  if (role == ROLE_LEADER) servos[channel].write(angle);
  delayWithFace(motorCurrentDelay);
}

bool pressingCheck(String cmd, int ms) {
  unsigned long start = millis();
  while ((int)(millis() - start) < ms) {
    serviceAll();
    if (currentCommand != cmd) {
      runStandPose(1);
      return false;
    }
    yield();
  }
  return true;
}

// ---------------------------------------------------------------- ESP-NOW handling
void handlePose8(const EspNowRxMsg& m) {
  if (role != ROLE_FOLLOWER || m.hdr.len != sizeof(Pose8Payload)) return;
  unsigned long now = millis();
  bool resync = !gotAnyPose || (now - lastPoseRxMs > RESYNC_MS);
  if (!resync && (int16_t)(m.hdr.seq - lastPoseSeq) <= 0) { poseStale++; return; }

  if (gotAnyPose && !resync) {
    uint32_t gap = now - lastPoseRxMs;
    if (gap > intervalMaxGap) intervalMaxGap = gap;
  }
  Pose8Payload p;
  memcpy(&p, m.payload, sizeof(p));
  for (int i = 0; i < NUM_SERVOS; i++) tgtAngle[i] = (uint8_t)constrain((int)p.angles[i], 0, 180);

  lastPoseSeq = m.hdr.seq;
  lastPoseRxMs = now;
  lastPoseRssi = m.rssi;
  gotAnyPose = true;
  intervalPoses++;
  if (failsafeTripped) { failsafeTripped = false; Serial.println("Leader back, resumed"); }
}

void onMessage(const EspNowRxMsg& m) {
  if (m.hdr.type == ESPNOW_MSG_POSE8) handlePose8(m);
}

// ---------------------------------------------------------------- leader / follower services
void leaderStream() {
  if (role != ROLE_LEADER) return;
  unsigned long now = millis();
  unsigned long period = sequenceRunning ? STREAM_BUSY_MS : STREAM_IDLE_MS;
  if (now - lastPoseSendMs < period) return;
  lastPoseSendMs = now;
  Pose8Payload p;
  memcpy(p.angles, cmdAngle, NUM_SERVOS);
  espnowSend(ESPNOW_MSG_POSE8, (const uint8_t*)&p, sizeof(p));
}

void followerUpdate() {
  unsigned long now = millis();

  if (role == ROLE_FOLLOWER && gotAnyPose && !failsafeTripped && now - lastPoseRxMs > FAILSAFE_MS) {
    for (int i = 0; i < NUM_SERVOS; i++) tgtAngle[i] = 90;
    failsafeTripped = true;
    Serial.println("FAILSAFE: leader silent, servos -> 90");
  }

  if (role == ROLE_FOLLOWER && now - lastSlewMs >= 10) {
    lastSlewMs = now;
    for (int i = 0; i < NUM_SERVOS; i++) {
      float diff = (float)tgtAngle[i] - curAngle[i];
      if (fabsf(diff) < 0.5f) continue;
      curAngle[i] += constrain(diff, -slewStep, slewStep);
      servos[i].write((int)lroundf(curAngle[i]));
    }
  }

  if (role == ROLE_FOLLOWER && now - lastStatsMs >= 2000) {
    lastStatsMs = now;
    if (intervalPoses > 0) {
      Serial.printf("pose rx %lu/2s | max gap %lu ms | stale total %lu | RSSI %d\n",
                    (unsigned long)intervalPoses, (unsigned long)intervalMaxGap,
                    (unsigned long)poseStale, lastPoseRssi);
    }
    intervalPoses = 0;
    intervalMaxGap = 0;
  }
}

void pollSerial() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (buf.length()) {
        String cmd = buf;
        buf = "";
        handleCommand(cmd);
      }
    } else {
      buf += c;
    }
  }
}

void serviceAll() {
  espnowService();
  leaderStream();
  followerUpdate();
  batteryUpdate();
  updateOled();
  pollSerial();
}

// ---------------------------------------------------------------- sequences
void runSequence(const String& name) {
  if (role != ROLE_LEADER) { Serial.println("Only the leader runs sequences. Send 'leader' first."); return; }

  bool found = true;
  sequenceRunning = true;
  currentCommand = name;
  if      (name == "rest")     runRestPose();
  else if (name == "stand")    runStandPose(1);
  else if (name == "wave")     runWavePose();
  else if (name == "dance")    runDancePose();
  else if (name == "swim")     runSwimPose();
  else if (name == "point")    runPointPose();
  else if (name == "pushup")   runPushupPose();
  else if (name == "bow")      runBowPose();
  else if (name == "cute")     runCutePose();
  else if (name == "freaky")   runFreakyPose();
  else if (name == "worm")     runWormPose();
  else if (name == "shake")    runShakePose();
  else if (name == "shrug")    runShrugPose();
  else if (name == "dead")     runDeadPose();
  else if (name == "crab")     runCrabPose();
  else if (name == "forward")  runWalkPose();
  else if (name == "backward") runWalkBackward();
  else if (name == "left")     runTurnLeft();
  else if (name == "right")    runTurnRight();
  else found = false;
  sequenceRunning = false;
  currentCommand = "";

  if (found) Serial.println("Sequence done");
  else Serial.println("Unknown sequence. Names: rest stand wave dance swim point pushup bow cute freaky worm shake shrug dead crab forward backward left right");
}

// Wiggle ONE servo on THIS board only (not sent to the other board). For checking breadboard wiring.
void wiggleServo(const String& which) {
  String up = which;
  up.trim();
  up.toUpperCase();
  int idx = servoNameToIndex(up);
  if (idx < 0 && up.length() == 1 && isDigit(up[0])) idx = up.toInt();
  if (idx < 0 || idx >= NUM_SERVOS) {
    Serial.println("Usage: wiggle <R1 R2 L1 L2 R3 R4 L3 L4 or index 0-7>");
    return;
  }
  int home = (role == ROLE_LEADER) ? (int)cmdAngle[idx] : (int)lroundf(curAngle[idx]);
  Serial.printf("Wiggling %s (index %d, GPIO %d) on this board only\n", ServoNames[idx].c_str(), idx, servoPins[idx]);
  servos[idx].write(constrain(home - 25, 0, 180));
  delay(400);
  servos[idx].write(constrain(home + 25, 0, 180));
  delay(400);
  servos[idx].write(home);
  delay(200);
}

// Resolve "r1", "L4" or "0".."7" to a servo index, or -1.
int parseServoIndex(String s) {
  s.trim();
  s.toUpperCase();
  int idx = servoNameToIndex(s);
  if (idx < 0 && s.length() == 1 && isDigit(s[0])) idx = s.toInt();
  return (idx >= 0 && idx < NUM_SERVOS) ? idx : -1;
}

// Leader only: move ONE servo to an angle. It moves on the leader and is streamed to the follower.
void moveServoCmd(const String& args) {
  if (role != ROLE_LEADER) { Serial.println("Only the leader sends servo commands. Send 'leader' first."); return; }
  int sp = args.lastIndexOf(' ');
  int idx = (sp > 0) ? parseServoIndex(args.substring(0, sp)) : -1;
  if (idx < 0) { Serial.println("Usage: move <R1 R2 L1 L2 R3 R4 L3 L4 or 0-7> <angle 0-180>"); return; }
  int angle = constrain((int)args.substring(sp + 1).toInt(), 0, 180);
  lastPoseSendMs = 0;                 // send the new pose right away
  setServoAngle(idx, angle);
  Serial.printf("%s (index %d, GPIO %d) -> %d deg on the leader, streamed to the follower\n",
                ServoNames[idx].c_str(), idx, servoPins[idx], angle);
}

// Leader only: sweep ONE servo 45-135 deg five times, then back to 90. 'stop' ends it early.
void sweepServoCmd(const String& arg) {
  if (role != ROLE_LEADER) { Serial.println("Only the leader sends servo commands. Send 'leader' first."); return; }
  int idx = parseServoIndex(arg);
  if (idx < 0) { Serial.println("Usage: sweep <R1 R2 L1 L2 R3 R4 L3 L4 or 0-7>"); return; }
  Serial.printf("Sweeping %s (index %d, GPIO %d) 5 times, 'stop' ends it\n", ServoNames[idx].c_str(), idx, servoPins[idx]);
  sequenceRunning = true;
  currentCommand = "sweep";
  for (int cycle = 0; cycle < 5 && currentCommand == "sweep"; cycle++) {
    for (int a = 45; a <= 135 && currentCommand == "sweep"; a += 5) setServoAngle(idx, a);
    for (int a = 135; a >= 45 && currentCommand == "sweep"; a -= 5) setServoAngle(idx, a);
  }
  setServoAngle(idx, 90);
  sequenceRunning = false;
  currentCommand = "";
  Serial.println("Sweep done");
}

// ---------------------------------------------------------------- commands
void printHelp() {
  Serial.println("Commands: help, status, leader, follower, seq <name>, wiggle <servo>, move <servo> <angle>, sweep <servo>, batt, batt r <mohm>, batt peak, stop, power <dBm>, slew <deg>");
  Serial.println("Sequences: rest stand wave dance swim point pushup bow cute freaky worm shake shrug dead crab forward backward left right");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (!cmd.length()) return;

  if (sequenceRunning) {
    if (cmd == "stop") { currentCommand = ""; Serial.println("Stop requested"); }
    else Serial.println("Busy: only 'stop' works while a sequence runs");
    return;
  }

  if (cmd == "help") printHelp();
  else if (cmd == "status") {
    espnowPrintInfo();
    Serial.printf("  role %s | stale poses %lu | slew %.1f deg/10ms | TX power %d dBm\n",
                  role == ROLE_LEADER ? "LEADER" : "FOLLOWER",
                  (unsigned long)poseStale, slewStep, txPowerDbm);
  }
  else if (cmd == "leader") {
    role = ROLE_LEADER;
    for (int i = 0; i < NUM_SERVOS; i++) cmdAngle[i] = (uint8_t)lroundf(curAngle[i]);
    Serial.println("Role: LEADER");
  }
  else if (cmd == "follower") {
    role = ROLE_FOLLOWER;
    gotAnyPose = false;
    failsafeTripped = false;
    for (int i = 0; i < NUM_SERVOS; i++) { curAngle[i] = cmdAngle[i]; tgtAngle[i] = cmdAngle[i]; }
    Serial.println("Role: FOLLOWER");
  }
  else if (cmd.startsWith("raw ")) {
    // raw <servo> <0-180>: PHYSICAL angle on this board only, no calibration, not streamed.
    // For finding rest / stand angles. Run 'stop' or 'seq rest' afterwards to return to normal.
    String a = cmd.substring(4);
    int sp = a.indexOf(' ');
    int idx = (sp > 0) ? parseServoIndex(a.substring(0, sp)) : -1;
    if (idx < 0) { Serial.println("Usage: raw <servo> <angle 0-180>  (physical, uncalibrated)"); }
    else {
      int ang = constrain((int)a.substring(sp + 1).toInt(), 0, 180);
      servos[idx].writeRaw(ang);
      Serial.printf("RAW %s (GPIO %d) -> physical %d deg (this board only)\n", ServoNames[idx].c_str(), servoPins[idx], ang);
    }
  }
  else if (cmd == "batt") {
    batteryPrint();
    Serial.printf("  peak current %.0f mA (reset with 'batt peak')\n", battPeakMa);
  }
  else if (cmd == "batt peak") { battPeakMa = 0; Serial.println("Peak current reset"); }
  else if (cmd.startsWith("batt r ")) batterySetResistance(cmd.substring(7).toInt());
  else if (cmd.startsWith("wiggle ")) wiggleServo(cmd.substring(7));
  else if (cmd.startsWith("move ")) moveServoCmd(cmd.substring(5));
  else if (cmd.startsWith("sweep ")) sweepServoCmd(cmd.substring(6));
  else if (cmd.startsWith("seq ")) runSequence(cmd.substring(4));
  else if (cmd == "stop") Serial.println("Nothing is running");
  else if (cmd.startsWith("power ")) {
    txPowerDbm = constrain(cmd.substring(6).toInt(), 2, 20);
    esp_err_t e = esp_wifi_set_max_tx_power(txPowerDbm * 4);
    Serial.printf("TX power %d dBm -> %s\n", txPowerDbm, esp_err_to_name(e));
  }
  else if (cmd.startsWith("slew ")) {
    slewStep = constrain(cmd.substring(5).toFloat(), 0.5f, 45.0f);
    Serial.printf("Follower slew %.1f deg per 10 ms\n", slewStep);
  }
  else Serial.println("Unknown command (type help)");
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(1500);
  {
    uint64_t efuseMac = ESP.getEfuseMac();
    char idbuf[8];
    snprintf(idbuf, sizeof(idbuf), "%02X%02X", (uint8_t)((efuseMac >> 32) & 0xFF), (uint8_t)((efuseMac >> 40) & 0xFF));
    boardId = String(idbuf);
  }
  Serial.println("\n=== ESP-NOW sequence tester (8 servos) ===");
  Serial.println("Board ID " + boardId + " (type 'status' to see the full MAC)");

  for (int i = 0; i < NUM_SERVOS; i++) {
    cmdAngle[i] = 90;
    tgtAngle[i] = 90;
    curAngle[i] = 90.0f;
    servos[i].attach(servoPins[i], MIN_US, MAX_US, i);
    servos[i].write(90);
    delay(motorCurrentDelay);   // stagger to limit inrush current
  }

  // OLED + INA260 on the shared I2C bus (both optional: the tester runs without them).
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) Serial.println("OLED not found at 0x3C, display disabled");
  batteryBegin();
  drawBatteryScreen();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_max_tx_power(txPowerDbm * 4);
  delay(100);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (!espnowBegin("seq-tester")) Serial.println("ESP-NOW failed to start!");
  espnowOnMessage = onMessage;

  Serial.println("Servo map (index, name, signal GPIO):");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("  %d  %s  GPIO %d\n", i, ServoNames[i].c_str(), servoPins[i]);
  }
  Serial.printf("Role: FOLLOWER (type 'leader' on the board that should lead) | TX %d dBm\n", txPowerDbm);
  printHelp();
}

void loop() {
  serviceAll();
}
