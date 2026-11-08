/*
 * ESP-NOW servo tester (leader / follower) - ESP32-S3 SuperMini
 *
 * Flash the SAME sketch to both boards. Only BOARD changes:
 *   BOARD 1 -> servos on GPIO 2, 10, 12, 13  (starts as LEADER)
 *   BOARD 2 -> servos on GPIO 1,  4,  5,  7  (starts as FOLLOWER)
 *
 * Serial monitor 115200 baud, newline line ending. Commands (either board):
 *   help       list commands
 *   status     ESP-NOW info, peers, tx/rx counters, pose stats
 *   ping       link test: 50 pings, reports loss, RTT, RSSI (works from either board)
 *   leader     become leader
 *   follower   become follower (mirrors poses received from the leader)
 *   center     move own servos to 90
 *   sweep1     leader only: sweep servo 0 only, mirrored to the follower
 *   sweep4     leader only: sweep all 4 servos with phase offsets
 *   stop       leader only: stop streaming, go to 90
 *
 * Follower safety: angles are clamped, stale/out-of-order poses are dropped,
 * and if the leader goes quiet for FAILSAFE_MS the servos go to 90.
 *
 * Power: servos from an external 5-6 V supply, common ground with the ESP32.
 * Libraries: ESP32Servo. Enable "USB CDC On Boot" in the Arduino board menu.
 */

#define BOARD 1   // 1 or 2

#include <ESP32Servo.h>
#include <WiFi.h>
#include "espnow-comms.h"

#if BOARD == 1
const int servoPins[4] = {2, 10, 12, 13};
const char* BOARD_NAME = "tester-1";
#else
const int servoPins[4] = {1, 4, 5, 7};
const char* BOARD_NAME = "tester-2";
#endif

// Same pulse range as the main firmware. Narrow to 500-2400 if a servo buzzes at the ends.
const int MIN_US = 732;
const int MAX_US = 2929;

const int ANGLE_MIN = 10;          // safe sweep limits
const int ANGLE_MAX = 170;
const unsigned long POSE_PERIOD_MS = 20;    // 50 Hz pose stream
const unsigned long SWEEP_PERIOD_MS = 4000; // one full sweep cycle
const unsigned long FAILSAFE_MS = 500;
const unsigned long RESYNC_MS = 1000;       // accept any seq after this much silence

struct __attribute__((packed)) PingPayload { uint16_t id; uint32_t sentUs; };
struct __attribute__((packed)) PosePayload { uint8_t angles[4]; };

Servo servos[4];

enum Role { ROLE_LEADER, ROLE_FOLLOWER };
#if BOARD == 1
Role role = ROLE_LEADER;
#else
Role role = ROLE_FOLLOWER;
#endif

enum SweepMode { SWEEP_NONE, SWEEP_ONE, SWEEP_ALL };
SweepMode sweepMode = SWEEP_NONE;
unsigned long lastPoseSendMs = 0;

// --- follower state
bool gotAnyPose = false, failsafeTripped = false;
unsigned long lastPoseRxMs = 0;
uint16_t lastPoseSeq = 0;
uint32_t poseApplied = 0, poseStale = 0;
uint32_t intervalPoses = 0, intervalMaxGap = 0;
int8_t lastPoseRssi = 0;
unsigned long lastStatsMs = 0;

// --- ping test state
const int PING_COUNT = 50;
const unsigned long PING_INTERVAL_MS = 100;
bool pingActive = false;
int pingSent = 0;
unsigned long pingLastMs = 0, pingEndMs = 0;
uint32_t pingSentUs[PING_COUNT];
uint32_t pingRtt[PING_COUNT];
bool pingGot[PING_COUNT];
int pingRssiSum = 0;

// ---------------------------------------------------------------- servos
void applyAngles(const uint8_t a[4]) {
  for (int i = 0; i < 4; i++) servos[i].write(constrain((int)a[i], 0, 180));
}

void centerServos() {
  uint8_t a[4] = {90, 90, 90, 90};
  applyAngles(a);
}

// ---------------------------------------------------------------- messages
void handlePose(const EspNowRxMsg& m) {
  if (role != ROLE_FOLLOWER || m.hdr.len != sizeof(PosePayload)) return;
  unsigned long now = millis();
  bool resync = !gotAnyPose || (now - lastPoseRxMs > RESYNC_MS);
  if (!resync && (int16_t)(m.hdr.seq - lastPoseSeq) <= 0) { poseStale++; return; }

  if (gotAnyPose && !resync) {
    uint32_t gap = now - lastPoseRxMs;
    if (gap > intervalMaxGap) intervalMaxGap = gap;
  }
  PosePayload p;
  memcpy(&p, m.payload, sizeof(p));
  applyAngles(p.angles);

  lastPoseSeq = m.hdr.seq;
  lastPoseRxMs = now;
  lastPoseRssi = m.rssi;
  gotAnyPose = true;
  poseApplied++;
  intervalPoses++;
  if (failsafeTripped) { failsafeTripped = false; Serial.println("Leader back, resumed"); }
}

void handlePong(const EspNowRxMsg& m) {
  if (m.hdr.len != sizeof(PingPayload)) return;
  PingPayload p;
  memcpy(&p, m.payload, sizeof(p));
  if (!pingActive || p.id >= PING_COUNT || pingGot[p.id]) return;
  pingGot[p.id] = true;
  pingRtt[p.id] = micros() - p.sentUs;
  pingRssiSum += m.rssi;
}

void onMessage(const EspNowRxMsg& m) {
  switch (m.hdr.type) {
    case ESPNOW_MSG_PING:
      if (m.hdr.len == sizeof(PingPayload)) espnowSend(ESPNOW_MSG_PONG, m.payload, m.hdr.len);
      break;
    case ESPNOW_MSG_PONG: handlePong(m); break;
    case ESPNOW_MSG_POSE: handlePose(m); break;
  }
}

// ---------------------------------------------------------------- ping test
void startPing() {
  memset(pingGot, 0, sizeof(pingGot));
  pingSent = 0;
  pingRssiSum = 0;
  pingActive = true;
  pingLastMs = millis() - PING_INTERVAL_MS;
  pingEndMs = 0;
  Serial.printf("Pinging: %d packets, %lu ms apart...\n", PING_COUNT, PING_INTERVAL_MS);
}

void servicePing() {
  if (!pingActive) return;
  unsigned long now = millis();
  if (pingSent < PING_COUNT) {
    if (now - pingLastMs >= PING_INTERVAL_MS) {
      pingLastMs = now;
      PingPayload p = {(uint16_t)pingSent, micros()};
      pingSentUs[pingSent] = p.sentUs;
      espnowSend(ESPNOW_MSG_PING, (const uint8_t*)&p, sizeof(p));
      pingSent++;
      if (pingSent == PING_COUNT) pingEndMs = now;
    }
  } else if (now - pingEndMs > 500) {
    int got = 0;
    uint32_t mn = UINT32_MAX, mx = 0;
    uint64_t sum = 0;
    for (int i = 0; i < PING_COUNT; i++) {
      if (!pingGot[i]) continue;
      got++;
      sum += pingRtt[i];
      if (pingRtt[i] < mn) mn = pingRtt[i];
      if (pingRtt[i] > mx) mx = pingRtt[i];
    }
    Serial.printf("Ping done: %d/%d replies, loss %d%%\n", got, PING_COUNT, (PING_COUNT - got) * 100 / PING_COUNT);
    if (got > 0) {
      Serial.printf("  RTT ms: min %.2f  avg %.2f  max %.2f | avg RSSI %d dBm\n",
                    mn / 1000.0, sum / (double)got / 1000.0, mx / 1000.0, pingRssiSum / got);
    } else {
      Serial.println("  No replies. Is the other board powered, on channel 1, and running this sketch?");
    }
    pingActive = false;
  }
}

// ---------------------------------------------------------------- leader stream
void serviceSweep() {
  if (role != ROLE_LEADER || sweepMode == SWEEP_NONE) return;
  unsigned long now = millis();
  if (now - lastPoseSendMs < POSE_PERIOD_MS) return;
  lastPoseSendMs = now;

  const float mid = (ANGLE_MIN + ANGLE_MAX) / 2.0f, amp = (ANGLE_MAX - ANGLE_MIN) / 2.0f;
  PosePayload p;
  for (int i = 0; i < 4; i++) {
    if (sweepMode == SWEEP_ONE && i > 0) { p.angles[i] = 90; continue; }
    float phase = 2.0f * PI * (now % SWEEP_PERIOD_MS) / SWEEP_PERIOD_MS + i * (PI / 2.0f);
    p.angles[i] = (uint8_t)(mid + amp * sinf(phase));
  }
  applyAngles(p.angles);
  espnowSend(ESPNOW_MSG_POSE, (const uint8_t*)&p, sizeof(p));
}

// ---------------------------------------------------------------- follower failsafe + stats
void serviceFollower() {
  unsigned long now = millis();
  if (role == ROLE_FOLLOWER && gotAnyPose && !failsafeTripped && now - lastPoseRxMs > FAILSAFE_MS) {
    centerServos();
    failsafeTripped = true;
    Serial.println("FAILSAFE: leader silent, servos -> 90");
  }
  if (now - lastStatsMs >= 2000) {
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

// ---------------------------------------------------------------- serial commands
void printHelp() {
  Serial.println("Commands: help, status, ping, leader, follower, center, sweep1, sweep4, stop");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (!cmd.length()) return;

  if (cmd == "help") printHelp();
  else if (cmd == "status") {
    espnowPrintInfo();
    Serial.printf("  role %s | poses applied %lu, stale %lu\n",
                  role == ROLE_LEADER ? "LEADER" : "FOLLOWER",
                  (unsigned long)poseApplied, (unsigned long)poseStale);
  }
  else if (cmd == "ping") startPing();
  else if (cmd == "leader") { role = ROLE_LEADER; Serial.println("Role: LEADER"); }
  else if (cmd == "follower") {
    role = ROLE_FOLLOWER; sweepMode = SWEEP_NONE; gotAnyPose = false;
    Serial.println("Role: FOLLOWER");
  }
  else if (cmd == "center") { sweepMode = SWEEP_NONE; centerServos(); Serial.println("Servos -> 90"); }
  else if (cmd == "sweep1" || cmd == "sweep4") {
    if (role != ROLE_LEADER) { Serial.println("Only the leader streams. Send 'leader' first."); return; }
    sweepMode = (cmd == "sweep1") ? SWEEP_ONE : SWEEP_ALL;
    Serial.printf("Streaming %s\n", cmd.c_str());
  }
  else if (cmd == "stop") {
    sweepMode = SWEEP_NONE;
    uint8_t a[4] = {90, 90, 90, 90};
    applyAngles(a);
    if (role == ROLE_LEADER) espnowSend(ESPNOW_MSG_POSE, a, sizeof(a));
    Serial.println("Stopped, servos -> 90");
  }
  else Serial.println("Unknown command (type help)");
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.printf("\n=== ESP-NOW servo tester | board %d | %s ===\n", BOARD, BOARD_NAME);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  for (int i = 0; i < 4; i++) {
    servos[i].setPeriodHertz(50);
    servos[i].attach(servoPins[i], MIN_US, MAX_US);
  }
  centerServos();

  // ESP-NOW layer sends on the soft-AP interface, so bring the AP up on the shared channel.
  WiFi.mode(WIFI_AP);
  WiFi.softAP(BOARD_NAME, nullptr, ESPNOW_CHANNEL);
  delay(200);
  if (!espnowBegin(BOARD_NAME)) Serial.println("ESP-NOW failed to start!");
  espnowOnMessage = onMessage;

  Serial.printf("Role: %s | servo GPIOs %d,%d,%d,%d\n",
                role == ROLE_LEADER ? "LEADER" : "FOLLOWER",
                servoPins[0], servoPins[1], servoPins[2], servoPins[3]);
  printHelp();
}

void loop() {
  espnowService();
  servicePing();
  serviceSweep();
  serviceFollower();
  if (Serial.available()) handleCommand(Serial.readStringUntil('\n'));
}
