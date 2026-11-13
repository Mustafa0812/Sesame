/*
 * ESP-NOW sequence tester (PCA9685 version) - ESP32-S3 SuperMini, 8 servos per board
 *
 * Same as espnow-sequence-tester, but the 8 servos are driven by a PCA9685 16-channel
 * I2C PWM board (channels 0-7) instead of ESP32 LEDC pins. The PCA9685 shares the I2C bus
 * (SDA 6, SCL 11) with the OLED (0x3C) and INA260.
 * !! PCA9685 stays at its default 0x40, so the INA260 must be moved: solder-bridge its A0 pad
 * !! (VS) -> 0x41. battery.h scans 0x40-0x4F and checks the chip ID, so no code change needed. PCA9685 VCC = 3V3, V+ = 5-6 V servo supply, all grounds common.
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
 * ESP32. OLED: the sequence's animated face (face-bitmaps.h, same faces as sesame-firmware-main)
 * while a sequence runs, the battery screen the rest of the time.
 * Libraries: Adafruit SSD1306 + GFX (OLED). No servo library: raw PCA9685 register writes over Wire.
 * Enable "USB CDC On Boot".
 * NOTE: movement-sequences.h here is a copy of the one in sesame-firmware-main.
 */

#define ESPNOW_USE_STA   // station-mode ESP-NOW, no soft AP

#include <WiFi.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "espnow-comms.h"
#include "movement-sequences.h"
#include "face-bitmaps.h"
#include "battery.h"

// ---------------------------------------------------------------- OLED + battery (INA260)
// I2C: SDA GPIO6, SCL GPIO11 (shared by OLED 0x3C, INA260 0x41, PCA9685 0x40). Power both from 3V3.
// INA260 goes on the BATTERY side, before the buck: battery + -> IN+, IN- -> buck input +.
// See battery.h for the method. On a bench supply type 'batt r 0' (no internal resistance).
enum Role { ROLE_LEADER, ROLE_FOLLOWER };
bool sequenceRunning = false;            // a sequence (or sweep) is playing on this board
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
    oled.print("(0x41, SDA6 SCL11)");
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

void updateFaceFrame();                      // face animation, in the faces section below

// While a sequence runs the OLED shows its face (one redraw per new frame, ~1 per second);
// otherwise the battery screen every OLED_PERIOD_MS.
void updateOled() {
  if (!oledOk) return;
  if (battCurrentMa > battPeakMa) battPeakMa = battCurrentMa;
  if (sequenceRunning) { updateFaceFrame(); return; }
  unsigned long now = millis();
  if (now - lastOledMs < OLED_PERIOD_MS) return;
  lastOledMs = now;
  drawBatteryScreen();
}

// ---------------------------------------------------------------- servo driver (PCA9685)
// 12-bit PWM at 50 Hz: 4096 ticks per 20 ms = 4.88 us per tick. No library, plain Wire.
#define PCA_ADDR      0x40      // default address; the INA260 is moved off 0x40 (A0 -> VS = 0x41)
#define PCA_MODE1     0x00
#define PCA_MODE2     0x01
#define PCA_LED0_ON_L 0x06
#define PCA_PRESCALE  0xFE
#define PCA_HZ        50
#define PCA_OSC_HZ    25000000UL

bool pcaOk = false;
// Supply dips from servo current can make an I2C write fail or reset the PCA9685 (it then wakes up asleep
// with every output off). Both used to lose servo moves silently; now they are retried / repaired and counted.
uint32_t pcaI2cErr = 0;                      // failed write attempts (each is retried once)
uint32_t pcaLost = 0;                        // writes that failed twice: a servo move was lost
uint32_t pcaResets = 0;                      // times the PCA9685 was found reset and re-initialised
long pcaLastUs[16];                          // last pulse per channel, to restore after a PCA reset (-1 = none)
unsigned long lastPcaCheckMs = 0;

bool pcaBegin() {
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() != 0) return false;
  auto wr = [](uint8_t reg, uint8_t val) {
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  };
  wr(PCA_MODE1, 0x10);                                           // sleep: required to set the prescaler
  wr(PCA_PRESCALE, (uint8_t)(PCA_OSC_HZ / (4096UL * PCA_HZ) - 1 + 0.5));   // 121 -> ~50 Hz
  wr(PCA_MODE2, 0x04);                                           // totem-pole outputs
  wr(PCA_MODE1, 0x20);                                           // wake, register auto-increment
  delay(5);                                                      // oscillator settle
  return true;
}

// pulse width in microseconds on one channel (0 = no pulse, servo limp)
void pcaSetUs(uint8_t ch, long us) {
  uint16_t off = (uint16_t)(us * 4096UL * PCA_HZ / 1000000UL);
  if (ch < 16) pcaLastUs[ch] = us;
  for (int attempt = 0; attempt < 2; attempt++) {
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(PCA_LED0_ON_L + 4 * ch);
    Wire.write(0); Wire.write(0);
    if (us <= 0) { Wire.write(0); Wire.write(0x10); }            // full-off bit
    else         { Wire.write(off & 0xFF); Wire.write(off >> 8); }
    if (Wire.endTransmission() == 0) return;
    pcaI2cErr++;
  }
  pcaLost++;
}

// Once a second: a PCA9685 that lost power comes back in sleep mode (MODE1 bit 4) with all outputs off.
// Re-initialise it and send every channel its last pulse again.
void pcaCheck() {
  if (!pcaOk) return;
  unsigned long now = millis();
  if (now - lastPcaCheckMs < 1000) return;
  lastPcaCheckMs = now;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(PCA_MODE1);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)PCA_ADDR, (uint8_t)1) != 1) { pcaI2cErr++; return; }
  uint8_t mode1 = Wire.read();
  if (!(mode1 & 0x10)) return;               // awake: fine
  pcaResets++;
  Serial.printf("PCA9685 was reset (MODE1 0x%02X, supply dip?): restoring outputs\n", mode1);
  if (!pcaBegin()) return;
  for (int ch = 0; ch < 16; ch++) if (pcaLastUs[ch] >= 0) pcaSetUs(ch, pcaLastUs[ch]);
}

// Per-servo calibration, in servo-index order (R1 R2 L1 L2 R4 R3 L3 L4, same as servoChannels = PCA channels 0-7).
// The sequences think in "logical" angles where 90 = rest. The angle sent to the servo is
//     physical = CAL_REST[i] + CAL_DIR[i] * (logical - 90)      (clamped to 0..180)
// CAL_REST = the measured physical angle where the robot stands straight (rest pose).
// CAL_DIR  = +1, or -1 to flip a joint whose real travel goes the other way (also what you
//            need when rest sits near 0 or 180 and the pose has to move toward the end stop).
// Calibrated on the PCA9685 at 500-2500 us: all eight joints rest at 90 (full range for every sequence).
//   Directions: R1 R2 L1 L2 +1, R4 R3 L3 L4 -1. Stand (physical): R1 135, R2 45, L1 45, L2 135, R4 180, R3 0, L3 180, L4 0.
const int CAL_REST[8] = { 90, 90, 90, 90, 90, 90, 90, 90 };   // L1, L2 measured at 500-2500 us (80, 94). R3: rest 90, stand 0, dir -1 (full range). R4: rest 90, stand 180, dir -1 (full range). L3: rest 90, stand 180, dir -1 (full range). L4: rest 90, stand 0, dir -1 (full range). R2: rest 90, stand ok (45), dir +1. R1: rest 90, stand above 90 (135), dir +1 (re-measured on the PCA).
const int CAL_DIR[8]  = {  1,  1,  1,  1, -1, -1, -1, -1 };   // R4 -1: rest 90, stand (logical 0) = physical 180. R3 -1: rest 90, stand (logical 180) = physical 0. L3 -1: rest 90, stand (logical 0) = physical 180. L4 -1: rest 90, stand (logical 180) = physical 0
// R2: straight = 47 on the PCA9685 (was 56). The stand pose is no longer 11: re-check R2's stand angle.

int calibratedAngle(int idx, int logical) {
  if (idx < 0 || idx > 7) return constrain(logical, 0, 180);
  return constrain(CAL_REST[idx] + CAL_DIR[idx] * (constrain(logical, 0, 180) - 90), 0, 180);
}

class PcaServo {
 public:
  void attach(int ch, int minUs, int maxUs, int index = -1) {
    channel = ch;
    idx = index;
    minPulse = minUs;
    maxPulse = maxUs;
    attached = pcaOk;
  }
  void writeUs(long us) { if (attached) pcaSetUs(channel, constrain(us, 400, 2600)); }   // raw pulse width
  void setRange(int minUs, int maxUs) { minPulse = minUs; maxPulse = maxUs; }
  void release() { if (attached) pcaSetUs(channel, 0); }             // no pulse: servo goes limp
  void write(int angle) { writeRaw(calibratedAngle(idx, angle)); }   // logical (90 = rest) -> physical
  void writeRaw(int angle) {             // physical angle, no calibration
    if (!attached) return;
    angle = constrain(angle, 0, 180);
    pcaSetUs(channel, minPulse + (long)(maxPulse - minPulse) * angle / 180);
  }
 private:
  int channel = -1;
  int idx = -1;
  int minPulse = 500, maxPulse = 2500;
  bool attached = false;
};

// ---------------------------------------------------------------- config
const int NUM_SERVOS = 8;
// PCA9685 channel map (channel = servo index, only 0-7 used, 8-15 stay unused):
//   ch 0 R1 right hip      ch 4 R4 right leg
//   ch 1 R2 right hip      ch 5 L4 left leg
//   ch 2 L1 left hip       ch 6 L3 left leg
//   ch 3 L2 left hip       ch 7 R3 right leg
// Array is in servo-index order (R1 R2 L1 L2 R4 R3 L3 L4): R4 -> 4, R3 -> 7, L3 -> 6, L4 -> 5. CAL_REST/CAL_DIR follow the name.
const int servoChannels[NUM_SERVOS] = {0, 1, 2, 3, 4, 7, 6, 5};
// Measured on the PCA9685: 500..2500 us gives the full 180 deg, so physical 90 = 1500 us (true center).
// The old LEDC sketch used 732..2929; CAL_REST values measured with that range do NOT transfer.
const int MIN_US = 500;
const int MAX_US = 2500;

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

PcaServo servos[NUM_SERVOS];
int rawAngle[NUM_SERVOS] = {-1, -1, -1, -1, -1, -1, -1, -1};   // last 'raw'/'jog' physical angle, -1 = none

Role role = ROLE_FOLLOWER;
String runningName = "";                     // sequence in progress, for telemetry (stays set until it really ends)

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
uint8_t lastPoseMac[6] = {0};               // sender of the last accepted pose (each sender numbers its own packets)

// Lockstep playback. When the leader starts a sequence it announces it 3 times, 30 ms apart:
// "SYNC:<leader id>:<run>:<ms until start>:<name>". Everyone starts 60 ms after the first announce (a follower
// that only hears the 2nd or 3rd still starts on time), and every follower plays the same sequence from its own copy of
// movement-sequences.h, so lost pose packets can no longer drop steps. The leader keeps streaming
// poses; a follower that missed all three announces simply copies that stream as before.
bool localRun = false;                       // follower: playing an announced sequence itself
uint16_t syncRunId = 0;                      // leader: number of the current run (random start, set in setup)
String syncName = "";                        // leader: what to announce (a sequence name or "stop")
int syncSendsLeft = 0;
unsigned long lastSyncSendMs = 0;
unsigned long syncStartMs = 0;               // leader: when the announced run starts
const unsigned long SYNC_LEAD_MS = 60;
String syncPendKey = "", syncPendName = "", syncLastKey = "";   // follower: received announce
unsigned long syncPendMs = 0;                // follower: when to start it
const char* const SEQ_NAMES[] = { "rest", "stand", "wave", "dance", "swim", "point", "pushup", "bow", "cute",
                                  "freaky", "worm", "shake", "shrug", "dead", "crab", "forward", "backward",
                                  "left", "right" };
bool isSequenceName(const String& n) {
  for (const char* k : SEQ_NAMES) if (n == k) return true;
  return false;
}

// ---------------------------------------------------------------- forward declarations
void serviceAll();
void handleCommand(String cmd);

// ---------------------------------------------------------------- faces (OLED, while a sequence runs)
// Same faces and animation modes as sesame-firmware-main: each sequence in movement-sequences.h picks
// its face with setFaceWithMode(). Each new frame is one OLED refresh (~25 ms of I2C). Most faces run
// at 1 frame/s, and leader and follower draw the same frames at the same moments, so they stay in step.
struct FaceEntry { const char* name; const unsigned char* const* frames; };
static const uint8_t MAX_FACE_FRAMES = 6;

#define MAKE_FACE_FRAMES(name) \
  const unsigned char* const face_##name##_frames[MAX_FACE_FRAMES] = { \
    epd_bitmap_##name, epd_bitmap_##name##_1, epd_bitmap_##name##_2, \
    epd_bitmap_##name##_3, epd_bitmap_##name##_4, epd_bitmap_##name##_5 };
#define X(name) MAKE_FACE_FRAMES(name)
FACE_LIST
#undef X
#undef MAKE_FACE_FRAMES

const FaceEntry faceEntries[] = {
#define X(name) { #name, face_##name##_frames },
  FACE_LIST
#undef X
};

struct FaceFps { const char* name; uint8_t fps; };
const FaceFps faceFpsTable[] = { { "point", 5 }, { "dead", 2 }, { "idle_blink", 7 } };   // all others 1 frame/s

const unsigned char* const* faceFrames = nullptr;   // null = no face yet in this sequence
uint8_t faceFrameCount = 0, faceFrameIndex = 0, faceFpsNow = 1;
int8_t faceDir = 1;
FaceAnimMode faceMode = FACE_ANIM_LOOP;
bool faceDone = false;
String faceName = "";
unsigned long lastFaceFrameMs = 0;

void faceReset() { faceFrames = nullptr; faceName = ""; }

void drawFaceFrame() {
  if (!oledOk || !sequenceRunning || !faceFrames || !faceFrames[faceFrameIndex]) return;
  oled.clearDisplay();
  oled.drawBitmap(0, 0, faceFrames[faceFrameIndex], 128, 64, 1);
  oled.display();
}

void setFace(const String& name) {
  if (name == faceName && faceFrames) return;
  faceName = name;
  faceFrames = nullptr;
  for (const FaceEntry& e : faceEntries)
    if (name.equalsIgnoreCase(e.name)) { faceFrames = e.frames; break; }
  if (!faceFrames || !faceFrames[0]) faceFrames = face_defualt_frames;   // (sic) its name in face-bitmaps.h
  faceFrameCount = 0;
  while (faceFrameCount < MAX_FACE_FRAMES && faceFrames[faceFrameCount]) faceFrameCount++;
  faceFpsNow = 1;
  for (const FaceFps& f : faceFpsTable) if (name.equalsIgnoreCase(f.name)) faceFpsNow = f.fps;
  faceFrameIndex = 0;
  faceDir = 1;
  faceDone = false;
  lastFaceFrameMs = millis();
  drawFaceFrame();
}

void setFaceMode(FaceAnimMode mode) { faceMode = mode; faceDir = 1; faceDone = false; }
void setFaceWithMode(const String& name, FaceAnimMode mode) { setFaceMode(mode); setFace(name); }
void enterIdle() {}                          // no idle face: the battery screen shows between sequences

// Next animation frame when it is due (from updateOled while a sequence runs).
void updateFaceFrame() {
  if (!faceFrames || faceFrameCount <= 1 || (faceMode == FACE_ANIM_ONCE && faceDone)) return;
  unsigned long now = millis();
  if (now - lastFaceFrameMs < 1000UL / faceFpsNow) return;
  lastFaceFrameMs = now;
  if (faceMode == FACE_ANIM_LOOP) {
    faceFrameIndex = (faceFrameIndex + 1) % faceFrameCount;
  } else if (faceMode == FACE_ANIM_ONCE) {
    if (faceFrameIndex + 1 >= faceFrameCount) { faceDone = true; return; }   // stay on the last frame
    faceFrameIndex++;
  } else {                                   // boomerang: 0 1 2 1 0 1 ...
    int next = faceFrameIndex + faceDir;
    if (next < 0 || next >= faceFrameCount) { faceDir = -faceDir; next = faceFrameIndex + faceDir; }
    faceFrameIndex = (uint8_t)next;
  }
  drawFaceFrame();
}

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
  if (role == ROLE_LEADER || localRun) servos[channel].write(angle);
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
  if (localRun) { lastPoseRxMs = now; return; }   // playing the sequence ourselves; the stream is the fallback
  // A different sender (Pi <-> leader handover) has its own packet counter: start over instead of
  // dropping its packets as 'stale' until the failsafe trips.
  bool otherSender = memcmp(m.mac, lastPoseMac, 6) != 0;
  if (gotAnyPose && otherSender && now - lastPoseRxMs < 200) return;   // two senders at once: stay with the current one
  bool resync = !gotAnyPose || otherSender || (now - lastPoseRxMs > RESYNC_MS);
  if (!resync && (int16_t)(m.hdr.seq - lastPoseSeq) <= 0) { poseStale++; return; }

  if (gotAnyPose && !resync) {
    uint32_t gap = now - lastPoseRxMs;
    if (gap > intervalMaxGap) intervalMaxGap = gap;
  }
  Pose8Payload p;
  memcpy(&p, m.payload, sizeof(p));
  for (int i = 0; i < NUM_SERVOS; i++) tgtAngle[i] = (uint8_t)constrain((int)p.angles[i], 0, 180);

  lastPoseSeq = m.hdr.seq;
  memcpy(lastPoseMac, m.mac, 6);
  lastPoseRxMs = now;
  lastPoseRssi = m.rssi;
  gotAnyPose = true;
  intervalPoses++;
  if (failsafeTripped) { failsafeTripped = false; Serial.println("Leader back, resumed"); }
}

// Remote commands from the Pi gateway: TEXT payload "CMD:<id|*>:<command>". Commands are only
// queued here (we are inside espnowService) and run from serviceAll(), the same path as serial input.
// The queue keeps their order (e.g. "seq wave" then "stop"); when it is full the oldest is dropped.
const int RCMD_SLOTS = 4;
String remoteCmds[RCMD_SLOTS];
int rcmdHead = 0, rcmdCount = 0;
void queueRemoteCmd(const String& c) {
  if (rcmdCount == RCMD_SLOTS) { rcmdHead = (rcmdHead + 1) % RCMD_SLOTS; rcmdCount--; }
  remoteCmds[(rcmdHead + rcmdCount) % RCMD_SLOTS] = c;
  rcmdCount++;
}

void onMessage(const EspNowRxMsg& m) {
  if (m.hdr.type == ESPNOW_MSG_POSE8) handlePose8(m);
  else if (m.hdr.type == ESPNOW_MSG_TEXT) {
    const char* t = (const char*)m.payload;
    if (strncmp(t, "SYNC:", 5) == 0) {           // "SYNC:<leader id>:<run>:<ms>:<name|stop>" from the leader
      if (role != ROLE_FOLLOWER) return;
      String s = String(t + 5);
      int c1 = s.indexOf(':'), c2 = s.indexOf(':', c1 + 1), c3 = s.indexOf(':', c2 + 1);
      if (c1 <= 0 || c2 < 0 || c3 < 0) return;
      String key = s.substring(0, c2), name = s.substring(c3 + 1);
      if (key == syncLastKey && name != "stop") return;   // repeat of an announce we already played
      syncPendKey = key;
      syncPendName = name;
      syncPendMs = millis() + (unsigned long)constrain(s.substring(c2 + 1, c3).toInt(), 0, 200);
      return;
    }
    if (localRun && strncmp(t, "TLM:", 4) == 0) {
      // The leader's telemetry says it has finished (run=-): end our walk too, in case its 'stop' was lost.
      String s = String(t + 4);
      int c = s.indexOf(':');
      if (c > 0 && syncLastKey.startsWith(s.substring(0, c) + ":") && s.indexOf(";run=-;") >= 0) currentCommand = "";
      return;
    }
    if (strncmp(t, "CMD:", 4) != 0) return;
    String s = String(t + 4);
    int c = s.indexOf(':');
    if (c <= 0) return;
    String id = s.substring(0, c);
    if (id != "*" && !id.equalsIgnoreCase(boardId)) return;   // not for this board
    queueRemoteCmd(s.substring(c + 1));
  }
}

// Status line for the Pi, every second and right after a role change or sequence start/end:
//   TLM:<id>:role=L|F;run=<sequence or ->;v=<V>;ma=<mA>;pct=<%>;up=<uptime s>;rst=<reset reason at boot>
// 'up' lets the ground station notice a reboot; 'rst' says why (e.g. brownout = the supply dipped).
unsigned long lastTlmMs = 0;
bool tlmNow = false;
const char* resetReason = "unknown";
void requestTelemetry() { tlmNow = true; }
void sendTelemetry() {
  unsigned long now = millis();
  if (!tlmNow && now - lastTlmMs < 1000) return;
  tlmNow = false;
  lastTlmMs = now;
  char buf[180];
  snprintf(buf, sizeof(buf), "TLM:%s:role=%c;run=%s;v=%.2f;ma=%.0f;pct=%d;up=%lu;rst=%s;i2c=%lu;lost=%lu;pcar=%lu",
           boardId.c_str(), role == ROLE_LEADER ? 'L' : 'F',
           sequenceRunning ? (runningName.length() ? runningName.c_str() : "busy") : "-",
           battInaOk ? battBusV : 0.0f, battInaOk ? battCurrentMa : 0.0f,
           (battInaOk && battHaveFilter && !battStale) ? battPercent : -1,
           now / 1000UL, resetReason,
           (unsigned long)pcaI2cErr, (unsigned long)pcaLost, (unsigned long)pcaResets);
  espnowSendText(String(buf));
}

// The role survives reboots (kept in flash), so a brownout or reset no longer turns a leader into a follower.
Preferences prefs;
void saveRole() { prefs.putUChar("role", role == ROLE_LEADER ? 1 : 0); }

const char* resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "poweron";
    case ESP_RST_EXT:       return "external";
    case ESP_RST_SW:        return "software";
    case ESP_RST_PANIC:     return "crash";
    case ESP_RST_INT_WDT:   return "int-watchdog";
    case ESP_RST_TASK_WDT:  return "task-watchdog";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_BROWNOUT:  return "brownout";
    case ESP_RST_DEEPSLEEP: return "deepsleep";
    default:                return "unknown";
  }
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

  if (localRun) return;                      // playing an announced sequence: servos are driven directly
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

void playSequence(const String& name);

void leaderSyncAnnounce() {
  if (!syncSendsLeft || role != ROLE_LEADER) return;
  unsigned long now = millis();
  if (lastSyncSendMs && now - lastSyncSendMs < 30) return;
  lastSyncSendMs = now ? now : 1;
  syncSendsLeft--;
  long wait = (long)(syncStartMs - now);
  if (wait < 0 || syncName == "stop") wait = 0;
  espnowSendText("SYNC:" + boardId + ":" + String(syncRunId) + ":" + String(wait) + ":" + syncName);
}

void followerSync() {
  if (!syncPendName.length()) return;
  if (role != ROLE_FOLLOWER) { syncPendName = ""; return; }
  if (sequenceRunning) {
    // 'stop', or the leader already started the next one: end ours (walks and turns stop at the next step)
    bool isStop = syncPendName == "stop";
    if (localRun && (isStop ? syncPendKey == syncLastKey : syncPendKey != syncLastKey)) currentCommand = "";
    if (syncPendName == "stop") syncPendName = "";
    return;                                  // a new sequence starts once this one has ended
  }
  if (syncPendName != "stop" && (long)(millis() - syncPendMs) < 0) return;   // not time to start yet
  String name = syncPendName, key = syncPendKey;
  syncPendName = "";
  if (name == "stop" || millis() - syncPendMs > 1000 || !isSequenceName(name)) return;   // late: keep copying the stream
  syncLastKey = key;
  localRun = true;
  for (int i = 0; i < NUM_SERVOS; i++) cmdAngle[i] = (uint8_t)lroundf(curAngle[i]);
  Serial.printf("Playing '%s' together with the leader\n", name.c_str());
  playSequence(name);
  localRun = false;
  for (int i = 0; i < NUM_SERVOS; i++) { curAngle[i] = cmdAngle[i]; tgtAngle[i] = cmdAngle[i]; }
  gotAnyPose = false;                        // take the leader's stream up again from its next packet
  failsafeTripped = false;
}

// Remote commands get exactly one reply: ACK (accepted) or ERR:<id>:<cmd>:<reason> (refused).
// The Pi re-sends a command until it hears a reply, so an accepted command that arrives again within 1.2 s
// is a re-send: it gets the ACK again and is not run a second time. Refused commands are judged afresh.
String lastRcmd = "", lastRreply = "";
unsigned long lastRcmdMs = 0;
void serviceRemoteCmd() {
  if (!rcmdCount) return;
  String c = remoteCmds[rcmdHead];
  rcmdHead = (rcmdHead + 1) % RCMD_SLOTS;
  rcmdCount--;
  String lc = c;
  lc.trim();
  lc.toLowerCase();
  if (lc == lastRcmd && lastRreply.startsWith("ACK:") && millis() - lastRcmdMs < 1200) {
    lastRcmdMs = millis();                   // still re-sending: keep treating it as the same command
    espnowSendText(lastRreply);
    return;
  }
  String reply;
  if (sequenceRunning && lc != "stop")
    reply = "ERR:" + boardId + ":" + c + ":busy running " + (runningName.length() ? runningName : String("a sequence"));
  else if (lc.startsWith("seq ") && role != ROLE_LEADER)
    reply = "ERR:" + boardId + ":" + c + ":not the leader";
  else
    reply = "ACK:" + boardId + ":" + c;
  lastRcmd = lc;
  lastRreply = reply;
  lastRcmdMs = millis();
  espnowSendText(reply);
  if (reply.startsWith("ACK:")) handleCommand(c);
}

void serviceAll() {
  espnowService();
  pcaCheck();
  leaderSyncAnnounce();
  leaderStream();
  followerUpdate();
  batteryUpdate();
  updateOled();
  pollSerial();
  sendTelemetry();
  serviceRemoteCmd();
  followerSync();
}

// ---------------------------------------------------------------- sequences
void runSequence(const String& name) {
  if (role != ROLE_LEADER) { Serial.println("Only the leader runs sequences. Send 'leader' first."); return; }
  if (!isSequenceName(name)) {
    Serial.println("Unknown sequence. Names: rest stand wave dance swim point pushup bow cute freaky worm shake shrug dead crab forward backward left right");
    return;
  }
  syncRunId++;                               // announce it so the followers play it in step
  syncName = name;
  syncSendsLeft = 3;
  lastSyncSendMs = 0;
  syncStartMs = millis() + SYNC_LEAD_MS;
  faceReset();                               // the screen keeps the battery view until the sequence sets its face
  sequenceRunning = true;                    // busy while waiting: remote commands are refused, not nested
  runningName = name;
  leaderSyncAnnounce();
  while ((long)(millis() - syncStartMs) < 0) { serviceAll(); delay(1); }
  playSequence(name);
}

void playSequence(const String& name) {
  bool found = true;
  faceReset();                               // always draw the sequence's face, even if it ran last time
  sequenceRunning = true;
  currentCommand = name;
  runningName = name;
  requestTelemetry();                 // the dashboard shows "running <name>" right away
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
  // The forward/backward and left/right function bodies were renamed in movement-sequences.h
  // (not the servo maths), so the command names map straight through again.
  else if (name == "forward")  runWalkPose();
  else if (name == "backward") runWalkBackward();
  else if (name == "left")     runTurnLeft();
  else if (name == "right")    runTurnRight();
  else found = false;
  sequenceRunning = false;
  currentCommand = "";
  runningName = "";
  faceReset();
  lastOledMs = 0;                            // battery screen again right away
  requestTelemetry();

  Serial.println(found ? "Sequence done" : "Unknown sequence");
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
  Serial.printf("Wiggling %s (index %d, CH %d) on this board only\n", ServoNames[idx].c_str(), idx, servoChannels[idx]);
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
  Serial.printf("%s (index %d, CH %d) -> %d deg on the leader, streamed to the follower\n",
                ServoNames[idx].c_str(), idx, servoChannels[idx], angle);
}

// Leader only: sweep ONE servo 45-135 deg five times, then back to 90. 'stop' ends it early.
void sweepServoCmd(const String& arg) {
  if (role != ROLE_LEADER) { Serial.println("Only the leader sends servo commands. Send 'leader' first."); return; }
  int idx = parseServoIndex(arg);
  if (idx < 0) { Serial.println("Usage: sweep <R1 R2 L1 L2 R3 R4 L3 L4 or 0-7>"); return; }
  Serial.printf("Sweeping %s (index %d, CH %d) 5 times, 'stop' ends it\n", ServoNames[idx].c_str(), idx, servoChannels[idx]);
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
// fullsweep <servo> [minDeg maxDeg]: slow PHYSICAL angle sweep (uncalibrated), default 0..180 in 2 deg steps.
// Prints the angle every 10 deg, then returns and HOLDS at physical 90 (the middle of the code's range)
// so you can look at the horn and 'jog' from there. 'stop' ends it early (servo goes limp).
// This board only, not streamed.
void fullSweepCmd(const String& args) {
  String a = args;
  a.trim();
  int sp = a.indexOf(' ');
  int idx = parseServoIndex(sp > 0 ? a.substring(0, sp) : a);
  int lo = 0, hi = 180;
  if (sp > 0) {
    String r = a.substring(sp + 1);
    r.trim();
    int sp2 = r.indexOf(' ');
    if (sp2 > 0) { lo = r.substring(0, sp2).toInt(); hi = r.substring(sp2 + 1).toInt(); }
  }
  if (idx < 0 || lo < 0 || hi > 180 || lo > 90 || hi < 90) {
    Serial.println("Usage: fullsweep <servo> [minDeg maxDeg]  (0-180, range must include 90; default 0 180)");
    return;
  }
  Serial.printf("Full sweep %s (CH %d) %d..%d deg, then hold at 90. 'stop' ends it\n",
                ServoNames[idx].c_str(), servoChannels[idx], lo, hi);
  sequenceRunning = true;
  currentCommand = "fullsweep";
  for (int ang = lo; ang <= hi && currentCommand == "fullsweep"; ang += 2) {
    servos[idx].writeRaw(ang);
    if (ang % 10 == 0) Serial.printf("%d deg\n", ang);
    delayWithFace(40);
  }
  for (int ang = hi; ang >= 90 && currentCommand == "fullsweep"; ang -= 2) {
    servos[idx].writeRaw(ang);
    if (ang % 10 == 0) Serial.printf("%d deg\n", ang);
    delayWithFace(40);
  }
  if (currentCommand == "fullsweep") {
    servos[idx].writeRaw(90);
    rawAngle[idx] = 90;
    Serial.printf("Holding %s at physical 90 deg. Use 'jog %s +/-n' to adjust, 'off %s' to release\n",
                  ServoNames[idx].c_str(), ServoNames[idx].c_str(), ServoNames[idx].c_str());
  } else {
    servos[idx].release();
    Serial.println("Stopped, servo limp");
  }
  sequenceRunning = false;
  currentCommand = "";
}

// sweepus <servo> [minUs maxUs]: slow PHYSICAL sweep by pulse width, to find the servo's real range.
// Default 1000-2000 us (safe for any servo); widen it a little at a time, e.g. 'sweepus r1 600 2400'.
// Prints the pulse every 100 us: note where the horn starts and stops moving. 'stop' ends it early.
// This board only, not streamed, ignores calibration. The servo goes limp afterwards.
void sweepUsCmd(const String& args) {
  String a = args;
  a.trim();
  int sp = a.indexOf(' ');
  int idx = parseServoIndex(sp > 0 ? a.substring(0, sp) : a);
  long lo = 1000, hi = 2000;
  if (sp > 0) {
    String r = a.substring(sp + 1);
    r.trim();
    int sp2 = r.indexOf(' ');
    if (sp2 > 0) { lo = r.substring(0, sp2).toInt(); hi = r.substring(sp2 + 1).toInt(); }
  }
  if (idx < 0 || lo < 400 || hi > 2600 || hi - lo < 100) {
    Serial.println("Usage: sweepus <servo> [minUs maxUs]  (400-2600, at least 100 apart; default 1000 2000)");
    return;
  }
  Serial.printf("Sweeping %s (CH %d) %ld..%ld us, 2 round trips, 'stop' ends it\n",
                ServoNames[idx].c_str(), servoChannels[idx], lo, hi);
  sequenceRunning = true;
  currentCommand = "sweepus";
  for (int cycle = 0; cycle < 2 && currentCommand == "sweepus"; cycle++) {
    for (long us = lo; us <= hi && currentCommand == "sweepus"; us += 10) {
      servos[idx].writeUs(us);
      if (us % 100 == 0) Serial.printf("%ld us\n", us);
      delayWithFace(40);
    }
    for (long us = hi; us >= lo && currentCommand == "sweepus"; us -= 10) {
      servos[idx].writeUs(us);
      if (us % 100 == 0) Serial.printf("%ld us\n", us);
      delayWithFace(40);
    }
  }
  servos[idx].release();
  sequenceRunning = false;
  currentCommand = "";
  Serial.println("Sweep done, servo limp");
}

void printHelp() {
  Serial.println("Commands: help, status, leader, follower, seq <name>, raw <servo> <angle>, jog <servo> <+/-deg>, us <servo> <us>, fullsweep <servo> [minDeg maxDeg], sweepus <servo> [min max], range <min> <max>, off [servo|all], wiggle <servo>, move <servo> <angle>, sweep <servo>, batt, batt r <mohm>, batt peak, stop, power <dBm>, slew <deg>");
  Serial.println("Sequences: rest stand wave dance swim point pushup bow cute freaky worm shake shrug dead crab forward backward left right");
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (!cmd.length()) return;

  if (sequenceRunning) {
    if (cmd == "stop") {
      currentCommand = "";
      Serial.println("Stop requested");
      if (role == ROLE_LEADER && !localRun) { syncName = "stop"; syncSendsLeft = 2; lastSyncSendMs = 0; }
    }
    else Serial.println("Busy: only 'stop' works while a sequence runs");
    return;
  }

  if (cmd == "help") printHelp();
  else if (cmd == "status") {
    espnowPrintInfo();
    Serial.printf("  role %s | stale poses %lu | slew %.1f deg/10ms | TX power %d dBm\n",
                  role == ROLE_LEADER ? "LEADER" : "FOLLOWER",
                  (unsigned long)poseStale, slewStep, txPowerDbm);
    Serial.printf("  PCA9685: I2C write errors %lu, lost servo moves %lu, resets repaired %lu | reset reason %s\n",
                  (unsigned long)pcaI2cErr, (unsigned long)pcaLost, (unsigned long)pcaResets, resetReason);
  }
  else if (cmd == "leader") {
    if (role != ROLE_LEADER) {          // only on a change: a repeated 'leader' must not reset the stream
      role = ROLE_LEADER;
      for (int i = 0; i < NUM_SERVOS; i++) cmdAngle[i] = (uint8_t)lroundf(curAngle[i]);
      saveRole();
    }
    requestTelemetry();
    Serial.println("Role: LEADER");
  }
  else if (cmd == "follower") {
    if (role != ROLE_FOLLOWER) {        // only on a change: a repeated 'follower' must not move the servos
      role = ROLE_FOLLOWER;
      syncSendsLeft = 0;
      gotAnyPose = false;
      failsafeTripped = false;
      for (int i = 0; i < NUM_SERVOS; i++) { curAngle[i] = cmdAngle[i]; tgtAngle[i] = cmdAngle[i]; }
      saveRole();
    }
    requestTelemetry();
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
      rawAngle[idx] = ang;
      Serial.printf("RAW %s (CH %d) -> physical %d deg (this board only)\n", ServoNames[idx].c_str(), servoChannels[idx], ang);
    }
  }
  else if (cmd.startsWith("jog ")) {
    // jog <servo> <+/-deg>: nudge the last 'raw' angle of that servo (physical, this board only)
    String a = cmd.substring(4);
    int sp = a.indexOf(' ');
    int idx = (sp > 0) ? parseServoIndex(a.substring(0, sp)) : -1;
    if (idx < 0) Serial.println("Usage: jog <servo> <+/-deg>  (needs a prior 'raw <servo> <angle>')");
    else if (rawAngle[idx] < 0) Serial.println("Set a start angle first: raw <servo> <angle>");
    else {
      rawAngle[idx] = constrain(rawAngle[idx] + (int)a.substring(sp + 1).toInt(), 0, 180);
      servos[idx].writeRaw(rawAngle[idx]);
      Serial.printf("JOG %s (CH %d) -> physical %d deg\n", ServoNames[idx].c_str(), servoChannels[idx], rawAngle[idx]);
    }
  }
  else if (cmd.startsWith("off")) {
    // off [servo|all]: stop the pulse so the servo goes limp (default: all)
    String a = cmd.substring(3);
    a.trim();
    if (a.length() == 0 || a == "all") {
      for (int i = 0; i < NUM_SERVOS; i++) { servos[i].release(); rawAngle[i] = -1; }
      Serial.println("All servos limp");
    } else {
      int idx = parseServoIndex(a);
      if (idx < 0) Serial.println("Usage: off [servo|all]");
      else { servos[idx].release(); rawAngle[idx] = -1; Serial.printf("%s limp\n", ServoNames[idx].c_str()); }
    }
  }
  else if (cmd.startsWith("us ")) {
    // us <servo> <pulse in microseconds 400-2600>: exact pulse width, bypasses angle maths. This board only.
    String a = cmd.substring(3);
    int sp = a.indexOf(' ');
    int idx = (sp > 0) ? parseServoIndex(a.substring(0, sp)) : -1;
    if (idx < 0) Serial.println("Usage: us <servo> <microseconds>");
    else {
      long us = constrain((long)a.substring(sp + 1).toInt(), 400L, 2600L);
      servos[idx].writeUs(us);
      Serial.printf("US %s (CH %d) -> %ld us\n", ServoNames[idx].c_str(), servoChannels[idx], us);
    }
  }
  else if (cmd.startsWith("sweepus ")) sweepUsCmd(cmd.substring(8));
  else if (cmd.startsWith("fullsweep ")) fullSweepCmd(cmd.substring(10));
  else if (cmd.startsWith("range ")) {
    // range <minUs> <maxUs>: pulse width for physical 0 and 180 deg, all servos (until reboot)
    String a = cmd.substring(6);
    int sp = a.indexOf(' ');
    int lo = (sp > 0) ? a.substring(0, sp).toInt() : 0;
    int hi = (sp > 0) ? a.substring(sp + 1).toInt() : 0;
    if (lo < 400 || hi > 2600 || hi - lo < 500) Serial.println("Usage: range <min 400+> <max 2600-> (at least 500 apart), e.g. range 500 2500");
    else {
      for (int i = 0; i < NUM_SERVOS; i++) servos[i].setRange(lo, hi);
      Serial.printf("Pulse range now %d..%d us for 0..180 deg (edit MIN_US/MAX_US to keep it)\n", lo, hi);
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
  resetReason = resetReasonName(esp_reset_reason());
  Serial.printf("Reset reason: %s\n", resetReason);
  prefs.begin("sesame", false);
  role = prefs.getUChar("role", 0) == 1 ? ROLE_LEADER : ROLE_FOLLOWER;   // last role, kept across reboots

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);
  for (int ch = 0; ch < 16; ch++) pcaLastUs[ch] = -1;
  pcaOk = pcaBegin();
  if (!pcaOk) Serial.printf("PCA9685 NOT found at 0x%02X: servos disabled (check wiring)\n", PCA_ADDR);

  for (int i = 0; i < NUM_SERVOS; i++) {
    cmdAngle[i] = 90;
    tgtAngle[i] = 90;
    curAngle[i] = 90.0f;
    servos[i].attach(servoChannels[i], MIN_US, MAX_US, i);
    servos[i].write(90);
    delay(motorCurrentDelay);   // stagger to limit inrush current
  }

  // OLED + INA260 on the shared I2C bus (both optional: the tester runs without them).
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
  syncRunId = (uint16_t)random(65536);
  espnowOnMessage = onMessage;

  Serial.println("Servo map (index, name, PCA9685 channel):");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("  %d  %s  CH %d\n", i, ServoNames[i].c_str(), servoChannels[i]);
  }
  Serial.printf("Role: %s (restored from flash; 'leader' / 'follower' to change) | TX %d dBm\n",
                role == ROLE_LEADER ? "LEADER" : "FOLLOWER", txPowerDbm);
  printHelp();
}

void loop() {
  serviceAll();
}
