/*
 * face-tester.ino — Preview every face in face-bitmaps.h on an SSD1306 OLED
 * AND drive two servos as a single-leg test. No WiFi, no web server.
 *
 * Target board: Wemos/LOLIN D1 Mini (ESP8266).
 *
 * The faces cycle on the OLED while two servos (one "leg" = hip + knee) sweep
 * back and forth together. Everything is non-blocking (millis-based) so the
 * servos move smoothly while the display animates. Face names are printed to
 * Serial (115200).
 *
 * ---- SETUP ----
 *   1. Keep this file in a folder named "face-tester".
 *   2. face-bitmaps.h must sit in this same folder (a copy is already here).
 *   3. Install ESP8266 board support:
 *        File > Preferences > Additional Boards Manager URLs, add:
 *        http://arduino.esp8266.com/stable/package_esp8266com_index.json
 *        then Boards Manager > install "esp8266 by ESP8266 Community".
 *   4. Install the "Adafruit SSD1306" and "Adafruit GFX" libraries.
 *      (Servo is bundled with the ESP8266 core — no install needed.)
 *   5. Tools > Board > "LOLIN(WEMOS) D1 R2 & mini", select the port, upload.
 *
 * ---- WIRING ----
 *   OLED VCC -> 3V3        OLED SDA -> D2 (GPIO4)
 *   OLED GND -> G          OLED SCL -> D1 (GPIO5)
 *
 *   Hip servo  signal -> D5 (GPIO14)
 *   Knee servo signal -> D6 (GPIO12)
 *
 *   >> Power the servos from an EXTERNAL 5V supply, NOT the D1 Mini's pins.
 *      Two servos can brown out the board over USB. Tie all grounds together
 *      (servo GND <-> external supply GND <-> D1 Mini G).
 */

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>
#include "face-bitmaps.h"

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define OLED_I2C_ADDR 0x3C   // most SSD1306 modules are 0x3C; some are 0x3D

// I2C pins for the Wemos D1 Mini (ESP8266). D2 = GPIO4 (SDA), D1 = GPIO5 (SCL).
#define I2C_SDA D2
#define I2C_SCL D1

// Servo pins (one leg = two joints). D5 = GPIO14, D6 = GPIO12 — both safe on ESP8266.
#define HIP_PIN  D5
#define KNEE_PIN D6

// How long to linger on each face, and the playback rate for animated frames.
const unsigned long FACE_DISPLAY_MS = 2500;
const int           ANIM_FPS        = 5;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ======================================================================
// Faces
// ======================================================================
static const uint8_t MAX_FACE_FRAMES = 6;

// Build a frame-pointer array for each face (identical to the main firmware).
// Frames that aren't defined in face-bitmaps.h are weak symbols that resolve
// to nullptr, so countFrames() below stops at the first missing frame.
#define MAKE_FACE_FRAMES(name) \
  const unsigned char* const face_##name##_frames[] = { \
    epd_bitmap_##name, epd_bitmap_##name##_1, epd_bitmap_##name##_2, \
    epd_bitmap_##name##_3, epd_bitmap_##name##_4, epd_bitmap_##name##_5 \
  };

#define X(name) MAKE_FACE_FRAMES(name)
FACE_LIST
#undef X
#undef MAKE_FACE_FRAMES

struct FaceEntry {
  const char* name;
  const unsigned char* const* frames;
  uint8_t maxFrames;
};

const FaceEntry faceEntries[] = {
#define X(name) { #name, face_##name##_frames, MAX_FACE_FRAMES },
  FACE_LIST
#undef X
};

const size_t FACE_COUNT = sizeof(faceEntries) / sizeof(faceEntries[0]);

uint8_t countFrames(const unsigned char* const* frames, uint8_t maxFrames) {
  if (frames == nullptr || frames[0] == nullptr) return 0;
  uint8_t count = 0;
  for (uint8_t i = 0; i < maxFrames; i++) {
    if (frames[i] == nullptr) break;
    count++;
  }
  return count;
}

void showBitmap(const unsigned char* bitmap) {
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
}

// Non-blocking face-cycling state.
size_t        faceIndex   = 0;
uint8_t       curFrames   = 0;
uint8_t       frameIdx    = 0;
unsigned long faceStartMs = 0;
unsigned long lastFrameMs = 0;

void loadFace(size_t i) {
  faceIndex   = i;
  curFrames   = countFrames(faceEntries[i].frames, faceEntries[i].maxFrames);
  frameIdx    = 0;
  faceStartMs = millis();
  lastFrameMs = millis();
  if (curFrames > 0) {
    showBitmap(faceEntries[i].frames[0]);
    Serial.print(F("Face: "));    Serial.print(faceEntries[i].name);
    Serial.print(F("  frames=")); Serial.println(curFrames);
  }
}

// Advance to the next face that actually has bitmap data (skips e.g. "stand").
void advanceFace() {
  size_t next = faceIndex;
  for (size_t tries = 0; tries < FACE_COUNT; tries++) {
    next = (next + 1) % FACE_COUNT;
    if (countFrames(faceEntries[next].frames, faceEntries[next].maxFrames) > 0) break;
  }
  loadFace(next);
}

void updateFace() {
  unsigned long now = millis();
  if (curFrames > 1 && now - lastFrameMs >= (1000UL / ANIM_FPS)) {
    lastFrameMs = now;
    frameIdx = (frameIdx + 1) % curFrames;
    showBitmap(faceEntries[faceIndex].frames[frameIdx]);
  }
  if (now - faceStartMs >= FACE_DISPLAY_MS) {
    advanceFace();
  }
}

// ======================================================================
// One-leg servo test (hip + knee): continuous back-and-forth sweep
// ======================================================================
Servo legHip;
Servo legKnee;

// Sweep range (degrees) and speed. Conservative range so nothing slams the
// end-stops — widen SWEEP_MIN/MAX once you've confirmed the safe travel.
const int SWEEP_MIN = 30;
const int SWEEP_MAX = 150;

int sweepPos = 90;   // current angle, shared by both servos
int sweepDir = 1;    // +1 = toward MAX, -1 = toward MIN

unsigned long lastLegMs = 0;
const unsigned long LEG_STEP_MS  = 15;  // how often we nudge the servos
const int           LEG_DEG_STEP = 2;   // degrees per nudge (lower = slower/smoother)

void updateLeg() {
  unsigned long now = millis();
  if (now - lastLegMs < LEG_STEP_MS) return;
  lastLegMs = now;

  sweepPos += sweepDir * LEG_DEG_STEP;
  if (sweepPos >= SWEEP_MAX) { sweepPos = SWEEP_MAX; sweepDir = -1; }
  if (sweepPos <= SWEEP_MIN) { sweepPos = SWEEP_MIN; sweepDir =  1; }

  legHip.write(sweepPos);
  legKnee.write(sweepPos);   // swap to (180 - sweepPos) to mirror the knee
}

// ======================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz Fast Mode for smoother frame pushes

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println(F("SSD1306 allocation failed — check wiring, pins, and I2C address."));
    while (1) { delay(100); }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("Face + Leg Test"));
  display.println(F("Cycling faces..."));
  display.display();
  Serial.println(F("Face + one-leg test started."));
  delay(1500);

  // Servos: start centered, then sweep from loop().
  legHip.attach(HIP_PIN);
  legKnee.attach(KNEE_PIN);
  legHip.write(sweepPos);
  legKnee.write(sweepPos);

  // Land on the first face that actually has bitmap data.
  loadFace(0);
  if (curFrames == 0) advanceFace();
}

void loop() {
  updateFace();
  updateLeg();
  yield();
}
