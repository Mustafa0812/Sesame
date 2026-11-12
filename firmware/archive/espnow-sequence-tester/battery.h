#pragma once
// ======================================================================
// Battery level from an INA260 (voltage / current monitor), shown on the OLED
// ======================================================================
// WIRING (the INA260 must see the BATTERY voltage, so it goes BEFORE the buck):
//   battery +  ->  INA260 IN+
//   INA260 IN- ->  buck converter input +   (and the rest of the robot)
//   battery -  ->  common ground (also the ESP32, buck and servo grounds)
//   INA260 VCC 3V3, GND, SDA = GPIO6, SCL = GPIO11 (same I2C bus as the OLED).
//   If the INA260 were placed AFTER the buck it would only see a flat 5.1 V and
//   the percentage would be meaningless until the pack is nearly dead.
//
// HOW THE PERCENTAGE IS WORKED OUT
//   1. Read pack voltage (V) and current (I) from the INA260 four times a second.
//   2. Load compensation: pack voltage sags when servos draw current, so
//        open-circuit voltage = V + I * R_internal
//      R_internal defaults to BATT_DEFAULT_MOHM. It is a GUESS for two small
//      Li-ion cells; tune it with the 'batt r <mohm>' serial command (see below).
//   3. Smooth it with a slow low-pass filter (BATT_FILTER_TAU_MS) so servo surges
//      do not make the percentage jump.
//   4. Divide by the number of cells and look it up in a Li-ion voltage curve.
//   This is an estimate from voltage only (no coulomb counting). It reads best
//   when the robot is not under heavy load, and assumes the cells are balanced.
//
// TUNING R_internal: with the robot standing still note the voltage ('batt'),
//   then run a servo-heavy move (e.g. dance) and note voltage and current.
//   R (mOhm) = (V_idle - V_load) / (I_load_A - I_idle_A) * 1000. Set it with
//   'batt r <mohm>'. When testing from a BENCH SUPPLY (no internal resistance)
//   use 'batt r 0', otherwise the percentage reads high under load.
//
// Serial commands (see sesame-firmware-main.ino):  batt   |   batt r <mohm>
// HTTP: GET /api/battery
// The robot runs fine without an INA260; the overlay is simply not drawn.
// ======================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

// ---- Configuration ---------------------------------------------------
#define BATT_CELLS              2       // 2S pack (e.g. 2x 14500 Li-ion), 6.0-8.4 V
#define BATT_DEFAULT_MOHM       200     // pack internal resistance guess (milliohms)
#define BATT_LOW_PERCENT        15      // at or below: the display shows "!"
#define BATT_UPDATE_MS          250     // INA260 read rate
#define BATT_FILTER_TAU_MS      6000UL  // low-pass time constant
#define BATT_STALE_MS           2000UL  // no good INA260 read for this long -> "--"
#define BATT_OVERLAY_ENABLED    1       // 0 = never draw on the OLED
#define BATT_SHOW_PERCENT_TEXT  1       // 0 = icon only

// ---- INA260 registers -------------------------------------------------
#define BATT_REG_CONFIG   0x00
#define BATT_REG_CURRENT  0x01   // signed, 1.25 mA per LSB
#define BATT_REG_VOLTAGE  0x02   // unsigned, 1.25 mV per LSB
#define BATT_REG_POWER    0x03   // unsigned, 10 mW per LSB
#define BATT_REG_MFR_ID   0xFE   // reads 0x5449 ("TI")
#define BATT_REG_DIE_ID   0xFF   // upper 12 bits read 0x227 for the INA260

// Open-circuit voltage per cell -> state of charge (typical Li-ion curve).
// Conservative: 0% at 3.27 V per cell, well above the 3.0 V protection cut-off.
struct BattPoint { float cellV; uint8_t pct; };
static const BattPoint BATT_CURVE[] = {
  {3.27f, 0},  {3.61f, 5},  {3.69f, 10}, {3.71f, 15}, {3.73f, 20},
  {3.75f, 25}, {3.77f, 30}, {3.79f, 35}, {3.80f, 40}, {3.82f, 45},
  {3.84f, 50}, {3.85f, 55}, {3.87f, 60}, {3.91f, 65}, {3.95f, 70},
  {3.98f, 75}, {4.02f, 80}, {4.08f, 85}, {4.11f, 90}, {4.15f, 95},
  {4.20f, 100}
};

// ---- State -------------------------------------------------------------
bool battInaOk = false;
uint8_t battInaAddr = 0;
float battBusV = 0;                 // measured pack voltage (V)
float battCurrentMa = 0;            // measured current (mA, + = discharging)
int battPowerMw = 0;
float battOcvFiltered = 0;          // filtered, load-compensated pack voltage
bool battHaveFilter = false;
int battPercent = 0;                // 0..100 (valid when battHaveFilter && !battStale)
bool battLow = false;
bool battStale = false;
bool battDirty = false;             // something visible changed: redraw the display
int battInternalMohm = BATT_DEFAULT_MOHM;
uint32_t battErrors = 0;
unsigned long battLastReadMs = 0, battLastGoodMs = 0;

// ---- INA260 helpers -------------------------------------------------------
static bool battRead16(uint8_t addr, uint8_t reg, uint16_t& val) {
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

static bool battWrite16(uint8_t addr, uint8_t reg, uint16_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  return Wire.endTransmission() == 0;
}

static float battPercentFromCell(float v) {
  const int n = sizeof(BATT_CURVE) / sizeof(BATT_CURVE[0]);
  if (v <= BATT_CURVE[0].cellV) return 0.0f;
  if (v >= BATT_CURVE[n - 1].cellV) return 100.0f;
  for (int i = 1; i < n; i++) {
    if (v <= BATT_CURVE[i].cellV) {
      float t = (v - BATT_CURVE[i - 1].cellV) / (BATT_CURVE[i].cellV - BATT_CURVE[i - 1].cellV);
      return BATT_CURVE[i - 1].pct + t * (BATT_CURVE[i].pct - BATT_CURVE[i - 1].pct);
    }
  }
  return 100.0f;
}

// Call once after Wire.begin(). Optional: if no INA260 answers, nothing is drawn.
bool batteryBegin() {
  battInaOk = false;
  battInaAddr = 0;
  for (uint8_t addr = 0x40; addr <= 0x4F; addr++) {
    uint16_t mfr = 0, die = 0;
    if (!battRead16(addr, BATT_REG_MFR_ID, mfr)) continue;
    if (!battRead16(addr, BATT_REG_DIE_ID, die)) continue;
    if (mfr == 0x5449 && (die >> 4) == 0x227) { battInaAddr = addr; break; }
  }
  if (battInaAddr == 0) {
    Serial.println(F("Battery: no INA260 found at 0x40-0x4F, battery display disabled"));
    return false;
  }
  battWrite16(battInaAddr, BATT_REG_CONFIG, 0x8000);   // reset
  delay(5);
  // 4-sample averaging, 1.1 ms conversions, continuous mode
  if (!battWrite16(battInaAddr, BATT_REG_CONFIG, 0x6327)) {
    Serial.println(F("Battery: INA260 configuration failed, battery display disabled"));
    return false;
  }
  battInaOk = true;
  battLastGoodMs = millis();
  Serial.printf("Battery: INA260 found at 0x%02X (%d cells, R=%d mOhm)\n",
                battInaAddr, BATT_CELLS, battInternalMohm);
  return true;
}

// Call often (non-blocking). Reads the INA260 at BATT_UPDATE_MS.
void batteryUpdate() {
  if (!battInaOk) return;
  unsigned long now = millis();
  if (battLastReadMs != 0 && now - battLastReadMs < BATT_UPDATE_MS) return;
  unsigned long dt = (battLastReadMs == 0) ? BATT_UPDATE_MS : now - battLastReadMs;
  battLastReadMs = now;

  // Mark the reading stale (and redraw once) if the INA260 stops answering.
  bool stale = (now - battLastGoodMs > BATT_STALE_MS);
  if (stale != battStale) { battStale = stale; battDirty = true; }

  uint16_t rawI, rawV, rawP;
  if (!battRead16(battInaAddr, BATT_REG_CURRENT, rawI) ||
      !battRead16(battInaAddr, BATT_REG_VOLTAGE, rawV) ||
      !battRead16(battInaAddr, BATT_REG_POWER, rawP)) {
    battErrors++;
    return;
  }
  battLastGoodMs = now;
  if (battStale) { battStale = false; battDirty = true; }

  battBusV = rawV * 0.00125f;
  battCurrentMa = (int16_t)rawI * 1.25f;
  battPowerMw = (int)rawP * 10;

  // Add back the voltage lost across the pack's internal resistance.
  float amps = (battCurrentMa > 0) ? battCurrentMa / 1000.0f : 0.0f;
  float ocv = battBusV + amps * (battInternalMohm / 1000.0f);

  if (!battHaveFilter) {
    battOcvFiltered = ocv;
    battHaveFilter = true;
    battDirty = true;
  } else {
    float alpha = (float)dt / (BATT_FILTER_TAU_MS + dt);
    battOcvFiltered += alpha * (ocv - battOcvFiltered);
  }

  int shown = (int)lroundf(battPercentFromCell(battOcvFiltered / BATT_CELLS));
  bool low = (shown <= BATT_LOW_PERCENT);
  if (shown != battPercent || low != battLow) {
    battPercent = shown;
    battLow = low;
    battDirty = true;
  }
}

// True once after something visible changed, so the caller can redraw the face.
bool batteryOverlayDirty() {
  if (!battDirty) return false;
  battDirty = false;
  return true;
}

// Draw a small battery icon (and percent) in the bottom-right corner of the
// display. Call after drawing the face and BEFORE display.display().
void batteryDrawOverlay(Adafruit_GFX& gfx) {
#if BATT_OVERLAY_ENABLED
  if (!battInaOk) return;
  const int iw = 18, ih = 9;                      // icon size including the nub
  int ix = gfx.width() - iw - 1;
  int iy = gfx.height() - ih - 1;
  bool unknown = battStale || !battHaveFilter;

  String txt;
#if BATT_SHOW_PERCENT_TEXT
  if (unknown) txt = "--";
  else txt = String(battLow ? "!" : "") + String(battPercent) + "%";
#endif
  int textW = txt.length() * 6;
  int boxLeft = ix - (textW ? textW + 3 : 1);

  gfx.fillRect(boxLeft, iy - 1, gfx.width() - boxLeft, ih + 2, 0);   // black backing
  gfx.drawRect(ix, iy, iw - 2, ih, 1);                                // body
  gfx.fillRect(ix + iw - 2, iy + 2, 2, ih - 4, 1);                    // nub
  if (!unknown) {
    int inner = iw - 6;                                               // 12 px
    int fw = (battPercent * inner + 50) / 100;
    if (battPercent > 0 && fw < 1) fw = 1;
    if (fw > inner) fw = inner;
    if (fw > 0) gfx.fillRect(ix + 2, iy + 2, fw, ih - 4, 1);
  }
  if (textW) {
    gfx.setTextSize(1);
    gfx.setTextColor(1);
    gfx.setTextWrap(false);
    gfx.setCursor(ix - textW - 2, iy + 1);
    gfx.print(txt);
    gfx.setTextWrap(true);
  }
#endif
}

void batteryPrint() {
  if (!battInaOk) { Serial.println(F("Battery: no INA260")); return; }
  float cellV = battOcvFiltered / BATT_CELLS;
  Serial.printf("Battery: %.2f V, %.0f mA, %d mW | load-compensated %.2f V (%.2f V/cell) | %d%%%s%s | R=%d mOhm | read errors %lu\n",
                battBusV, battCurrentMa, battPowerMw, battOcvFiltered, cellV, battPercent,
                battLow ? " LOW" : "", (battStale || !battHaveFilter) ? " (no data)" : "",
                battInternalMohm, (unsigned long)battErrors);
}

void batterySetResistance(int mohm) {
  battInternalMohm = constrain(mohm, 0, 2000);
  battHaveFilter = false;       // restart the filter so the new R takes effect at once
  battDirty = true;
  Serial.printf("Battery: internal resistance set to %d mOhm\n", battInternalMohm);
}

String batteryJson() {
  if (!battInaOk) return String("{\"present\":false}");
  String j = "{\"present\":true,";
  j += "\"voltage\":" + String(battBusV, 3) + ",";
  j += "\"current_ma\":" + String(battCurrentMa, 0) + ",";
  j += "\"power_mw\":" + String(battPowerMw) + ",";
  j += "\"compensated_voltage\":" + String(battOcvFiltered, 3) + ",";
  j += "\"cells\":" + String(BATT_CELLS) + ",";
  j += "\"percent\":" + String(battPercent) + ",";
  j += "\"low\":" + String(battLow ? "true" : "false") + ",";
  j += "\"valid\":" + String((battHaveFilter && !battStale) ? "true" : "false") + ",";
  j += "\"internal_mohm\":" + String(battInternalMohm);
  j += "}";
  return j;
}
