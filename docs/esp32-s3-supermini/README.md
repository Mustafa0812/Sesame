# ESP32-S3 SuperMini: Sesame Port Notes

> [!NOTE]
> These are working notes kept while porting Sesame to the ESP32-S3 SuperMini. Sections are dated and
> some describe earlier stages. **Current state:** the bots run
> [`firmware/espnow-sequence-tester-pca`](../../firmware/espnow-sequence-tester-pca/) and the gateway runs
> [`firmware/espnow-gateway`](../../firmware/espnow-gateway/) (see [`firmware/README.md`](../../firmware/README.md)).
> The bench testers named below were moved to [`firmware/archive/`](../../firmware/archive/README.md) on 2026-11-12.
>
> The vendor's own pinout sheet is kept in [vendor-pinout.md](vendor-pinout.md) for reference, but parts of it are wrong (section 2).

Living notes for running the Sesame firmware on the ESP32-S3 SuperMini board
(Ulegqin, pack of 3). Updated as we learn more.

**Status (2026-11-07):** pin changes applied; IMU and ESP-NOW support added
(see section 6). Not yet compiled or tested on hardware.

## 1. Hardware

| Item | Value | Source |
|---|---|---|
| Board | ESP32-S3 SuperMini ("ESP32 MINI") | Amazon listing |
| Chip | ESP32-S3 (QFN56) rev v0.2, embedded flash + PSRAM (= ESP32-S3FH4R2) | **esptool, 2026-11-07** (pinout image wrongly says S3FN8) |
| Flash / PSRAM | **4 MB (XMC) / 2 MB quad PSRAM (AP_3v3)** | **esptool** (pinout image wrongly says 8 MB / none) |
| USB mode | USB-Serial/JTAG | esptool |
| MAC (bot 1) | ac:27:6e:cc:b9:20 (so AP name `Sesame-B920`) | esptool |
| USB | Type-C, native USB (power, programming, debug) | Pinout image |
| Wireless | 2.4 GHz Wi-Fi 802.11 b/g/n, Bluetooth 5.0 LE | Pinout image |
| Supply | 5 V (USB) or 5 V pin; 3V3 pin outputs 3.3 V, max 500 mA | Pinout image |
| Size | 18 x 28 mm | Pinout image |

**Arduino IDE settings for this board:** ESP32S3 Dev Module, Flash Size **4MB**,
Partition "Default 4MB with spiffs", PSRAM Disabled or QSPI (**never OPI**),
USB CDC On Boot Enabled, Upload Speed 115200 if uploads are unreliable.
If an upload fails, enter download mode by hand (section 2).

**The vendor pinout image is unreliable.** It got the chip, flash, PSRAM and
header pins wrong. Trust esptool and the numbers printed on the board.

## 2. Pinout (corrected 2026-11-07, from the ESPboards diagram)

The vendor image's pinout was wrong: it showed 12 pins per side, and GPIO0,
GPIO18 and GPIO45 on the header. Symptom: with SCL on "GPIO18" the I2C scan
found nothing, because the header position the vendor called GPIO18 is
really GPIO8.

### Header: left side (top to bottom, USB-C at the top)

| GPIO | Function | Use in Sesame |
|---|---|---|
| 43 | TX (UART0) | Keep free |
| 44 | RX (UART0) | Keep free |
| 1 | GPIO, ADC1_CH0 | Servo 2 |
| 2 | GPIO, ADC1_CH1 | Servo 5 |
| 3 | GPIO, ADC1_CH2 | Spare, but strapping pin (JTAG select). Avoid |
| 4 | GPIO, ADC1_CH3 | Servo 0 |
| 5 | GPIO, ADC1_CH4 | Servo 1 |
| 6 | GPIO, ADC1_CH5 | **I2C SDA** |
| 7 | GPIO, ADC1_CH6 | Servo 3 |

### Header: right side (top to bottom)

| GPIO | Function | Use in Sesame |
|---|---|---|
| 5V | 5 V in/out | |
| GND | Ground | |
| 3V3 | 3.3 V out, max 500 mA | OLED + IMU power |
| 13 | GPIO | Servo 7 |
| 12 | GPIO | Servo 6 |
| 11 | GPIO | **I2C SCL** |
| 10 | GPIO, ADC1_CH9 | Servo 4 |
| 9 | GPIO, ADC1_CH8 | Spare |
| 8 | GPIO, ADC1_CH7 | Spare |

### Not on the header
- GPIO0: BOOT button.
- GPIO48: onboard WS2812 RGB LED.
- GPIO18, GPIO45 and others: underside pads only (per ESPboards).
- GPIO19/20: USB.

### Pins to avoid for servos

| GPIO | Reason |
|---|---|
| 3 | Strapping pin (boot behaviour) |
| 43, 44 | UART0 TX/RX, keep for serial |
| 6, 11 | Used by I2C |

### Download mode
Hold BOOT, press and release RESET, then release BOOT.

## 3. What the Sesame firmware assumes

File: `firmware/sesame-firmware-main/sesame-firmware-main.ino`

- 8 hobby servos driven **directly from GPIOs** using `ESP32Servo`
  (`servos[i].attach(servoPins[i], 732, 2929)`).
- **No PCA9685.** There is no `Adafruit_PWMServoDriver` anywhere in the repo.
- I2C is used **only for an SSD1306 128x64 OLED** at address `0x3C`
  (`Wire.begin(I2C_SDA, I2C_SCL)`).
- Currently set for the Lolin S2 Mini: `I2C_SDA 33`, `I2C_SCL 35`,
  `servoPins = {1, 2, 4, 6, 8, 10, 13, 14}`.
- Wi-Fi AP "Sesame-Controller", captive portal, web server, mDNS.

The other sketches differ:
- `debugging-firmware/sesame-motor-tester/sesame-motor-tester.ino` uses the same direct-GPIO servo pins.
- `archive/face-tester/face-tester.ino` targets a Wemos D1 Mini (ESP8266), not an ESP32.

### Do we need the PCA9685?
No. The ESP32-S3 has 8 LEDC PWM channels and any GPIO can use one, so 8 servos
can be driven directly. Use a PCA9685 only if you need more than 8 servos.

## 4. Pin change (APPLIED 2026-11-07)

Edits to `sesame-firmware-main.ino`:

| Setting | Current (S2 Mini) | Proposed (S3 SuperMini) |
|---|---|---|
| `I2C_SDA` | 33 | **6** |
| `I2C_SCL` | 35 | **11** |
| `servoPins[8]` | `{1, 2, 4, 6, 8, 10, 13, 14}` | **`{4, 5, 1, 7, 10, 2, 12, 13}`** (L1/L2 swapped after assembly, 2026-11-08: now **`{4, 5, 7, 1, 10, 2, 12, 13}`**) |

History: first proposed SDA 9 / SCL 18 (vendor pinout), then 9 / 8 (corrected
pinout), but the I2C scan found nothing. `i2c-pin-finder` showed the MPU6050
(0x68) is actually wired to **SDA = GPIO6, SCL = GPIO11**, so the code now
matches the wiring (option B) and the two servos that used 6 and 11 moved.

Rationale:
- Every proposed pin is broken out on this board.
- None is a strapping, USB, UART or LED pin.
- The servo set is the Distro Board V3 order, except servo 2 (6 -> 1) and
  servo 5 (11 -> 2), which gave up their pins to I2C. The other six servos
  keep their V3 pins.
- I2C SDA 6 / SCL 11 matches where the IMU is actually wired.
- The old S2 Mini lines would be commented out, not deleted, so it is easy to revert.
- GPIO14 (used by the S2 Mini map) does not exist on this board.

Not changing: the pulse range of 732-2929 us (see below).

## 5. Wiring and power notes

- Power the servos from an **external 5-6 V supply**, not the board's 5V pin.
  Eight servos can draw amps.
- **Share ground** between the servo supply and the ESP32.
- OLED and IMU: 3V3, GND, SDA = GPIO6, SCL = GPIO11.
- Signal wires: each servo signal wire goes to its GPIO from section 4.

## 6. Firmware changes for IMU + ESP-NOW (APPLIED 2026-11-07)

Sketch folder: `firmware/sesame-firmware-main/`

- **Headers moved** (`git mv`) from `firmware/` into the sketch folder, because
  Arduino only compiles files in the sketch's own folder: `face-bitmaps.h`,
  `movement-sequences.h`, `captive-portal.h`.
- **New `imu.h`:** MPU6050 / MPU6500 / MPU9250 driver using only `Wire`.
  - Detects the chip by WHO_AM_I (0x68 = MPU6050, 0x70 = 6500, 0x71 = 9250,
    0x73 = 9255); unknown IDs are tried anyway with a warning.
  - Tries address 0x68, then 0x69.
  - +-2 g, +-250 deg/s, 100 Hz, gyro bias calibrated at boot (keep the robot still).
  - Pitch/roll from a complementary filter. Axis directions depend on mounting.
  - Optional: the robot boots and runs without an IMU.
  - The MPU9250 magnetometer (AK8963) is not used.
- **New `espnow-comms.h`:** generic message layer, no behaviours yet.
  - Broadcast messages with a header (magic, version, type, seq, length).
  - Radio callback only queues; handling happens in `loop()`.
  - Heartbeat every 2 s carries the bot's name; peer table with RSSI.
  - Hook `espnowOnMessage` for future behaviours.
  - Supports ESP32 Arduino core 2.x and 3.x. RSSI only on 3.x (0 on 2.x).
  - Not encrypted.
- **Unique names per bot** from the last 2 MAC bytes: AP `Sesame-XXXX`,
  mDNS `sesame-xxxx.local`, ESP-NOW name = AP name. Wi-Fi password unchanged.
- **Soft AP pinned to channel 1** (`ESPNOW_CHANNEL`) so both bots match.
- IMU + ESP-NOW are serviced from `loop()` and during movement waits
  (`serviceBackground()`), so they keep running while the robot walks.

### New serial commands (115200 baud)
| Command | Does |
|---|---|
| `imu` | Print accel, gyro, pitch, roll, temperature |
| `imu cal` | Recalibrate gyro bias (keep robot still) |
| `en info` | ESP-NOW status, radio channel, counters, peers |
| `en send <text>` | Broadcast a text message to other bots |

The serial command buffer was raised from 32 to 128 characters for `en send`.

### Standalone IMU tester
`firmware/archive/imu-tester/imu-tester.ino`: MPU6050-only test sketch (SDA 6, SCL 11),
no servos or OLED. Commands: `s` I2C scan, `c` gyro calibrate, `p` pause,
`m` text/Serial Plotter output, `r` re-init, `h` help.

### IMU + servo tester
`firmware/archive/imu-servo-tester/imu-servo-tester.ino`: IMU and servos together
(SDA 6, SCL 11, servos `{4, 5, 7, 1, 10, 2, 12, 13}`, L1/L2 swapped after assembly). Servos start limp.
Commands: `<id> <angle>`, `all <angle>`, `sweep <id>`, `tilt <a> <b>` (servo a
follows roll, b follows pitch), `gain`, `flip`, `stop`, `imu cal`, `p`, `h`.
The status line shows IMU read errors, so I2C trouble while servos run
(brown-out, noise) shows up as a rising `err` count. Needs the ESP32Servo library.

Extended with an **SSD1306 OLED (0x3C)** and an **INA260 (0x40)**, all three on the
same I2C bus (SDA 6, SCL 11). Libraries: ESP32Servo, Adafruit SSD1306, Adafruit GFX
(IMU and INA260 are read over `Wire` directly). Each device is optional.
- INA260 goes **in series with the servo supply's positive wire**
  (supply + -> IN+, IN- -> servo V+ rail), all grounds common.
- Shows volts, mA, watts, peak mA and a current bar on the OLED and serial;
  flags `LOW V` under 4.5 V (a sagging servo supply). `peak` resets the peak.
- INA260 settings: 4-sample averaging, 1.1 ms conversions (about 9 ms per update).
  Limits: 36 V, 15 A.
- OLED refresh is 4 Hz because each refresh blocks the I2C bus about 25 ms.

### I2C pin finder (when the scan finds nothing)
`firmware/archive/i2c-pin-finder/i2c-pin-finder.ino`: finds which GPIOs a device is
really wired to, without trusting any pinout. Phase 1 reads each GPIO with
pull-down and pull-up (pins held HIGH are the MPU6050's SDA/SCL pull-ups, which
also proves the module has power). Phase 2 scans SDA/SCL pairs among those pins.
Disconnect servos before running. `r` reruns, `f` forces a scan of all pins.

### New HTTP endpoint
- `GET /api/imu` returns JSON with chip, accel, gyro, pitch, roll, temperature.

### Known limitations
- Joining a router (station mode or the web UI's Wi-Fi connect) moves the
  radio off channel 1 and breaks ESP-NOW with the other bot.
- If the OLED is missing the firmware still halts at boot (unchanged behaviour).
- ~~Relative links to the moved headers in `firmware/README.md` are now broken.~~ Fixed 2026-11-13.

### Battery level on the OLED (INA260), added 2026-11-08 (TEST ONLY, not in the main firmware)
`firmware/archive/espnow-sequence-tester/battery.h`, used by `espnow-sequence-tester.ino` (its own battery screen on the
OLED). It was first wired into `sesame-firmware-main` and then removed again; `battery.h` still has an
overlay drawing function (`batteryDrawOverlay`) for putting the icon on the face if it goes back in.
Not yet compiled or tested on hardware.
- **Wiring:** INA260 on the BATTERY side, before the buck converter: battery + -> IN+, IN- -> buck
  input +, grounds common. INA260 on the shared I2C bus (SDA 6, SCL 11, address 0x40). After the buck
  it would only see 5.1 V and could not show a battery level.
- **Pack assumed:** 2S Li-ion (2x 14500), 8.4 V full. Change `BATT_CELLS` and the `BATT_CURVE` table
  for other packs.
- **Method:** voltage + I * R_internal (load compensation, `BATT_DEFAULT_MOHM` = 200 is a guess),
  6 s low-pass filter, per-cell Li-ion lookup. Voltage-only estimate, no coulomb counting.
- **Display (tester):** big percent, volts, mA, peak mA, per-cell volts, internal resistance and a level
  bar ("LOW!" at or below 15%, "--" if the INA260 stops answering). Refreshed every 500 ms.
- **Serial:** `batt` prints volts/amps/percent/peak; `batt r <mohm>` sets the internal resistance
  (use `batt r 0` on a bench supply); `batt peak` resets the peak current.
- **Tune R:** (V_idle - V_load) / (I_load - I_idle) while a servo-heavy move runs.

## 6b. Servo calibration and bench status (2026-11-08)

**Sketch:** `firmware/archive/espnow-sequence-tester/espnow-sequence-tester.ino` (two 8-servo bots, leader streams poses to follower). Not yet compiled/verified after the latest edits.

**Calibration model.** Sequences use logical angles (90 = rest). Just before the servo, `calibratedAngle()` converts:
`physical = CAL_REST[i] + CAL_DIR[i] * (logical - 90)`, clamped 0..180. Same table on leader and follower; the stream carries logical angles.

| Servo | GPIO | CAL_REST | CAL_DIR | stand (physical) |
|---|---|---|---|---|
| R1 | 4 | 7 | +1 | 52 |
| R2 | 5 | 56 (horn remounted; was 11) | +1 | 11 |
| L1 | 7 | 87 | +1 | 45 |
| L2 | 1 | 0 | +1 | 45 |
| R4, R3, L3, L4 | 10, 2, 12, 13 | 90 (uncalibrated) | +1 | - |

- L1/L2 GPIOs were swapped after assembly: pin map is `{4, 5, 7, 1, 10, 2, 12, 13}` (main firmware, sequence tester, imu-servo-tester). Calibration values follow the servo, not the name: the servo that measured 87 is on GPIO 7 (L1), the one that measured 0 is on GPIO 1 (L2). Assumption, re-check with `move L1 90` / `move L2 90`.
- R2 originally rested at 11, leaving only 11 deg of downward travel (stand needs 45). Angle 56 crashed into the body, 160 did not move. Fixed by remounting the horn so straight = 56.
- Legs R4/R3/L3/L4 still need rest angles. Use `raw <servo> <angle>` (physical, uncalibrated, this board only) to find them, then fill `CAL_REST`.
- Hobby servos have no position feedback: code only knows the commanded angle. The `angles` command was tried and reverted.
- `move` takes logical angles, `raw` takes physical ones (`move r2 90` = `raw r2 56`).
- `sesame-firmware-main.ino` is NOT ported yet: still ESP32Servo, flat `servoSubtrim` only (cannot flip a joint), no TX power setting. Planned: LEDC driver, 14 dBm, calibration table.
- Optional, not done: per-servo min/max physical limits to protect against body collisions.

**Tester commands added:** `batt`, `batt r <mohm>`, `batt peak`, `raw <servo> <angle>`. OLED shows board ID, role, battery screen. Boot banner prints Board ID (last 2 MAC bytes) to tell the two boards apart; both serial windows once appeared to follow the same board (check MAC with `status`).

**Open issues:** wave pose on follower's L3 looked odd (not diagnosed; try `sweep L3`, check `pose rx`); periodic ping loss; one watchdog reset on board 2.

## 7. Parked ideas (two-bot behaviours)
- Fall detection using the IMU, alert sent to the other bot over ESP-NOW.
  Realistically the other bot can only relay the alert or trigger a
  self-righting attempt; it can't physically help.
- Follow the leader using ESP-NOW RSSI (rough distance only, no direction).

## 8. Open points / to verify

- [ ] The image has inconsistent labels (onboard LED shown as GPIO8 in one place,
      with "48" printed next to the LED on the board; GPIO45 labelled as an ADC pin).
      Trust the numbers printed on the physical board over the tables.
- [ ] I don't think the firmware uses the onboard LED (no `pinMode` calls found),
      but this has not been checked in full.
- [ ] The pulse range of 732-2929 us is wider than most servos accept (about
      500-2500 us). Test with `sesame-motor-tester.ino` first to avoid stalling the
      servos at the extremes.
- [ ] Confirm which physical servo (hip/knee, which leg) is wired to each index 0-7.
- [ ] Confirm whether an OLED is being used.
- [ ] Confirm the "S3FN8" chip against the marking on the module shield.
