# Archived sketches

Earlier bench testers, kept for reference. None of them is part of the working system. For the two bots, the
gateway and the Pi ground station, see the table at the top of [../README.md](../README.md).

| Sketch | What it was | Why it is archived |
|---|---|---|
| `espnow-sequence-tester` | Leader/follower sequence tester with servos on ESP32 LEDC pins | Superseded by `espnow-sequence-tester-pca` (the bots use a PCA9685). It has no ground-station commands or telemetry, so the dashboard cannot see or drive it. |
| `espnow-servo-tester` | First 4-servo ESP-NOW leader/follower test (soft-AP, ESP32Servo) | Superseded by the sequence tester |
| `espnow-tester-board1`, `espnow-tester-board2` | Per-board copies of the 4-servo ESP-NOW test | Superseded. board2 uses 8 dBm TX power, which lost ~97 % of packets at close range. |
| `espnow-radio-diag` | Raw ESP-NOW link diagnostic | Debug tool. It defaults to channel 11; type `chan 1` to match the system. |
| `pca9685-servo-tester` | PCA9685 bench tester | The bot sketch's `raw`, `jog`, `us`, `off` and `range` commands cover it. |
| `imu-servo-tester`, `imu-tester` | MPU6050 + servo bench sketches on the bot | The tilt IMU is now on the Pi (`ground-station/imu_publisher.py`). imu-servo-tester also puts the INA260 at 0x40, which collides with the PCA9685. |
| `i2c-pin-finder` | Finds which GPIOs an I2C device is on | One-off utility |
| `single-servo-tester` | Sweeps one servo on GPIO 12 | Obsolete |
| `servo-speed-tester` | Arduino Uno servo timing sketch | Not for the ESP32 bots |
| `face-tester` | OLED face preview for an ESP8266 D1 Mini | Not for the ESP32 bots. Its face-bitmaps.h is a copy of the one in sesame-firmware-main. |

To use one again, open its `.ino` from this folder in the Arduino IDE. Each sketch folder is self-contained.
