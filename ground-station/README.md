# Sesame ground station

```
imu_publisher.py    --\
                       >-- Mosquitto (MQTT) --> app.py --> USB --> ESP-NOW gateway --> Bot A, Bot B
bmp388_publisher.py --/                           \--> dashboard  http://127.0.0.1:8000
```

## What runs where

| Device | Runs |
|---|---|
| Both bots (ESP32-S3 + PCA9685) | `firmware/espnow-sequence-tester-pca` |
| Gateway (ESP32 WROOM-32, USB to the Pi) | `firmware/espnow-gateway` |
| Raspberry Pi | Mosquitto, `app.py`, `imu_publisher.py`, `bmp388_publisher.py` |

## First-time setup on the Pi

```
sudo apt install mosquitto mosquitto-clients i2c-tools python3-venv
sudo raspi-config        # Interfaces -> SPI (MPU6500) and I2C (BMP388) -> enable, then reboot
sudo usermod -aG dialout $USER   # serial access, log out and back in
cd ground-station && python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

## Every time (three terminals, each: `cd ground-station && source .venv/bin/activate`)

```
python3 app.py                          # controller + dashboard -> open http://127.0.0.1:8000
python3 imu_publisher.py --host localhost       # hold the IMU level when it starts
python3 bmp388_publisher.py --host localhost
```

Keep **one** dashboard tab open.

## Using the dashboard

1. **Robots**: both bots appear with battery, signal and their role tag. Click **Leader → Bot A** (or B).
   The other bot becomes the follower automatically, and the choice sticks: if a bot reboots or reports the
   wrong role, the ground station sets it again.
2. **Sequences**: a button sends the sequence to the leader. The leader announces it over the radio and the
   follower plays the same sequence from its own memory, in step, so lost radio packets can't make it skip steps.
   The line under the buttons says whether the leader received the command (it is re-sent up to 3 times until
   the bot answers), or why it was refused. Buttons are locked while any bot is running a sequence.
   **STOP** interrupts walks and turns on both bots; poses finish on their own.
3. **Body tilt**: switch on and both bots lean the way the IMU is tilted (the Pi leads, both bots follow).
   Switch off and they return to stand and your chosen leader is restored. **Set level** (IMU card) makes the
   IMU's current position the new level.
4. **Activity**: every command, reply, refusal and reboot. A bot that reboots with reason `brownout` during
   sequences has a supply problem (battery low, or the servos pull the voltage down).

## MQTT topics (prefix `sesame`)

| Topic | From -> to | Payload |
|---|---|---|
| `sesame/tilt` | imu_publisher -> app | `{"pitch": deg, "roll": deg}` (25 Hz) |
| `sesame/env/temperature` | bmp388_publisher -> app | number, deg C |
| `sesame/env/pressure` | bmp388_publisher -> app | number, hPa |
| `sesame/imu/zero` | app -> imu_publisher | `1` (Set level) |
| `sesame/state` | app -> anyone | bots, leader, body tilt (1 Hz) |

## Troubleshooting

- `Cannot start: port 8000 is already in use`: another `app.py` is running (`pkill -f app.py`).
- Gateway chip red: check `ls /dev/ttyUSB*` and the `dialout` group. Pass the port if needed: `python3 app.py /dev/ttyUSB0`.
- IMU not reading: the MPU6500 is on SPI0 with NCS on CE0 (GPIO8, pin 24). Check SPI is enabled
  (`ls /dev/spidev0.0`) and the wiring in `imu.py`. `WHO_AM_I 0x00` or `0xFF` means no answer (wiring);
  an MPU6050 / GY-521 has no SPI and cannot be used this way.
- BMP388 chip red / read errors: it is on I2C now. `sudo i2cdetect -y 1` must show `77` (or `76` if SDO is
  grounded: pass `--addr 0x76`). Nothing there: check SDA/SCL and that the board's CS pin is not pulled low.
- A bot never appears: it must run `espnow-sequence-tester-pca` (only that sketch sends telemetry) on channel 1.
- "no reply from …": the bot is off, out of range, or the radio is overloaded (very close range at high TX power).
- **Bots skip moves (both bots, mid-routine).** First check the power, not the code. Two bots on ONE bench supply
  start every move at the same instant in lockstep, their servo current peaks add up, the supply hits its current
  limit and the voltage sags: servos slow down or miss moves. Fix: one supply per bot (battery or bench), at least
  ~5 A per bot, short thick leads to the PCA9685's V+ and a 1000 µF capacitor across V+/GND.
  To rule out the software: after a bad run, plug USB in without powering off and type `status`. The line
  `PCA9685: I2C write errors 0, lost servo moves 0, resets repaired 0 | reset reason poweron` means every move
  reached the servo driver and the bot did not reboot, so the problem is after the driver (supply or servos).
  Non-zero counts point to I²C wiring or supply noise; a reset reason of `brownout` means the ESP32's own supply dipped.
- **It works on USB but not wireless.** USB also powers the ESP32 from the PC, which takes its current off the
  servo supply. If the same routine is perfect with USB plugged in (commanded from the dashboard), the cause is
  power, not the radio.
- Checking the radio link: plug the follower into USB (Serial Monitor, 115200). While the leader is idle the
  follower prints `pose rx 20/2s | max gap ... ms` every 2 s; 20 of 20 means no loss. During a sequence it prints
  `Playing '<name>' together with the leader`. Heavy loss with the bots close together (under ~50 cm): type
  `power 8` on both bots. The default 14 dBm suits 1-3 m.
