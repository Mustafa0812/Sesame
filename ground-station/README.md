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
sudo raspi-config        # Interfaces -> I2C -> enable, then reboot
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
2. **Sequences**: buttons run on the leader; the follower copies it. The line under the buttons says whether
   the bot received the command (it is re-sent up to 3 times until the bot answers), or why it was refused.
   Buttons are locked while a sequence runs. **STOP** interrupts walks and turns; poses finish on their own.
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
- A bot never appears: it must run `espnow-sequence-tester-pca` (only that sketch sends telemetry) on channel 1.
- "no reply from …": the bot is off, out of range, or the radio is overloaded (very close range at high TX power).
