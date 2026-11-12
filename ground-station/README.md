# Sesame ground station

```
imu_publisher.py   --\
                      >-- Mosquitto (MQTT) --> app.py (controller + dashboard) --> ESP-NOW gateway --> bots
bmp388_publisher.py -/                              \--> InfluxDB (optional)
```

## Run on the Pi

```
cd ground-station && python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt adafruit-circuitpython-bmp3xx

# terminal 1: controller + dashboard (http://localhost:8000)
LOCAL_IMU=0 MQTT_HOST=localhost python3 app.py
# terminal 2: IMU tilt publisher (hold the IMU level when it starts)
python3 imu_publisher.py --host localhost
# terminal 3: temperature + pressure publisher
python3 bmp388_publisher.py --host localhost
```

No hardware? `DEMO=1 python3 app.py` fakes two bots, an IMU and a BMP388.

## MQTT topics (prefix `sesame`)

| topic | direction | payload |
|---|---|---|
| `sesame/tilt` | in | `{"pitch": deg, "roll": deg}` (pitch + = lean forward, roll + = lean right) |
| `sesame/env/temperature` | in | plain number, deg C |
| `sesame/env/pressure` | in | plain number, hPa |
| `sesame/body/enable` | in | `{"enabled": true, "target": "A1B2"}` |
| `sesame/cmd` | in | `{"target": "A1B2", "text": "seq wave"}` |
| `sesame/state` | out | gateway, bots and body-tilt state, once a second |

Optional InfluxDB 1.x logging (database `SRH_ES_iot`): set `INFLUX_HOST` and `pip install influxdb`.

## Dashboard

- **Robots**: one 3D panel per bot with a LEADER / FOLLOWER tag. Sequences you start (or that a bot reports in
  its telemetry) are replayed in the model using the firmware's own timings; followers mirror the leader.
  The body shell is the real STL from `hardware/printing/stl`; the legs are modelled from the part dimensions.
  If the model faces the wrong way, change **Model front**.
- **Body tilt** / **Walk mode**: IMU-driven control. **IMU orientation**: 3D attitude + gravity-vector trail.
- **Temperature / Pressure**: last 15 minutes from the BMP388.

Regenerate the simulator timelines after editing `firmware/sesame-firmware-main/movement-sequences.h`:
`python3 tools/gen_sequences.py`.

## Known limits

- The limb STLs in `hardware/printing/stl` are print-layout parts (mirrored pairs overlap), so the legs are
  stylised rather than the real meshes. Which end is the front, and the hip swing direction, were inferred from the
  firmware (`bow` pose, gait pairing) and should be checked against the real bot.
- No login: anyone on the network can send commands. Keep it on a trusted network.
