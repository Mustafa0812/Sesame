"""Standalone IMU tilt publisher: reads the MPU6500 and publishes tilt angles over MQTT.

Run it on whatever device has the IMU (a Pi, or any Linux board with I2C). The ground station
(app.py with MQTT_HOST set) subscribes to the topic and drives the bot over ESP-NOW.

    python3 imu_publisher.py --host <broker> [--rate 25] [--invert-pitch] [--invert-roll] [--swap]

Publishes  <prefix>/tilt  {"pitch": deg, "roll": deg}    pitch + = lean forward, roll + = lean right
Listens    <prefix>/imu/zero  (any payload)              re-levels: the current pose becomes 0, 0
The IMU is levelled automatically at startup, so hold it in the neutral pose when you start the script.
"""
import argparse
import json
import os
import time

import paho.mqtt.client as mqtt

from imu import Imu


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=os.environ.get("MQTT_HOST", "localhost"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("MQTT_PORT", "1883")))
    ap.add_argument("--user", default=os.environ.get("MQTT_USER"))
    ap.add_argument("--password", default=os.environ.get("MQTT_PASS", ""))
    ap.add_argument("--prefix", default=os.environ.get("MQTT_PREFIX", "sesame"))
    ap.add_argument("--rate", type=float, default=25.0, help="messages per second")
    ap.add_argument("--invert-pitch", action="store_true")
    ap.add_argument("--invert-roll", action="store_true")
    ap.add_argument("--swap", action="store_true", help="swap the pitch and roll axes")
    a = ap.parse_args()

    imu = Imu()
    imu.start()
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:   # paho 1.x
        c = mqtt.Client()
    if a.user:
        c.username_pw_set(a.user, a.password)

    off = {"pitch": 0.0, "roll": 0.0, "rezero": True}
    topic_tilt, topic_zero = f"{a.prefix}/tilt", f"{a.prefix}/imu/zero"

    def on_connect(client, userdata, flags, rc, *extra):
        client.subscribe(topic_zero)

    def on_message(client, userdata, msg):
        off["rezero"] = True

    c.on_connect, c.on_message = on_connect, on_message
    c.connect_async(a.host, a.port, 30)
    c.loop_start()

    period = 1.0 / a.rate
    print(f"Publishing tilt to {a.host}:{a.port} topic {topic_tilt} at {a.rate:g} Hz")
    last_err = ""
    while True:
        time.sleep(period)
        if not imu.fresh():
            if imu.error != last_err:
                print("IMU not reading:", imu.error)
                last_err = imu.error
            continue
        last_err = ""
        if off["rezero"]:
            off["pitch"], off["roll"], off["rezero"] = imu.pitch, imu.roll, False
            print(f"Level set: pitch {off['pitch']:.1f}, roll {off['roll']:.1f}")
        p, r = imu.pitch - off["pitch"], imu.roll - off["roll"]
        if a.swap:
            p, r = r, p
        if a.invert_pitch:
            p = -p
        if a.invert_roll:
            r = -r
        c.publish(topic_tilt, json.dumps({"pitch": round(p, 1), "roll": round(r, 1)}))


if __name__ == "__main__":
    main()
