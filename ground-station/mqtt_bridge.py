"""MQTT side of the ground station, following the lab pattern:
    publishers (IMU, BMP388) -> Mosquitto -> this controller/subscriber -> ESP-NOW gateway + dashboard.

Environment: MQTT_HOST (default localhost), MQTT_PORT (1883), MQTT_USER, MQTT_PASS, MQTT_PREFIX (sesame),
             MQTT_PUBLISH_IMU=1 to publish this Pi's own IMU tilt.

Topics (P = prefix):
  in   P/tilt                 {"pitch": <deg>, "roll": <deg>}  pitch + = lean forward, roll + = lean right
  in   P/env/temperature      plain number, degrees C   (bmp388_publisher.py)
  in   P/env/pressure         plain number, hPa         (bmp388_publisher.py)
  in   P/body/enable          {"enabled": true, "target": "A1B2"}
  in   P/cmd                  {"target": "A1B2", "text": "seq wave"}   same whitelist as the dashboard
  out  P/state                JSON once a second: gateway, bots (telemetry), body-tilt status
  out  P/imu                  {"pitch":..,"roll":..} 10 Hz from the Pi's own IMU (MQTT_PUBLISH_IMU=1)
"""
import json
import os
import threading
import time

import paho.mqtt.client as mqtt


class MqttBridge:
    def __init__(self, gw, body, guide, imu, store, cmd_fn):
        self.gw, self.body, self.guide, self.imu, self.store, self.cmd_fn = gw, body, guide, imu, store, cmd_fn
        self.p = os.environ.get("MQTT_PREFIX", "sesame").strip("/")
        self.publish_imu = os.environ.get("MQTT_PUBLISH_IMU") == "1"
        try:
            self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:      # paho 1.x
            self.c = mqtt.Client()
        user = os.environ.get("MQTT_USER")
        if user:
            self.c.username_pw_set(user, os.environ.get("MQTT_PASS", ""))
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = self._on_disconnect
        self.c.on_message = self._on_message
        self.connected = False

    def start(self):
        self.c.connect_async(os.environ.get("MQTT_HOST", "localhost"), int(os.environ.get("MQTT_PORT", "1883")), 30)
        self.c.loop_start()
        threading.Thread(target=self._publisher, daemon=True).start()

    def _on_connect(self, client, userdata, flags, rc, *extra):
        self.connected = True
        for t in ("tilt", "env/temperature", "env/pressure", "body/enable", "cmd"):
            client.subscribe(f"{self.p}/{t}")

    def _on_disconnect(self, *args):
        self.connected = False

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic[len(self.p) + 1:]
            if topic in ("env/temperature", "env/pressure"):
                self.store.add_metric(topic.split("/")[1], float(msg.payload))
                return
            d = json.loads(msg.payload)
            if topic == "tilt":
                self.body.set_external(d["pitch"], d["roll"])
            elif topic == "body/enable":
                target = str(d.get("target", ""))
                if d.get("enabled") and not (target.isalnum() and self.body.source_ok()):
                    return
                self.body.set_enabled(bool(d.get("enabled")), target)
            elif topic == "cmd":
                self.cmd_fn(str(d.get("target", "*")), str(d.get("text", "")).strip())
        except (ValueError, KeyError, TypeError):
            pass   # malformed message: ignore

    def _publisher(self):
        n = 0
        while True:
            time.sleep(0.1)
            if not self.connected:
                continue
            n += 1
            if self.publish_imu and self.imu.fresh():
                self.c.publish(f"{self.p}/imu", json.dumps(
                    {"pitch": round(self.guide.pitch, 1), "roll": round(self.guide.roll, 1)}))
            if n % 10 == 0:
                snap = self.gw.snapshot()
                snap["body"] = {"enabled": self.body.enabled, "target": self.body.target,
                                "status": self.body.status}
                self.c.publish(f"{self.p}/state", json.dumps(snap))
