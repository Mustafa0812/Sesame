"""MQTT front door for the ground station. Enabled when the MQTT_HOST environment variable is set.

Environment: MQTT_HOST, MQTT_PORT (1883), MQTT_USER, MQTT_PASS, MQTT_PREFIX (sesame),
             MQTT_PUBLISH_IMU=1 to publish this Pi's own IMU tilt.

Topics (P = prefix):
  in   P/tilt           {"pitch": <deg>, "roll": <deg>}  pitch + = lean forward, roll + = lean right,
                        relative to level. Publish at 10-30 Hz. While fresh (<0.5 s) it drives body tilt.
  in   P/body/enable    {"enabled": true, "target": "A1B2"}      start/stop body-tilt mode
  in   P/cmd            {"target": "A1B2", "text": "seq wave"}   same whitelist as the web UI
  out  P/state          JSON, once a second: gateway, bots (telemetry), body-tilt status
  out  P/imu            {"pitch":..,"roll":..} 10 Hz from the Pi's own IMU (MQTT_PUBLISH_IMU=1)
"""
import json
import os
import threading
import time

import paho.mqtt.client as mqtt


class MqttBridge:
    def __init__(self, gw, body, guide, imu, sequences):
        self.gw, self.body, self.guide, self.imu, self.sequences = gw, body, guide, imu, sequences
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
        self.c.on_message = self._on_message
        self.connected = False

    def start(self):
        self.c.connect_async(os.environ["MQTT_HOST"], int(os.environ.get("MQTT_PORT", "1883")), 30)
        self.c.loop_start()
        threading.Thread(target=self._publisher, daemon=True).start()

    def _on_connect(self, client, userdata, flags, rc, *extra):
        self.connected = True
        for t in ("tilt", "body/enable", "cmd"):
            client.subscribe(f"{self.p}/{t}")

    def _on_message(self, client, userdata, msg):
        try:
            d = json.loads(msg.payload)
            topic = msg.topic[len(self.p) + 1:]
            if topic == "tilt":
                self.body.set_external(d["pitch"], d["roll"])
            elif topic == "body/enable":
                target = str(d.get("target", ""))
                if d.get("enabled") and not (target.isalnum() and self.body.source_ok()):
                    return
                self.body.set_enabled(bool(d.get("enabled")), target)
            elif topic == "cmd":
                target, text = str(d.get("target", "*")), str(d.get("text", "")).strip()
                ok = text in ("leader", "follower", "stop", "status") or (
                    text.startswith("seq ") and text[4:] in self.sequences)
                if ok and (target == "*" or target.isalnum()):
                    self.gw.command(target, text)
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
