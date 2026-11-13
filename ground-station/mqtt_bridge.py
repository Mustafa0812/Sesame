"""MQTT subscriber of the ground station (same pattern as the lab: publishers -> Mosquitto -> controller).

Environment: MQTT_HOST (default localhost), MQTT_PORT (1883), MQTT_USER, MQTT_PASS, MQTT_PREFIX (sesame).

  in   <prefix>/tilt               {"pitch": deg, "roll": deg}   from imu_publisher.py (25 Hz)
  in   <prefix>/env/temperature    plain number, deg C           from bmp388_publisher.py
  in   <prefix>/env/pressure       plain number, hPa             from bmp388_publisher.py
  out  <prefix>/state              JSON once a second: bots, leader, body-tilt status
  out  <prefix>/imu/zero           "1" when you press Set level (imu_publisher.py re-levels)
"""
import json
import math
import os
import threading
import time

import paho.mqtt.client as mqtt


class MqttBridge:
    def __init__(self, on_tilt, on_env, state_fn):
        self.on_tilt, self.on_env, self.state_fn = on_tilt, on_env, state_fn
        self.p = os.environ.get("MQTT_PREFIX", "sesame").strip("/")
        self.host = os.environ.get("MQTT_HOST", "localhost")
        try:
            self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:   # paho-mqtt 1.x
            self.c = mqtt.Client()
        if os.environ.get("MQTT_USER"):
            self.c.username_pw_set(os.environ["MQTT_USER"], os.environ.get("MQTT_PASS", ""))
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = lambda *a: setattr(self, "connected", False)
        self.c.on_message = self._on_message
        self.connected = False

    def start(self):
        self.c.reconnect_delay_set(min_delay=1, max_delay=5)   # back within seconds after a broker restart
        self.c.connect_async(self.host, int(os.environ.get("MQTT_PORT", "1883")), 30)
        self.c.loop_start()
        threading.Thread(target=self._publish_state, daemon=True).start()

    def _on_connect(self, client, *args):
        self.connected = True
        for t in ("tilt", "env/temperature", "env/pressure"):
            client.subscribe(f"{self.p}/{t}")

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic[len(self.p) + 1:]
            if topic == "tilt":
                d = json.loads(msg.payload)
                p, r = float(d["pitch"]), float(d["roll"])
                if math.isfinite(p) and math.isfinite(r):      # NaN would break the dashboard's JSON
                    self.on_tilt(p, r)
            elif topic.startswith("env/"):
                v = float(msg.payload)
                if math.isfinite(v):
                    self.on_env(topic[4:], v)
        except (ValueError, KeyError, TypeError):
            pass   # malformed message: ignore

    def publish(self, topic, payload):
        """Publish under the prefix, e.g. publish('imu/zero', '1') makes imu_publisher.py re-level."""
        if not self.connected:
            return False
        self.c.publish(f"{self.p}/{topic}", payload)
        return True

    def _publish_state(self):
        while True:
            time.sleep(1)
            if self.connected:
                try:
                    self.c.publish(f"{self.p}/state", json.dumps(self.state_fn()))
                except Exception:
                    pass
