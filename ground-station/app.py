"""Sesame ground station.

    python3 app.py [/dev/ttyUSB0]          then open http://localhost:8000   (or http://<pi-ip>:8000)

Architecture (same pattern as the lab):
    imu_publisher.py  --\
                         >-- MQTT broker (Mosquitto) --> this controller --> ESP-NOW gateway --> bots
    bmp388_publisher.py -/                                      |
                                                  dashboard (SSE)  +  optional InfluxDB

Environment: MQTT_HOST/PORT/USER/PASS/PREFIX (see mqtt_bridge.py), INFLUX_HOST (see influx_writer.py),
             LOCAL_IMU=0 when the IMU publishes over MQTT, DEMO=1 for fake bots and sensors.
"""
import json
import os
import sys
import threading
import time

from flask import Flask, Response, jsonify, request, send_from_directory

from bodytilt import BodyTilt
from gateway import Gateway
from guidance import Guidance
from imu import Imu
from store import Store

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")
STL_DIR = os.path.join(HERE, "..", "hardware", "printing", "stl")
MODELS = {  # name used by the dashboard -> STL file (assembly-consistent parts only)
    "frame": "Internal-Frame-v121.stl",
    "bottom": "Bottom-Cover-v121.stl",
    "top": "top-covers/Top-Cover-Enclosed-v117.stl",
}
SEQUENCES = ["rest", "stand", "wave", "dance", "swim", "point", "pushup", "bow", "cute", "freaky",
             "worm", "shake", "shrug", "dead", "crab", "forward", "backward", "left", "right"]
DEMO = os.environ.get("DEMO") == "1"

store = Store()
gw = Gateway(sys.argv[1] if len(sys.argv) > 1 else None)
imu = Imu()


def do_cmd(target, text):
    """Send a whitelisted command to a bot; sequence starts are also announced to the dashboard simulator."""
    text = str(text).strip()
    target = str(target)
    ok = text in ("leader", "follower", "stop", "status") or (text.startswith("seq ") and text[4:] in SEQUENCES)
    if not ok or not (target == "*" or target.isalnum()):
        return False
    if text.startswith("seq "):
        store.add_event(target, text[4:])
    elif text == "stop":
        store.add_event(target, "stop")
    return gw.command(target, text)


class Link:
    """What the walking-guidance mode sees: the gateway, but sequence starts are announced to the simulator."""
    def __getattr__(self, name):
        return getattr(gw, name)

    def command(self, target, text):
        if text.startswith("seq "):
            store.add_event(target, text[4:])
        elif text == "stop":
            store.add_event(target, "stop")
        return gw.command(target, text)


guide = Guidance(imu, Link())
body = BodyTilt(guide, gw)


def current_tilt():
    if body.ext_fresh():
        return {"pitch": body.ext_pitch, "roll": body.ext_roll, "src": "mqtt"}
    if imu.fresh():
        return {"pitch": guide.pitch, "roll": guide.roll, "src": "local"}
    return None


# ---- start the data sources
if DEMO:
    from demo import Demo
    Demo(gw, body, store).start()
else:
    gw.start()
    if os.environ.get("LOCAL_IMU", "1") != "0":
        imu.start()
    from mqtt_bridge import MqttBridge
    bridge = MqttBridge(gw, body, guide, imu, store, do_cmd)
    bridge.start()
if os.environ.get("INFLUX_HOST"):
    import influx_writer
    influx_writer.start(store, current_tilt, gw)


def watch_runs():
    """If a bot starts a sequence without us sending it (serial, another controller), still simulate it."""
    while True:
        time.sleep(0.25)
        for b in gw.snapshot()["bots"]:
            run = (b.get("tlm") or {}).get("run", "-")
            prev = store.last_run.get(b["mac"], "-")
            store.last_run[b["mac"]] = run
            if run != "-" and run != prev and b.get("id"):
                recent = [e for e in store.events_since(0) if e["name"] == run and time.time() - e["t"] < 4]
                if not recent:
                    store.add_event(b["id"], run, offset_ms=1000, source="telemetry")


threading.Thread(target=watch_runs, daemon=True).start()

app = Flask(__name__, static_folder=STATIC, static_url_path="/static")


def bots_payload():
    out = []
    for b in gw.snapshot()["bots"]:
        t = b.get("tlm") or {}
        if not b.get("id"):
            continue
        out.append({"id": b["id"], "mac": b["mac"], "online": b["online"], "age": b["age"], "rssi": b["rssi"],
                    "role": t.get("role", "?"), "run": t.get("run", "-"), "volts": t.get("v"),
                    "ma": t.get("ma"), "pct": t.get("pct")})
    return sorted(out, key=lambda x: x["id"])


def frame(last_id):
    snap = gw.snapshot()
    return {
        "t": time.time(),
        "tilt": current_tilt(),
        "pose": list(body.last_pose) if body.enabled else None,
        "body": {"enabled": body.enabled, "target": body.target, "status": body.status,
                 "gain": body.gain, "max_drop": body.max_drop},
        "walk": {"enabled": guide.enabled, "target": guide.target, "status": guide.status},
        "bots": bots_payload(),
        "env": {"temperature": store.metric("temperature"), "pressure": store.metric("pressure")},
        "gateway": {"connected": snap["connected"], "ready": snap["gateway"], "stats": snap["stats"]},
        "events": store.events_since(last_id),
    }


@app.get("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.get("/model/<name>.stl")
def model(name):
    if name not in MODELS:
        return "not found", 404
    return send_from_directory(STL_DIR, MODELS[name])


@app.get("/api/config")
def config():
    return jsonify(sequences=SEQUENCES, demo=DEMO)


@app.get("/api/history")
def history():
    return jsonify(temperature=store.history("temperature"), pressure=store.history("pressure"))


@app.get("/api/stream")
def stream():
    def gen():
        last_id = store.latest_event_id()
        while True:
            f = frame(last_id)
            if f["events"]:
                last_id = f["events"][-1]["id"]
            yield "data: " + json.dumps(f) + "\n\n"
            time.sleep(0.05)
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.post("/api/cmd")
def cmd():
    d = request.get_json(force=True)
    if do_cmd(d.get("target", "*"), d.get("text", "")):
        return jsonify(ok=True)
    return jsonify(ok=False, error="command refused or gateway not connected"), 400


# ---- walking mode (tilt the IMU, the bot walks that way)
@app.post("/api/imu/zero")
def imu_zero():
    guide.zero()
    return jsonify(ok=True)


@app.post("/api/imu/config")
def imu_config():
    d = request.get_json(force=True)
    guide.dead = min(40.0, max(5.0, float(d.get("dead", guide.dead))))
    guide.invert_pitch = bool(d.get("invert_pitch", guide.invert_pitch))
    guide.invert_roll = bool(d.get("invert_roll", guide.invert_roll))
    guide.swap = bool(d.get("swap", guide.swap))
    return jsonify(ok=True)


@app.post("/api/imu/enable")
def imu_enable():
    d = request.get_json(force=True)
    target = str(d.get("target", ""))
    if d.get("enabled") and not (target.isalnum() and imu.fresh()):
        return jsonify(ok=False, error="pick a bot and make sure the local IMU is reading"), 400
    if d.get("enabled") and body.enabled:
        body.set_enabled(False)
    return jsonify(ok=guide.set_enabled(bool(d.get("enabled")), target))


# ---- body-tilt mode (the bot leans the way the IMU is tilted)
@app.post("/api/body/config")
def body_config():
    d = request.get_json(force=True)
    body.gain = min(4.0, max(0.5, float(d.get("gain", body.gain))))
    body.max_drop = min(80.0, max(10.0, float(d.get("max_drop", body.max_drop))))
    return jsonify(ok=True)


@app.post("/api/body/enable")
def body_enable():
    d = request.get_json(force=True)
    target = str(d.get("target", ""))
    if d.get("enabled") and not (target.isalnum() and body.source_ok()):
        return jsonify(ok=False, error="pick a bot and make sure tilt is arriving (IMU publisher running)"), 400
    return jsonify(ok=body.set_enabled(bool(d.get("enabled")), target))


def pick_host():
    """Linux: listen on IPv4 and IPv6 (dual stack) so http://localhost works whichever one the browser tries
    first. Windows sockets are IPv6-only on '::', so use plain IPv4 there."""
    import socket
    if os.name != "nt" and socket.has_ipv6:
        try:
            s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
            s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
            s.bind(("::", 0))
            s.close()
            return "::"
        except OSError:
            pass
    return "0.0.0.0"


def port_in_use(port):
    """True if something already accepts connections on this port (checked by connecting, which also works
    on Windows where a second bind to a busy port can silently succeed)."""
    import socket
    for addr in ("127.0.0.1", "::1"):
        try:
            socket.create_connection((addr, port), timeout=0.5).close()
            return True
        except OSError:
            continue
    return False


if __name__ == "__main__":
    host, port = pick_host(), int(os.environ.get("PORT", "8000"))
    if port_in_use(port):
        sys.exit("\nCannot start: port %d is already in use.\n"
                 "Another app.py is probably still running. Stop it (Ctrl+C in its terminal, or "
                 "'pkill -f app.py') or use another port: PORT=8001 python3 app.py\n" % port)
    print("\nSesame ground station: open http://127.0.0.1:%d  (or http://<this-device-ip>:%d)\n" % (port, port))
    app.run(host=host, port=port, threaded=True)
