r"""Sesame ground station: controller + dashboard.

    python3 app.py [/dev/ttyUSB0]          then open http://127.0.0.1:8000  (or http://<pi-ip>:8000)

    imu_publisher.py  ---\
                          >-- Mosquitto (MQTT) --> app.py --> USB serial --> ESP-NOW gateway --> bots
    bmp388_publisher.py --/                          \--> dashboard (one shared 10 Hz stream)

Roles: you choose ONE leader in the dashboard; every other bot is made a follower. The choice is
re-applied automatically if a bot reports a different role (for example after it rebooted).
While body tilt is on, the Pi leads and every bot is a follower.
"""
import json
import os
import socket
import sys
import threading
import time
from collections import deque

from flask import Flask, Response, jsonify, request, send_from_directory

from bodytilt import BodyTilt
from gateway import Gateway

HERE = os.path.dirname(os.path.abspath(__file__))
STATIC = os.path.join(HERE, "static")
SEQUENCES = ["stand", "rest", "wave", "dance", "swim", "point", "pushup", "bow", "cute", "freaky",
             "worm", "shake", "shrug", "dead", "crab", "forward", "backward", "left", "right"]
ROLE_RETRY_S = 3.0        # re-send a role at most this often while a bot reports the wrong one
ENV_KEEP = 3600           # temperature/pressure samples kept (1 h at 1 Hz)

gw = Gateway(sys.argv[1] if len(sys.argv) > 1 else None)
body = BodyTilt(gw)
leader = {"id": None}     # the bot the user chose as leader (None = not chosen yet)
last_role_send = {}       # bot id -> time a role command was last sent
env_hist = {"temperature": deque(maxlen=ENV_KEEP), "pressure": deque(maxlen=ENV_KEEP)}
bridge = None


def on_env(name, value):
    if name in env_hist:
        env_hist[name].append((round(time.time(), 1), value))


# ---------------------------------------------------------------- roles
def wanted_role(bot_id):
    if body.enabled or (body.handover and time.time() < body.roles_at):
        return "F"                                   # the Pi is streaming poses: everyone follows
    if leader["id"] is None:
        return None
    return "L" if bot_id == leader["id"] else "F"


def send_role(bot_id, role):
    last_role_send[bot_id] = time.time()
    gw.command(bot_id, "leader" if role == "L" else "follower", user=False)


def keep_roles():
    if body.handover:
        return                                       # handover() is setting the roles right now
    bots = [b for b in gw.bot_list() if b["online"]]
    if leader["id"] is None and not body.enabled:
        leaders = [b["id"] for b in bots if b["role"] == "L"]
        if len(leaders) == 1:                        # adopt the existing leader after a restart
            leader["id"] = leaders[0]
            gw.note(f"using {leaders[0]} as leader (it already reports leader)")
    now = time.time()
    for b in bots:
        want = wanted_role(b["id"])
        if b["run"] not in ("-", ""):
            continue                                 # busy bots refuse role changes; retry when it finishes
        if want and b["role"] != want and now - last_role_send.get(b["id"], 0) > ROLE_RETRY_S:
            gw.note(f"{b['id']} reports {'leader' if b['role'] == 'L' else 'follower'}, setting it to "
                    f"{'leader' if want == 'L' else 'follower'}")
            send_role(b["id"], want)


def apply_roles_now():
    bots = [b for b in gw.bot_list() if b["online"]]
    for b in sorted(bots, key=lambda b: wanted_role(b["id"]) == "L"):   # followers first: never two leaders
        want = wanted_role(b["id"])
        if want:
            send_role(b["id"], want)


def handover():
    """Body tilt was switched off: the Pi streams the stand pose for 0.6 s, then sends the roles, and keeps
    streaming stand until the chosen leader reports it is leading (at most 2.5 s), so no bot goes without poses."""
    if not body.handover or time.time() < body.roles_at:
        return
    if not body.roles_sent:
        body.roles_sent = True
        apply_roles_now()
        return
    lead = next((b for b in gw.bot_list() if b["id"] == leader["id"]), None)
    if lead is None or lead["role"] == "L":
        body.end_hold()


# ---------------------------------------------------------------- live frame (built once, shared by all tabs)
class Hub:
    def __init__(self):
        self.cv = threading.Condition()
        self.seq, self.data = 0, "{}"

    def publish(self, data):
        with self.cv:
            self.seq += 1
            self.data = data
            self.cv.notify_all()

    def wait(self, seen, timeout=15):
        with self.cv:
            self.cv.wait_for(lambda: self.seq != seen, timeout)
            return self.seq, self.data


hub = Hub()


def build_frame():
    bots = gw.bot_list()
    for i, b in enumerate(bots):
        b["slot"] = "Bot " + ("AB"[i] if i < 2 else str(i + 1))
        b["want"] = wanted_role(b["id"])
    now = time.time()
    env = {}
    for k, h in env_hist.items():
        env[k] = list(h[-1]) if h and now - h[-1][0] < 30 else None
    log_seq, log = gw.log_view()
    return {
        "t": round(now, 2),
        "gateway": gw.ser is not None,
        "mqtt": bool(bridge and bridge.connected),
        "tilt": {"pitch": round(body.pitch, 1), "roll": round(body.roll, 1)} if body.tilt_fresh() else None,
        "env": env,
        "bots": bots,
        "leader": leader["id"],
        "body": {"enabled": body.enabled, "status": body.status, "gain": body.gain, "max_drop": body.max_drop,
                 "handover": body.handover},
        "cmd": gw.last_command(),
        "log": log,
        "log_seq": log_seq,
    }


def state_for_mqtt():
    return {"leader": leader["id"], "body_tilt": body.enabled,
            "bots": [{k: b.get(k) for k in ("id", "online", "role", "run", "volts", "pct", "rssi")} for b in gw.bot_list()]}


def maintenance():
    n = 0
    while True:
        time.sleep(0.05)
        n += 1
        try:
            gw.tick()                       # command re-sends (20 Hz)
            handover()                      # body tilt -> leader
            if n % 2 == 0:
                hub.publish(json.dumps(build_frame()))   # dashboard frame (10 Hz)
            if n % 10 == 0:
                keep_roles()                # 2 Hz
        except Exception as e:          # never let the loop die
            gw.note(f"internal error: {e!r}")
            time.sleep(1)


# ---------------------------------------------------------------- web
app = Flask(__name__, static_folder=STATIC, static_url_path="/static")


def err(msg, code=400):
    return jsonify(ok=False, error=msg), code


def busy_bot():
    """A bot that is running a sequence refuses role changes, so wait for it (or STOP it) first."""
    return next((b for b in gw.bot_list() if b["online"] and b["run"] not in ("-", "")), None)


@app.get("/")
def index():
    return send_from_directory(STATIC, "index.html")


@app.get("/api/config")
def config():
    return jsonify(sequences=SEQUENCES)


@app.get("/api/history")
def history():
    return jsonify({k: list(v) for k, v in env_hist.items()})


@app.get("/api/stream")
def stream():
    def gen():
        seen = -1
        yield "retry: 1500\n\n"
        while True:
            seen, data = hub.wait(seen)
            yield f"data: {data}\n\n"
    return Response(gen(), mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"})


@app.post("/api/leader")
def set_leader():
    bot_id = str((request.get_json(force=True) or {}).get("id", ""))
    if bot_id not in {b["id"] for b in gw.bot_list()}:
        return err("unknown bot")
    busy = busy_bot()
    if busy:
        return err(f"{busy['id']} is running '{busy['run']}'. Wait for it to finish, then choose the leader.")
    leader["id"] = bot_id
    gw.note(f"leader set to {bot_id}")
    if body.enabled or body.handover:            # land in the stand pose first; handover() sends the roles
        if body.enabled:
            body.set_enabled(False)
            gw.note("body tilt switched off because a leader was chosen")
        return jsonify(ok=True)
    apply_roles_now()                            # followers first, so there is never a second leader
    return jsonify(ok=True)


@app.post("/api/seq")
def run_sequence():
    name = str((request.get_json(force=True) or {}).get("name", ""))
    if name not in SEQUENCES:
        return err("unknown sequence")
    if body.enabled:
        return err("Body tilt is on. Switch it off to run sequences.")
    lead = next((b for b in gw.bot_list() if b["id"] == leader["id"]), None)
    if lead is None:
        return err("Choose a leader first.")
    if not lead["online"]:
        return err(f"The leader ({lead['id']}) is offline.")
    if body.handover or lead["role"] != "L":
        return err(f"{lead['id']} is still switching to leader. Try again in a second.")
    busy = busy_bot()
    if busy:
        return err(f"{busy['id']} is still running '{busy['run']}'.")
    rec = gw.command(lead["id"], f"seq {name}")
    return jsonify(ok=rec["state"] != "failed", id=rec["id"], error=rec.get("error", ""))


@app.post("/api/stop")
def stop():
    rec = gw.command("*", "stop")
    return jsonify(ok=rec["state"] != "failed", id=rec["id"], error=rec.get("error", ""))


@app.post("/api/level")
def set_level():
    """Ask imu_publisher.py to treat the IMU's current attitude as level."""
    if not (bridge and bridge.publish("imu/zero", "1")):
        return err("MQTT is not connected.")
    gw.note("asked the IMU publisher to set level")
    return jsonify(ok=True)


@app.post("/api/body")
def body_cfg():
    d = request.get_json(force=True) or {}
    if "gain" in d:
        body.gain = min(4.0, max(0.5, float(d["gain"])))
    if "max_drop" in d:
        body.max_drop = min(80.0, max(10.0, float(d["max_drop"])))
    if "enabled" in d:
        on = bool(d["enabled"])
        if on and not body.tilt_fresh():
            return err("No tilt data. Start imu_publisher.py and check MQTT.")
        busy = busy_bot()
        if on and busy:
            return err(f"{busy['id']} is running '{busy['run']}'. Wait for it to finish first.")
        was = body.enabled
        body.set_enabled(on)
        if on != was:
            gw.note("body tilt " + ("on: all bots follow the Pi" if on else "off: returning to stand, then to the leader"))
        if on:
            apply_roles_now()                        # off: handover() sends the roles after the stand pose
    return jsonify(ok=True)


# ---------------------------------------------------------------- start
def pick_host():
    """Linux: listen on IPv4 + IPv6 so http://localhost works either way. Windows: plain IPv4."""
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
    for addr in ("127.0.0.1", "::1"):
        try:
            socket.create_connection((addr, port), timeout=0.5).close()
            return True
        except OSError:
            continue
    return False


if __name__ == "__main__":
    port = int(os.environ.get("PORT", "8000"))
    if port_in_use(port):        # checked before touching the serial port, so a second copy changes nothing
        sys.exit(f"\nCannot start: port {port} is already in use - app.py is probably already running.\n"
                 f"Stop the other one (Ctrl+C in its terminal, or: pkill -f app.py) or use PORT=8001.\n")
    gw.start()
    try:
        from mqtt_bridge import MqttBridge
        bridge = MqttBridge(body.set_tilt, on_env, state_for_mqtt)
        bridge.start()
    except ImportError:
        print("paho-mqtt not installed: no tilt / BMP388 data (pip install paho-mqtt)")
    threading.Thread(target=maintenance, daemon=True).start()
    print(f"\nSesame ground station: open http://127.0.0.1:{port}  (or http://<this-pi-ip>:{port})\n")
    app.run(host=pick_host(), port=port, threaded=True)
