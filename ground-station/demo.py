"""Demo mode (DEMO=1): fake bots, IMU tilt and BMP388 data so the dashboard can be tried without hardware.
Commands to the bots are intercepted: 'seq X' makes a fake bot report that sequence for its real duration."""
import json
import math
import os
import random
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))


class Demo:
    def __init__(self, gw, body, store):
        self.gw, self.body, self.store = gw, body, store
        seq = json.load(open(os.path.join(HERE, "static", "data", "sequences.json")))["sequences"]
        self.dur = {k: v["duration"] / 1000.0 for k, v in seq.items()}
        self.bots = {
            "A1B2": {"mac": "AA:BB:CC:00:00:01", "role": "L", "run": "-", "until": 0, "pct": 82.0},
            "C3D4": {"mac": "AA:BB:CC:00:00:02", "role": "F", "run": "-", "until": 0, "pct": 74.0},
        }
        gw.send = lambda line: True
        gw.command = self.command
        gw.ser = object()          # makes the dashboard show "gateway connected"
        gw.ready = "@READY DEMO ch1 tx14"

    def command(self, target, text):
        text = text.strip()
        for bid, b in self.bots.items():
            if target not in ("*", bid):
                continue
            if text in ("leader", "follower"):
                b["role"] = "L" if text == "leader" else "F"
            elif text == "stop":
                b["run"], b["until"] = "-", 0
            elif text.startswith("seq ") and b["role"] == "L":
                name = text[4:]
                b["run"], b["until"] = name, time.time() + self.dur.get(name, 3)
        return True

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        t0, n = time.time(), 0
        while True:
            time.sleep(0.04)
            n += 1
            t = time.time() - t0
            self.body.set_external(18 * math.sin(t * 0.7), 14 * math.sin(t * 0.45 + 1))
            if n % 25 == 0:
                self.store.add_metric("temperature", 24.0 + 1.5 * math.sin(t / 40) + random.uniform(-0.05, 0.05))
                self.store.add_metric("pressure", 1013.0 + 2.0 * math.sin(t / 90) + random.uniform(-0.03, 0.03))
            if n % 12 == 0:
                now = time.time()
                for bid, b in self.bots.items():
                    if b["until"] and now > b["until"]:
                        b["run"], b["until"] = "-", 0
                    b["pct"] = max(5.0, b["pct"] - 0.0015)
                    bot = self.gw._bot(b["mac"])
                    bot.update(id=bid, name="seq-tester", rssi=-48 - random.randint(0, 6), seen=now)
                    running = b["run"] != "-"
                    bot["tlm"] = {"role": b["role"], "run": b["run"], "v": f"{7.0 + b['pct'] / 100 * 1.2:.2f}",
                                  "ma": f"{(420 if running else 90) + random.randint(-15, 15)}", "pct": f"{b['pct']:.0f}"}
