"""Serial bridge to the ESP-NOW gateway (firmware/espnow-gateway).

Keeps a table of bots seen over the air (heartbeats + telemetry) and lets the web UI
send commands. Protocol lines from the gateway all start with '@'.
"""
import glob
import threading
import time

import serial

STALE_S = 10  # a bot counts as offline after this many seconds of silence


def find_port():
    """First USB serial port that looks like an ESP32-S3 (Linux/Pi)."""
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return None


class Gateway:
    def __init__(self, port=None, baud=115200):
        self.fixed_port = port
        self.port = port or find_port()
        self.baud = baud
        self.ser = None
        self.lock = threading.Lock()
        self.bots = {}      # mac -> {id, name, rssi, seen, tlm: {...}}
        self.log = []       # recent lines for the UI
        self.stats = ""
        self.ready = ""
        self._stop = False

    # ---- lifecycle
    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while not self._stop:
            try:
                if self.port is None:
                    self.port = self.fixed_port or find_port()
                    if self.port is None:
                        self._note("no gateway serial port found, retrying")
                        time.sleep(2)
                        continue
                # CP2102/CH340 boards (WROOM-32) reset the ESP32 when DTR/RTS toggle on open.
                # Hold both low before opening so the gateway keeps running.
                s = serial.Serial()
                s.port, s.baudrate, s.timeout = self.port, self.baud, 0.5
                s.dtr = False
                s.rts = False
                s.open()
                self.ser = s
                self._note(f"opened {self.port}")
                while not self._stop:
                    raw = self.ser.readline()
                    if raw:
                        self._parse(raw.decode("utf-8", "replace").strip())
            except (serial.SerialException, OSError) as e:
                self._note(f"serial error: {e}")
                self.ser = None
                self.port = None
                time.sleep(2)

    # ---- sending
    def send(self, line):
        if not self.ser:
            return False
        with self.lock:
            self.ser.write((line.strip() + "\n").encode())
        return True

    def command(self, target, text):
        """target = board id or '*'; text = what you'd type on the bot's serial."""
        return self.send(f"cmd {target} {text}")

    # ---- receiving
    def _note(self, s):
        self.log.append(f"{time.strftime('%H:%M:%S')} {s}")
        del self.log[:-200]

    def _bot(self, mac):
        return self.bots.setdefault(mac, {"id": "", "name": "", "rssi": 0, "seen": 0, "tlm": {}})

    def _parse(self, line):
        if not line.startswith("@"):
            return
        parts = line.split(" ", 3)
        tag = parts[0]
        if tag == "@READY":
            self.ready = line
        elif tag == "@STAT":
            self.stats = line[6:]
        elif tag == "@HB" and len(parts) >= 4:
            b = self._bot(parts[1])
            b.update(rssi=int(parts[2]), name=parts[3], seen=time.time())
        elif tag == "@RX" and len(parts) >= 4:
            b = self._bot(parts[1])
            b.update(rssi=int(parts[2]), seen=time.time())
            text = parts[3]
            if text.startswith("TLM:"):
                bid, _, rest = text[4:].partition(":")
                b["id"] = bid
                b["tlm"] = dict(kv.split("=", 1) for kv in rest.split(";") if "=" in kv)
            else:
                self._note(f"{parts[1]}: {text}")
        elif tag == "@ERR":
            self._note(line)

    def snapshot(self):
        now = time.time()
        out = []
        for mac, b in list(self.bots.items()):
            if b["name"] == "gateway":
                continue
            out.append({"mac": mac, **b, "age": round(now - b["seen"], 1),
                        "online": now - b["seen"] < STALE_S})
        return {"bots": out, "log": self.log[-40:], "gateway": self.ready,
                "stats": self.stats, "connected": self.ser is not None}
