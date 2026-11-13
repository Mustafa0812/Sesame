"""Serial link to the ESP-NOW gateway (firmware/espnow-gateway) and the table of bots it hears.

ESP-NOW broadcasts can be lost, so every command is re-sent until each target bot answers, up to RETRIES
sends. A bot answers each command exactly once: "ACK:<id>:<command>" (accepted) or
"ERR:<id>:<command>:<reason>" (refused). It recognises a re-send and repeats its answer without running it again.

Gateway -> Pi lines all start with '@':  @READY, @HB <mac> <rssi> <name>, @RX <mac> <rssi> <text>, @OK, @ERR ...
Pi -> gateway:                           cmd <id|*> <text>,  pose a0..a7
"""
import glob
import threading
import time
from collections import deque

import serial

OFFLINE_S = 6          # a bot counts as offline after this long without telemetry or heartbeat
ACK_TIMEOUT_S = 0.35   # re-send a command if no ACK arrived within this time
RETRIES = 3            # total sends per command


def find_port():
    """First USB serial port that looks like an ESP32 (Linux / Raspberry Pi)."""
    for pat in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        ports = sorted(glob.glob(pat))
        if ports:
            return ports[0]
    return None


class Gateway:
    def __init__(self, port=None, baud=115200):
        self.fixed_port, self.port, self.baud = port, None, baud
        self.ser = None
        self.ready = ""
        self.lock = threading.RLock()        # bots, pending commands, log (re-entrant: sends happen under it)
        self.write_lock = threading.Lock()   # serial writes
        self.bots = {}                       # board id -> state dict
        self.macs = {}                       # MAC -> board id
        self.pending = []                    # commands still waiting for ACKs
        self.last_cmd = None                 # last command started by the user (for the dashboard)
        self.log = deque(maxlen=60)
        self.log_seq = 0
        self._next_id = 1

    # ------------------------------------------------------------------ serial
    def start(self):
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self):
        last_err = ""
        while True:
            s = None
            try:
                self.port = self.fixed_port or find_port()
                if self.port is None:
                    if last_err != "none":
                        self.note("no gateway on USB (/dev/ttyUSB*, /dev/ttyACM*), retrying")
                        last_err = "none"
                    time.sleep(3)
                    continue
                # CP2102/CH340 boards (WROOM-32) reset the ESP32 when DTR/RTS toggle on open: hold both low.
                s = serial.Serial()
                s.port, s.baudrate, s.timeout = self.port, self.baud, 0.5
                s.dtr = False
                s.rts = False
                s.open()
                self.ser = s
                last_err = ""
                self.note(f"gateway connected on {self.port}")
                while True:
                    raw = s.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", "replace").strip()
                    try:
                        self._parse(line)
                    except Exception as e:          # e.g. a line cut short by a gateway reset: skip it
                        self.note(f"skipped a garbled gateway line ({e!r}): {line[:80]}")
            except (serial.SerialException, OSError) as e:
                if self.ser is not None:
                    self.note(f"gateway disconnected: {e}")
                elif str(e) != last_err:             # could not open: say why, once
                    hint = " (add yourself to the dialout group)" if "ermission" in str(e) else ""
                    self.note(f"cannot open {self.port}: {e}{hint}")
                last_err = str(e)
                self.ser = None
                try:
                    if s is not None:
                        s.close()                    # free the device so a replugged gateway gets the same name
                except Exception:
                    pass
                time.sleep(2)

    def send(self, line):
        s = self.ser
        if s is None:
            return False
        try:
            with self.write_lock:
                s.write((line + "\n").encode())
            return True
        except (serial.SerialException, OSError):
            return False

    # ------------------------------------------------------------------ commands
    def command(self, target, text, user=True):
        """Send a bot command (same text you would type on the bot's serial). target = board id or '*'.
        Returns the command record; it is re-sent until every target ACKs (see tick())."""
        now = time.time()
        with self.lock:
            if target == "*":
                need = {b["id"] for b in self.bots.values() if now - b["seen"] < OFFLINE_S}
            else:
                need = {target}
            kind = "role" if text in ("leader", "follower") else ("user" if user else "other")
            rec = {"id": self._next_id, "target": target, "text": text, "need": need, "acked": {},
                   "t0": now, "last": now, "tries": 1, "state": "sent", "user": user, "kind": kind}
            self._next_id += 1
            # A newer command replaces older unanswered ones of the same kind (user commands, or role
            # commands), so a late re-send can never run after it (e.g. 'seq wave' after STOP).
            if kind != "other":
                for r in self.pending:
                    if r["kind"] == kind and r["state"] == "sent" and (r["target"] == target or "*" in (r["target"], target)):
                        r["state"] = "superseded"
            self.pending.append(rec)
            if user:
                self.last_cmd = rec
            ok = self.send(f"cmd {target} {text}")   # under the lock, so the wire order matches the supersede order
            if not ok:
                rec["state"] = "failed"
                rec["error"] = "gateway not connected"
                self._note(f"✗ '{text}' to {target}: gateway not connected")
        return rec

    def tick(self):
        """Call every ~50 ms: re-sends commands that have not been ACKed yet."""
        now = time.time()
        with self.lock:   # re-send under the lock: a command superseded a moment ago is never written
            for rec in self.pending:
                if rec["state"] != "sent" or now - rec["last"] < ACK_TIMEOUT_S:
                    continue
                if rec["tries"] < RETRIES:
                    rec["tries"] += 1
                    rec["last"] = now
                    self.send(f"cmd {rec['target']} {rec['text']}")
                else:
                    missing = sorted(rec["need"] - set(rec["acked"]))
                    rec["state"] = "partial" if rec["acked"] else "failed"
                    rec["error"] = "no reply from " + (", ".join(missing) if missing else "any bot")
                    self._note(f"✗ '{rec['text']}' to {rec['target']}: {rec['error']}")
            self.pending = [r for r in self.pending if r["state"] == "sent" or now - r["t0"] < 10]

    def _ack(self, bid, text):
        now = time.time()
        for rec in self.pending:
            if rec["state"] != "sent" or rec["text"].lower() != text.strip().lower():
                continue
            if rec["target"] != "*" and rec["target"].lower() != bid.lower():
                continue
            if bid not in rec["acked"]:
                rec["acked"][bid] = round((now - rec["t0"]) * 1000)
            if rec["need"] <= set(rec["acked"]):
                rec["state"] = "acked"
                ms = max(rec["acked"].values())
                self._note(f"✓ '{rec['text']}' received by {', '.join(sorted(rec['acked']))} ({ms} ms"
                           + (f", {rec['tries']} sends)" if rec["tries"] > 1 else ")"))
            break

    def _refused(self, bid, text, reason):
        """The bot received the command but refused it (firmware sends ERR:<id>:<cmd>:<reason>)."""
        for rec in reversed(self.pending):
            if rec["state"] not in ("sent", "acked") or rec["text"].lower() != text.strip().lower():
                continue
            if rec["target"] != "*" and rec["target"].lower() != bid.lower():
                continue
            rec["state"] = "rejected"
            rec["error"] = f"{bid} refused it: {reason}"
            self._note(f"✗ '{text}' refused by {bid}: {reason}")
            return
        self._note(f"{bid} refused '{text}': {reason}")

    # ------------------------------------------------------------------ incoming lines
    def note(self, s):
        with self.lock:
            self._note(s)

    def _note(self, s):
        self.log.append(f"{time.strftime('%H:%M:%S')}  {s}")
        self.log_seq += 1

    def _parse(self, line):
        if not line.startswith("@"):
            return                                     # boot messages etc.
        parts = line.split(" ", 3)
        tag = parts[0]
        with self.lock:
            if tag == "@READY":
                self.ready = line
                self._note("gateway ready: " + line[7:])
            elif tag == "@ERR":
                self._note("gateway " + line)
            elif tag == "@HB" and len(parts) >= 3:
                bid = self.macs.get(parts[1])
                if bid in self.bots:
                    self.bots[bid].update(rssi=int(parts[2]), seen=time.time())
            elif tag == "@RX" and len(parts) >= 4:
                mac, rssi, text = parts[1], int(parts[2]), parts[3]
                if text.startswith("TLM:"):
                    self._telemetry(mac, rssi, text[4:])
                elif text.startswith("ACK:"):
                    bid, _, cmd = text[4:].partition(":")
                    self._ack(bid, cmd)
                elif text.startswith("ERR:"):
                    bid, _, rest = text[4:].partition(":")
                    cmd, _, reason = rest.rpartition(":")
                    self._refused(bid, cmd, reason)
                elif text.startswith("SYNC:"):
                    pass                                # leader -> follower sequence announce, not for the Pi
                else:
                    self._note(f"{self.macs.get(mac, mac)}: {text}")

    def _telemetry(self, mac, rssi, body):
        bid, _, rest = body.partition(":")
        kv = dict(p.split("=", 1) for p in rest.split(";") if "=" in p)
        now = time.time()
        b = self.bots.setdefault(bid, {"id": bid, "role": "?", "run": "-", "up": None, "rebooted_at": 0, "rst": ""})
        self.macs[mac] = bid
        up = int(kv["up"]) if kv.get("up", "").isdigit() else None
        if up is not None and b["up"] is not None and up + 2 < b["up"]:
            b["rebooted_at"] = now
            b["rst"] = kv.get("rst", "?")
            self._note(f"⚠ {bid} rebooted (reset reason: {b['rst']})")
        b.update(mac=mac, rssi=rssi, seen=now, role=kv.get("role", "?"), run=kv.get("run", "-"),
                 volts=kv.get("v"), ma=kv.get("ma"), pct=kv.get("pct"), up=up, rst=kv.get("rst", b["rst"]))

    # ------------------------------------------------------------------ dashboard view
    def bot_list(self):
        now = time.time()
        with self.lock:
            out = []
            for b in sorted(self.bots.values(), key=lambda x: x["id"]):
                d = dict(b)
                d["online"] = now - b["seen"] < OFFLINE_S
                d["age"] = round(now - b["seen"], 1)
                d["rebooted_ago"] = round(now - b["rebooted_at"]) if b["rebooted_at"] else None
                out.append(d)
            return out

    def last_command(self):
        with self.lock:
            r = self.last_cmd
            if not r:
                return None
            return {"id": r["id"], "target": r["target"], "text": r["text"], "state": r["state"],
                    "tries": r["tries"], "acked": dict(r["acked"]), "error": r.get("error", "")}

    def log_view(self, n=40):
        with self.lock:
            return self.log_seq, list(self.log)[-n:]
