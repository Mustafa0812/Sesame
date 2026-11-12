"""IMU guidance mode: tilt the IMU, the chosen bot walks that way. Level = stop.

Needs the bot to be the leader (it runs the sequences); enabling guidance sends 'leader' to it.
Commands are the same ones the web UI sends: seq forward|backward|left|right and stop.
"""
import threading
import time

SEND_GAP_S = 0.3        # minimum time between commands to the bot
RESTART_GAP_S = 0.7     # after 'stop', wait this long before starting the next sequence
RESEND_AFTER_S = 2.5    # sequence ended by itself while still tilted -> start it again


class Guidance:
    def __init__(self, imu, gw):
        self.imu, self.gw = imu, gw
        self.enabled = False
        self.target = ""
        self.dead = 15.0            # degrees of tilt needed to start moving
        self.invert_pitch = False
        self.invert_roll = False
        self.swap = False           # swap which tilt axis means forward/back vs left/right
        self.off_pitch = 0.0        # "level" reference, set by zero()
        self.off_roll = 0.0
        self.cmd = ""               # direction currently commanded
        self.last_send = 0.0
        self.last_start = 0.0
        self.stopped_at = 0.0
        self.pitch = 0.0            # tilt after offsets/inversion, for the UI
        self.roll = 0.0
        self.status = "off"
        threading.Thread(target=self._run, daemon=True).start()

    # ---- UI controls
    def zero(self):
        self.off_pitch, self.off_roll = self.imu.pitch, self.imu.roll

    def set_enabled(self, on, target=None):
        if on:
            if not target or target == "*":
                return False
            self.target = target
            self.gw.command(target, "leader")
            self.cmd = ""
            self.enabled = True
        else:
            self.enabled = False
            self._stop_bot()
        return True

    def _stop_bot(self):
        if self.target:
            self.gw.command(self.target, "stop")
        self.cmd = ""
        self.stopped_at = time.time()
        self.last_send = time.time()

    # ---- tilt -> direction
    def _tilt(self):
        p = self.imu.pitch - self.off_pitch
        r = self.imu.roll - self.off_roll
        if self.swap:
            p, r = r, p
        self.pitch = -p if self.invert_pitch else p
        self.roll = -r if self.invert_roll else r

    def _direction(self):
        p, r = self.pitch, self.roll
        # hysteresis: once moving, keep going until the tilt drops below 2/3 of the threshold
        limit = self.dead * (0.67 if self.cmd else 1.0)
        if max(abs(p), abs(r)) < limit:
            return ""
        if abs(p) >= abs(r):
            return "forward" if p > 0 else "backward"
        return "right" if r > 0 else "left"

    def _bot_running(self):
        for b in self.gw.snapshot()["bots"]:
            if b["id"] == self.target:
                return b["tlm"].get("run", "-") != "-"
        return False

    # ---- control loop
    def _run(self):
        while True:
            time.sleep(0.05)
            self._tilt()
            if not self.enabled:
                self.status = "off"
                continue
            now = time.time()
            if not self.imu.fresh():
                self.status = "IMU lost: stopped"
                if self.cmd:
                    self._stop_bot()
                continue
            want = self._direction()
            self.status = want or "level"
            if now - self.last_send < SEND_GAP_S:
                continue
            if want != self.cmd:
                if self.cmd:                       # something is running: stop it first
                    self._stop_bot()
                    continue
                if want and now - self.stopped_at >= RESTART_GAP_S:
                    self.gw.command(self.target, f"seq {want}")
                    self.cmd, self.last_send, self.last_start = want, now, now
            elif want and now - self.last_start > RESEND_AFTER_S and not self._bot_running():
                self.gw.command(self.target, f"seq {want}")   # sequence finished: keep walking
                self.last_send = self.last_start = now
