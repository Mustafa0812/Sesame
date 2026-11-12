"""Body-tilt mode: the Pi streams servo poses so the bots lean the way the IMU is tilted.

The Pi acts as the leader (it sends the same POSE8 packets a leader bot streams), so while it is on every bot
must be a follower (app.py takes care of that). Tilt comes from MQTT (imu_publisher.py -> sesame/tilt).

Joint order is the firmware's servo INDEX order (names as in the PCA sketch):
    0 R1 hip   1 R2 hip   2 L1 hip   3 L2 hip   4 R4 lift   5 R3 lift   6 L3 lift   7 L4 lift
Stand pose = runStandPose: hips 135 45 45 135, lifts 0 180 0 180 (by index).
Legs (from the turn gaits): hip 0 + lift 5, hip 3 + lift 7, hip 1 + lift 4, hip 2 + lift 6.
Front legs are hips 0 and 2 (the 'bow' pose folds only lifts 5 and 6).
A lift joint moves toward 90 to lower its corner of the body, so a tilt lowers the legs on that side.
"""
import threading
import time

STAND = [135, 45, 45, 135, 0, 180, 0, 180]
SEND_PERIOD_S = 0.04          # 25 Hz; a follower's failsafe trips after 500 ms of silence
TILT_STALE_S = 0.5

# (index, direction toward "lowered") for each corner, from the stand pose:
#   idx 4 starts at 0 and lowers by increasing; idx 5 starts at 180 and lowers by decreasing; etc.
#   Corners (inferred from the 'bow' pose: R1/L1 legs are the front): FR = 5, FL = 6, BR = 4, BL = 7.
CORNERS = {"FR": (5, -1), "FL": (6, +1), "BR": (4, +1), "BL": (7, -1)}


def _dead(v, dz):
    if abs(v) <= dz:
        return 0.0
    return v - dz if v > 0 else v + dz


class BodyTilt:
    def __init__(self, gw):
        self.gw = gw
        self.enabled = False
        self.gain = 1.5          # servo degrees of leg drop per degree of tilt
        self.max_drop = 60.0     # never fold a lift joint more than this far from stand
        self.dead = 3.0          # degrees of tilt ignored
        self.smooth = 0.35       # 0..1, higher = snappier
        self.pitch = self.roll = 0.0
        self.tilt_time = 0.0
        self.d = {k: 0.0 for k in CORNERS}
        self.status = "off"
        self._hold_until = 0.0   # after switching off, keep streaming the stand pose until this time
        threading.Thread(target=self._run, daemon=True).start()

    def set_tilt(self, pitch, roll):
        """Tilt in degrees relative to level: pitch + = lean forward, roll + = lean right."""
        self.pitch, self.roll, self.tilt_time = float(pitch), float(roll), time.time()

    def tilt_fresh(self):
        return time.time() - self.tilt_time < TILT_STALE_S

    def set_enabled(self, on):
        if on and not self.enabled:
            self.d = {k: 0.0 for k in CORNERS}
        if not on and self.enabled:
            self._hold_until = time.time() + 1.0      # land softly in the stand pose before the leader takes over
        self.enabled = bool(on)

    def pose(self, hold=False):
        p = 0.0 if hold else _dead(self.pitch, self.dead)
        r = 0.0 if hold else _dead(self.roll, self.dead)
        fwd, back, right, left = max(p, 0), max(-p, 0), max(r, 0), max(-r, 0)
        goal = {"FR": fwd + right, "FL": fwd + left, "BR": back + right, "BL": back + left}
        a = list(STAND)
        for k, (idx, sign) in CORNERS.items():
            t = min(self.max_drop, max(0.0, self.gain * goal[k]))
            self.d[k] += self.smooth * (t - self.d[k])
            a[idx] = STAND[idx] + sign * self.d[k]
        return [int(round(min(180, max(0, v)))) for v in a]

    def _run(self):
        while True:
            time.sleep(SEND_PERIOD_S)
            if self.enabled:
                fresh = self.tilt_fresh()
                self.status = "streaming" if fresh else "no tilt data: holding stand"
                self.gw.send("pose " + " ".join(map(str, self.pose(hold=not fresh))))
            elif time.time() < self._hold_until:
                self.status = "returning to stand"
                self.gw.send("pose " + " ".join(map(str, STAND)))
            else:
                self.status = "off"
