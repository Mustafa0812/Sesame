"""Body-tilt mode: the Pi streams servo poses so the bot leans the way the IMU is tilted.

The Pi acts as the leader (it sends the same POSE8 packets a leader bot streams); the target bot is
put in follower mode. Joint order is the servo INDEX order used by the firmware:
    0 R1 hip(front-right)  1 R2 hip(back-right)  2 L1 hip(front-left)  3 L2 hip(back-left)
    4 R3 lift(back-right)  5 R4 lift(front-right) 6 L4 lift(front-left) 7 L3 lift(back-left)
(Legs: R1+R4, R2+R3, L1+L4, L2+L3. Front/back is inferred from the 'bow' pose, see README.)

Stand pose (logical angles, same as runStandPose): hips 135 45 45 135, lifts 0 180 0 180.
A lift joint moves toward 90 to lower its corner of the body, so a tilt lowers the legs on that side.
"""
import threading
import time

STAND = [135, 45, 45, 135, 0, 180, 0, 180]
SEND_PERIOD_S = 0.04   # 25 Hz stream; the bot's failsafe trips after 500 ms of silence


def _dead(v, dz):
    if abs(v) <= dz:
        return 0.0
    return v - dz if v > 0 else v + dz


class BodyTilt:
    def __init__(self, guide, gw):
        self.guide, self.gw = guide, gw
        self.enabled = False
        self.target = ""
        self.gain = 1.5        # servo degrees of leg drop per degree of tilt
        self.max_drop = 60.0   # never fold a lift joint more than this far from stand
        self.dead = 3.0        # degrees of tilt ignored
        self.smooth = 0.35     # 0..1, higher = snappier
        self.d = [0.0] * 4     # smoothed drop for FR, FL, BR, BL
        self.status = "off"
        self._finishing = False
        self.ext_pitch = 0.0   # tilt received from elsewhere (MQTT); wins over the local IMU while fresh
        self.ext_roll = 0.0
        self.ext_time = 0.0
        threading.Thread(target=self._run, daemon=True).start()

    def set_external(self, pitch, roll):
        """Final tilt in degrees, relative to level: pitch + = lean forward, roll + = lean right.
        The sender decides the convention, so the invert/swap settings do not apply to it."""
        self.ext_pitch, self.ext_roll, self.ext_time = float(pitch), float(roll), time.time()

    def ext_fresh(self):
        return time.time() - self.ext_time < 0.5

    def source_ok(self):
        return self.ext_fresh() or self.guide.imu.fresh()

    def pose(self, hold=False):
        g = self.guide
        pitch, roll = (self.ext_pitch, self.ext_roll) if self.ext_fresh() else (g.pitch, g.roll)
        p = 0.0 if hold else _dead(pitch, self.dead)    # + = lean forward (nose down)
        r = 0.0 if hold else _dead(roll, self.dead)     # + = lean right
        fwd, back, right, left = max(p, 0), max(-p, 0), max(r, 0), max(-r, 0)
        goal = [fwd + right, fwd + left, back + right, back + left]   # FR, FL, BR, BL
        for i in range(4):
            t = min(self.max_drop, max(0.0, self.gain * goal[i]))
            self.d[i] += self.smooth * (t - self.d[i])
        fr, fl, br, bl = self.d
        a = list(STAND)
        a[4] = STAND[4] + br    # R3 lift, back-right
        a[5] = STAND[5] - fr    # R4 lift, front-right
        a[6] = STAND[6] + fl    # L4 lift, front-left
        a[7] = STAND[7] - bl    # L3 lift, back-left
        return [int(round(min(180, max(0, v)))) for v in a]

    def set_enabled(self, on, target=None):
        if on:
            if not target or target == "*":
                return False
            self.guide.set_enabled(False)          # only one mode at a time
            self.target = target
            self.d = [0.0] * 4
            self.gw.command(target, "follower")    # the Pi is the leader now
            time.sleep(0.2)
            self.enabled = True
        elif self.enabled:
            self.enabled = False
            threading.Thread(target=self._finish, daemon=True).start()
        return True

    def _finish(self):
        """Hold the stand pose for a second (so the bot doesn't hit its failsafe), then hand the
        bot back to normal leader mode in the stand pose."""
        self._finishing = True
        end = time.time() + 1.0
        while time.time() < end:
            self.gw.send("pose " + " ".join(map(str, STAND)))
            time.sleep(SEND_PERIOD_S)
        self.gw.command(self.target, "leader")
        time.sleep(0.3)
        self.gw.command(self.target, "seq stand")
        self._finishing = False

    def _run(self):
        while True:
            time.sleep(SEND_PERIOD_S)
            if not self.enabled:
                self.status = "off"
                continue
            fresh = self.source_ok()
            a = self.pose(hold=not fresh)
            self.status = ("stream (mqtt)" if self.ext_fresh() else "stream") if fresh else "tilt lost: holding stand"
            self.gw.send("pose " + " ".join(map(str, a)))
