"""Turn the firmware's movement-sequences.h into timed joint keyframes for the dashboard simulator.

    python3 tools/gen_sequences.py            (from ground-station/)

Reads ../firmware/sesame-firmware-main/movement-sequences.h and writes static/data/sequences.json:
    { "wave": {"duration": 4200, "events": [[t_ms, servo_index, angle], ...]}, ... }
Servo index order is the firmware's (0 R1, 1 R2, 2 L1, 3 L2, 4 R3, 5 R4, 6 L4, 7 L3).

It interprets only the small C subset the sequences use: setServoAngle, delayWithFace, pressingCheck,
runStandPose, for-loops (constant or walkCycles bounds) and for(i<8) setServoAngle(i, n). Everything else
(faces, Serial prints, state flags) is ignored. Timing matches the firmware: every setServoAngle waits
motorCurrentDelay (20 ms), pressingCheck waits frameDelay (100 ms), walkCycles = 10.
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "..", "firmware", "sesame-firmware-main", "movement-sequences.h")
OUT = os.path.join(HERE, "..", "static", "data", "sequences.json")

MOTOR_DELAY = 20
FRAME_DELAY = 100
WALK_CYCLES = 10

SEQUENCES = {  # command name -> function
    "rest": "runRestPose", "stand": "runStandPose", "wave": "runWavePose", "dance": "runDancePose",
    "swim": "runSwimPose", "point": "runPointPose", "pushup": "runPushupPose", "bow": "runBowPose",
    "cute": "runCutePose", "freaky": "runFreakyPose", "worm": "runWormPose", "shake": "runShakePose",
    "shrug": "runShrugPose", "dead": "runDeadPose", "crab": "runCrabPose",
    "forward": "runWalkPose", "backward": "runWalkBackward", "left": "runTurnLeft", "right": "runTurnRight",
}


def enum_map(src):
    body = re.search(r"enum ServoName[^{]*\{(.*?)\};", src, re.S).group(1)
    return {m.group(1): int(m.group(2)) for m in re.finditer(r"(\w+)\s*=\s*(\d+)", body)}


def functions(src):
    funcs = {}
    for m in re.finditer(r"inline\s+void\s+(\w+)\s*\([^)]*\)\s*\{", src):
        i, depth = m.end(), 1
        while depth:
            depth += {"{": 1, "}": -1}.get(src[i], 0)
            i += 1
        funcs[m.group(1)] = src[m.end():i - 1]
    return funcs


def strip(code):
    code = re.sub(r"//[^\n]*", "", code)
    return re.sub(r"/\*.*?\*/", "", code, flags=re.S)


class Sim:
    def __init__(self, names, funcs):
        self.names, self.funcs = names, funcs
        self.t = 0
        self.events = []

    def run_block(self, code, env):
        i = 0
        while i < len(code):
            code_i = code[i:].lstrip()
            i = len(code) - len(code_i)
            if not code_i:
                break
            m = re.match(r"for\s*\(\s*int\s+(\w+)\s*=\s*(\d+)\s*;\s*\1\s*<\s*(\w+)\s*;\s*\1\+\+\s*\)\s*", code_i)
            if m:
                var, start, bound = m.group(1), int(m.group(2)), m.group(3)
                end = WALK_CYCLES if bound == "walkCycles" else int(bound)
                rest = code_i[m.end():]
                if rest.startswith("{"):
                    j, depth = 1, 1
                    while depth:
                        depth += {"{": 1, "}": -1}.get(rest[j], 0)
                        j += 1
                    inner, consumed = rest[1:j - 1], m.end() + j
                else:   # single statement body
                    j = rest.index(";") + 1
                    inner, consumed = rest[:j], m.end() + j
                for n in range(start, end):
                    self.run_block(inner, {**env, var: n})
                i += consumed
                continue
            j = code_i.find(";")
            if j < 0:
                break
            stmt, i = code_i[:j].strip(), i + j + 1
            self.stmt(stmt, env)

    def stmt(self, s, env):
        # "if (...) <statement>" -> only keep the statement for the conditions we can ignore
        m = re.match(r"if\s*\(.*?\)\s*(.*)$", s, re.S)
        if m and not s.startswith("if (!pressingCheck"):
            s = m.group(1).strip()
        for call in re.finditer(r"setServoAngle\(\s*(\w+)\s*,\s*(\d+)\s*\)", s):
            a, v = call.group(1), int(call.group(2))
            idx = int(a) if a.isdigit() else env[a] if a in env else self.names[a]
            self.events.append([self.t, idx, v])
            self.t += MOTOR_DELAY
        m = re.search(r"delayWithFace\(\s*(\d+)\s*\)", s)
        if m:
            self.t += int(m.group(1))
        if "pressingCheck" in s:
            self.t += FRAME_DELAY
        m = re.match(r"runStandPose\(\s*(\d)\s*\)", s)
        if m:
            self.run_block(strip(self.funcs["runStandPose"]).replace("if (face == 1)", "if (0)"), env)


def main():
    src = open(SRC, encoding="utf-8").read().replace("\r\n", "\n")
    names, funcs = enum_map(src), functions(src)
    out = {}
    for cmd, fn in SEQUENCES.items():
        sim = Sim(names, funcs)
        if fn == "runStandPose":
            sim.run_block(strip(funcs[fn]), {})
        else:
            sim.run_block(strip(funcs[fn]), {})
        out[cmd] = {"duration": sim.t, "events": sim.events}
        print(f"{cmd:9s} {len(sim.events):4d} moves  {sim.t/1000:5.1f} s")
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    json.dump({"order": ["R1", "R2", "L1", "L2", "R3", "R4", "L4", "L3"], "sequences": out},
              open(OUT, "w"), separators=(",", ":"))
    print("wrote", os.path.normpath(OUT))


if __name__ == "__main__":
    sys.exit(main())
