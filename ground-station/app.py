"""Sesame ground station: web UI on the Pi.

    python3 app.py [/dev/ttyACM0]      then open http://<pi-ip>:8000
"""
import json
import os
import sys

from flask import Flask, jsonify, request

from gateway import Gateway
from bodytilt import BodyTilt
from guidance import Guidance
from imu import Imu

SEQUENCES = ["rest", "stand", "wave", "dance", "swim", "point", "pushup", "bow", "cute", "freaky",
             "worm", "shake", "shrug", "dead", "crab", "forward", "backward", "left", "right"]

gw = Gateway(sys.argv[1] if len(sys.argv) > 1 else None)
gw.start()
imu = Imu()
if os.environ.get("LOCAL_IMU", "1") != "0":   # LOCAL_IMU=0 when the IMU publishes over MQTT instead
    imu.start()
guide = Guidance(imu, gw)
body = BodyTilt(guide, gw)
if os.environ.get("MQTT_HOST"):
    from mqtt_bridge import MqttBridge
    MqttBridge(gw, body, guide, imu, SEQUENCES).start()
app = Flask(__name__)

PAGE = """<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<title>Sesame ground station</title>
<style>
body{font:15px system-ui;margin:16px;max-width:900px;background:#111;color:#eee}
button{margin:3px;padding:8px 12px;border:0;border-radius:6px;background:#2d6cdf;color:#fff}
button.stop{background:#c0392b}table{border-collapse:collapse;width:100%}
td,th{padding:5px 8px;border-bottom:1px solid #333;text-align:left}.off{opacity:.4}
pre{background:#000;padding:8px;height:160px;overflow:auto}select{padding:6px}
</style>
<h2>Sesame ground station <small id=gws></small></h2>
<table><thead><tr><th>ID<th>Role<th>Running<th>Volts<th>mA<th>Batt<th>RSSI<th>Seen</tr></thead>
<tbody id=bots></tbody></table>
<p>Target: <select id=target><option value="*">all bots</option></select>
<button onclick="send('leader')">Leader</button>
<button onclick="send('follower')">Follower</button>
<button class=stop onclick="send('stop')">STOP</button></p>
<div id=seqs></div>
<h3>IMU guidance <small id=imus></small></h3>
<p>Bot: <select id=gtarget></select>
<button onclick="gz()">Set level</button>
<button id=gbtn onclick="gtoggle()">Enable</button>
Dead zone <input id=dead type=range min=5 max=40 value=15 oninput="gcfg()"> <span id=deadv>15</span>&deg;<br>
<label><input type=checkbox id=ip onchange="gcfg()"> invert fwd/back</label>
<label><input type=checkbox id=ir onchange="gcfg()"> invert left/right</label>
<label><input type=checkbox id=sw onchange="gcfg()"> swap axes</label></p>
<h3>Body tilt <small id=bts></small></h3>
<p>The bot leans the way the IMU is tilted (Pi streams the pose; the bot is set to follower). Uses the same bot, level and invert settings as above.<br>
<button id=bbtn onclick="btoggle()">Enable body tilt</button>
Gain <input id=bgain type=range min=5 max=40 value=15 oninput="bcfg()"> <span id=bgainv>1.5</span>
Max drop <input id=bmax type=range min=10 max=80 value=60 oninput="bcfg()"> <span id=bmaxv>60</span>&deg;</p>
<p id=msg style="color:#f66"></p>
<h3>Log</h3><pre id=log></pre>
<script>
const SEQ=__SEQ__;
document.getElementById('seqs').innerHTML=SEQ.map(s=>`<button onclick="send('seq ${s}')">${s}</button>`).join('');
async function send(text){
  await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({target:document.getElementById('target').value,text})});
}
let gon=false, bon=false;
function bcfg(){const g=document.getElementById('bgain').value/10, m=+document.getElementById('bmax').value;
  document.getElementById('bgainv').textContent=g.toFixed(1);document.getElementById('bmaxv').textContent=m;
  post('/api/body/config',{gain:g,max_drop:m});}
async function act(u,b){const m=document.getElementById('msg');m.textContent='';
  try{const r=await post(u,b);const j=await r.json();if(!r.ok||j.ok===false)m.textContent=j.error||'refused (is a bot selected and is tilt arriving?)';}
  catch(e){m.textContent='request failed: '+e;}}
async function btoggle(){await act('/api/body/enable',{enabled:!bon,target:document.getElementById('gtarget').value});}
function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b||{})});}
function gcfg(){document.getElementById('deadv').textContent=document.getElementById('dead').value;
  post('/api/imu/config',{dead:+document.getElementById('dead').value,invert_pitch:document.getElementById('ip').checked,
  invert_roll:document.getElementById('ir').checked,swap:document.getElementById('sw').checked});}
function gz(){post('/api/imu/zero');}
async function gtoggle(){await act('/api/imu/enable',{enabled:!gon,target:document.getElementById('gtarget').value});}
async function imutick(){
  try{
    const s=await (await fetch('/api/imu')).json();
    gon=s.enabled; bon=s.body;
    document.getElementById('bbtn').textContent=bon?'Disable body tilt':'Enable body tilt';
    document.getElementById('bts').textContent=bon?s.body_status:(s.mqtt?'(MQTT tilt arriving)':'(no tilt received yet)');
    document.getElementById('gbtn').textContent=gon?'Disable':'Enable';
    document.getElementById('imus').textContent=s.ok?`pitch ${s.pitch.toFixed(0)}\\u00b0 roll ${s.roll.toFixed(0)}\\u00b0 | ${s.status}`:`IMU: ${s.error}`;
  }catch(e){}
}
setInterval(imutick,200);
async function tick(){
  try{
    const s=await (await fetch('/api/state')).json();
    document.getElementById('gws').textContent=s.connected?'(gateway connected)':'(gateway NOT connected)';
    const sel=document.getElementById('target'), cur=sel.value;
    const ids=s.bots.filter(b=>b.id).map(b=>b.id);
    sel.innerHTML='<option value="*">all bots</option>'+ids.map(i=>`<option>${i}</option>`).join('');
    sel.value=(ids.includes(cur)||cur=='*')?cur:'*';
    const gs=document.getElementById('gtarget'), gcur=gs.value;
    if(gs.options.length!=ids.length){gs.innerHTML=ids.map(i=>`<option>${i}</option>`).join('');if(ids.includes(gcur))gs.value=gcur;}
    document.getElementById('bots').innerHTML=s.bots.map(b=>{const t=b.tlm||{};
      return `<tr class="${b.online?'':'off'}"><td>${b.id||b.mac}<td>${t.role||''}<td>${t.run||''}<td>${t.v||''}<td>${t.ma||''}<td>${t.pct>=0?t.pct+'%':'?'}<td>${b.rssi}<td>${b.age}s</tr>`}).join('');
    document.getElementById('log').textContent=s.log.join('\\n');
  }catch(e){}
}
setInterval(tick,1000);tick();
</script>"""


@app.get("/")
def index():
    return PAGE.replace("__SEQ__", json.dumps(SEQUENCES))


@app.get("/api/state")
def state():
    return jsonify(gw.snapshot())


@app.post("/api/cmd")
def cmd():
    d = request.get_json(force=True)
    target = str(d.get("target", "*"))
    text = str(d.get("text", "")).strip()
    # Whitelist: only known sequences and the safe control words.
    ok = text in ("leader", "follower", "stop", "status") or (
        text.startswith("seq ") and text[4:] in SEQUENCES)
    if not ok:
        return jsonify(ok=False, error="command not allowed"), 400
    if target != "*" and not target.isalnum():
        return jsonify(ok=False, error="bad target"), 400
    return jsonify(ok=gw.command(target, text))


@app.get("/api/imu")
def imu_state():
    return jsonify(ok=imu.fresh(), error=imu.error, pitch=guide.pitch, roll=guide.roll,
                   enabled=guide.enabled, status=guide.status, target=guide.target,
                   body=body.enabled, body_status=body.status, mqtt=body.ext_fresh())


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
        return jsonify(ok=False, error="pick a bot and make sure the IMU is reading"), 400
    if d.get("enabled") and body.enabled:
        body.set_enabled(False)
    return jsonify(ok=guide.set_enabled(bool(d.get("enabled")), target))


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
        return jsonify(ok=False, error="pick a bot and make sure the IMU is reading"), 400
    return jsonify(ok=body.set_enabled(bool(d.get("enabled")), target))


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)
