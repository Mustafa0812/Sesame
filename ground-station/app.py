"""Sesame ground station: web UI on the Pi.

    python3 app.py [/dev/ttyACM0]      then open http://<pi-ip>:8000
"""
import json
import sys

from flask import Flask, jsonify, request

from gateway import Gateway

SEQUENCES = ["rest", "stand", "wave", "dance", "swim", "point", "pushup", "bow", "cute", "freaky",
             "worm", "shake", "shrug", "dead", "crab", "forward", "backward", "left", "right"]

gw = Gateway(sys.argv[1] if len(sys.argv) > 1 else None)
gw.start()
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
<h3>Log</h3><pre id=log></pre>
<script>
const SEQ=__SEQ__;
document.getElementById('seqs').innerHTML=SEQ.map(s=>`<button onclick="send('seq ${s}')">${s}</button>`).join('');
async function send(text){
  await fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({target:document.getElementById('target').value,text})});
}
async function tick(){
  try{
    const s=await (await fetch('/api/state')).json();
    document.getElementById('gws').textContent=s.connected?'(gateway connected)':'(gateway NOT connected)';
    const sel=document.getElementById('target'), cur=sel.value;
    const ids=s.bots.filter(b=>b.id).map(b=>b.id);
    sel.innerHTML='<option value="*">all bots</option>'+ids.map(i=>`<option>${i}</option>`).join('');
    sel.value=(ids.includes(cur)||cur=='*')?cur:'*';
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


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)
