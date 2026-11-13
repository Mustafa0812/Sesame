(() => {
'use strict';
const $ = id => document.getElementById(id);
let palette = {};        // CSS colour tokens, read once (and again when the light/dark scheme changes)
const css = v => palette[v] || (palette[v] = getComputedStyle(document.documentElement).getPropertyValue(v).trim());
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

let F = null;            // latest frame from the server
let dirty = false;       // a new frame arrived and is not drawn yet
let clockOffset = 0;     // server time - browser time (s)

/* ---------------------------------------------------------------- helpers */
let toastTimer;
function toast(msg) {
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), 3500);
}
async function post(url, body) {
  let r, j = {};
  try {
    r = await fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}) });
    j = await r.json().catch(() => ({}));
  } catch (e) { toast('Ground station not reachable'); throw e; }
  if (!r.ok || j.ok === false) { const e = new Error(j.error || 'HTTP ' + r.status); toast(e.message); throw e; }
  return j;
}
function setText(el, s) { if (el._v !== s) { el.textContent = s; el._v = s; } }
function setClass(el, c) { if (el.className !== c) el.className = c; }
function chip(id, ok, label) { const c = $(id); setClass(c, 'chip' + (ok ? ' ok' : '')); setText(c.lastElementChild, label); }
const slotOf = id => { const b = F && F.bots.find(x => x.id === id); return b ? `${b.slot} (${id})` : id; };
function fitCanvas(c) {
  const r = c.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 1.5);
  const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
  if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
  const ctx = c.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, W: r.width, H: r.height };
}

/* ---------------------------------------------------------------- robots + leader choice */
let botSig = '', segSig = '';
function renderBots() {
  const bots = F.bots, host = $('bots');
  const sig = bots.map(b => b.id).join(',');
  if (sig !== botSig) {
    botSig = sig;
    host.innerHTML = bots.length ? bots.map(b => `
      <div class="bot" data-id="${b.id}">
        <div class="bot-head"><span class="dot"></span><b>${b.slot}</b><code>${b.id}</code><span class="tag"></span><span class="pending"></span></div>
        <div class="stats">
          <div class="stat"><span>Battery</span><b data-k="pct">–</b><div class="batt"><i></i></div></div>
          <div class="stat"><span>Voltage</span><b data-k="v">–</b></div>
          <div class="stat"><span>Current</span><b data-k="ma">–</b></div>
          <div class="stat"><span>Signal</span><b data-k="rssi">–</b></div>
          <div class="stat"><span>Running</span><b data-k="run">–</b></div>
        </div>
        <div class="warn" hidden></div>
      </div>`).join('') : '<p class="muted">Waiting for the bots to report (needs the gateway and the bot firmware running)…</p>';
  }
  for (const b of bots) {
    const el = host.querySelector(`[data-id="${CSS.escape(b.id)}"]`);
    if (!el) continue;
    const role = b.role === 'L' || b.role === 'F' ? b.role : 'U';
    setClass(el, `bot ${role}${b.online ? '' : ' off'}`);
    setClass(el.querySelector('.dot'), 'dot' + (b.online ? ' on' : ''));
    const tag = el.querySelector('.tag');
    setClass(tag, 'tag ' + role);
    setText(tag, role === 'L' ? 'Leader' : role === 'F' ? 'Follower' : 'No role yet');
    let pend = '';
    if (!b.online) pend = `offline (${Math.round(b.age)} s)`;
    else if (b.want && b.want !== b.role) pend = `switching to ${b.want === 'L' ? 'leader' : 'follower'}…`;
    setText(el.querySelector('.pending'), pend);
    const pct = b.pct != null && +b.pct >= 0 ? +b.pct : null;
    setText(el.querySelector('[data-k=pct]'), pct == null ? '–' : pct + '%');
    const bar = el.querySelector('.batt i'), w = (pct || 0) + '%', col = pct != null && pct < 20 ? 'var(--bad)' : pct != null && pct < 40 ? 'var(--warn)' : 'var(--good)';
    if (bar._w !== w + col) { bar.style.width = w; bar.style.background = col; bar._w = w + col; }
    setText(el.querySelector('[data-k=v]'), b.volts ? (+b.volts).toFixed(2) + ' V' : '–');
    setText(el.querySelector('[data-k=ma]'), b.ma ? Math.round(+b.ma) + ' mA' : '–');
    setText(el.querySelector('[data-k=rssi]'), b.rssi != null ? b.rssi + ' dBm' : '–');
    setText(el.querySelector('[data-k=run]'), b.run && b.run !== '-' ? b.run : 'idle');
    const warn = el.querySelector('.warn');
    const msg = b.rebooted_ago != null && b.rebooted_ago < 180
      ? `⚠ Rebooted ${b.rebooted_ago} s ago (reset reason: ${b.rst || 'unknown'}). A brownout during sequences means the battery or servo supply is dipping.` : '';
    setText(warn, msg); warn.hidden = !msg;
  }
}
function renderLeaderSeg() {
  const bots = F.bots;
  const sig = bots.map(b => `${b.id}:${b.online}`).join('|') + `#${F.leader}#${F.body.enabled}`;
  if (sig === segSig) return;
  segSig = sig;
  const seg = $('leaderSeg');
  if (!bots.length) { seg.innerHTML = '<button disabled>no bots yet</button>'; return; }
  seg.innerHTML = bots.map(b => `<button data-id="${b.id}" class="${!F.body.enabled && b.id === F.leader ? 'on' : ''}"
      ${b.online ? '' : 'disabled title="offline"'}>${b.slot}</button>`).join('')
    + (F.body.enabled ? '<button class="on" disabled title="Body tilt is on: the Pi leads">Pi (body tilt)</button>' : '');
  seg.querySelectorAll('button[data-id]').forEach(btn => btn.onclick = () => post('/api/leader', { id: btn.dataset.id }).catch(() => {}));
}

/* ---------------------------------------------------------------- sequences */
function buildSequences(names) {
  $('seqGrid').innerHTML = names.map(n => `<button class="seq" data-seq="${n}">${n}</button>`).join('');
  $('seqGrid').querySelectorAll('.seq').forEach(b => b.onclick = () => post('/api/seq', { name: b.dataset.seq }).catch(() => {}));
}
function renderSequences() {
  const lead = F.bots.find(b => b.id === F.leader);
  const busy = F.bots.find(b => b.online && b.run && b.run !== '-');      // followers play along, so wait for every bot
  const running = busy ? busy.run : '';
  const switching = !!(lead && (lead.role !== 'L' || F.body.handover));
  const usable = !!(lead && lead.online && !F.body.enabled && !running && !switching);
  $('seqGrid').querySelectorAll('.seq').forEach(b => {
    if (b.disabled === usable) b.disabled = !usable;
    b.classList.toggle('running', b.dataset.seq === running);
  });
  const c = F.cmd, el = $('cmdStatus');
  let txt, cls = '';
  if (F.body.enabled) txt = 'Body tilt is on. Switch it off to run sequences.';
  else if (!lead) txt = 'Choose a leader above to run sequences.';
  else if (!lead.online) { txt = `The leader ${slotOf(lead.id)} is offline.`; cls = 'bad'; }
  else if (switching) { txt = `${slotOf(lead.id)} is switching to leader…`; cls = 'wait'; }
  else if (!c) txt = 'The leader runs the sequence and the follower plays it in step.';
  else {
    const who = c.target === '*' ? 'all bots' : slotOf(c.target);
    if (c.state === 'sent') { txt = `Sending '${c.text}' to ${who}…` + (c.tries > 1 ? ` (try ${c.tries} of 3)` : ''); cls = 'wait'; }
    else if (c.state === 'acked') { txt = `✓ ${who} received '${c.text}' (${Math.max(0, ...Object.values(c.acked))} ms)` + (running ? ` · running ${running}` : ''); cls = 'ok'; }
    else if (c.state === 'rejected') { txt = `✗ '${c.text}' — ${c.error}`; cls = 'bad'; }
    else { txt = `✗ '${c.text}' — ${c.error || 'no reply'}. Check the bot is on and in range.`; cls = 'bad'; }
  }
  setText(el, txt); setClass(el, 'cmd-status ' + cls);
}

/* ---------------------------------------------------------------- body tilt */
let sliderBusy = false, switchBusy = false;
function renderBody() {
  const b = F.body, sw = $('bodySwitch');
  if (!switchBusy && sw.checked !== b.enabled) sw.checked = b.enabled;
  const txt = b.enabled ? `On · ${b.status}` : b.status === 'returning to stand' ? 'Off · returning to stand'
    : F.tilt ? 'Off · tilt data arriving' : 'Off · no tilt data (start imu_publisher.py)';
  setText($('bodyStatus'), txt); setClass($('bodyStatus'), 'status' + (b.enabled ? ' live' : ''));
  if (!sliderBusy) {
    const g = String(Math.round(b.gain * 10)), m = String(Math.round(b.max_drop));
    if ($('gain').value !== g) $('gain').value = g;
    if ($('maxDrop').value !== m) $('maxDrop').value = m;
    setText($('gainV'), b.gain.toFixed(1)); setText($('maxV'), m);
  }
}

/* ---------------------------------------------------------------- IMU 3D (plain 2D canvas, no WebGL) */
const view = { yaw: 2.5, pitch: 0.45 };     // seen from behind and above, the way you hold the IMU
const trail = [];
let imuDirty = true, lastTiltKey = '';
function rotTilt(v, p, r) {            // pitch + = front (+Z) down, roll + = right (+X) down
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p);
  const x1 = v[0] * cr + v[1] * sr, y1 = -v[0] * sr + v[1] * cr, z1 = v[2];
  return [x1, y1 * cp - z1 * sp, y1 * sp + z1 * cp];
}
function project(v, W, H) {
  const cy = Math.cos(view.yaw), sy = Math.sin(view.yaw), cx = Math.cos(view.pitch), sx = Math.sin(view.pitch);
  const x = v[0] * cy + v[2] * sy, z = -v[0] * sy + v[2] * cy, y = v[1];
  const y2 = y * cx - z * sx, z2 = y * sx + z * cx;
  const d = 4.2, s = Math.min(W, H) * 0.36 * d / (d - z2);
  // world axes x right, y up, z front form a left-handed set: flip x on screen so the drawing is not a mirror image
  return [W / 2 - x * s, H / 2 + 14 - y2 * s, z2];
}
function drawImu() {
  const { ctx, W, H } = fitCanvas($('imuCanvas'));
  ctx.clearRect(0, 0, W, H);
  const muted = css('--muted'), line = css('--line');
  const ring = (f, color, width) => {
    ctx.beginPath();
    for (let i = 0; i <= 72; i++) { const a = i / 72 * Math.PI * 2, q = project(f(a), W, H); i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1]); }
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke();
  };
  ring(a => [Math.cos(a), Math.sin(a), 0], line, 1);
  ring(a => [0, Math.sin(a), Math.cos(a)], line, 1);
  ring(a => [Math.cos(a), 0, Math.sin(a)], css('--good'), 1.5);          // level ring
  const axis = (v, color, label) => {
    const o = project([0, 0, 0], W, H), q = project(v, W, H);
    ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(q[0], q[1]); ctx.strokeStyle = color; ctx.lineWidth = 1.2; ctx.stroke();
    ctx.fillStyle = color; ctx.font = '12px system-ui'; ctx.fillText(label, q[0] + 4, q[1] - 4);
  };
  axis([1.3, 0, 0], '#e5534b', 'right'); axis([0, 1.3, 0], css('--good'), 'up'); axis([0, 0, 1.3], css('--accent'), 'front');

  const t = F && F.tilt;
  if (!t) { ctx.fillStyle = muted; ctx.font = '13px system-ui'; ctx.fillText('No tilt data (imu_publisher.py → MQTT sesame/tilt)', 14, 22); return; }
  const p = t.pitch * Math.PI / 180, r = t.roll * Math.PI / 180;
  // trail of the up-vector tip
  for (let i = 1; i < trail.length; i++) {
    const a = project(trail[i - 1], W, H), b = project(trail[i], W, H);
    ctx.globalAlpha = 0.15 + 0.85 * i / trail.length;
    ctx.beginPath(); ctx.moveTo(a[0], a[1]); ctx.lineTo(b[0], b[1]); ctx.strokeStyle = css('--accent'); ctx.lineWidth = 2; ctx.stroke();
  }
  ctx.globalAlpha = 1;
  // board: box faces sorted back-to-front
  const hx = 0.58, hy = 0.06, hz = 0.4;
  const C = [[-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz], [-hx, -hy, hz], [hx, -hy, hz], [hx, hy, hz], [-hx, hy, hz]]
    .map(v => project(rotTilt(v, p, r), W, H));
  const faces = [[3, 2, 6, 7, '#35c28f'], [0, 1, 5, 4, '#145a46'], [4, 5, 6, 7, css('--accent-2')], [0, 3, 2, 1, '#1d6f57'], [1, 2, 6, 5, '#1f7d62'], [0, 4, 7, 3, '#1f7d62']];
  faces.map(f => ({ f, z: (C[f[0]][2] + C[f[1]][2] + C[f[2]][2] + C[f[3]][2]) / 4 })).sort((a, b) => a.z - b.z).forEach(({ f }) => {
    ctx.beginPath(); ctx.moveTo(C[f[0]][0], C[f[0]][1]);
    for (let k = 1; k < 4; k++) ctx.lineTo(C[f[k]][0], C[f[k]][1]);
    ctx.closePath(); ctx.fillStyle = f[4]; ctx.globalAlpha = 0.92; ctx.fill(); ctx.globalAlpha = 1;
    ctx.strokeStyle = 'rgba(0,0,0,.35)'; ctx.lineWidth = 1; ctx.stroke();
  });
  // nose arrow on top of the board, pointing to the front
  const n = [[-0.14, hy + 0.01, 0.05], [0.14, hy + 0.01, 0.05], [0, hy + 0.01, 0.36]].map(v => project(rotTilt(v, p, r), W, H));
  ctx.beginPath(); ctx.moveTo(n[0][0], n[0][1]); ctx.lineTo(n[1][0], n[1][1]); ctx.lineTo(n[2][0], n[2][1]); ctx.closePath();
  ctx.fillStyle = '#fff'; ctx.globalAlpha = 0.85; ctx.fill(); ctx.globalAlpha = 1;
  // up vector
  const up = rotTilt([0, 1, 0], p, r), o = project([0, 0, 0], W, H), tip = project(up, W, H);
  ctx.beginPath(); ctx.moveTo(o[0], o[1]); ctx.lineTo(tip[0], tip[1]); ctx.strokeStyle = css('--accent-2'); ctx.lineWidth = 2.5; ctx.stroke();
  ctx.beginPath(); ctx.arc(tip[0], tip[1], 5, 0, Math.PI * 2); ctx.fillStyle = css('--accent-2'); ctx.fill();
}
function renderImu() {
  const t = F.tilt;
  setText($('pitchV'), t ? t.pitch.toFixed(1) + '°' : '–'); setText($('rollV'), t ? t.roll.toFixed(1) + '°' : '–');
  const key = t ? t.pitch + ',' + t.roll : '';
  if (key !== lastTiltKey) {
    lastTiltKey = key; imuDirty = true;
    if (t) { trail.push(rotTilt([0, 1, 0], t.pitch * Math.PI / 180, t.roll * Math.PI / 180)); if (trail.length > 150) trail.shift(); }
  }
  if (imuDirty) { imuDirty = false; drawImu(); }
}
(function dragToRotate() {
  const c = $('imuCanvas'); let drag = null;
  c.addEventListener('pointerdown', e => { drag = [e.clientX, e.clientY]; c.setPointerCapture(e.pointerId); c.style.cursor = 'grabbing'; });
  c.addEventListener('pointermove', e => {
    if (!drag) return;
    view.yaw -= (e.clientX - drag[0]) * 0.01; view.pitch = clamp(view.pitch + (e.clientY - drag[1]) * 0.01, -1.2, 1.45);
    drag = [e.clientX, e.clientY]; drawImu();
  });
  const end = () => { drag = null; c.style.cursor = ''; };
  c.addEventListener('pointerup', end); c.addEventListener('pointercancel', end);
})();

/* ---------------------------------------------------------------- temperature / pressure charts */
const env = { temperature: [], pressure: [] };
const WINDOW_S = 1800;
let chartsDirty = true;
function drawChart(canvas, pts, color, digits) {
  const { ctx, W, H } = fitCanvas(canvas);
  ctx.clearRect(0, 0, W, H);
  const now = Date.now() / 1000 + clockOffset, data = pts.filter(p => now - p[0] <= WINDOW_S);
  ctx.font = '12px system-ui'; ctx.fillStyle = css('--muted');
  if (data.length < 2) { ctx.fillText(data.length ? 'Collecting data…' : 'Waiting for data (bmp388_publisher.py)…', 14, H / 2); return; }
  let lo = Infinity, hi = -Infinity;
  for (const p of data) { if (p[1] < lo) lo = p[1]; if (p[1] > hi) hi = p[1]; }
  if (hi - lo < 0.2) { const m = (hi + lo) / 2; lo = m - 0.1; hi = m + 0.1; }
  const pad = (hi - lo) * 0.15; lo -= pad; hi += pad;
  const L = 56, R = 12, T = 10, B = 24;
  const X = t => L + (W - L - R) * (1 - (now - t) / WINDOW_S), Y = v => T + (H - T - B) * (1 - (v - lo) / (hi - lo));
  ctx.strokeStyle = css('--line'); ctx.lineWidth = 1;
  for (let i = 0; i <= 3; i++) {
    const v = lo + (hi - lo) * i / 3, y = Math.round(Y(v)) + 0.5;
    ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - R, y); ctx.stroke(); ctx.fillText(v.toFixed(digits), 6, y + 4);
  }
  [[WINDOW_S, '-30 min'], [WINDOW_S * 2 / 3, '-20'], [WINDOW_S / 3, '-10'], [0, 'now']].forEach(([ago, lab]) => {
    const x = X(now - ago); ctx.fillText(lab, Math.min(W - R - 26, Math.max(L, x - 14)), H - 6);
  });
  // split where samples are missing (sensor stopped, tab was hidden) so a gap is not drawn as a straight line
  const runs = [[data[0]]];
  for (let i = 1; i < data.length; i++) {
    if (data[i][0] - data[i - 1][0] > 15) runs.push([]);
    runs[runs.length - 1].push(data[i]);
  }
  for (const run of runs) {
    ctx.beginPath();
    run.forEach((p, i) => { const x = X(p[0]), y = Y(p[1]); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.stroke();
    ctx.lineTo(X(run[run.length - 1][0]), H - B); ctx.lineTo(X(run[0][0]), H - B); ctx.closePath();
    ctx.globalAlpha = 0.12; ctx.fillStyle = color; ctx.fill(); ctx.globalAlpha = 1;
  }
}
function renderEnv() {
  for (const k of ['temperature', 'pressure']) {
    const v = F.env[k], a = env[k];
    if (v && (!a.length || v[0] > a[a.length - 1][0])) { a.push(v); if (a.length > 4000) a.shift(); chartsDirty = true; }
  }
  const t = F.env.temperature, p = F.env.pressure;
  const th = t ? t[1].toFixed(1) + '<small>°C</small>' : '–<small>°C</small>', ph = p ? p[1].toFixed(1) + '<small>hPa</small>' : '–<small>hPa</small>';
  if ($('tempV')._v !== th) { $('tempV').innerHTML = th; $('tempV')._v = th; }
  if ($('presV')._v !== ph) { $('presV').innerHTML = ph; $('presV')._v = ph; }
}
setInterval(() => { chartsDirty = true; }, 5000);   // keep the time axis moving

/* ---------------------------------------------------------------- log + header */
let logSeq = -1;
function renderLog() {
  if (F.log_seq === logSeq) return;
  logSeq = F.log_seq;
  $('log').textContent = F.log.slice().reverse().join('\n') || 'Nothing yet.';
}
function renderHeader(linkOk) {
  chip('chipLink', linkOk, linkOk ? 'Dashboard live' : 'Reconnecting…');
  if (!F) return;
  chip('chipGateway', F.gateway, 'Gateway');
  chip('chipMqtt', F.mqtt, 'MQTT');
  chip('chipTilt', !!F.tilt, 'IMU');
  chip('chipEnv', !!F.env.temperature, 'BMP388');
  const online = F.bots.filter(b => b.online).length;
  const lead = F.body.enabled ? 'the Pi leads (body tilt)' : F.leader ? 'leader ' + slotOf(F.leader) : 'no leader chosen';
  setText($('subtitle'), `${online} of ${F.bots.length} bots online · ${lead}`);
}

/* ---------------------------------------------------------------- main loop: draw only the newest frame */
let linkOk = false, lastFrameAt = 0, staleShown = false;
function render() {
  renderHeader(linkOk && Date.now() - lastFrameAt < 3000);
  renderLeaderSeg(); renderBots(); renderSequences(); renderBody(); renderImu(); renderEnv(); renderLog();
  if (chartsDirty) {
    chartsDirty = false;
    drawChart($('tempCanvas'), env.temperature, css('--accent-2'), 1);
    drawChart($('presCanvas'), env.pressure, css('--accent'), 1);
  }
}
function loop() {
  requestAnimationFrame(loop);
  if (dirty && F) { dirty = false; render(); }
  const stale = linkOk && lastFrameAt && Date.now() - lastFrameAt > 3000;   // stream open but no usable frames
  if (stale !== staleShown) { staleShown = stale; renderHeader(!stale && linkOk); if (stale) chip('chipLink', false, 'No data'); }
}

function wire() {
  $('stopBtn').onclick = () => post('/api/stop').catch(() => {});
  $('levelBtn').onclick = () => post('/api/level').then(() => { trail.length = 0; }).catch(() => {});
  $('bodySwitch').onchange = e => {
    const on = e.target.checked; switchBusy = true;
    post('/api/body', { enabled: on }).catch(() => { e.target.checked = !on; }).finally(() => { switchBusy = false; });
  };
  let timer;
  const push = () => { clearTimeout(timer); timer = setTimeout(() => { post('/api/body', { gain: $('gain').value / 10, max_drop: +$('maxDrop').value }).catch(() => {}).finally(() => { sliderBusy = false; }); }, 200); };
  $('gain').oninput = () => { sliderBusy = true; setText($('gainV'), ($('gain').value / 10).toFixed(1)); push(); };
  $('maxDrop').oninput = () => { sliderBusy = true; setText($('maxV'), $('maxDrop').value); push(); };
  window.addEventListener('resize', () => { imuDirty = true; chartsDirty = true; dirty = true; });
  matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { palette = {}; imuDirty = chartsDirty = dirty = true; });
}
async function loadHistory() {
  try {
    const h = await (await fetch('/api/history')).json();
    env.temperature = h.temperature || []; env.pressure = h.pressure || []; chartsDirty = true; dirty = true;
  } catch (e) { /* charts keep what they have */ }
}

async function init() {
  wire();
  try { const cfg = await (await fetch('/api/config')).json(); buildSequences(cfg.sequences); } catch (e) { toast('Could not load the sequence list'); }
  await loadHistory();
  // One live stream per visible tab. Hidden tabs let go of it: Chromium allows only 6 connections per
  // server, and open streams from background tabs would otherwise stall commands and page loads.
  let es = null, wasClosed = false, badFrames = 0;
  const open = () => {
    if (es) return;
    if (wasClosed) loadHistory();            // fill in the samples missed while the tab was hidden
    es = new EventSource('/api/stream');
    es.onopen = () => { linkOk = true; dirty = true; };
    es.onerror = () => { linkOk = false; renderHeader(false); };
    es.onmessage = m => {
      let f;
      try { f = JSON.parse(m.data); } catch (e) { if (++badFrames === 1) toast('Got an unreadable update from the ground station'); return; }
      if (!f || !f.bots) return;
      F = f; badFrames = 0; lastFrameAt = Date.now();
      clockOffset = F.t - Date.now() / 1000; dirty = true;
    };
  };
  const close = () => { if (es) { es.close(); es = null; linkOk = false; wasClosed = true; } };
  document.addEventListener('visibilitychange', () => (document.hidden ? close() : open()));
  if (!document.hidden) open();
  renderHeader(false); drawImu();
  requestAnimationFrame(loop);
}
init();
})();
