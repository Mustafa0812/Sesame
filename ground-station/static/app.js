(() => {
'use strict';
const $ = id => document.getElementById(id);
const css = v => getComputedStyle(document.documentElement).getPropertyValue(v).trim();
const DEG = Math.PI / 180;
const clamp = (v, a, b) => Math.min(b, Math.max(a, v));

/* ---------------------------------------------------------------- helpers */
function post(url, body) {
  return fetch(url, { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body || {}) })
    .then(async r => {
      const j = await r.json().catch(() => ({}));
      if (!r.ok || j.ok === false) throw new Error(j.error || 'Request refused');
      return j;
    });
}
let toastTimer;
function toast(msg) {
  const t = $('toast'); t.textContent = msg; t.classList.add('show');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.classList.remove('show'), 3500);
}
const act = (url, body) => post(url, body).catch(e => { toast(e.message); throw e; });

/* theme */
(function theme() {
  let saved = null; try { saved = localStorage.getItem('theme'); } catch (e) {}
  if (saved) document.documentElement.dataset.theme = saved;
  $('themeBtn').onclick = () => {
    const light = (document.documentElement.dataset.theme || (matchMedia('(prefers-color-scheme: light)').matches ? 'light' : 'dark')) === 'light';
    const next = light ? 'dark' : 'light';
    document.documentElement.dataset.theme = next;
    try { localStorage.setItem('theme', next); } catch (e) {}
    setTimeout(restyleCharts, 50);
  };
})();

/* ---------------------------------------------------------------- bot model */
const STAND = [135, 45, 45, 135, 0, 180, 0, 180];             // firmware servo-index order: R1 R2 L1 L2 R3 R4 L4 L3
const LIFT_STAND = { 4: 0, 5: 180, 6: 0, 7: 180 };
// leg = hip servo + lift servo. Front/back inferred from the 'bow' pose (R1/L1 front) and the turn-gait pairing.
const LEGS = [
  { side: 1, front: 1, hip: 0, lift: 5 }, { side: 1, front: -1, hip: 1, lift: 4 },
  { side: -1, front: 1, hip: 2, lift: 6 }, { side: -1, front: -1, hip: 3, lift: 7 },
];
const HIP_X = 30, HIP_Z = 28.2, HIP_Y = 24, UPPER = 36, LOWER = 58, SERVO_SPEED = 420; // mm, deg/s
const GROUND_Y = HIP_Y - LOWER * Math.cos(25 * DEG);
const ROLE_COLOR = { L: 0xff9f45, F: 0x4f9cff, '?': 0x8b98a9 };
const stlCache = {};
const stlLoader = () => new THREE.STLLoader();
// STL (x,y,z) -> robot frame: X = -(z-2.4) lateral (+ right), Y = y up, Z = x-26 forward. Rotation only (no mirroring).
const STL_TO_ROBOT = new THREE.Matrix4().set(0, 0, -1, 2.4, 0, 1, 0, 0, 1, 0, 0, -26, 0, 0, 0, 1);
function loadStl(name) {
  return stlCache[name] || (stlCache[name] = new Promise((res, rej) =>
    stlLoader().load('/model/' + name + '.stl', g => { g.applyMatrix4(STL_TO_ROBOT); g.computeVertexNormals(); res(g); }, undefined, rej)));
}
let modelRot = 0;
try { modelRot = parseInt(localStorage.getItem('modelRot') || '0', 10) || 0; } catch (e) {}

function makeRenderer(el) {
  const r = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  r.setPixelRatio(Math.min(devicePixelRatio || 1, 1.5));
  el.appendChild(r.domElement);
  return r;
}
function fit(renderer, camera, el) {
  const w = el.clientWidth, h = el.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix();
}
function lights(scene) {
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445066, 0.9));
  const d = new THREE.DirectionalLight(0xffffff, 0.8); d.position.set(120, 220, 140); scene.add(d);
}

class Bot3D {
  constructor(el) {
    this.el = el;
    this.renderer = makeRenderer(el);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(36, 1, 1, 2000);
    this.camera.position.set(200, 120, 250);
    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0, 2, 0); this.controls.enableDamping = true; this.controls.maxPolarAngle = Math.PI * 0.52;
    lights(this.scene);
    const grid = new THREE.GridHelper(420, 21, 0x4a5568, 0x2e3748); grid.position.y = GROUND_Y - 1; grid.material.opacity = 0.5; grid.material.transparent = true;
    this.scene.add(grid);
    this.view = new THREE.Group();      // model-front rotation
    this.root = new THREE.Group();      // tilt
    this.view.add(this.root); this.scene.add(this.view);
    this.legMat = new THREE.MeshStandardMaterial({ color: ROLE_COLOR['?'], roughness: 0.5, metalness: 0.1 });
    this.legs = LEGS.map(L => this.buildLeg(L));
    this.cur = STAND.slice(); this.tgt = STAND.slice();
    this.tilt = { p: 0, r: 0 };
    this.pb = null;
    this.setRole('?');
    this.setFront(modelRot);
    ['frame', 'bottom', 'top'].forEach(n => loadStl(n).then(g => {
      const m = new THREE.MeshStandardMaterial({ color: n === 'top' ? 0xe9eef7 : n === 'frame' ? 0xaab6c8 : 0x7d8aa0, roughness: 0.6,
        metalness: 0.05, transparent: n === 'top', opacity: n === 'top' ? 0.55 : 1, side: THREE.DoubleSide });
      this.root.add(new THREE.Mesh(g, m));
    }).catch(() => {}));
    new ResizeObserver(() => fit(this.renderer, this.camera, this.el)).observe(el);
    fit(this.renderer, this.camera, el);
    this.apply();
  }
  buildLeg(L) {
    const g = new THREE.Group(); g.position.set(L.side * HIP_X, HIP_Y, L.front * HIP_Z);
    const cyl = (len, r) => new THREE.CylinderGeometry(r, r, len, 14);
    const upper = new THREE.Mesh(cyl(UPPER, 4.5), this.legMat); upper.rotation.z = Math.PI / 2; upper.position.x = UPPER / 2; g.add(upper);
    const hip = new THREE.Mesh(new THREE.SphereGeometry(6.5, 16, 12), this.legMat); g.add(hip);
    const knee = new THREE.Group(); knee.position.x = UPPER; g.add(knee);
    const lower = new THREE.Mesh(cyl(LOWER, 3.6), this.legMat); lower.position.y = -LOWER / 2; knee.add(lower);
    const foot = new THREE.Mesh(new THREE.SphereGeometry(5, 14, 10), this.legMat); foot.position.y = -LOWER; knee.add(foot);
    knee.add(new THREE.Mesh(new THREE.SphereGeometry(5.5, 14, 10), this.legMat));
    this.root.add(g);
    return { L, g, knee };
  }
  setRole(role) {
    if (this.role === role) return; this.role = role;
    this.legMat.color.setHex(ROLE_COLOR[role] || ROLE_COLOR['?']);
  }
  setFront(k) { this.view.rotation.y = k * Math.PI / 2; }
  play(seq, offsetMs) { this.pb = { seq, t0: performance.now() - (offsetMs || 0), i: 0 }; }
  stop() { this.pb = null; this.tgt = STAND.slice(); }
  progress() {
    if (!this.pb) return null;
    const el = performance.now() - this.pb.t0;
    return { name: this.pb.name, el, dur: this.pb.seq.duration };
  }
  setPose(pose) { this.pb = null; this.tgt = pose.slice(); }
  setTilt(p, r) { this.tilt.p = p; this.tilt.r = r; }
  step(dt) {
    if (this.pb) {
      const el = performance.now() - this.pb.t0, ev = this.pb.seq.events;
      while (this.pb.i < ev.length && ev[this.pb.i][0] <= el) { this.tgt[ev[this.pb.i][1]] = ev[this.pb.i][2]; this.pb.i++; }
      if (el > this.pb.seq.duration + 300) this.pb = null;
    }
    const mx = SERVO_SPEED * dt;
    for (let j = 0; j < 8; j++) this.cur[j] += clamp(this.tgt[j] - this.cur[j], -mx, mx);
    this.apply();
  }
  apply() {
    for (const { L, g, knee } of this.legs) {
      const fwd = (this.cur[L.hip] - 90) * (L.side > 0 ? 1 : -1);            // forward swing, degrees
      g.rotation.y = L.side > 0 ? -fwd * DEG : Math.PI + fwd * DEG;
      const fold = Math.abs(this.cur[L.lift] - LIFT_STAND[L.lift]);
      knee.rotation.z = clamp(25 + fold, 0, 118) * DEG;
    }
    const k = 0.25;
    this.root.rotation.x += (this.tilt.p * DEG - this.root.rotation.x) * k;
    this.root.rotation.z += (-this.tilt.r * DEG - this.root.rotation.z) * k;
  }
  render() { this.controls.update(); this.renderer.render(this.scene, this.camera); }
}

/* ---------------------------------------------------------------- IMU 3D */
class ImuView {
  constructor(el) {
    this.el = el; this.renderer = makeRenderer(el);
    this.scene = new THREE.Scene(); this.camera = new THREE.PerspectiveCamera(40, 1, 1, 1000);
    this.camera.position.set(130, 95, 165);
    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true; lights(this.scene);
    const R = 70; this.R = R;
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(R, 24, 16), new THREE.MeshBasicMaterial({ color: 0x4f9cff, wireframe: true, transparent: true, opacity: 0.12 }));
    this.scene.add(sphere);
    const ring = new THREE.Mesh(new THREE.RingGeometry(R - 0.6, R, 64), new THREE.MeshBasicMaterial({ color: 0x3fb97a, side: THREE.DoubleSide, transparent: true, opacity: 0.6 }));
    ring.rotation.x = Math.PI / 2; this.scene.add(ring);
    const axes = new THREE.AxesHelper(R * 1.25); axes.material.transparent = true; axes.material.opacity = 0.5; this.scene.add(axes);
    this.board = new THREE.Group();
    const body = new THREE.Mesh(new THREE.BoxGeometry(60, 6, 40), new THREE.MeshStandardMaterial({ color: 0x1f6f5c, roughness: 0.5 }));
    const top = new THREE.Mesh(new THREE.BoxGeometry(60.2, 0.6, 40.2), new THREE.MeshStandardMaterial({ color: 0x35c28f }));
    top.position.y = 3.2;
    const nose = new THREE.Mesh(new THREE.ConeGeometry(6, 14, 16), new THREE.MeshStandardMaterial({ color: 0xff9f45 }));
    nose.rotation.x = Math.PI / 2; nose.position.set(0, 3, 27);       // +Z = forward
    this.board.add(body, top, nose); this.scene.add(this.board);
    this.N = 360; this.pts = new Float32Array(this.N * 3); this.cols = new Float32Array(this.N * 3); this.count = 0;
    const lg = new THREE.BufferGeometry();
    lg.setAttribute('position', new THREE.BufferAttribute(this.pts, 3)); lg.setAttribute('color', new THREE.BufferAttribute(this.cols, 3));
    lg.setDrawRange(0, 0);
    this.trail = new THREE.Line(lg, new THREE.LineBasicMaterial({ vertexColors: true })); this.scene.add(this.trail);
    this.tip = new THREE.Mesh(new THREE.SphereGeometry(3.2, 12, 10), new THREE.MeshBasicMaterial({ color: 0xff9f45 })); this.scene.add(this.tip);
    this.p = 0; this.r = 0; this.v = new THREE.Vector3();
    new ResizeObserver(() => fit(this.renderer, this.camera, this.el)).observe(el); fit(this.renderer, this.camera, el);
  }
  set(p, r) {
    this.p += (p - this.p) * 0.35; this.r += (r - this.r) * 0.35;
    this.board.rotation.set(this.p * DEG, 0, -this.r * DEG);
    this.v.set(0, 1, 0).applyQuaternion(this.board.quaternion).multiplyScalar(this.R);
    this.tip.position.copy(this.v);
  }
  pushTrail() {
    if (this.count === this.N) { this.pts.copyWithin(0, 3); this.count--; }
    this.pts.set([this.v.x, this.v.y, this.v.z], this.count * 3); this.count++;
    for (let i = 0; i < this.count; i++) {                // older = darker
      const a = (i + 1) / this.count; this.cols.set([0.31 * a + 0.1, 0.61 * a + 0.1, 1 * a], i * 3);
    }
    const g = this.trail.geometry; g.attributes.position.needsUpdate = true; g.attributes.color.needsUpdate = true; g.setDrawRange(0, this.count);
  }
  render() { this.controls.update(); this.renderer.render(this.scene, this.camera); }
}

/* ---------------------------------------------------------------- charts */
const charts = {};
function lineChart(canvasId, color, unit, windowSec, extra) {
  const ctx = $(canvasId).getContext('2d');
  return new Chart(ctx, {
    type: 'line', data: { datasets: [] },
    options: Object.assign({
      animation: false, responsive: true, maintainAspectRatio: false, parsing: false, normalized: true,
      interaction: { intersect: false, mode: 'nearest', axis: 'x' },
      plugins: { legend: { display: !!(extra && extra.legend), labels: { color: css('--muted'), boxWidth: 10 } },
        tooltip: { callbacks: { title: i => (i[0].parsed.x).toFixed(0) + ' s ago', label: i => i.dataset.label + ': ' + i.parsed.y.toFixed(2) + ' ' + unit } } },
      scales: {
        x: { type: 'linear', min: -windowSec, max: 0, ticks: { color: css('--muted'), maxTicksLimit: 6, callback: v => windowSec > 120 ? (v / 60).toFixed(0) + ' min' : v + ' s' }, grid: { color: css('--line') } },
        y: { ticks: { color: css('--muted') }, grid: { color: css('--line') }, grace: '10%' },
      },
    }, {}),
  });
}
function restyleCharts() {
  for (const c of Object.values(charts)) {
    c.options.scales.x.ticks.color = c.options.scales.y.ticks.color = css('--muted');
    c.options.scales.x.grid.color = c.options.scales.y.grid.color = css('--line');
    c.options.plugins.legend.labels.color = css('--muted'); c.update('none');
  }
}
function setSeries(chart, defs) {
  chart.data.datasets = defs.map(d => ({ label: d.label, data: [], borderColor: d.color, backgroundColor: d.fill || 'transparent',
    borderWidth: 2, pointRadius: 0, tension: 0.25, fill: !!d.fill }));
}

/* ---------------------------------------------------------------- app state */
const state = { sequences: [], timelines: {}, slots: [null, null], target: '*', bots: [], frame: null, env: { temperature: [], pressure: [] }, imuHist: [] };
let botRender = [], imuView = null;

/* ---------- robot panels */
function slotName(i) { return 'Bot ' + 'AB'[i]; }
function buildPanels() {
  const host = $('bots'); host.innerHTML = '';
  for (let i = 0; i < 2; i++) {
    const el = document.createElement('div'); el.className = 'bot'; el.dataset.slot = i;
    el.innerHTML = `
      <div class="bot-head">
        <div class="bot-name"><span class="dot"></span><b>${slotName(i)}</b><code>waiting…</code><span class="tag unknown"></span></div>
        <div class="seg">
          <button data-role="leader" class="sm">Leader</button><button data-role="follower" class="sm">Follower</button>
        </div>
      </div>
      <div class="viewport"><div class="overlay"><span class="seqname">Idle</span><span class="seqtime"></span></div></div>
      <div class="progress"><i></i></div>
      <div class="bot-stats">
        <div class="stat"><span>Battery</span><b class="s-pct">–</b><div class="batt"><i style="width:0"></i></div></div>
        <div class="stat"><span>Voltage</span><b class="s-v">–</b></div>
        <div class="stat"><span>Current</span><b class="s-ma">–</b></div>
        <div class="stat"><span>Signal</span><b class="s-rssi">–</b></div>
      </div>`;
    host.appendChild(el);
    el.querySelectorAll('[data-role]').forEach(b => b.onclick = () => {
      const id = state.slots[i]; if (!id) return toast(slotName(i) + ' has not reported yet');
      act('/api/cmd', { target: id, text: b.dataset.role }).catch(() => {});
    });
    try { botRender[i] = new Bot3D(el.querySelector('.viewport')); }
    catch (e) { el.querySelector('.viewport').innerHTML = '<div class="placeholder">3D view needs WebGL</div>'; }
  }
}
function botBySlot(i) { return state.bots.find(b => b.id === state.slots[i]); }
function roleOf(b) { return b ? (b.role === 'L' ? 'leader' : b.role === 'F' ? 'follower' : 'unknown') : 'unknown'; }

function assignSlots(bots) {
  for (const b of bots) if (!state.slots.includes(b.id)) { const k = state.slots.indexOf(null); if (k >= 0) state.slots[k] = b.id; }
}

let lastPanel = 0;
function updatePanels(f) {
  const now = performance.now(); if (now - lastPanel < 200) return; lastPanel = now;
  const tags = [];
  for (let i = 0; i < 2; i++) {
    const el = $('bots').children[i], b = botBySlot(i), role = roleOf(b);
    const piDriven = b && f.body.enabled && f.body.target === b.id;
    el.className = 'bot ' + (b ? role : '');
    el.querySelector('.dot').className = 'dot' + (b && b.online ? ' on' : '');
    el.querySelector('code').textContent = b ? b.id : 'waiting…';
    const tag = el.querySelector('.tag'); tag.className = 'tag ' + role;
    tag.innerHTML = b ? (role === 'unknown' ? 'No role' : role) + (piDriven ? ' <small>· Pi-driven</small>' : '') : 'Offline';
    tags.push(`<span class="tag ${role}">${slotName(i)} · ${b ? (role === 'unknown' ? 'no role' : role) : 'offline'}${piDriven ? ' <small>· Pi-driven</small>' : ''}</span>`);
    const pct = b && b.pct != null && +b.pct >= 0 ? +b.pct : null;
    el.querySelector('.s-pct').textContent = pct == null ? '–' : pct + '%';
    el.querySelector('.batt i').style.width = (pct || 0) + '%';
    el.querySelector('.batt i').style.background = pct != null && pct < 20 ? 'var(--bad)' : pct != null && pct < 40 ? 'var(--warn)' : 'var(--good)';
    el.querySelector('.s-v').textContent = b && b.volts ? (+b.volts).toFixed(2) + ' V' : '–';
    el.querySelector('.s-ma').textContent = b && b.ma ? Math.round(+b.ma) + ' mA' : '–';
    el.querySelector('.s-rssi').textContent = b ? b.rssi + ' dBm' : '–';
    el.querySelectorAll('[data-role]').forEach(x => x.classList.toggle('on', b && x.dataset.role === role));
    if (botRender[i]) botRender[i].setRole(b ? (b.role === 'L' || b.role === 'F' ? b.role : '?') : '?');
  }
  $('fleetTags').innerHTML = tags.join('');
}

/* ---------- sequence events: leader runs it, followers mirror it */
function handleEvent(e) {
  const bots = state.bots; if (!bots.length) return;
  const f = state.frame, targets = bots.filter(b => e.target === '*' || e.target === b.id);
  const leaders = targets.filter(b => b.role !== 'F');
  const mirror = bots.filter(b => b.role === 'F' && !(f && f.body.enabled && f.body.target === b.id));
  const all = new Set([...leaders, ...(leaders.length ? mirror : [])].map(b => b.id));
  for (const id of all) {
    const v = botRender[state.slots.indexOf(id)]; if (!v) continue;
    if (e.name === 'stop') { v.stop(); continue; }
    const seq = state.timelines[e.name]; if (!seq) continue;
    v.play(seq, e.offset || 0); v.pb.name = e.name;
  }
}
function updateProgress() {
  for (let i = 0; i < 2; i++) {
    const el = $('bots').children[i], v = botRender[i]; if (!v) continue;
    const p = v.progress();
    el.querySelector('.seqname').textContent = p ? p.name : 'Idle';
    el.querySelector('.seqtime').textContent = p ? (p.el / 1000).toFixed(1) + ' / ' + (p.dur / 1000).toFixed(1) + ' s' : '';
    el.querySelector('.progress i').style.width = p ? clamp(p.el / p.dur * 100, 0, 100) + '%' : '0';
  }
  const running = new Set(botRender.map(v => v && v.progress() && v.progress().name));
  document.querySelectorAll('.seq').forEach(b => b.classList.toggle('running', running.has(b.dataset.seq)));
}

/* ---------- selects, target, controls */
function syncSelects(f) {
  const opts = state.bots.map(b => `<option value="${b.id}">${slotName(state.slots.indexOf(b.id))} (${b.id})</option>`).join('') || '<option value="">no bots yet</option>';
  for (const id of ['bodyTarget', 'walkTarget']) {
    const s = $(id), cur = s.value;
    if (s.dataset.sig !== opts) { s.innerHTML = opts; s.dataset.sig = opts; if (cur) s.value = cur; }
  }
  if (f.body.enabled && f.body.target) $('bodyTarget').value = f.body.target;
}
function renderTargetSeg() {
  const seg = $('targetSeg'); const labels = [['*', 'All']].concat(state.slots.map((id, i) => id ? [id, slotName(i)] : null).filter(Boolean));
  const sig = JSON.stringify(labels) + state.target; if (seg.dataset.sig === sig) return; seg.dataset.sig = sig;
  seg.innerHTML = labels.map(([id, n]) => `<button class="${id === state.target ? 'on' : ''}" data-id="${id}">${n}</button>`).join('');
  seg.querySelectorAll('button').forEach(b => b.onclick = () => { state.target = b.dataset.id; seg.dataset.sig = ''; renderTargetSeg(); });
}
function updateControls(f) {
  const body = f.body, sw = $('bodySwitch');
  if (document.activeElement !== sw) sw.checked = body.enabled;
  $('bodyStatus').textContent = body.enabled ? `${body.status} → ${body.target}` : (f.tilt ? 'Off · tilt source: ' + f.tilt.src : 'Off · no tilt data');
  $('bodyStatus').classList.toggle('live', body.enabled);
  if (!state.sliderBusy) { $('gain').value = Math.round(body.gain * 10); $('gainV').textContent = body.gain.toFixed(1); $('maxDrop').value = body.max_drop; $('maxV').textContent = Math.round(body.max_drop); }
  $('walkBtn').textContent = f.walk.enabled ? 'Disable walk mode' : 'Enable walk mode';
  $('walkStatus').textContent = f.walk.enabled ? `${f.walk.status} → ${f.walk.target}` : 'Off';
  $('walkStatus').classList.toggle('live', f.walk.enabled);
}

/* ---------- frame handler */
let lastImuChart = 0;
function bootError(msg) { if (window.__boot) window.__boot(msg, 1); }

function onFrame(f) {
  $('boot').classList.add('done');
  state.frame = f; state.bots = f.bots; assignSlots(f.bots);
  const online = f.bots.filter(b => b.online).length;
  $('subtitle').textContent = `${online} of ${f.bots.length || 0} bots online · ${state.demo ? 'demo mode' : 'live'}`;
  setChip('chipGateway', f.gateway.connected, 'Gateway'); setChip('chipTilt', !!f.tilt, f.tilt ? 'IMU · ' + f.tilt.src : 'IMU');
  setChip('chipEnv', f.env.temperature != null, 'BMP388');
  for (const e of f.events) handleEvent(e);
  // body-tilt streaming drives the target bot's simulated pose + lean; otherwise the bot lies level
  for (let i = 0; i < 2; i++) {
    const v = botRender[i]; if (!v) continue;
    const driven = f.pose && f.body.enabled && f.body.target === state.slots[i];
    if (driven) { v.setPose(f.pose); v.setTilt(f.tilt ? f.tilt.pitch : 0, f.tilt ? f.tilt.roll : 0); }
    else v.setTilt(0, 0);
  }
  updatePanels(f); syncSelects(f); renderTargetSeg(); updateControls(f);
  if (f.tilt) {
    imuView && imuView.set(f.tilt.pitch, f.tilt.roll);
    $('pitchV').textContent = f.tilt.pitch.toFixed(1) + '°'; $('rollV').textContent = f.tilt.roll.toFixed(1) + '°';
    const now = f.t;
    if (now - lastImuChart > 0.1) {
      lastImuChart = now; state.imuHist.push([now, f.tilt.pitch, f.tilt.roll]);
      while (state.imuHist.length && now - state.imuHist[0][0] > 30) state.imuHist.shift();
      imuView && imuView.pushTrail();
    }
  } else { $('pitchV').textContent = $('rollV').textContent = '–'; }
  if (f.env.temperature != null) $('tempV').innerHTML = f.env.temperature.toFixed(1) + '<small>°C</small>';
  if (f.env.pressure != null) $('presV').innerHTML = f.env.pressure.toFixed(1) + '<small>hPa</small>';
  state.env.latest = f.env;
}
function setChip(id, ok, text) { const c = $(id); c.classList.toggle('ok', !!ok); c.lastChild.textContent = text; }

/* ---------- periodic chart refresh */
function refreshCharts() {
  const now = Date.now() / 1000;
  const rel = (arr, k) => arr.map(p => ({ x: p[0] - now, y: p[k] }));
  for (const [name, chart] of [['temperature', charts.temp], ['pressure', charts.pres]]) {
    chart.data.datasets[0].data = state.env[name].map(p => ({ x: p[0] - now, y: p[1] }));
    const ys = state.env[name].map(p => p[1]);
    if (ys.length) { chart.options.scales.y.min = Math.min(...ys) - (name === 'pressure' ? 0.3 : 0.3); chart.options.scales.y.max = Math.max(...ys) + 0.3; }
    chart.update('none');
  }
  charts.imu.data.datasets[0].data = rel(state.imuHist, 1); charts.imu.data.datasets[1].data = rel(state.imuHist, 2);
  charts.imu.update('none');
}
function collectEnv() {
  const e = state.env.latest, now = Date.now() / 1000;
  if (!e) return;
  for (const k of ['temperature', 'pressure']) {
    if (e[k] == null) continue;
    const a = state.env[k]; a.push([now, e[k]]);
    while (a.length && now - a[0][0] > 900) a.shift();
  }
}

/* ---------------------------------------------------------------- wiring */
function wire() {
  $('stopBtn').onclick = () => act('/api/cmd', { target: state.target, text: 'stop' }).catch(() => {});
  $('swapRoles').onclick = async () => {
    const ids = state.slots.filter(Boolean); if (ids.length < 2) return toast('Need both bots online to swap roles');
    const [a, b] = ids.map(id => state.bots.find(x => x.id === id)); if (!a || !b) return;
    const [lead, fol] = a.role === 'L' ? [a, b] : [b, a];
    try { await post('/api/cmd', { target: lead.id, text: 'follower' }); await post('/api/cmd', { target: fol.id, text: 'leader' }); } catch (e) { toast(e.message); }
  };
  $('modelRot').value = String(modelRot);
  $('modelRot').onchange = e => { modelRot = +e.target.value; try { localStorage.setItem('modelRot', modelRot); } catch (x) {} botRender.forEach(v => v && v.setFront(modelRot)); };

  $('bodySwitch').onchange = e => {
    const on = e.target.checked;
    act('/api/body/enable', { enabled: on, target: $('bodyTarget').value }).catch(() => { e.target.checked = !on; });
  };
  let t; const push = () => { clearTimeout(t); t = setTimeout(() => { post('/api/body/config', { gain: $('gain').value / 10, max_drop: +$('maxDrop').value }).catch(() => {}); state.sliderBusy = false; }, 150); };
  $('gain').oninput = () => { state.sliderBusy = true; $('gainV').textContent = ($('gain').value / 10).toFixed(1); push(); };
  $('maxDrop').oninput = () => { state.sliderBusy = true; $('maxV').textContent = $('maxDrop').value; push(); };

  const walkCfg = () => { $('deadV').textContent = $('dead').value; post('/api/imu/config', { dead: +$('dead').value, invert_pitch: $('invP').checked, invert_roll: $('invR').checked, swap: $('swapA').checked }).catch(() => {}); };
  ['dead', 'invP', 'invR', 'swapA'].forEach(id => $(id).oninput = $(id).onchange = walkCfg);
  $('zeroBtn').onclick = () => act('/api/imu/zero').catch(() => {});
  $('walkBtn').onclick = () => act('/api/imu/enable', { enabled: !state.frame.walk.enabled, target: $('walkTarget').value }).catch(() => {});
}

function buildSequences() {
  const g = $('seqGrid'); g.innerHTML = '';
  for (const s of state.sequences) {
    const b = document.createElement('button'); b.className = 'seq'; b.textContent = s; b.dataset.seq = s;
    b.onclick = () => {
      if (state.target !== '*') { const bot = state.bots.find(x => x.id === state.target); if (bot && bot.role === 'F') return toast('That bot is a follower. Sequences run on the leader.'); }
      act('/api/cmd', { target: state.target, text: 'seq ' + s }).catch(() => {});
    };
    g.appendChild(b);
  }
}

async function init() {
  if (window.__boot) __boot('Step 3/4: fetching sequences and history…');
  const [cfg, seqs, hist] = await Promise.all([
    fetch('/api/config').then(r => r.json()), fetch('/static/data/sequences.json').then(r => r.json()), fetch('/api/history').then(r => r.json()).catch(() => ({})),
  ]);
  state.sequences = cfg.sequences; state.demo = cfg.demo; state.timelines = seqs.sequences;
  state.env.temperature = hist.temperature || []; state.env.pressure = hist.pressure || [];
  if (window.__boot) __boot('Step 4/4: building the 3D views, waiting for live data from the server…');
  buildPanels(); buildSequences(); wire();
  try { imuView = new ImuView($('imuView')); } catch (e) { $('imuView').innerHTML = '<div class="placeholder">3D view needs WebGL</div>'; }

  charts.temp = lineChart('tempChart', css('--accent-2'), '°C', 900); setSeries(charts.temp, [{ label: 'Temperature', color: css('--accent-2'), fill: 'rgba(255,159,69,.12)' }]);
  charts.pres = lineChart('presChart', css('--accent'), 'hPa', 900); setSeries(charts.pres, [{ label: 'Pressure', color: css('--accent'), fill: 'rgba(79,156,255,.12)' }]);
  charts.imu = lineChart('imuChart', css('--accent'), '°', 30, { legend: true });
  setSeries(charts.imu, [{ label: 'Pitch', color: css('--accent') }, { label: 'Roll', color: css('--accent-2') }]);
  charts.imu.options.scales.y.suggestedMin = -30; charts.imu.options.scales.y.suggestedMax = 30;

  setInterval(collectEnv, 1000); setInterval(refreshCharts, 1000);
  const es = new EventSource('/api/stream');
  es.onopen = () => setChip('chipLink', true, 'Live'); es.onerror = () => setChip('chipLink', false, 'Reconnecting');
  es.onmessage = m => { try { onFrame(JSON.parse(m.data)); } catch (e) { console.error(e); } };

  let last = performance.now(), trailT = 0;
  (function loop(t) {
    requestAnimationFrame(loop);
    if (document.hidden) { last = t; return; }
    const dt = Math.min((t - last) / 1000, 0.1); last = t;
    botRender.forEach(v => { if (v) { v.step(dt); v.render(); } });
    imuView && imuView.render(); updateProgress();
  })(last);
}
init().catch(e => { console.error(e); toast('Dashboard failed to start: ' + e.message); });
})();
