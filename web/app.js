import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

const ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'];
const GRIP = 'gripper_joint';
const TIP = 'link5';                 // точка, которой управляет джойстик
const LABEL = { joint1: 'j1', joint2: 'j2', joint3: 'j3', joint4: 'j4', joint5: 'j5', gripper_joint: 'губка' };
const DEG = 180 / Math.PI;

const state = {};                    // q, радианы
const limits = {};
let jointDefs = [];
const jointGroups = {};
const linkGroups = {};
let tipGroup = null;

// ---------------------------------------------------------------- 3D
const sceneHost = document.getElementById('scene');
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x24282d);

const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 20);
camera.up.set(0, 0, 1);              // у робота Z вверх, у three.js по умолчанию Y
camera.position.set(0.5, -0.9, 0.45);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(devicePixelRatio);
sceneHost.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0, 0.2);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.update();

// гизмо: маркер запястья, тянем оси — IK ведёт руку
const gizmo = new THREE.Mesh(new THREE.SphereGeometry(0.012, 16, 12),
  new THREE.MeshStandardMaterial({ color: 0x4f9dff, emissive: 0x123a63, roughness: 0.4 }));
scene.add(gizmo);
const tcontrols = new TransformControls(camera, renderer.domElement);
tcontrols.setMode('translate');
tcontrols.setSpace('world');
tcontrols.setSize(0.7);
tcontrols.attach(gizmo);
scene.add(tcontrols);
let gizmoTarget = null;
let gizmoDragging = false;
tcontrols.addEventListener('dragging-changed', (e) => {
  gizmoDragging = e.value;
  controls.enabled = !e.value;
});
tcontrols.addEventListener('objectChange', () => {
  if (gizmoDragging) gizmoTarget = gizmo.position.clone();
});

scene.add(new THREE.HemisphereLight(0xffffff, 0x2a2f36, 1.7));
const sun = new THREE.DirectionalLight(0xffffff, 1.9);
sun.position.set(0.7, -0.8, 1.2);
scene.add(sun);

const grid = new THREE.GridHelper(1, 10, 0x59616b, 0x343a41);
grid.rotation.x = Math.PI / 2;       // XY-плоскость, Z вверх
scene.add(grid);
scene.add(new THREE.AxesHelper(0.12));

function resize() {
  const w = sceneHost.clientWidth || 1;
  const h = sceneHost.clientHeight || 1;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
addEventListener('resize', resize);

// ---------------------------------------------------------------- URDF
const vec = (s) => (s ? s.trim().split(/\s+/).map(Number) : [0, 0, 0]);

function buildRobot(text) {
  const doc = new DOMParser().parseFromString(text, 'text/xml');
  const links = {};
  for (const el of doc.getElementsByTagName('link')) {
    const visual = el.getElementsByTagName('visual')[0];
    const box = visual?.getElementsByTagName('box')[0];
    if (!box) continue;
    const origin = visual.getElementsByTagName('origin')[0];
    links[el.getAttribute('name')] = {
      size: vec(box.getAttribute('size')),
      xyz: vec(origin?.getAttribute('xyz')),
      rpy: vec(origin?.getAttribute('rpy')),
    };
  }

  const joints = [];
  for (const el of doc.getElementsByTagName('joint')) {
    const origin = el.getElementsByTagName('origin')[0];
    const axis = el.getElementsByTagName('axis')[0];
    const limit = el.getElementsByTagName('limit')[0];
    joints.push({
      name: el.getAttribute('name'),
      type: el.getAttribute('type'),
      parent: el.getElementsByTagName('parent')[0]?.getAttribute('link'),
      child: el.getElementsByTagName('child')[0]?.getAttribute('link'),
      xyz: vec(origin?.getAttribute('xyz')),
      rpy: vec(origin?.getAttribute('rpy')),
      axis: vec(axis?.getAttribute('xyz') || '0 0 1'),
      lo: parseFloat(limit?.getAttribute('lower') ?? -1.5),
      hi: parseFloat(limit?.getAttribute('upper') ?? 1.5),
    });
  }

  const colors = [0x9aa7b1, 0xc2cad2, 0x8fb3d9, 0x7fa8c9, 0xb7c9a8, 0xd9c48f, 0xd9a48f, 0xa8b7c9];
  const byParent = new Map();
  for (const j of joints) {
    if (!byParent.has(j.parent)) byParent.set(j.parent, []);
    byParent.get(j.parent).push(j);
  }
  let colorIndex = 0;

  function addLink(name, parent) {
    const group = new THREE.Group();
    parent.add(group);
    linkGroups[name] = group;

    const v = links[name];
    if (v) {
      const material = new THREE.MeshStandardMaterial({
        color: colors[colorIndex++ % colors.length],
        roughness: 0.55,
        metalness: 0.15,
      });
      const mesh = new THREE.Mesh(new THREE.BoxGeometry(...v.size), material);
      mesh.position.set(...v.xyz);
      mesh.quaternion.setFromEuler(new THREE.Euler(...v.rpy));
      group.add(mesh);
    }

    for (const j of byParent.get(name) || []) {
      const jg = new THREE.Group();
      jg.position.set(...j.xyz);
      jg.userData.rpy = new THREE.Quaternion().setFromEuler(new THREE.Euler(...j.rpy));
      group.add(jg);
      jointGroups[j.name] = jg;
      jointDefs.push(j);
      if (j.type !== 'fixed') limits[j.name] = [j.lo, j.hi];
      addLink(j.child, jg);
    }
  }

  addLink('base_link', scene);
  tipGroup = linkGroups[TIP];
  makeSliders();
  applyState();
}

// ---------------------------------------------------------------- состояние
function setJoints(qmap) {
  for (const j of jointDefs) {
    const jg = jointGroups[j.name];
    if (!jg) continue;
    const q = j.type === 'fixed' ? 0 : (qmap[j.name] ?? 0);
    const axis = new THREE.Vector3(...j.axis).normalize();
    const rot = new THREE.Quaternion().setFromAxisAngle(axis, q);
    jg.quaternion.copy(jg.userData.rpy).multiply(rot);
  }
  scene.updateMatrixWorld(true);
}

function applyState() {
  setJoints(state);
  updateReadout();
}

let previewLine = null;

function showPreview(points) {
  clearPreview();
  if (points.length < 2) return;
  const line = new THREE.Line(new THREE.BufferGeometry().setFromPoints(points),
    new THREE.LineBasicMaterial({ color: 0x7fc4ff }));
  const end = new THREE.Mesh(new THREE.SphereGeometry(0.008, 12, 8),
    new THREE.MeshBasicMaterial({ color: 0x7fc4ff }));
  end.position.copy(points[points.length - 1]);
  line.userData.end = end;
  scene.add(line, end);
  previewLine = line;
}

function clearPreview() {
  if (!previewLine) return;
  scene.remove(previewLine);
  if (previewLine.userData.end) scene.remove(previewLine.userData.end);
  previewLine.geometry.dispose();
  previewLine.material.dispose();
  previewLine = null;
}

function tipPosition() {
  const p = new THREE.Vector3();
  if (tipGroup) tipGroup.getWorldPosition(p);
  return p;
}

function updateReadout() {
  const rows = [];
  for (const name of [...ARM, GRIP]) {
    rows.push(`<tr><td>${LABEL[name] ?? name}</td><td>${((state[name] ?? 0) * DEG).toFixed(1)}°</td></tr>`);
  }
  const p = tipPosition();
  rows.push(`<tr><td>x / y / z</td><td>${(p.x * 1000).toFixed(0)} / ${(p.y * 1000).toFixed(0)} / ${(p.z * 1000).toFixed(0)} мм</td></tr>`);
  document.getElementById('readout').innerHTML = rows.join('');
}

// ---------------------------------------------------------------- ползунки
function makeSliders() {
  const box = document.getElementById('panel-joints');
  box.innerHTML = '';
  for (const name of [...ARM, GRIP]) {
    const [lo, hi] = limits[name] ?? [-1.5, 1.5];
    const row = document.createElement('div');
    row.className = 'row';
    row.dataset.joint = name;
    row.innerHTML = `<label>${LABEL[name] ?? name}</label>
      <input type="range" min="${lo}" max="${hi}" step="0.005" value="${state[name] ?? 0}">
      <input type="number" class="num" step="1" value="${((state[name] ?? 0) * DEG).toFixed(1)}">`;
    const slider = row.querySelector('input[type=range]');
    const num = row.querySelector('input[type=number]');

    slider.addEventListener('input', () => {
      const value = parseFloat(slider.value);
      state[name] = value;
      num.value = (value * DEG).toFixed(1);
      applyState();
      queueJoint(name, value);
    });

    num.addEventListener('change', () => {
      let deg = parseFloat(num.value);
      if (!Number.isFinite(deg)) { num.value = ((state[name] ?? 0) * DEG).toFixed(1); return; }
      const rad = Math.min(hi, Math.max(lo, deg / DEG));
      state[name] = rad;
      slider.value = rad;
      num.value = (rad * DEG).toFixed(1);
      applyState();
      queueJoint(name, rad);
    });

    box.appendChild(row);
  }
}

function syncSliders() {
  for (const row of document.querySelectorAll('#panel-joints .row')) {
    if (row.contains(document.activeElement)) continue;   // не мешаем вводить
    const name = row.dataset.joint;
    const value = state[name] ?? 0;
    row.querySelector('input[type=range]').value = value;
    row.querySelector('input[type=number]').value = (value * DEG).toFixed(1);
  }
}

// ---------------------------------------------------------------- отправка
const pending = {};
let sendTimer = 0;

function queueJoint(name, value) {
  pending[name] = value;
  if (!sendTimer) sendTimer = setTimeout(flush, 50);
}

function flush() {
  sendTimer = 0;
  const batch = { ...pending };
  for (const key in pending) delete pending[key];
  for (const [name, value] of Object.entries(batch)) {
    post({ type: 'joint', name, value });
  }
}

async function postTo(url, body) {
  try {
    const res = await fetch(url, { method: 'POST', body: JSON.stringify(body) });
    return await res.json();
  } catch {
    return { ok: false, error: 'нет связи' };
  }
}

function post(body) {
  return postTo('/cmd', body);
}

// ---------------------------------------------------------------- связь
const statusEl = document.getElementById('status');
const dotEl = document.getElementById('dot');

let linkState = null;
function setLink(ok, text) {
  if (linkState === `${ok}|${text}`) return;
  linkState = `${ok}|${text}`;
  statusEl.textContent = text;
  dotEl.className = `dot ${ok ? 'on' : 'off'}`;
}

const events = new EventSource('/events');

events.onmessage = (e) => {
  const msg = JSON.parse(e.data);
  msg.names.forEach((name, i) => { state[name] = msg.positions[i]; });
  applyState();
  syncSliders();
  setLink(true, 'на связи');
};

events.onerror = () => setLink(false, 'нет связи');

// ---------------------------------------------------------------- режимы
let mode = 'joints';
let programRunning = false;
const tabs = document.querySelectorAll('.tabs button');
tabs.forEach((tab) => tab.addEventListener('click', () => {
  mode = tab.dataset.mode;
  tabs.forEach((t) => t.classList.toggle('active', t === tab));
  document.getElementById('panel-joints').hidden = mode !== 'joints';
  document.getElementById('panel-stick').hidden = mode !== 'stick';
}));

// ---------------------------------------------------------------- джойстик
const pad = document.getElementById('pad');
const knob = document.getElementById('knob');
let jx = 0;                          // вправо +
let jy = 0;                          // вниз + (экран)
let zvel = 0;

function stickMove(e) {
  const rect = pad.getBoundingClientRect();
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  let dx = (e.clientX - cx) / (rect.width / 2);
  let dy = (e.clientY - cy) / (rect.height / 2);
  const len = Math.hypot(dx, dy);
  if (len > 1) { dx /= len; dy /= len; }
  jx = Math.abs(dx) < 0.12 ? 0 : dx;
  jy = Math.abs(dy) < 0.12 ? 0 : dy;
  knob.style.left = `${50 + dx * 40}%`;
  knob.style.top = `${50 + dy * 40}%`;
}

function stickEnd() {
  jx = 0;
  jy = 0;
  knob.style.left = '50%';
  knob.style.top = '50%';
}

pad.addEventListener('pointerdown', (e) => {
  pad.setPointerCapture(e.pointerId);
  stickMove(e);
});
pad.addEventListener('pointermove', (e) => {
  if (pad.hasPointerCapture(e.pointerId)) stickMove(e);
});
pad.addEventListener('pointerup', stickEnd);
pad.addEventListener('pointercancel', stickEnd);

const zslider = document.getElementById('zslider');
zslider.addEventListener('input', () => { zvel = parseFloat(zslider.value); });
function zEnd() {
  zvel = 0;
  zslider.value = 0;
}
zslider.addEventListener('change', zEnd);
zslider.addEventListener('pointerup', zEnd);

// шаг за тик: 2 мм по X/Y, 1.75 мм по Z (20 Гц -> ~4 см/с)
// горизонталь стика двигает ось Y, вертикаль — ось X:
// так «вправо» на экране остаётся вправо, а «вверх» — от себя
let ikFails = 0;
setInterval(async () => {
  if (mode !== 'stick' || !tipGroup || programRunning || gizmoTarget) return;
  const vx = -jy * 0.002;            // стик вверх -> +X
  const vy = jx * 0.002;             // стик вправо -> +Y
  const vz = zvel * 0.00175;
  if (Math.hypot(vx, vy, vz) < 0.0002) return;
  const p = tipPosition();
  const res = await post({ type: 'pose', x: p.x + vx, y: p.y + vy, z: p.z + vz });
  if (res.ok) {
    ikFails = 0;
  } else if (++ikFails === 4) {
    setLink(true, 'IK не находит решение — точка вне зоны');
  }
}, 50);

// гизмо: ведём руку к маркеру шагами по 2 см
setInterval(async () => {
  if (!gizmoTarget || programRunning) return;
  const p = tipPosition();
  const d = gizmoTarget.clone().sub(p);
  if (d.length() < 0.003) { gizmoTarget = null; return; }
  d.clampLength(0, 0.02);
  const res = await post({ type: 'pose', x: p.x + d.x, y: p.y + d.y, z: Math.max(0.02, p.z + d.z) });
  if (!res.ok) gizmoTarget = null;
}, 60);

// ---------------------------------------------------------------- кнопки
document.getElementById('stop').addEventListener('click', async () => {
  stickEnd();
  zEnd();
  clearPreview();
  gizmoTarget = null;
  await postTo('/program/stop', {});
  await post({ type: 'stop' });
});

document.getElementById('home').addEventListener('click', async () => {
  stickEnd();
  zEnd();
  clearPreview();
  await post({ type: 'home' });
});

document.getElementById('open').addEventListener('click', () => {
  post({ type: 'grip', name: 'open' });
});

document.getElementById('close').addEventListener('click', () => {
  post({ type: 'grip', name: 'close' });
});


// ---------------------------------------------------------------- программа
const steps = [];
const stepsEl = document.getElementById('steps');
const progStatus = document.getElementById('prog-status');
const progSpeed = document.getElementById('prog-speed');
const progDesc = document.getElementById('prog-desc');
const AXES = ['j1', 'j2', 'j3', 'j4', 'j5'];
let stepCounter = 0;
let lastError = '';
let singleRow = null;

const esc = (s) => String(s).replace(/[&<>"']/g, (c) => (
  { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));

function renderSteps() {
  stepsEl.innerHTML = '';
  steps.forEach((step, i) => {
    const li = document.createElement('li');
    const head = `<div class="step-top">
        <input class="pname" value="${esc(step.name ?? `pos_${i + 1}`)}" spellcheck="false" title="имя — можно переименовать">
        <button data-act="run" title="выполнить только этот шаг">▶</button>
        ${step.type === 'wait' ? '' : '<button data-act="preview" title="показать траекторию">👁</button>'}
        <button data-act="dup" title="дублировать">⧉</button>
        <button data-act="up" title="выше">↑</button>
        <button data-act="down" title="ниже">↓</button>
        ${step.type === 'wait' ? '' : '<button data-act="take" title="заменить текущей позой">⟲</button>'}
        <button data-act="del" title="удалить">✕</button>
      </div>`;
    let body;
    if (step.type === 'wait') {
      body = `<div class="step-meta">
          <label class="mini">ждать
            <input class="ms" type="number" min="50" max="60000" step="50" value="${step.ms}"> мс
          </label>
        </div>`;
    } else {
      const q = step.joints.map((v, k) => `${AXES[k]} ${(v * DEG).toFixed(0)}`).join(' · ');
      body = `<div class="step-meta">
          <label class="mini">пауза
            <input class="pause" type="number" min="0" max="10" step="0.1" value="${step.pause.toFixed(1)}">
          </label>
          <label class="mini">скорость
            <input class="speed" type="number" min="0.1" max="1" step="0.05" value="${step.speed.toFixed(2)}">
          </label>
        </div>
        <div class="step-q">${q} · губка ${(step.gripper * DEG).toFixed(0)}°</div>`;
    }
    li.innerHTML = head + body;
    stepsEl.appendChild(li);
  });
  if (!programRunning) progStatus.textContent = `точек: ${steps.length}`;
}

function setRunUI(run) {
  programRunning = run;
  if (run) gizmoTarget = null;
  tcontrols.enabled = !run;
  document.querySelectorAll('#panel-joints input, #home, #open, #close, #add-step, #add-wait, #save-program, #load-program, #run-program, #prog-speed, #loop-program, #clear-program')
    .forEach((el) => { el.disabled = run; });
  document.getElementById('zslider').disabled = run;
  document.getElementById('pad').style.pointerEvents = run ? 'none' : '';
}

stepsEl.addEventListener('click', (e) => {
  if (programRunning) return;
  const btn = e.target.closest('button[data-act]');
  if (!btn) return;
  const i = [...stepsEl.children].indexOf(btn.closest('li'));
  const act = btn.dataset.act;
  if (act === 'run') { runOne(i); return; }
  if (act === 'preview') { previewStep(i); return; }
  if (act === 'dup') {
    stepCounter += 1;
    const copy = JSON.parse(JSON.stringify(steps[i]));
    copy.name = copy.type === 'wait' ? `пауза_${stepCounter}` : `pos_${stepCounter}`;
    steps.splice(i + 1, 0, copy);
    renderSteps();
    return;
  }
  if (act === 'del') steps.splice(i, 1);
  else if (act === 'up' && i > 0) [steps[i - 1], steps[i]] = [steps[i], steps[i - 1]];
  else if (act === 'down' && i < steps.length - 1) [steps[i], steps[i + 1]] = [steps[i + 1], steps[i]];
  else if (act === 'take') {
    steps[i].joints = ARM.map((n) => state[n] ?? 0);
    steps[i].gripper = state[GRIP] ?? 0.3;
  }
  renderSteps();
});

stepsEl.addEventListener('change', (e) => {
  if (programRunning) return;
  const row = e.target.closest('li');
  if (!row) return;
  const i = [...stepsEl.children].indexOf(row);
  const v = parseFloat(e.target.value);
  if (e.target.classList.contains('pname')) {
    steps[i].name = e.target.value.trim() || `pos_${i + 1}`;
    e.target.value = steps[i].name;
  } else if (e.target.classList.contains('pause')) {
    steps[i].pause = Number.isFinite(v) ? Math.max(0, Math.min(10, v)) : 0.3;
    e.target.value = steps[i].pause.toFixed(1);
  } else if (e.target.classList.contains('speed')) {
    steps[i].speed = Number.isFinite(v) ? Math.max(0.1, Math.min(1, v)) : 0.4;
    e.target.value = steps[i].speed.toFixed(2);
  } else if (e.target.classList.contains('ms')) {
    steps[i].ms = Number.isFinite(v) ? Math.max(50, Math.min(60000, v)) : 1000;
    e.target.value = steps[i].ms;
  }
});

function highlightRow(index) {
  [...stepsEl.children].forEach((li, i) => li.classList.toggle('running', i === index));
}

async function runOne(i) {
  if (programRunning) return;
  lastError = '';
  singleRow = i;
  clearPreview();
  const res = await postTo('/program/run', { steps: [steps[i]] });
  if (!res.ok) {
    singleRow = null;
    progStatus.textContent = `ошибка: ${res.error ?? 'не запустилось'}`;
    return;
  }
  setRunUI(true);
  progStatus.textContent = `точка ${i + 1}: ${steps[i].name}`;
}

async function previewStep(i) {
  if (programRunning) return;
  const res = await postTo('/program/plan', { step: steps[i] });
  if (!res.ok) { progStatus.textContent = `предпросмотр: ${res.error}`; return; }
  const orig = {};
  res.names.forEach((n) => { orig[n] = state[n]; });
  const pts = [];
  for (const p of res.points) {
    res.names.forEach((n, k) => { state[n] = p[k]; });
    setJoints(state);
    pts.push(tipPosition().clone());
  }
  res.names.forEach((n) => { state[n] = orig[n]; });
  setJoints(state);
  showPreview(pts);
  progStatus.textContent = `траектория: ${res.points.length} точек`;
}

document.getElementById('add-step').addEventListener('click', () => {
  if (programRunning) return;
  stepCounter += 1;
  steps.push({
    type: 'pose',
    name: `pos_${stepCounter}`,
    joints: ARM.map((n) => state[n] ?? 0),
    gripper: state[GRIP] ?? 0.3,
    pause: 0.3,
    speed: Math.max(0.1, Math.min(1, parseFloat(progSpeed.value) || 0.4)),
  });
  renderSteps();
});

document.getElementById('add-wait').addEventListener('click', () => {
  if (programRunning) return;
  stepCounter += 1;
  steps.push({ type: 'wait', name: `пауза_${stepCounter}`, ms: 1000 });
  renderSteps();
});

// --- библиотека последних программ
async function refreshLibrary() {
  try {
    const items = await (await fetch('/library')).json();
    const box = document.getElementById('lib-list');
    box.innerHTML = '';
    for (const it of items) {
      const li = document.createElement('li');
      const when = (it.saved || '').replace('T', ' ').slice(0, 16);
      li.innerHTML = `<div class="lib-item" data-path="${esc(it.path)}">
          <div class="lib-name">${esc(it.name)}</div>
          <div class="lib-desc">${esc(it.description || '')}</div>
          <div class="lib-date">${when}</div>
        </div>
        <button class="lib-del" data-path="${esc(it.path)}" title="убрать из списка">✕</button>`;
      box.appendChild(li);
    }
  } catch {
    /* сервер недоступен */
  }
}

document.getElementById('lib-list').addEventListener('click', async (e) => {
  const del = e.target.closest('.lib-del');
  if (del) {
    await postTo('/program/forget', { path: del.dataset.path });
    refreshLibrary();
    return;
  }
  const item = e.target.closest('.lib-item');
  if (item) loadPath(item.dataset.path);
});

// --- окно сохранения/загрузки (серверное, с папками)
const fsModal = document.getElementById('fs-modal');
const fsList = document.getElementById('fs-list');
const fsPath = document.getElementById('fs-path');
const fsName = document.getElementById('fs-name');
const fsTitle = document.getElementById('fs-title');
let fsCurrent = null;
let fsMode = 'save';
let lastFsPath = '';

function openFs(mode) {
  fsMode = mode;
  fsTitle.textContent = mode === 'load' ? 'Загрузить программу' : 'Сохранить программу';
  document.getElementById('fs-name').hidden = mode === 'load';
  document.getElementById('fs-save').hidden = mode === 'load';
  fsOpen(lastFsPath);
}

async function fsOpen(path) {
  const query = path ? `?path=${encodeURIComponent(path)}` : '';
  try {
    const res = await (await fetch(`/fs${query}`)).json();
    if (!res.ok) { progStatus.textContent = `ошибка: ${res.error}`; return; }
    fsCurrent = res;
    lastFsPath = res.path;
    fsPath.value = res.path;
    fsList.innerHTML = '';
    const up = document.createElement('div');
    up.className = 'fs-item dir';
    up.textContent = '↑ ..';
    up.addEventListener('click', () => fsOpen(res.parent));
    fsList.appendChild(up);
    for (const dir of res.dirs) {
      const el = document.createElement('div');
      el.className = 'fs-item dir';
      el.textContent = `📁 ${dir.name}`;
      el.addEventListener('click', () => fsOpen(dir.path));
      fsList.appendChild(el);
    }
    for (const file of res.files) {
      const el = document.createElement('div');
      el.className = 'fs-item file';
      el.textContent = `📄 ${file.name}`;
      el.addEventListener('click', () => {
        if (fsMode === 'load') loadPath(file.path);
        else fsName.value = file.name;
      });
      fsList.appendChild(el);
    }
    fsModal.hidden = false;
  } catch {
    progStatus.textContent = 'нет связи с сервером';
  }
}

function applyProgram(program) {
  steps.length = 0;
  (program.steps || []).forEach((s, i) => {
    if (s.type === 'wait') {
      steps.push({ type: 'wait', name: s.name || `пауза_${i + 1}`,
        ms: Math.max(50, Math.min(60000, Number(s.ms ?? 1000))) });
    } else {
      steps.push({ type: 'pose', name: s.name || `pos_${i + 1}`,
        joints: s.joints.map(Number), gripper: Number(s.gripper ?? 0.3),
        pause: Number(s.pause ?? 0.3),
        speed: Math.max(0.1, Math.min(1, Number(s.speed ?? 0.4))) });
    }
  });
  stepCounter = steps.length;
  renderSteps();
  progDesc.value = program.description || '';
}

async function loadPath(path) {
  try {
    const res = await (await fetch(`/program/load?path=${encodeURIComponent(path)}`)).json();
    if (!res.ok) { progStatus.textContent = `ошибка: ${res.error}`; return; }
    applyProgram(res.program);
    progStatus.textContent = `загружено: ${steps.length} точек`;
    fsModal.hidden = true;
    refreshLibrary();
  } catch {
    progStatus.textContent = 'ошибка загрузки';
  }
}

document.getElementById('save-program').addEventListener('click', () => {
  if (!steps.length) { progStatus.textContent = 'нет точек'; return; }
  openFs('save');
});
document.getElementById('load-program').addEventListener('click', () => openFs('load'));
document.getElementById('fs-up').addEventListener('click', () => {
  if (fsCurrent) fsOpen(fsCurrent.parent);
});
fsPath.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') fsOpen(fsPath.value);
});
document.getElementById('fs-cancel').addEventListener('click', () => { fsModal.hidden = true; });
fsModal.addEventListener('click', (e) => { if (e.target === fsModal) fsModal.hidden = true; });

document.getElementById('fs-save').addEventListener('click', async () => {
  if (!fsCurrent || fsMode !== 'save') return;
  const raw = fsName.value.trim() || 'program.json';
  const name = raw.endsWith('.json') ? raw : `${raw}.json`;
  const path = `${fsCurrent.path.replace(/\/+$/, '')}/${name}`;
  const res = await postTo('/program/save', {
    path,
    program: { description: progDesc.value, steps },
  });
  if (!res.ok) { progStatus.textContent = `ошибка: ${res.error ?? 'не сохранилось'}`; return; }
  fsModal.hidden = true;
  progStatus.textContent = `сохранено: ${res.path}`;
  refreshLibrary();
});

document.getElementById('run-program').addEventListener('click', async () => {
  if (!steps.length) { progStatus.textContent = 'нет точек'; return; }
  lastError = '';
  singleRow = null;
  clearPreview();
  const loop = document.getElementById('loop-program').checked;
  const res = await postTo('/program/run', { steps, loop });
  if (!res.ok) { progStatus.textContent = `ошибка: ${res.error ?? 'не запустилось'}`; return; }
  setRunUI(true);
  progStatus.textContent = loop ? 'выполняется по кругу' : `выполняется 1 из ${res.total}`;
});

document.getElementById('stop-program').addEventListener('click', async () => {
  clearPreview();
  await postTo('/program/stop', {});
});

document.getElementById('clear-program').addEventListener('click', () => {
  if (programRunning || !steps.length) return;
  if (!confirm('Очистить список точек?')) return;
  steps.length = 0;
  stepCounter = 0;
  renderSteps();
  progStatus.textContent = 'точек: 0';
});

async function pollProgram() {
  try {
    const st = await (await fetch('/program/status')).json();
    if (st.running) {
      if (!programRunning) setRunUI(true);
      highlightRow(singleRow !== null ? singleRow : st.index - 1);
      progStatus.textContent = st.name ? `выполняется: ${st.name}`
        : `выполняется ${st.index} из ${st.total}`;
    } else if (programRunning) {
      setRunUI(false);
      highlightRow(-1);
      singleRow = null;
      progStatus.textContent = `точек: ${steps.length}`;
    }
    if (!st.running && st.error && st.error !== lastError) {
      lastError = st.error;
      progStatus.textContent = `ошибка: ${st.error}`;
    }
  } catch {
    /* сервер недоступен */
  }
}

setInterval(pollProgram, 500);
renderSteps();
refreshLibrary();

// ---------------------------------------------------------------- 3D / логи
let viewMode = '3d';
const viewTabs = document.querySelectorAll('.view-tabs button');
viewTabs.forEach((tab) => tab.addEventListener('click', () => {
  viewMode = tab.dataset.view;
  viewTabs.forEach((t) => t.classList.toggle('active', t === tab));
  document.getElementById('scene').hidden = viewMode !== '3d';
  const logs = document.getElementById('logs');
  logs.hidden = viewMode !== 'logs';
  if (viewMode === '3d') {
    resize();
  } else {
    requestAnimationFrame(() => { logs.scrollTop = logs.scrollHeight; });
  }
}));

let lastSeq = 0;

async function pollLogs() {
  try {
    const res = await fetch(`/logs?since=${lastSeq}`);
    const items = await res.json();
    if (!items.length) return;
    const box = document.getElementById('logs');
    const atBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 30;
    for (const it of items) {
      lastSeq = Math.max(lastSeq, it.seq);
      const line = document.createElement('div');
      line.className = `log l${it.level}`;
      line.textContent = `${it.t}  ${it.name}: ${it.msg}`;
      box.appendChild(line);
    }
    while (box.childElementCount > 600) box.firstElementChild.remove();
    if (atBottom) box.scrollTop = box.scrollHeight;
  } catch {
    /* сервер недоступен — попробуем в следующий раз */
  }
}

setInterval(pollLogs, 1000);
pollLogs();

// ---------------------------------------------------------------- старт
resize();
renderer.setAnimationLoop(() => {
  controls.update();
  if (!gizmoDragging && !gizmoTarget) gizmo.position.copy(tipPosition());
  if (viewMode === '3d') renderer.render(scene, camera);
});

fetch('/robot.urdf')
  .then((r) => r.text())
  .then(buildRobot)
  .catch(() => setLink(false, 'не удалось прочитать robot.urdf'));
