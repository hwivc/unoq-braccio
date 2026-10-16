// Braccio dashboard: sign in, 3D arm from the URDF + live transforms, camera
// with the cubes it sees, sorting, teaching (web_teach), colours, a live log
// of every ROS node, manual moves and settings. Served by web_dashboard.py.

import * as THREE from './vendor/three.module.min.js';
import { STLLoader } from './vendor/STLLoader.js';
import { OrbitControls } from './vendor/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const store = {
  get(key, fallback) { try { return localStorage.getItem(key) ?? fallback; } catch { return fallback; } },
  set(key, value) { try { localStorage.setItem(key, value); } catch { /* private mode */ } },
};

// ---------------------------------------------------------------- api

async function api(path, body) {
  const options = body === undefined ? {} : {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', 'X-Braccio': '1' },
    body: JSON.stringify(body),
  };
  const response = await fetch(path, { credentials: 'same-origin', ...options });
  let data = {};
  try { data = await response.json(); } catch { /* empty */ }
  if (response.status === 401 && path !== '/api/login' && path !== '/api/join') showAuth();
  if (!response.ok) throw Object.assign(new Error(data.error || response.statusText), { status: response.status, data });
  return data;
}

let toastTimer;
function toast(text) {
  const el = $('toast');
  el.textContent = text;
  el.classList.remove('hidden');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => el.classList.add('hidden'), 3200);
}

// ---------------------------------------------------------------- appearance

function applyAppearance() {
  const theme = store.get('theme', 'system');
  if (theme === 'system') delete document.documentElement.dataset.theme;
  else document.documentElement.dataset.theme = theme;
  document.documentElement.dataset.solid = store.get('solid', 'false');
  setSegmented($('theme'), theme);
  $('solid').checked = store.get('solid', 'false') === 'true';
}

function setSegmented(group, value) {
  for (const b of group.querySelectorAll('button')) b.setAttribute('aria-checked', String(b.dataset.value === value));
}
function segmentedValue(group) {
  return group.querySelector('button[aria-checked="true"]')?.dataset.value;
}
function wireSegmented(group, onChange) {
  group.addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    setSegmented(group, b.dataset.value);
    onChange?.(b.dataset.value);
  });
}

// ---------------------------------------------------------------- sign in

function showAuth(needsOwner = false) {
  closeEvents();
  $('app').classList.add('hidden');
  $('auth').classList.remove('hidden');
  $('stage').classList.add('hidden');
  if (needsOwner) {
    $('join-title').textContent = 'Set Up Braccio';
    $('join-sub').textContent = 'Enter the pairing code shown in the dashboard terminal, then create the owner account.';
    $('show-login').classList.add('hidden');
    showForm('join');
  } else {
    showForm('login');
  }
}
function showForm(which) {
  $('login-form').classList.toggle('hidden', which !== 'login');
  $('join-form').classList.toggle('hidden', which !== 'join');
  (which === 'login' ? $('login-name') : $('join-code')).focus();
}

$('show-join').onclick = () => showForm('join');
$('show-login').onclick = () => showForm('login');
$('login-form').onsubmit = async (e) => {
  e.preventDefault();
  $('login-error').textContent = '';
  try {
    await api('/api/login', { name: $('login-name').value, password: $('login-password').value });
    $('login-password').value = '';
    start();
  } catch (err) { $('login-error').textContent = err.message; }
};
$('join-form').onsubmit = async (e) => {
  e.preventDefault();
  $('join-error').textContent = '';
  try {
    await api('/api/join', { code: $('join-code').value, name: $('join-name').value, password: $('join-password').value });
    $('join-password').value = '';
    start();
  } catch (err) { $('join-error').textContent = err.message; }
};

// ---------------------------------------------------------------- 3D view

const view = {
  renderer: null, scene: null, camera: null, controls: null,
  links: new Map(), ready: false, posed: false,
};

function rpyQuaternion([r, p, y]) {
  // URDF rpy = fixed-axis roll, pitch, yaw  ==  intrinsic Z-Y-X.
  return new THREE.Quaternion().setFromEuler(new THREE.Euler(r, p, y, 'ZYX'));
}

function initView() {
  if (view.renderer) return;
  const canvas = $('scene');
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;

  const scene = new THREE.Scene();
  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);       // ROS is Z-up
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 20);
  camera.up.set(0, 0, 1);
  const home = () => camera.position.set(0.55, -0.55, 0.42).multiplyScalar(window.innerWidth < 720 ? 1.45 : 1);
  home();

  scene.add(new THREE.HemisphereLight(0xffffff, 0x8899aa, 1.1));
  const key = new THREE.DirectionalLight(0xffffff, 2.2);
  key.position.set(0.6, -0.4, 1.2);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  key.shadow.camera.left = key.shadow.camera.bottom = -0.5;
  key.shadow.camera.right = key.shadow.camera.top = 0.5;
  key.shadow.radius = 6;
  scene.add(key);
  const rim = new THREE.DirectionalLight(0xbfd4ff, 0.8);
  rim.position.set(-0.8, 0.6, 0.5);
  scene.add(rim);

  const ground = new THREE.Mesh(new THREE.CircleGeometry(0.6, 64), new THREE.ShadowMaterial({ opacity: 0.22 }));
  ground.receiveShadow = true;
  scene.add(ground);
  const grid = new THREE.PolarGridHelper(0.5, 8, 5, 64, 0x8890a0, 0x8890a0);
  grid.rotation.x = Math.PI / 2;
  grid.material.transparent = true;
  grid.material.opacity = 0.18;
  scene.add(grid);

  const controls = new OrbitControls(camera, canvas);
  controls.target.set(0, 0, 0.16);
  controls.enableDamping = true;
  controls.minDistance = 0.25;
  controls.maxDistance = 2.5;
  controls.maxPolarAngle = Math.PI * 0.495;
  canvas.addEventListener('dblclick', () => {
    home();
    controls.target.set(0, 0, 0.16);
  });

  Object.assign(view, { renderer, scene, camera, controls });
  new ResizeObserver(resize).observe(canvas);
  resize();
  const loop = () => {
    requestAnimationFrame(loop);
    if (document.hidden) return;
    controls.update();
    renderer.render(scene, camera);
  };
  loop();
}

function resize() {
  const canvas = $('scene');
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (!w || !h) return;
  view.renderer.setSize(w, h, false);
  view.camera.aspect = w / h;
  // Centre the arm in the part of the screen the floating panel leaves free.
  const panel = $('panel');
  const inStage = canvas.parentElement === $('stage');
  if (inStage && panel && !$('app').classList.contains('hidden')) {
    const r = panel.getBoundingClientRect();
    const top = 80;                                   // below the status capsule
    let free = { x0: 0, x1: w, y0: top, y1: h - 80 }; // above the tab bar
    if (r.width < w * 0.6) free.x0 = Math.min(r.right, w * 0.5);   // panel at the side
    else free.y1 = Math.max(top + 120, r.top);                     // panel at the bottom
    view.camera.setViewOffset(w, h, w / 2 - (free.x0 + free.x1) / 2, h / 2 - (free.y0 + free.y1) / 2, w, h);
  } else {
    view.camera.clearViewOffset();
  }
  view.camera.updateProjectionMatrix();
}

async function loadScene() {
  const scene = await api('/api/scene');
  if (!Object.keys(scene.links).length) { setTimeout(loadScene, 2000); return; }
  const loader = new STLLoader();
  const geometries = new Map();
  const meshGeometry = (index) => {
    if (!geometries.has(index)) {
      geometries.set(index, new Promise((resolve, reject) => loader.load(`/api/mesh/${index}.stl`, (g) => {
        g.computeVertexNormals();
        resolve(g);
      }, undefined, reject)));
    }
    return geometries.get(index);
  };
  for (const group of view.links.values()) view.scene.remove(group);
  view.links.clear();
  for (const [name, visuals] of Object.entries(scene.links)) {
    const group = new THREE.Group();
    group.visible = false;
    view.scene.add(group);
    view.links.set(name, group);
    for (const v of visuals) {
      const [r, g, b, a] = v.color;
      const material = new THREE.MeshStandardMaterial({
        color: new THREE.Color(r, g, b), roughness: 0.42, metalness: 0.04,
        transparent: a < 1, opacity: a,
      });
      let geometry;
      const geo = v.geometry;
      if (geo.type === 'mesh') geometry = await meshGeometry(geo.mesh).catch(() => null);
      else if (geo.type === 'box') geometry = new THREE.BoxGeometry(...geo.size);
      else if (geo.type === 'sphere') geometry = new THREE.SphereGeometry(geo.radius, 24, 16);
      else if (geo.type === 'cylinder') {
        geometry = new THREE.CylinderGeometry(geo.radius, geo.radius, geo.length, 32);
        geometry.rotateX(Math.PI / 2);   // three: Y axis, URDF: Z axis
      }
      if (!geometry) continue;
      const mesh = new THREE.Mesh(geometry, material);
      if (geo.scale) mesh.scale.set(...geo.scale);
      mesh.position.set(...v.origin.slice(0, 3));
      mesh.quaternion.copy(rpyQuaternion(v.origin.slice(3)));
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
    }
  }
  view.ready = true;
}

function updatePoses(links) {
  let any = false;
  for (const [name, p] of Object.entries(links || {})) {
    const group = view.links.get(name);
    if (!group) continue;
    group.position.set(p[0], p[1], p[2]);
    group.quaternion.set(p[3], p[4], p[5], p[6]);
    group.visible = true;
    any = true;
  }
  view.posed = any;
}
// ---------------------------------------------------------------- app state

const app = {
  info: null, user: null, events: null, state: null,
  tab: 'sort', selected: new Set(), dragging: false, lastState: '', placed: [],
  cameraBig: false, cameraKind: 'camera',
  colorsKey: '', tableKey: '', editColors: false, addTimer: null, addPreview: null, addNameTyped: false,
  teachSeq: null, teachSession: '', teachDone: true, models: [], modelsFor: '', jogStep: 1,
};

const COLOR_CSS = {
  red: '#ff3b30', orange: '#ff9500', yellow: '#ffcc00', green: '#34c759', cyan: '#32ade6',
  blue: '#007aff', purple: '#af52de', magenta: '#ff2d92', pink: '#ff6fae', white: '#f2f2f7', black: '#1c1c1e',
};
const swatches = {};
const colorCss = (name) => swatches[name] || COLOR_CSS[String(name || '').replace(/\d+$/, '')] || '#8e8e93';
const nice = (name) => String(name).replace(/(\d+)$/, ' $1').replace(/^./, (c) => c.toUpperCase());
const el = (tag, className, text) => {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined) node.textContent = text;
  return node;
};
const reducedMotion = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;

const STEPS = {
  look: ['DETECTING', 'TARGET_CONFIRMED', 'GO_HOME'],
  reach: ['MOVE_ABOVE_CUBE', 'VERIFY_CUBE', 'DESCEND'],
  grip: ['GRASP', 'LIFT'],
  check: ['VERIFY_GRASP', 'VERIFY_FAILED'],
  drop: ['MOVE_TO_BIN', 'LOWER', 'RELEASE', 'RETREAT', 'VERIFY_PLACEMENT', 'PLACED'],
};
const ORDER = Object.keys(STEPS);

function describe(task) {
  const c = task.current || {};
  if (c.narration && !['ATTENTION', 'REST', 'ATTRACT'].includes(task.state)) return c.narration;
  const cube = c.cube || c.bin;
  const cubeText = cube ? `${nice(cube).toLowerCase()} cube` : 'cube';
  return ({
    STARTING: 'Starting…',
    GO_HOME: 'Moving clear of the camera',
    DETECTING: 'Looking for cubes',
    TARGET_CONFIRMED: `Found a ${cubeText}`,
    MOVE_ABOVE_CUBE: `Reaching for the ${cubeText}`,
    VERIFY_CUBE: 'Checking the colour',
    DESCEND: 'Lowering',
    GRASP: 'Gripping',
    LIFT: 'Lifting',
    VERIFY_GRASP: 'Checking it has the cube',
    VERIFY_FAILED: 'Missed — trying again',
    MOVE_TO_BIN: task.mode === 'exhibition' ? `Moving the ${cubeText} to a new spot` : `Carrying the ${cubeText}`,
    PLACED: 'Put it down',
    REST: 'Resting (servos cooling)',
    ATTRACT: `Doing a little ${c.trick || 'dance'}`,
    ATTENTION: 'Needs a hand',
    PATTERN_DONE: 'Pattern complete!',
    LOWER: 'Lowering',
    RELEASE: 'Letting go',
    RETREAT: 'Moving back',
    VERIFY_PLACEMENT: 'Checking the drop',
    COMPLETE: 'Done',
    STOPPED: 'Stopped',
    FAILED: 'Could not finish',
    DETECTION_FAILED: 'No cubes found',
    TARGET_UNREACHABLE: 'A cube is out of reach',
  })[task.state] || 'Ready';
}

const canControl = () => ['owner', 'operator'].includes(app.user?.role);
const driving = () => !!app.state?.control?.mine;

// ---------------------------------------------------------------- start

async function start() {
  let me;
  try { me = await api('/api/me'); } catch { setTimeout(start, 2000); return; }
  if (!me.user) { showAuth(me.needs_owner); return; }
  app.user = me.user;
  app.info = await api('/api/info');
  Object.assign(swatches, app.info.swatches || {});
  $('auth').classList.add('hidden');
  $('stage').classList.remove('hidden');
  $('app').classList.remove('hidden');
  document.body.dataset.role = app.user.role;
  $('avatar').textContent = app.user.user.slice(0, 1);
  buildSort();
  buildTrain();
  buildMove();
  if (!app.showWired) { wireShow(); wireCollect(); app.showWired = true; }
  buildSettings();
  initTerminal();
  applyCamera();
  if (location.pathname === '/show') startKiosk();
  initView();
  loadScene().catch(() => setTimeout(loadScene, 2000));
  openEvents();
  setTab(store.get('tab', 'sort'));
  moveTabIndicator();
  reframe();
}

async function refreshInfo() {
  try {
    app.info = await api('/api/info');
    fillSessions();
  } catch { /* offline for a moment */ }
}

function openEvents() {
  closeEvents();
  const events = new EventSource('/api/events');
  events.onmessage = (e) => onState(JSON.parse(e.data));
  events.onerror = () => {
    setStatus('Reconnecting…', '', 'warn');
    if (events.readyState === EventSource.CLOSED) setTimeout(start, 2000);
  };
  app.events = events;
}
function closeEvents() { app.events?.close(); app.events = null; }

// A hidden tab drops its live streams (browsers allow only ~6 connections per
// server, and it saves battery) and reconnects when it is shown again.
document.addEventListener('visibilitychange', () => {
  if (!app.user || $('app').classList.contains('hidden')) return;
  const cams = [$('camera-pip'), $('camera-full')];
  if (document.hidden) {
    closeEvents();
    for (const img of cams) if (img.getAttribute('src')) { img.dataset.wasStreaming = '1'; img.removeAttribute('src'); }
  } else {
    openEvents();
    for (const img of cams) if (img.dataset.wasStreaming) { delete img.dataset.wasStreaming; img.src = streamUrl(); }
  }
});

// ---------------------------------------------------------------- live state

function setStatus(title, sub, kind) {
  $('status-title').textContent = title;
  $('status-sub').textContent = sub;
  $('status-dot').className = `dot ${kind}`;
}

function onState(s) {
  const previous = app.state;
  app.state = s;
  updatePoses(s.links);
  appendLogs(s.logs);
  renderColors(s.colors || []);
  renderTable(s);
  renderModelLine(s.selected);
  renderTrain(s);
  renderCollect(s);
  renderShow(s);
  updateWindow(s.exhibit?.window_yaw);
  if (app.kiosk) renderKiosk(s);

  // stage placeholder
  const waiting = !view.posed;
  $('stage-empty').classList.toggle('hidden', !waiting || app.cameraBig);
  $('stage-empty-text').textContent = s.mode === 'offline'
    ? (armDev ? `${armDev.detail || 'The arm is not connected.'} See Settings › Devices.`
      : 'The arm is not connected. Start real.launch.py or sim.launch.py.')
    : 'Loading the 3D model…';

  // status capsule
  const modeText = { real: 'Real arm', sim: 'Simulation', offline: 'Not connected' }[s.mode];
  const controlText = s.control ? (s.control.mine ? 'You have control' : `${s.control.user} has control`) : '';
  const sub = [modeText, controlText].filter(Boolean).join(' · ');
  const teaching = s.teach && !s.teach.done;
  const armDev = s.devices?.arm;
  if (armDev && armDev.state === 'searching') setStatus('No arm connected', armDev.detail || sub, 'bad');
  else if (armDev && armDev.state === 'starting') setStatus('Arm powering up…', sub, 'busy');
  else if (s.mode === 'offline') setStatus('Arm offline', sub, 'bad');
  else if (s.task.running) setStatus(describe(s.task), sub, 'busy');
  else if (teaching) setStatus(`${s.teach.kind === 'collect' ? 'Collecting' : 'Teaching'} · ${PHASE_TITLES[s.teach.phase] || 'Learning'}`, sub, 'busy');
  else if (['FAILED', 'DETECTION_FAILED', 'TARGET_UNREACHABLE'].includes(s.task.state)) setStatus(describe(s.task), sub, 'warn');
  else setStatus(s.task.state === 'COMPLETE' ? 'Done' : 'Ready', sub, 'ok');

  // control banner + enabled state
  const banner = $('control-banner');
  if (!canControl()) {
    banner.classList.remove('hidden');
    $('control-text').textContent = 'You can watch. Ask an owner if you need to control the arm.';
    $('take-control').classList.add('hidden');
  } else if (s.control && !s.control.mine) {
    banner.classList.remove('hidden');
    $('control-text').textContent = `${s.control.user} is controlling the arm from ${article(s.control.device)}.`;
    $('take-control').classList.remove('hidden');
    $('take-control').textContent = 'Take Over';
  } else {
    banner.classList.add('hidden');
  }
  const blocked = !canControl() || (s.control && !s.control.mine) || s.mode === 'offline';
  for (const node of document.querySelectorAll('#panel button:not(#grabber):not(#take-control):not(.model-line), #panel input, #panel select')) {
    if (node.closest('#control-banner')) continue;
    node.disabled = blocked;
  }
  $('stop').disabled = !canControl() || s.mode === 'offline';
  $('stop').classList.toggle('armed', s.task.running || !!teaching);
  $('release-control').classList.toggle('hidden', !driving());

  // sorting progress
  const running = s.task.running;
  $('sort-setup').classList.toggle('hidden', running);
  $('sort-running').classList.toggle('hidden', !running);
  if (running) {
    if (!previous?.task.running) { app.placed = []; renderPlaced(); }
    $('run-title').textContent = describe(s.task);
    const colors = (s.task.colors || []).map((c) => (c === 'all' ? 'all colours' : nice(c).toLowerCase()));
    $('run-sub').textContent = s.task.mode === 'exhibition'
      ? `Exhibition mode${s.task.by ? ` · started by ${s.task.by}` : ''}`
      : `Sorting ${colors.join(', ')}${s.task.by ? ` · started by ${s.task.by}` : ''}`;
    const at = ORDER.findIndex((k) => STEPS[k].includes(s.task.state));
    for (const li of $('steps').children) {
      const i = ORDER.indexOf(li.dataset.step);
      li.className = at < 0 ? '' : i < at ? 'done' : i === at ? 'now' : '';
    }
    if (s.task.state === 'RELEASE' && app.lastState !== 'RELEASE') {
      const color = s.task.current?.bin || s.task.current?.cube;
      app.placed.push(color);
      renderPlaced();
      celebrate($('sorted-count'), 40, [colorCss(color), '#ffffff']);
    }
  } else if (previous?.task.running && s.task.state === 'COMPLETE') {
    const n = app.placed.length;
    toast(n ? `Done — sorted ${n} cube${n === 1 ? '' : 's'}!` : 'Done — no cubes it could pick.');
    if (n) celebrate(null, 160);
  }
  if (s.task.state === 'PATTERN_DONE' && app.lastState !== 'PATTERN_DONE') celebrate(null, app.kiosk ? 220 : 120);
  app.lastState = s.task.state;

  // joints follow the arm unless being dragged
  if (!app.dragging && s.servo) {
    s.servo.forEach((v, i) => {
      const input = $(`joint-${i}`);
      if (input && document.activeElement !== input) { input.value = v; input.nextElementSibling.value = `${v}°`; }
    });
  }

  // camera
  const hasCamera = s.cameras.includes('camera');
  $('pip').classList.toggle('hidden', !hasCamera && !app.cameraBig);
  if (hasCamera && !$('camera-pip').getAttribute('src') && !app.cameraBig) $('camera-pip').src = streamUrl();
  const n = s.cubes?.list?.length || 0;
  $('pip-count').classList.toggle('hidden', !n);
  $('pip-count').textContent = `${n} cube${n === 1 ? '' : 's'}`;

  if (!$('settings').classList.contains('hidden')) {
    $('raw').textContent = `now     ${JSON.stringify(s.servo)}\ntarget  ${JSON.stringify(s.target)}\nmode    ${s.mode}\nstate   ${s.task.state}`;
  }
}

const article = (device) => (/^[aeiou]/i.test(device || '') ? `an ${device}` : `a ${device || 'browser'}`);

function renderPlaced() {
  $('placed').replaceChildren(...app.placed.map((c) => {
    const s = el('span', 'swatch');
    s.title = nice(c || 'cube');
    s.style.background = colorCss(c);
    return s;
  }));
  const counter = $('sorted-count');
  counter.textContent = String(app.placed.length);
  counter.classList.remove('bump');
  void counter.offsetWidth;          // restart the animation
  counter.classList.add('bump');
}

// ---------------------------------------------------------------- control

async function ensureControl() {
  if (driving()) return true;
  if (!canControl()) return false;
  try {
    await api('/api/control/take', {});
    return true;
  } catch (err) {
    toast(err.message);
    return false;
  }
}

$('take-control').onclick = async () => {
  const holder = app.state?.control?.user;
  if (holder && !confirm(`Take control from ${holder}?`)) return;
  try { await api('/api/control/take', { force: true }); } catch (err) { toast(err.message); }
};

async function drive(path, body) {
  if (!(await ensureControl())) return null;
  try { return await api(path, body); } catch (err) { toast(err.message); return null; }
}

async function stopEverything() {
  if (!canControl()) return;
  try { await api('/api/stop', {}); toast('Stopped.'); } catch (err) { toast(err.message); }
}
$('stop').onclick = stopEverything;
document.addEventListener('keydown', (e) => {
  if (e.key !== 'Escape') return;
  if (!$('settings').classList.contains('hidden')) { closeSettings(); return; }
  if (!$('cam-panel').classList.contains('hidden')) { toggleAdjust(false); return; }
  if (!$('app').classList.contains('hidden')) stopEverything();
});

// ---------------------------------------------------------------- sort tab

function buildSort() {
  $('color-chips').onclick = onColorChip;
  setSegmented($('drop-mode'), store.get('drop', 'auto'));
  app.colorsKey = '';
  renderColors(app.info.colors.map((name) => ({ name, swatch: swatches[name] })));
}

function renderColors(colors) {
  for (const c of colors) if (c.swatch) swatches[c.name] = c.swatch;
  const key = JSON.stringify([colors.map((c) => c.name), [...app.selected], app.editColors]);
  if (key === app.colorsKey) return;
  app.colorsKey = key;
  const names = colors.map((c) => c.name);
  for (const name of [...app.selected]) if (!names.includes(name)) app.selected.delete(name);
  const chips = $('color-chips');
  const items = [];
  if (!app.editColors) {
    const all = chip('All', null, app.selected.size === 0);
    all.dataset.all = '1';
    items.push(all);
  }
  for (const name of names) {
    const b = chip(nice(name), name, !app.editColors && app.selected.has(name));
    if (app.editColors) {
      b.classList.add('removable');
      b.setAttribute('aria-label', `Remove ${nice(name)}`);
      b.removeAttribute('aria-pressed');
      b.append(el('span', 'chip-x', '×'));
    }
    items.push(b);
  }
  const add = chip('Add colour', null, false);
  add.classList.add('add-chip');
  add.removeAttribute('aria-pressed');
  add.dataset.add = '1';
  add.prepend(el('span', 'chip-plus', '+'));
  items.push(add);
  chips.replaceChildren(...items);
  $('edit-colors').textContent = app.editColors ? 'Done' : 'Edit';
  $('edit-colors').setAttribute('aria-pressed', String(app.editColors));
  updateStartLabel();
}

async function onColorChip(e) {
  const b = e.target.closest('.chip');
  if (!b) return;
  if (b.dataset.add) { openAddColor(); return; }
  if (app.editColors) {
    const name = b.dataset.color;
    if (!confirm(`Forget the colour ${nice(name)}? Cubes of that colour will be ignored.`)) return;
    const done = await drive('/api/colors/remove', { name });
    if (done) toast(`Removed ${nice(name)}.`);
    return;
  }
  if (b.dataset.all) app.selected.clear();
  else if (app.selected.has(b.dataset.color)) app.selected.delete(b.dataset.color);
  else app.selected.add(b.dataset.color);
  app.colorsKey = '';
  app.tableKey = '';
  renderColors(app.state?.colors || []);
}

$('edit-colors').onclick = () => {
  app.editColors = !app.editColors;
  app.colorsKey = '';
  renderColors(app.state?.colors || []);
};

function chip(label, color, pressed) {
  const b = el('button', color ? 'chip' : 'chip plain-chip');
  b.setAttribute('aria-pressed', String(pressed));
  if (color) {
    b.dataset.color = color;
    const sw = el('span', 'swatch');
    sw.style.background = colorCss(color);
    b.append(sw);
  }
  b.append(label);
  return b;
}

function updateStartLabel() {
  const n = app.selected.size;
  $('start-sort').querySelector('span').textContent =
    n === 0 ? 'Sort All Cubes' : n === 1 ? `Pick ${nice([...app.selected][0])} Cubes` : `Sort ${n} Colours`;
}

const allowed = (color) => app.selected.size === 0 || app.selected.has(color);

function renderTable(s) {
  const list = s.cubes?.list || [];
  const seen = Object.entries(s.cubes?.pixels || {}).filter(([, pct]) => pct >= 0.05).map(([c]) => c);
  const key = JSON.stringify([list.map((c) => [c.color, c.known]), [...app.selected], s.cameras,
    s.devices?.camera?.state, s.devices?.camera?.detail, (s.cubes?.rejected || []).slice(0, 1).map((r) => [r.color, r.reason]), seen,
    Object.keys(s.cubes?.pixels || {}).length]);
  if (key === app.tableKey) return;
  app.tableKey = key;
  const box = $('table-cubes');
  if (!s.cameras.includes('camera') || (s.devices?.camera && s.devices.camera.state !== 'streaming')) {
    const cam = s.devices?.camera;
    const note = el('button', 'secondary small empty-note link-note',
      cam?.detail ? `Camera: ${cam.detail} · Devices ›` : 'No camera picture yet. · Devices ›');
    note.onclick = openDevices;
    box.replaceChildren(note);
    return;
  }
  if (!list.length) {
    box.replaceChildren(whyNoCubes(s));
    return;
  }
  const groups = new Map();
  for (const c of list) {
    const g = groups.get(c.color) || { n: 0, known: 0, unknown: 0 };
    g.n += 1;
    if (c.known === true) g.known += 1;
    if (c.known === false) g.unknown += 1;
    groups.set(c.color, g);
  }
  box.replaceChildren(...[...groups.entries()].map(([color, g]) => {
    const tile = el('div', `cube-tile${allowed(color) ? '' : ' off'}`);
    const cube = el('span', 'cube-icon');
    cube.style.setProperty('--c', colorCss(color));
    const count = el('strong', 'cube-count', String(g.n));
    const name = el('span', 'cube-name', nice(color));
    const note = el('span', 'cube-note',
      !allowed(color) ? 'not sorting' : g.unknown && !g.known ? 'not taught there' : g.unknown ? `${g.known} reachable` : 'ready');
    tile.append(cube, count, name, note);
    return tile;
  }));
}

const REJECT_HINT = {
  'too big': 'put it on the table, further from the camera',
  'too small': 'it is too far away or only a speck of that colour',
  'cut off by the edge': 'move it into the picture',
  'not square': 'it may be the arm, a shadow or two cubes touching',
};

// Colour detection only sees saved colours, as solid cube-sized blobs: say which part failed.
function whyNoCubes(s) {
  const wrap = el('div', 'why-none');
  const rejected = s.cubes?.rejected || [];
  const pixels = s.cubes?.pixels || {};
  const names = Object.keys(pixels);
  if (!names.length) {
    wrap.append(el('p', 'small', 'No colours saved yet, so there is nothing to look for. ' +
      'Put a cube on the table, press Add colour and tap the cube in the picture.'));
    const add = el('button', 'btn primary small-btn', 'Add colour');
    add.onclick = openAddColor;
    wrap.append(add);
    return wrap;
  }
  if (rejected.length) {
    const r = rejected[0];
    wrap.append(el('p', 'small', `Seeing ${nice(r.color).toLowerCase()}, but ${r.reason}: ${REJECT_HINT[r.reason] || 'see the dashed box on the camera'}.`));
  } else if (names.length && names.every((c) => pixels[c] < 0.05)) {
    wrap.append(el('p', 'small', `No ${names.join(', ')} pixels in view. If a cube is on the table, the camera ` +
      'sees its colour differently from when it was measured (another camera or light): put that cube under ' +
      'the camera and save it again with Add colour, same name.'));
    const fix = el('button', 'btn tinted small-btn', 'Re-measure a colour');
    fix.onclick = openAddColor;
    wrap.append(fix);
  } else {
    wrap.append(el('p', 'secondary small', 'No cubes in view. Put some on the table!'));
  }
  const tip = el('button', 'btn plain small-btn', cameraKindLabel('mask', true));
  tip.onclick = () => setCameraKind(app.cameraKind === 'mask' ? 'camera' : 'mask');
  wrap.append(tip);
  return wrap;
}

function renderModelLine(selected) {
  const line = $('sort-model');
  const info = selected?.info;
  if (app.state?.mode === 'sim') { line.classList.add('hidden'); return; }
  line.classList.remove('hidden');
  line.textContent = info
    ? `Session ${selected.session} · ${modelLabel(info.name)} · ${KIND_NAMES[info.chosen] || info.chosen}${info.error_deg != null ? ` · ±${info.error_deg}°` : ''} ›`
    : selected?.session ? `Session ${selected.session} has no model yet — teach it ›` : 'No teaching session yet — teach the arm ›';
}
$('sort-model').onclick = () => setTab('train');

wireSegmented($('drop-mode'), (v) => store.set('drop', v));
$('start-sort').onclick = async () => {
  $('sort-error').textContent = '';
  const body = { colors: [...app.selected], drop: $('drop-advanced').value || segmentedValue($('drop-mode')) };
  if (!(await ensureControl())) return;
  try { await api('/api/task/start', body); } catch (err) { $('sort-error').textContent = err.message; }
};
$('stop-sort').onclick = () => drive('/api/task/stop', {});

// ---- add a colour: the one cube under the camera, measured live

// Adding a colour: the camera fills the screen and the person taps the cube,
// so the arm, cables or the table are never measured by mistake.
function openAddColor() {
  $('add-color').classList.remove('hidden');
  $('add-name').value = '';
  app.addNameTyped = false;
  app.addPoint = null;
  app.addOpenedBig = !app.cameraBig;
  if (!app.cameraBig) swapCamera();
  document.body.classList.add('picking');
  pollAddColor();
}
function closeAddColor() {
  $('add-color').classList.add('hidden');
  clearTimeout(app.addTimer);
  app.addTimer = null;
  app.addPreview = null;
  app.addPoint = null;
  document.body.classList.remove('picking');
  if (app.addOpenedBig && app.cameraBig) swapCamera();
  app.addOpenedBig = false;
}

// A tap on the big camera picture -> pixel in the camera image (undoing zoom, rotate, mirror).
function imagePoint(e) {
  const cubes = app.state?.cubes;
  if (!cubes?.width) return null;
  const r = $('frame-full').getBoundingClientRect();
  const { adjust } = cameraSettings();
  let x = (e.clientX - (r.left + r.width / 2)) / (adjust.zoom / 100);
  let y = (e.clientY - (r.top + r.height / 2)) / (adjust.zoom / 100);
  if (adjust.rotate === 180) { x = -x; y = -y; }
  if (adjust.mirror) x = -x;
  const scale = Math.min(r.width / cubes.width, r.height / cubes.height);
  const ix = (x + r.width / 2 - (r.width - cubes.width * scale) / 2) / scale;
  const iy = (y + r.height / 2 - (r.height - cubes.height * scale) / 2) / scale;
  if (ix < 0 || iy < 0 || ix >= cubes.width || iy >= cubes.height) return null;
  return { x: Math.round(ix), y: Math.round(iy) };
}
$('overlay-full').addEventListener('click', (e) => {
  if (!document.body.classList.contains('picking')) return;
  const point = imagePoint(e);
  if (!point) return;
  app.addPoint = point;
  app.addNameTyped = false;
  pollAddColor();
});

async function pollAddColor() {
  clearTimeout(app.addTimer);
  if ($('add-color').classList.contains('hidden')) return;
  const seen = await drive('/api/colors/preview', app.addPoint || {});
  if (seen) {
    app.addPreview = seen.seen ? seen : null;
    $('add-swatch').style.background = seen.seen ? seen.swatch : 'transparent';
    $('add-swatch').classList.toggle('empty', !seen.seen);
    $('add-seen').textContent = seen.seen ? `Seeing ${nice(seen.name).toLowerCase()} (hue ${seen.hue})` : 'Looking…';
    const known = (app.state?.colors || []).some((c) => c.name === ($('add-name').value.trim() || seen.name));
    $('add-why').textContent = seen.seen
      ? (seen.clash ? `Too close to ${seen.clash}: the camera could not tell them apart.`
        : known ? 'Already saved: saving measures it again under this light.' : 'Looks good. Name it and save.')
      : seen.why;
    if (seen.seen && !seen.tapped) $('add-why').textContent += ' Not this one? Tap your cube in the picture.';
    $('add-why').classList.toggle('warn-text', !!seen.clash || (!seen.seen && !!app.addPoint));
    if (seen.seen && !app.addNameTyped) $('add-name').value = seen.name;
    $('add-save').disabled = !seen.seen || !!seen.clash;
  }
  app.addTimer = setTimeout(pollAddColor, 900);
}
$('add-name').oninput = () => { app.addNameTyped = true; };
$('add-cancel').onclick = closeAddColor;
$('add-save').onclick = async () => {
  const done = await drive('/api/colors/add', { name: $('add-name').value.trim().toLowerCase(), ...(app.addPoint || {}) });
  if (!done) return;
  toast(`Added ${nice(done.name)}! It can be sorted now.`);
  celebrate($('add-swatch'), 70, [colorCss(done.name), '#ffffff']);
  closeAddColor();
};

// ---------------------------------------------------------------- train tab

const PHASE_TITLES = {
  starting: 'Getting ready', menu: 'Ready to learn', reading: 'Looking…', moving: 'Moving…',
  above: 'Step 1 · Above the cube', grab: 'Step 2 · Around the cube', close: 'Step 3 · Close & lift',
  check: 'Step 4 · Got it?', training: 'Learning…', testing: 'Trying on my own…', test_ready: 'Test time!',
  teach_here: 'New spot', correct: 'Missed', drop: 'Drop spots', review: 'Review', stopped: 'Stopped',
  camera_moved: 'Camera moved?', ended: 'Done',
  collect_setup: 'Ready to collect', collect_handover: 'Hand me the cube', collect_lost: 'Lost the cube',
  collect_give_back: 'Take the cube',
};
const STEP_OF = { menu: 0, reading: 0, above: 1, grab: 2, close: 3, check: 4 };
const BUSY = ['starting', 'reading', 'moving', 'training', 'testing'];
const JOG = ['above', 'grab', 'close', 'drop'];
const CHOICES = {
  record: ['Record Example', 'primary'], test: ['Test Me', 'tinted'], drops: ['Drop Spots', 'tinted'],
  review: ['Review', 'tinted'], undo: ['Undo Last', 'tinted'],
  next: ['Next', 'primary'], cancel: ['Cancel', 'tinted'],
  'check:yes': ['Holding It ✓', 'success'], 'check:no': ['Dropped It', 'tinted'],
  'correct:yes': ['Show Me', 'primary'], 'correct:no': ['Skip', 'tinted'],
  'teach_here:yes': ['Teach Here', 'primary'], 'teach_here:no': ['Skip', 'tinted'],
  go: ['Go!', 'primary'], skip: ['Skip', 'tinted'],
  'drop:save': ['Save Spot', 'primary'], 'drop:skip': ['Skip Colour', 'tinted'], 'drop:cancel': ['Done', 'tinted'],
  check: ['Go Through Them', 'primary'], done: ['Done', 'tinted'], delete: ['Delete', 'destructive'], keep: ['Keep', 'tinted'],
  home: ['Open & Go Home', 'primary'], stay: ['Stay Here', 'tinted'],
  carry_on: ['Carry On', 'primary'], quit: ['End', 'tinted'],
  start: ['Start Collecting', 'primary'], ok: ['OK', 'primary'],
  'collect_handover:quit': ['Stop Collecting', 'tinted'], 'collect_lost:quit': ['Stop Collecting', 'tinted'],
};
const KIND_NAMES = { knn: 'Nearest neighbours', poly1: 'Flat plane', poly2: 'Curve', poly3: 'Wiggly curve', mlp: 'Neural net' };
const modelLabel = (name) => (name ? String(name).replace('model_', 'Model ').replace(/ 0+(\d)/, ' $1') : 'model');

function buildTrain() {
  wireSegmented($('jog-step'), (v) => { app.jogStep = Number(v); });
  const rows = $('jog-rows');
  rows.replaceChildren();
  app.info.joints.forEach((name, i) => {
    const row = el('div', 'jog-row');
    row.append(el('span', 'jog-label', JOINT_LABELS[i]));
    const minus = el('button', 'jog-btn', '−');
    minus.setAttribute('aria-label', `${JOINT_LABELS[i]} down`);
    const out = el('output', 'jog-value', '–');
    out.id = `jog-${i}`;
    const plus = el('button', 'jog-btn', '+');
    plus.setAttribute('aria-label', `${JOINT_LABELS[i]} up`);
    holdRepeat(minus, () => nudge(i, -app.jogStep));
    holdRepeat(plus, () => nudge(i, app.jogStep));
    row.append(minus, out, plus);
    if (i === 5) {
      const open = el('button', 'btn tinted small-btn', 'Open');
      const grip = el('button', 'btn tinted small-btn', 'Grip');
      open.onclick = () => drive('/api/gripper', { close: false });
      grip.onclick = () => drive('/api/gripper', { close: true });
      row.append(open, grip);
      row.classList.add('gripper-row');
    }
    rows.append(row);
  });
  fillSessions();
}

// Press and hold a jog button to keep moving, as with the teach_pick keys.
function holdRepeat(button, action) {
  let timer = null;
  const stop = () => { clearTimeout(timer); timer = null; };
  button.addEventListener('pointerdown', (e) => {
    if (button.disabled) return;
    e.preventDefault();
    action();
    const again = (delay) => { timer = setTimeout(() => { action(); again(140); }, delay); };
    again(420);
  });
  for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) button.addEventListener(ev, stop);
  button.addEventListener('keydown', (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); action(); } });
}

let nudging = false;
async function nudge(joint, delta) {
  if (nudging) return;           // one at a time: the arm is slower than a finger
  nudging = true;
  await drive('/api/nudge', { joint, delta });
  nudging = false;
}

function fillSessions() {
  const sessions = app.info.sessions || [];
  const current = app.state?.selected?.session || app.info.default_session || '';
  for (const select of [$('train-session'), $('session'), $('collect-session')]) {
    const keep = select.value || current;
    select.replaceChildren(...sessions.map((s) => new Option(`${s.name} · ${s.examples} examples`, s.name)));
    if (!sessions.length) select.append(new Option('No sessions yet', ''));
    if (keep) select.value = keep;
  }
  loadReview();
}

async function loadModels(session) {
  app.modelsFor = session;
  try { app.models = session ? (await api(`/api/models?session=${encodeURIComponent(session)}`)).models : []; } catch { app.models = []; }
  const select = $('train-model');
  select.replaceChildren(new Option('Latest model', ''), ...app.models.map((m) => new Option(
    `${modelLabel(m.name)} · ${KIND_NAMES[m.chosen] || m.chosen || '?'}${m.error_deg != null ? ` · ±${m.error_deg}°` : ''}`, m.name)));
  select.value = app.state?.selected?.model || '';
}

$('train-session').onchange = async (e) => {
  const done = await drive('/api/select', { session: e.target.value, model: '' });
  if (done) loadModels(e.target.value);
};
$('train-model').onchange = (e) => drive('/api/select', { session: $('train-session').value, model: e.target.value });
$('new-session').onclick = () => {
  const box = $('new-session-box');
  box.classList.toggle('hidden');
  if (!box.classList.contains('hidden')) $('new-session-name').focus();
  updateTeachButton();
};
$('new-session-name').oninput = updateTeachButton;

function updateTeachButton() {
  const fresh = !$('new-session-box').classList.contains('hidden');
  const session = (app.info.sessions || []).find((s) => s.name === $('train-session').value);
  $('start-teach').querySelector('span').textContent = fresh
    ? `Start Session ${$('new-session-name').value.trim() || '…'}`
    : session?.examples ? `Continue Teaching ${session.name}` : 'Start Teaching';
}

$('start-teach').onclick = async () => {
  $('train-error').textContent = '';
  const fresh = !$('new-session-box').classList.contains('hidden');
  const session = fresh ? $('new-session-name').value.trim() : $('train-session').value;
  if (!session) { $('train-error').textContent = 'Name the new session first.'; return; }
  if (!(await ensureControl())) return;
  try {
    await api('/api/teach/start', { session });
    $('new-session-box').classList.add('hidden');
    $('new-session-name').value = '';
  } catch (err) { $('train-error').textContent = err.message; }
};

$('end-teach').onclick = async () => {
  const t = app.state?.teach;
  if (t?.choices.includes('quit')) { drive('/api/teach/answer', { choice: 'quit' }); return; }
  if (!confirm('End teaching now? Examples recorded so far are kept.')) return;
  drive('/api/teach/stop', {});
};

function renderTrain(s) {
  const t = s.teach && s.teach.kind !== 'collect' ? s.teach : null;
  const live = !!t && !t.done;
  $('train-setup').classList.toggle('hidden', live);
  $('train-live').classList.toggle('hidden', !live);

  // teaching just ended: fresh sessions and models
  if (!live && !app.teachDone) refreshInfo().then(() => loadModels($('train-session').value));
  app.teachDone = !live;

  if (!live) {
    const sel = s.selected || {};
    if (document.activeElement !== $('train-session') && sel.session && $('train-session').value !== sel.session) {
      $('train-session').value = sel.session;
    }
    const modelKey = `${sel.session}|${sel.info?.name}`;
    if (modelKey !== app.modelsFor + '|' + app.lastModelName) {
      app.lastModelName = sel.info?.name;
      loadModels(sel.session || '');
    }
    if (document.activeElement !== $('train-model')) $('train-model').value = sel.model || '';
    renderStats($('train-stats'), sel.info ? [
      [sel.info.examples ?? '–', 'examples'],
      [sel.info.error_deg != null ? `±${sel.info.error_deg}°` : '–', 'typical error'],
      [KIND_NAMES[sel.info.chosen] || '–', 'winning model'],
    ] : [['0', 'examples'], ['–', 'typical error'], ['–', 'winning model']]);
    renderBoard($('leaderboard'), sel.info?.scores, sel.info?.chosen, false);
    updateTeachButton();
    return;
  }

  // live teaching
  $('teach-title').textContent = PHASE_TITLES[t.phase] || t.phase;
  $('teach-session').textContent = t.session;
  const busy = BUSY.includes(t.phase);
  $('teach-prompt').textContent = t.prompt;
  $('teach-prompt').classList.toggle('busy', busy);
  const step = STEP_OF[t.phase];
  if (step !== undefined) app.teachStep = step;
  const inExample = ['above', 'grab', 'close', 'check'].includes(t.phase) || (t.phase === 'moving' && t.reading);
  for (const li of $('teach-steps').children) {
    const i = [...li.parentNode.children].indexOf(li);
    li.className = !inExample && step === undefined ? '' : i < app.teachStep ? 'done' : i === app.teachStep ? 'now' : '';
  }
  const reading = $('teach-reading');
  reading.classList.toggle('hidden', !t.reading);
  if (t.reading) {
    reading.replaceChildren();
    const sw = el('span', 'swatch');
    sw.style.background = colorCss(t.reading.color);
    reading.append(sw, el('span', '', `${nice(t.reading.color)} cube at (${t.reading.x}, ${t.reading.y}) px · ${t.reading.angle}° · wobble ${t.reading.jitter} px`));
  }
  $('teach-review').classList.toggle('hidden', !t.review.length);
  $('teach-review').textContent = t.review.join('\n');
  renderChoices(t);
  $('jog').classList.toggle('hidden', !JOG.includes(t.phase));
  const target = s.target || s.servo;
  if (target) target.forEach((v, i) => { const o = $(`jog-${i}`); if (o) o.value = `${v}°`; });
  renderStats($('teach-stats'), [
    [String(t.examples), 'examples'],
    [t.model?.error_deg != null ? `±${t.model.error_deg}°` : '–', 'typical error'],
    [t.success || '–', 'recent tests'],
  ]);
  renderBoard($('teach-board'), t.scores, t.model?.chosen, t.phase === 'training');
  handleTeachEvents(t);
}

function renderChoices(t, box = $('teach-choices')) {
  const key = `${t.phase}|${t.choices.join(',')}`;
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  box.replaceChildren(...t.choices.filter((c) => c !== 'quit' || !['menu', 'collect_setup'].includes(t.phase)).map((choice) => {
    const [label, kind] = CHOICES[`${t.phase}:${choice}`] || CHOICES[choice] || [nice(choice), 'tinted'];
    const b = el('button', `btn ${kind}`, label);
    if (choice === 'record') b.classList.add('wide');
    b.onclick = async () => {
      box.dataset.key = '';            // allow re-render even if the server answers fast
      for (const other of box.children) other.disabled = true;
      await drive('/api/teach/answer', { choice });
    };
    return b;
  }));
  if (!t.choices.length) box.append(el('div', 'thinking', ''));
}

function renderStats(box, items) {
  box.replaceChildren(...items.map(([value, label]) => {
    const tile = el('div', 'stat');
    tile.append(el('strong', '', String(value)), el('span', '', label));
    return tile;
  }));
}

function renderBoard(box, scores, chosen, racing) {
  const entries = Object.entries(scores || {}).filter(([, v]) => v != null);
  box.classList.toggle('racing', racing);
  const key = JSON.stringify([entries, chosen, racing]);
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  if (!entries.length) {
    box.replaceChildren(el('p', 'secondary small empty-note', 'Record a few examples and the models start racing.'));
    return;
  }
  const best = Math.min(...entries.map(([, v]) => v));
  entries.sort((a, b) => a[1] - b[1]);
  box.replaceChildren(...entries.map(([kind, err]) => {
    const row = el('div', `race-row${kind === chosen ? ' winner' : ''}`);
    const name = el('span', 'race-name', `${kind === chosen ? '🏆 ' : ''}${KIND_NAMES[kind] || kind}`);
    const track = el('span', 'race-track');
    const bar = el('span', 'race-bar');
    bar.style.width = `${Math.max(8, Math.min(100, (best / err) * 100))}%`;
    track.append(bar);
    row.append(name, track, el('span', 'race-value', `${err.toFixed(1)}°`));
    return row;
  }));
}

function handleTeachEvents(t) {
  const events = t.events || [];
  const top = events.length ? events[events.length - 1].seq : 0;
  if (app.teachSeq === null || app.teachSession !== t.session || top < app.teachSeq) {
    app.teachSeq = app.teachSeq === null ? top : 0;   // first sight: no replay of old news
    app.teachSession = t.session;
    if (app.teachSeq === top) return;
  }
  for (const e of events) {
    if (e.seq <= app.teachSeq) continue;
    app.teachSeq = e.seq;
    if (['saved', 'picked', 'missed', 'warn', 'error', 'stopped', 'trained'].includes(e.kind)) toast(e.text);
    if (e.kind === 'saved') celebrate($('teach-stats'), 70);
    if (e.kind === 'picked') celebrate(null, 180);
    if (e.kind === 'missed') shake($('train-live'));
    if (e.kind === 'trained') flash($('teach-board'));
  }
}

function shake(node) { node.classList.remove('shake'); void node.offsetWidth; node.classList.add('shake'); }
function flash(node) { node.classList.remove('flash'); void node.offsetWidth; node.classList.add('flash'); }

// ---------------------------------------------------------------- collect tab (auto-collect)

function wireCollect() {
  const save = (body) => drive('/api/collect/settings', body);
  holdRepeat($('col-minus'), () => nudge(0, -2));   // smaller base angle = turn right
  holdRepeat($('col-plus'), () => nudge(0, 2));
  $('col-set-left').onclick = () => save({ set_end: 'left' });
  $('col-set-right').onclick = () => save({ set_end: 'right' });
  wireSegmented($('collect-step'), (v) => save({ step: Number(v) }));
  $('collect-seed').onchange = (e) => save({ seed: e.target.value });
  wireSegmented($('collect-mode'), (v) => save({ mode: v }));
  $('collect-list').onchange = (e) => { if (e.target.dataset.id) save({ toggle: e.target.dataset.id }); };
  $('collect-skip').onclick = () => save({ skip_arc: true });
  $('start-collect').onclick = async () => {
    $('collect-error').textContent = '';
    const session = $('collect-session').value;
    if (!session) { $('collect-error').textContent = 'Teach an example first (Train tab).'; return; }
    if (!(await ensureControl())) return;
    try { await api('/api/collect/start', { session }); } catch (err) { $('collect-error').textContent = err.message; }
  };
  $('end-collect').onclick = async () => {
    const t = app.state?.teach;
    if (t?.choices.includes('quit') && t.phase === 'collect_setup') { drive('/api/teach/answer', { choice: 'quit' }); return; }
    if (!confirm('End auto-collect now? Examples saved so far are kept.')) return;
    drive('/api/teach/stop', {});
  };
}

function renderCollect(s) {
  const t = s.teach && s.teach.kind === 'collect' ? s.teach : null;
  const live = !!t && !t.done;
  $('collect-setup').classList.toggle('hidden', live);
  $('collect-live').classList.toggle('hidden', !live);
  if (!live && app.collectWasLive) refreshInfo().then(() => loadModels($('train-session').value));
  app.collectWasLive = live;
  if (!live) {
    const sel = s.selected?.session;
    if (sel && document.activeElement !== $('collect-session') && $('collect-session').value !== sel) $('collect-session').value = sel;
    return;
  }
  const c = t.collect;
  const setup = t.phase === 'collect_setup';
  $('collect-title').textContent = PHASE_TITLES[t.phase] || t.phase;
  $('collect-session-pill').textContent = t.session;
  $('collect-prompt').textContent = t.prompt;
  $('collect-prompt').classList.toggle('busy', BUSY.includes(t.phase));
  $('collect-settings').classList.toggle('hidden', !setup);

  // what to repeat: one example, or every ticked one (a taught line)
  const all = c.settings.mode === 'all';
  if (!document.activeElement?.closest?.('#collect-mode')) setSegmented($('collect-mode'), c.settings.mode);
  $('collect-one').classList.toggle('hidden', all);
  $('collect-all').classList.toggle('hidden', !all);
  const listKey = JSON.stringify(c.seeds.map((e) => [e.id, e.ticked, e.todo]));
  if ($('collect-list').dataset.key !== listKey) {
    $('collect-list').dataset.key = listKey;
    const inOrder = [...c.seeds].reverse();      // taught order: the line from the base outwards
    $('collect-list').replaceChildren(...inOrder.map((e, i) => {
      const row = el('label', 'cell seed-row');
      const box = el('input');
      Object.assign(box, { type: 'checkbox', checked: e.ticked });
      box.dataset.id = e.id;
      const text = el('span', 'seed-text');
      text.append(el('strong', '', `${i + 1}. ${nice(e.color || 'cube')} at (${e.x}, ${e.y}) px`),
        el('span', 'secondary small', `base ${e.base}° · ${e.todo ? `${e.todo} spots to do` : 'arc done'}`));
      row.append(box, text);
      return row;
    }));
  }

  // which example to repeat
  const seedKey = JSON.stringify([c.seeds, c.seed]);
  if ($('collect-seed').dataset.key !== seedKey && document.activeElement !== $('collect-seed')) {
    $('collect-seed').dataset.key = seedKey;
    $('collect-seed').replaceChildren(...c.seeds.map((e, i) => new Option(
      `${i === 0 ? 'Newest · ' : ''}${nice(e.color || 'cube')} at (${e.x}, ${e.y}) px · base ${e.base}°`, e.id)));
    if (c.seed) $('collect-seed').value = c.seed;
  }
  const [lo, hi] = c.window;
  $('col-left').textContent = `${hi}°`;
  $('col-right').textContent = `${lo}°`;
  const base = (s.target || s.servo || [])[0];
  $('col-now').textContent = base == null ? '–' : `${base}°`;
  $('col-value').value = base == null ? '–' : `${base}°`;
  if (!document.activeElement?.closest?.('#collect-step')) setSegmented($('collect-step'), String(c.settings.step));

  // progress while it runs (and the result after)
  const p = c.progress;
  $('collect-progress').classList.toggle('hidden', !p.total);
  $('collect-arc').textContent = p.arcs ? `Arc ${p.arc || 1} of ${p.arcs}` : '';
  $('collect-skip').classList.toggle('hidden', setup || p.arcs < 1);
  if (p.total) {
    $('collect-bar').style.width = `${Math.round((p.done / p.total) * 100)}%`;
    renderStats($('collect-stats'), [[String(p.saved), 'new examples'], [String(p.skipped), 'skipped'],
      [String(Math.max(0, p.total - p.done)), p.base != null && !setup ? `left · now ${p.base}°` : 'left']]);
  }
  renderChoices(t, $('collect-choices'));
  renderStats($('collect-model'), [[String(t.examples), 'examples in total'],
    [t.model?.error_deg != null ? `±${t.model.error_deg}°` : '–', 'typical error'],
    [setup ? String(c.planned) : '–', 'spots to do']]);
  handleTeachEvents(t);
}

// ---------------------------------------------------------------- show tab (exhibition mode)

function wireShow() {
  holdRepeat($('win-minus'), () => nudge(0, -2));   // smaller base angle = turn right
  holdRepeat($('win-plus'), () => nudge(0, 2));
  const save = (body) => drive('/api/exhibit/settings', body);
  $('set-left').onclick = () => save({ set_end: 'left' });
  $('set-right').onclick = () => save({ set_end: 'right' });
  $('clear-window').onclick = () => save({ clear_window: true });
  wireSegmented($('show-rest'), (v) => save({ rest_s: Number(v) }));
  wireSegmented($('show-moves'), (v) => save({ moves_per_round: Number(v) }));
  $('show-attract').onchange = (e) => save({ attract: e.target.checked });
  $('show-patterns').onclick = (e) => {
    const b = e.target.closest('.chip');
    if (!b) return;
    const on = new Set(app.state?.exhibit?.settings?.patterns || []);
    if (on.has(b.dataset.pattern)) on.delete(b.dataset.pattern); else on.add(b.dataset.pattern);
    if (!on.size) { toast('Keep at least one pattern.'); return; }
    save({ patterns: [...on] });
  };
  $('start-show').onclick = async () => {
    $('show-error').textContent = '';
    if (!(await ensureControl())) return;
    try { await api('/api/task/start', { mode: 'exhibition' }); } catch (err) { $('show-error').textContent = err.message; }
  };
  $('stop-show').onclick = () => drive('/api/task/stop', {});
}

const minutes = (seconds) => {
  if (!(seconds > 0)) return '–';
  const m = Math.floor(seconds / 60);
  return m < 60 ? `${m} min` : `${Math.floor(m / 60)} h ${m % 60} min`;
};
function exhibitStats(task) {
  const c = task.current || {};
  const picks = c.picks || 0;
  return {
    moved: c.moved || 0,
    rate: picks ? `${Math.round(((picks - (c.misses || 0)) / picks) * 100)}%` : '–',
    time: minutes(c.started ? Date.now() / 1000 - c.started : 0),
    round: c.round || 0,
    attention: task.state === 'ATTENTION' ? c.attention : '',
  };
}

const PATTERN_NAMES = { line: 'Line by colour', arc: 'Circle', zones: 'Colour zones', swap: 'Swap', shuffle: 'Shuffle' };

function renderShow(s) {
  const ex = s.exhibit;
  if (!ex) return;
  const running = s.task.running && s.task.mode === 'exhibition';
  $('show-setup').classList.toggle('hidden', running);
  $('show-running').classList.toggle('hidden', !running);
  const set = ex.settings;
  $('win-left').textContent = set.base_max == null ? 'none' : `${set.base_max}°`;
  $('win-right').textContent = set.base_min == null ? 'none' : `${set.base_min}°`;
  const base = (s.target || s.servo || [])[0];
  $('win-now').textContent = base == null ? '–' : `${base}°`;
  $('win-value').value = base == null ? '–' : `${base}°`;
  const inside = base == null || ((set.base_min == null || base >= set.base_min) && (set.base_max == null || base <= set.base_max));
  $('win-now').parentElement.classList.toggle('outside', !inside);
  $('win-note').textContent = `${ex.spots} taught spot${ex.spots === 1 ? '' : 's'} inside the window`
    + (ex.spots < 2 ? ' — teach more spots there (Train tab), or widen the window.' : '. Cubes are only put down on these.')
    + (set.base_min == null && set.base_max == null ? ' No window set: it may turn anywhere it was taught.' : '');
  $('win-note').classList.toggle('warn-text', ex.spots < 2);
  if (!document.activeElement?.closest?.('#show-rest')) setSegmented($('show-rest'), String(set.rest_s));
  if (!document.activeElement?.closest?.('#show-moves')) setSegmented($('show-moves'), String(set.moves_per_round));
  $('show-attract').checked = !!set.attract;
  const playlistKey = JSON.stringify(set.patterns);
  if ($('show-patterns').dataset.key !== playlistKey) {
    $('show-patterns').dataset.key = playlistKey;
    $('show-patterns').replaceChildren(...Object.entries(PATTERN_NAMES).map(([key, label]) => {
      const b = chip(label, null, set.patterns.includes(key));
      b.dataset.pattern = key;
      return b;
    }));
  }
  if (!running) return;
  const st = exhibitStats(s.task);
  $('show-title').textContent = describe(s.task);
  $('show-sub').textContent = `Round ${st.round}${s.task.by ? ` · started by ${s.task.by}` : ''}`;
  if ($('show-count').textContent !== String(st.moved)) {
    $('show-count').textContent = String(st.moved);
    bump($('show-count'));
    if (st.moved) celebrate($('show-count'), 50);
  }
  $('show-attention').classList.toggle('hidden', !st.attention);
  $('show-attention').textContent = st.attention || '';
  renderStats($('show-stats'), [[st.rate, 'picked first time'], [st.time, 'running'], [String(s.task.current?.picks || 0), 'picks tried']]);
}
function bump(node) { node.classList.remove('bump'); void node.offsetWidth; node.classList.add('bump'); }

// The base window drawn on the floor of the 3D view: where the arm may turn.
function updateWindow(yaw) {
  if (!view.scene) return;
  const key = JSON.stringify(yaw);
  if (view.windowKey === key) return;
  view.windowKey = key;
  if (view.windowMesh) { view.scene.remove(view.windowMesh); view.windowMesh.geometry.dispose(); view.windowMesh = null; }
  if (!yaw) return;
  const geometry = new THREE.RingGeometry(0.07, 0.46, 48, 1, yaw[0], Math.max(0.01, yaw[1] - yaw[0]));
  const material = new THREE.MeshBasicMaterial({ color: 0x34c759, transparent: true, opacity: 0.22, depthWrite: false, side: THREE.DoubleSide });
  view.windowMesh = new THREE.Mesh(geometry, material);
  view.windowMesh.position.z = 0.002;
  view.scene.add(view.windowMesh);
}

// ---------------------------------------------------------------- big screen (/show)

function startKiosk() {
  app.kiosk = true;
  document.body.classList.add('kiosk');
  $('kiosk-hud').classList.remove('hidden');
  $('terminal').classList.remove('hidden');
  $('terminal').classList.add('tall');
  $('pip').dataset.size = 'l';
  // the Pi's own network address, so it reads right whatever address this screen used
  const address = (app.info.addresses || []).find((a) => !a.startsWith('<')) || location.host;
  $('kiosk-join').textContent = `Watch on your phone: http://${address}`;
  if (!app.cameraBig) swapCamera();
}

function renderKiosk(s) {
  const exhibition = s.task.running && s.task.mode === 'exhibition';
  const teaching = s.teach && !s.teach.done;
  $('kiosk-now').textContent = s.mode === 'offline' ? 'The arm is resting (not connected)'
    : s.task.running ? describe(s.task)
      : teaching ? `Learning from a person: ${(PHASE_TITLES[s.teach.phase] || '').toLowerCase()}`
        : 'Waiting for the next show…';
  const st = exhibition ? exhibitStats(s.task) : { moved: app.placed.length, rate: '–', time: '–', attention: '' };
  if ($('kiosk-moved').textContent !== String(st.moved)) {
    $('kiosk-moved').textContent = String(st.moved);
    bump($('kiosk-moved'));
    if (st.moved) celebrate($('kiosk-moved'), 90);
  }
  $('kiosk-rate').textContent = st.rate;
  $('kiosk-time').textContent = st.time;
  $('kiosk-attention').classList.toggle('hidden', !st.attention);
  $('kiosk-attention').textContent = st.attention || '';
}

// ---------------------------------------------------------------- move tab

const JOINT_LABELS = ['Base', 'Shoulder', 'Elbow', 'Wrist', 'Roll', 'Gripper'];

function buildMove() {
  const poses = $('poses');
  poses.replaceChildren();
  for (const name of ['ready', 'rest', 'pickup', 'drop', 'wave']) {
    if (!app.info.poses[name]) continue;
    const b = chip(name === 'ready' ? 'Home' : nice(name), null, false);
    b.removeAttribute('aria-pressed');
    b.onclick = () => drive('/api/pose', { name });
    poses.append(b);
  }
  $('gripper').onclick = (e) => {
    const b = e.target.closest('button');
    if (b) drive('/api/gripper', { close: b.dataset.value === 'close' });
  };

  const joints = $('joints');
  joints.replaceChildren();
  app.info.joints.forEach((name, i) => {
    const [lo, hi] = app.info.limits[name];
    const row = el('div', 'joint');
    const label = el('label', '', JOINT_LABELS[i]);
    label.htmlFor = `joint-${i}`;
    const input = el('input');
    Object.assign(input, { type: 'range', id: `joint-${i}`, min: lo, max: hi, step: 1, value: Math.round((lo + hi) / 2) });
    row.append(label, input, el('output', '', '–'));
    joints.append(row);
  });
  let pending = null, sending = false;
  const flush = async () => {
    if (sending || !pending) return;
    sending = true;
    const values = pending;
    pending = null;
    await drive('/api/move', { values });
    sending = false;
    if (pending) flush();
  };
  joints.addEventListener('input', (e) => {
    if (e.target.type !== 'range') return;
    app.dragging = true;
    e.target.nextElementSibling.value = `${e.target.value}°`;
    pending = app.info.joints.map((_, i) => Number($(`joint-${i}`).value));
    flush();
  });
  joints.addEventListener('change', () => { setTimeout(() => { app.dragging = false; }, 600); });
}

// ---------------------------------------------------------------- tabs + panel

const TABS = ['sort', 'train', 'collect', 'show', 'move'];
function setTab(tab) {
  app.tab = TABS.includes(tab) ? tab : 'sort';
  store.set('tab', app.tab);
  for (const name of TABS) $(`tab-${name}`).classList.toggle('hidden', app.tab !== name);
  for (const b of $('tabs').querySelectorAll('button')) b.setAttribute('aria-selected', String(b.dataset.tab === app.tab));
  $('panel').classList.remove('collapsed');
  if (app.tab === 'train') refreshInfo();
  moveTabIndicator();
  reframe();
}
// The panel animates for ~0.4 s; reframe the 3D view once it has settled.
function reframe() { if (view.renderer) setTimeout(resize, 450); }
function moveTabIndicator() {
  const b = $('tabs').querySelector('button[aria-selected="true"]');
  if (!b) return;
  const ind = $('tab-indicator');
  ind.style.width = `${b.offsetWidth}px`;
  ind.style.transform = `translateX(${b.offsetLeft - 4}px)`;
}
$('tabs').onclick = (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  if (b.dataset.tab === app.tab) { $('panel').classList.toggle('collapsed'); reframe(); }
  else setTab(b.dataset.tab);
};
$('grabber').onclick = () => { $('panel').classList.toggle('collapsed'); reframe(); };
window.addEventListener('resize', moveTabIndicator);

// ---------------------------------------------------------------- live log terminal

const term = { atBottom: true, lines: 0 };

function initTerminal() {
  const wanted = store.get('term', window.innerWidth > 720 ? '1' : '0') === '1';
  showTerminal(wanted);
  $('terminal').classList.toggle('tall', store.get('termTall', '0') === '1');
  const body = $('term-body');
  body.addEventListener('scroll', () => {
    term.atBottom = body.scrollHeight - body.scrollTop - body.clientHeight < 24;
    if (term.atBottom) $('term-jump').classList.add('hidden');
  });
}
function showTerminal(on) {
  $('terminal').classList.toggle('hidden', !on);
  document.body.classList.toggle('term-on', on);
  $('toggle-term').setAttribute('aria-pressed', String(on));
  store.set('term', on ? '1' : '0');
  if (on) scrollTerminal();
}
$('toggle-term').onclick = () => showTerminal($('terminal').classList.contains('hidden'));
$('term-close').onclick = () => showTerminal(false);
$('term-size').onclick = () => {
  const tall = !$('terminal').classList.contains('tall');
  $('terminal').classList.toggle('tall', tall);
  store.set('termTall', tall ? '1' : '0');
  scrollTerminal();
};
$('term-jump').onclick = () => { term.atBottom = true; scrollTerminal(); $('term-jump').classList.add('hidden'); };

const shortNode = (name) => String(name || '').replace(/^braccio_/, '').replace(/_node$/, '').slice(0, 16);

function appendLogs(logs) {
  if (!logs?.length) return;
  const body = $('term-body');
  const fragment = document.createDocumentFragment();
  for (const e of logs) {
    const line = el('div', `term-line ${String(e.level).toLowerCase()}`);
    line.append(el('span', 'term-t', e.t), el('span', 'term-n', shortNode(e.node)), el('span', 'term-m', e.text));
    fragment.append(line);
  }
  body.append(fragment);
  term.lines += logs.length;
  while (term.lines > 400) { body.firstChild?.remove(); term.lines -= 1; }
  if (term.atBottom) scrollTerminal();
  else $('term-jump').classList.remove('hidden');
}
function scrollTerminal() { const body = $('term-body'); body.scrollTop = body.scrollHeight; }

// ---------------------------------------------------------------- camera: swap, size, boxes, picture

const CAM_SIZES = ['s', 'm', 'l', 'xl'];
const ADJUST_DEFAULT = { zoom: 100, bright: 100, contrast: 100, rotate: 0, mirror: false };
function cameraSettings() {
  let adjust = ADJUST_DEFAULT;
  try { adjust = { ...ADJUST_DEFAULT, ...JSON.parse(store.get('camAdjust', '{}')) }; } catch { /* default */ }
  return { size: store.get('camSize', 'm'), boxes: store.get('camBoxes', '1') === '1', adjust };
}
function applyCamera() {
  const { size, boxes, adjust } = cameraSettings();
  $('pip').dataset.size = CAM_SIZES.includes(size) ? size : 'm';
  $('cam-boxes').setAttribute('aria-pressed', String(boxes));
  $('cam-smaller').disabled = size === CAM_SIZES[0];
  $('cam-bigger').disabled = size === CAM_SIZES[CAM_SIZES.length - 1];
  const transform = `scale(${adjust.zoom / 100}) rotate(${adjust.rotate}deg) scaleX(${adjust.mirror ? -1 : 1})`;
  for (const frame of [$('frame-pip'), $('frame-full')]) {
    frame.style.setProperty('--cam-transform', transform);
    frame.style.setProperty('--cam-filter', `brightness(${adjust.bright}%) contrast(${adjust.contrast}%)`);
  }
  $('adj-zoom').value = adjust.zoom; $('adj-zoom-out').value = `${(adjust.zoom / 100).toFixed(1)}×`;
  $('adj-bright').value = adjust.bright; $('adj-bright-out').value = `${adjust.bright}%`;
  $('adj-contrast').value = adjust.contrast; $('adj-contrast-out').value = `${adjust.contrast}%`;
  $('adj-rotate').setAttribute('aria-pressed', String(adjust.rotate === 180));
  $('adj-mirror').setAttribute('aria-pressed', String(adjust.mirror));
}
function setAdjust(change) {
  store.set('camAdjust', JSON.stringify({ ...cameraSettings().adjust, ...change }));
  applyCamera();
}
function stepSize(delta) {
  const i = CAM_SIZES.indexOf(cameraSettings().size);
  store.set('camSize', CAM_SIZES[Math.max(0, Math.min(CAM_SIZES.length - 1, i + delta))]);
  applyCamera();
}
$('cam-smaller').onclick = () => stepSize(-1);
$('cam-bigger').onclick = () => stepSize(1);
$('cam-boxes').onclick = () => { store.set('camBoxes', cameraSettings().boxes ? '0' : '1'); applyCamera(); };
function toggleAdjust(on) {
  const open = on ?? $('cam-panel').classList.contains('hidden');
  $('cam-panel').classList.toggle('hidden', !open);
  $('cam-adjust').setAttribute('aria-expanded', String(open));
}
$('cam-adjust').onclick = () => toggleAdjust();
$('adj-zoom').oninput = (e) => setAdjust({ zoom: Number(e.target.value) });
$('adj-bright').oninput = (e) => setAdjust({ bright: Number(e.target.value) });
$('adj-contrast').oninput = (e) => setAdjust({ contrast: Number(e.target.value) });
$('adj-rotate').onclick = () => setAdjust({ rotate: cameraSettings().adjust.rotate === 180 ? 0 : 180 });
$('adj-mirror').onclick = () => setAdjust({ mirror: !cameraSettings().adjust.mirror });
$('adj-reset').onclick = () => { store.set('camAdjust', '{}'); applyCamera(); };

function streamUrl() { return `/api/camera/${app.cameraKind}?t=${Date.now()}`; }

function swapCamera() {
  app.cameraBig = !app.cameraBig;
  const canvas = $('scene');
  if (app.cameraBig) {
    $('camera-pip').removeAttribute('src');
    $('camera-pip').classList.add('hidden');
    $('overlay-pip').classList.add('hidden');
    $('frame-pip').prepend(canvas);
    $('camera-full').src = streamUrl();
    $('frame-full').classList.remove('hidden');
    $('pip-label').textContent = '3D';
    $('frame-pip').setAttribute('aria-label', 'Show the 3D view');
  } else {
    $('camera-full').removeAttribute('src');
    $('frame-full').classList.add('hidden');
    $('stage').prepend(canvas);
    $('camera-pip').classList.remove('hidden');
    $('overlay-pip').classList.remove('hidden');
    $('camera-pip').src = streamUrl();
    $('pip-label').textContent = cameraKindLabel(app.cameraKind);
    $('frame-pip').setAttribute('aria-label', 'Enlarge camera');
  }
  requestAnimationFrame(resize);
}
$('frame-pip').onclick = swapCamera;
$('frame-pip').onkeydown = (e) => { if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); swapCamera(); } };

// Boxes around the cubes the dashboard sees, drawn over the picture.
function drawOverlays(time) {
  requestAnimationFrame(drawOverlays);
  if (document.hidden || !app.state) return;
  const { boxes } = cameraSettings();
  const targets = [[$('overlay-pip'), 'cover', !app.cameraBig], [$('overlay-full'), 'contain', app.cameraBig]];
  for (const [canvas, fit, visible] of targets) {
    if (!visible) continue;
    const ctx = canvas.getContext('2d');
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    if (!w || !h) continue;
    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const cubes = app.state.cubes || {};
    if (!cubes.width) continue;
    const scale = fit === 'cover' ? Math.max(w / cubes.width, h / cubes.height) : Math.min(w / cubes.width, h / cubes.height);
    const ox = (w - cubes.width * scale) / 2, oy = (h - cubes.height * scale) / 2;
    const map = ([x1, y1, x2, y2]) => [ox + x1 * scale, oy + y1 * scale, ox + x2 * scale, oy + y2 * scale];
    const small = w < 300;
    if (boxes) {
      const pulse = reducedMotion() ? 1 : 0.78 + 0.22 * Math.sin(time / 220);
      for (const c of cubes.list || []) {
        const ok = allowed(c.color);
        bracket(ctx, map(c.box), colorCss(c.color), ok ? pulse : 0.45, !ok);
        if (!small) label(ctx, map(c.box), `${nice(c.color)}${c.known === false ? ' · not taught' : ''}`, colorCss(c.color), ok ? 1 : 0.55);
      }
      const spots = app.state.teach && !app.state.teach.done ? app.state.teach.spots || [] : [];
      for (const [x, y, kind] of spots) {
        ctx.beginPath();
        ctx.arc(ox + x * scale, oy + y * scale, small ? 2 : 3.5, 0, Math.PI * 2);
        ctx.fillStyle = kind === 'sweep' ? 'rgba(52, 199, 89, 0.9)' : 'rgba(10, 132, 255, 0.95)';
        ctx.fill();
      }
      for (const r of cubes.rejected || []) {
        const [x1, y1, x2, y2] = map(r.box);
        ctx.save();
        ctx.setLineDash([5, 4]);
        ctx.lineWidth = 2;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.85)';
        ctx.shadowColor = 'rgba(0, 0, 0, 0.6)';
        ctx.shadowBlur = 4;
        ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
        ctx.restore();
        if (!small) label(ctx, [x1, y1], `${nice(r.color)}? ${r.reason}`, colorCss(r.color), 0.85);
      }
      const ghosts = app.state.task.running ? app.state.task.current?.targets || [] : [];
      const ghostSize = 28 * scale;
      for (const [gx, gy, color, done] of ghosts) {
        const x = ox + gx * scale, y = oy + gy * scale;
        ctx.save();
        ctx.strokeStyle = color ? colorCss(color) : '#ffffff';
        ctx.fillStyle = color ? colorCss(color) : '#ffffff';
        ctx.lineWidth = 2;
        ctx.setLineDash(done ? [] : [4, 4]);
        ctx.globalAlpha = done ? 0.9 : 0.75;
        ctx.strokeRect(x - ghostSize / 2, y - ghostSize / 2, ghostSize, ghostSize);
        ctx.globalAlpha = done ? 0.35 : 0.12;
        ctx.fillRect(x - ghostSize / 2, y - ghostSize / 2, ghostSize, ghostSize);
        if (done && !small) {
          ctx.globalAlpha = 1;
          ctx.fillStyle = '#fff';
          ctx.font = '700 14px -apple-system, "Segoe UI", sans-serif';
          ctx.fillText('✓', x - 5, y + 5);
        }
        ctx.restore();
      }
      const px = app.state.task.running && app.state.task.current?.pixel;
      const place = app.state.task.running && app.state.task.current?.place_pixel;
      if (px && place) {
        ctx.save();
        ctx.setLineDash([6, 6]);
        ctx.lineDashOffset = reducedMotion() ? 0 : -time / 40;
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(ox + px[0] * scale, oy + px[1] * scale);
        ctx.lineTo(ox + place[0] * scale, oy + place[1] * scale);
        ctx.stroke();
        ctx.restore();
      }
      if (place) {
        target(ctx, ox + place[0] * scale, oy + place[1] * scale, time, small, '#34c759');
        if (!small) label(ctx, [ox + place[0] * scale + 14, oy + place[1] * scale - 4], 'goes here', '#34c759', 1);
      }
      if (px) target(ctx, ox + px[0] * scale, oy + px[1] * scale, time, small);
    }
    if (app.addPoint && fit === 'contain') {
      const tx = ox + app.addPoint.x * scale, ty = oy + app.addPoint.y * scale;
      ctx.save();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.shadowColor = 'rgba(0, 0, 0, 0.7)';
      ctx.shadowBlur = 4;
      ctx.beginPath();
      ctx.arc(tx, ty, 7, 0, Math.PI * 2);
      ctx.moveTo(tx - 13, ty); ctx.lineTo(tx - 4, ty); ctx.moveTo(tx + 4, ty); ctx.lineTo(tx + 13, ty);
      ctx.moveTo(tx, ty - 13); ctx.lineTo(tx, ty - 4); ctx.moveTo(tx, ty + 4); ctx.lineTo(tx, ty + 13);
      ctx.stroke();
      ctx.restore();
    }
    if (app.addPreview?.box) {
      ctx.save();
      ctx.setLineDash([6, 5]);
      ctx.lineWidth = 2;
      ctx.strokeStyle = '#ffffff';
      const [x1, y1, x2, y2] = map(app.addPreview.box);
      ctx.strokeRect(x1 - 4, y1 - 4, x2 - x1 + 8, y2 - y1 + 8);
      ctx.restore();
      if (!small) label(ctx, [x1 - 4, y1 - 4, x2 + 4, y2 + 4], `new: ${app.addPreview.name}?`, app.addPreview.swatch, 1);
    }
  }
}
// Corner brackets just outside the cube: a dark outline under the cube's own
// colour, so they show on any table and never melt into the cube itself.
function bracket(ctx, box, color, alpha, dashed) {
  const pad = Math.max(4, (box[2] - box[0]) * 0.18);
  const [x1, y1, x2, y2] = [box[0] - pad, box[1] - pad, box[2] + pad, box[3] + pad];
  const len = Math.max(5, Math.min(x2 - x1, y2 - y1) * 0.3);
  const path = () => {
    ctx.beginPath();
    for (const [x, y, dx, dy] of [[x1, y1, 1, 1], [x2, y1, -1, 1], [x1, y2, 1, -1], [x2, y2, -1, -1]]) {
      ctx.moveTo(x, y + dy * len); ctx.lineTo(x, y); ctx.lineTo(x + dx * len, y);
    }
  };
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  if (dashed) ctx.setLineDash([3, 4]);
  ctx.strokeStyle = 'rgba(0, 0, 0, 0.55)';
  ctx.lineWidth = dashed ? 4 : 5.5;
  path();
  ctx.stroke();
  ctx.strokeStyle = color;
  ctx.shadowColor = color;
  ctx.shadowBlur = 10;
  ctx.lineWidth = dashed ? 2 : 3;
  path();
  ctx.stroke();
  ctx.restore();
}
function label(ctx, [x1, y1], text, color, alpha) {
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.font = '600 12px -apple-system, "SF Pro Text", "Segoe UI", Roboto, sans-serif';
  const width = ctx.measureText(text).width + 14;
  const y = Math.max(2, y1 - 24);
  ctx.fillStyle = 'rgba(0, 0, 0, 0.62)';
  ctx.beginPath();
  if (ctx.roundRect) ctx.roundRect(x1, y, width, 19, 9.5);
  else ctx.rect(x1, y, width, 19);
  ctx.fill();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(x1 + 9, y + 9.5, 3.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.fillStyle = '#fff';
  ctx.fillText(text, x1 + 16, y + 13.5);
  ctx.restore();
}
function target(ctx, x, y, time, small, color = '#ff3b30') {
  const r = (small ? 10 : 18) + (reducedMotion() ? 0 : 4 * Math.sin(time / 160));
  ctx.save();
  ctx.strokeStyle = color;
  ctx.shadowColor = color;
  ctx.shadowBlur = 12;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(x, y, r, 0, Math.PI * 2);
  for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
    ctx.moveTo(x + dx * (r - 6), y + dy * (r - 6)); ctx.lineTo(x + dx * (r + 8), y + dy * (r + 8));
  }
  ctx.stroke();
  ctx.restore();
}
requestAnimationFrame(drawOverlays);

// ---------------------------------------------------------------- celebrations

const confetti = { parts: [], running: false };
const PARTY = ['#ff3b30', '#ff9500', '#ffcc00', '#34c759', '#32ade6', '#007aff', '#af52de', '#ff2d92'];

function celebrate(origin, count = 100, colors = PARTY) {
  if (reducedMotion() || store.get('party', 'true') !== 'true') return;
  const canvas = $('confetti');
  let rect = origin?.getBoundingClientRect?.();
  if (rect && !rect.width) rect = null;   // hidden (another tab): burst from the middle
  const x = rect ? rect.left + rect.width / 2 : window.innerWidth / 2;
  const y = rect ? rect.top + rect.height / 2 : window.innerHeight * 0.35;
  const palette = colors.length > 2 ? colors : [...colors, ...PARTY.slice(0, 3)];
  for (let i = 0; i < count; i += 1) {
    const angle = Math.random() * Math.PI * 2;
    const speed = (rect ? 4 : 7) + Math.random() * (rect ? 6 : 9);
    confetti.parts.push({
      x, y, vx: Math.cos(angle) * speed, vy: Math.sin(angle) * speed - (rect ? 4 : 6),
      size: 5 + Math.random() * 6, spin: Math.random() * 6, turn: (Math.random() - 0.5) * 0.4,
      color: palette[i % palette.length], life: 1,
    });
  }
  if (!confetti.running) {
    confetti.running = true;
    canvas.width = window.innerWidth; canvas.height = window.innerHeight;
    requestAnimationFrame(confettiFrame);
  }
}
function confettiFrame() {
  const canvas = $('confetti');
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  for (const p of confetti.parts) {
    p.vy += 0.32; p.vx *= 0.985; p.vy *= 0.985;
    p.x += p.vx; p.y += p.vy; p.spin += p.turn; p.life -= 0.011;
    ctx.save();
    ctx.globalAlpha = Math.max(0, Math.min(1, p.life * 2));
    ctx.translate(p.x, p.y);
    ctx.rotate(p.spin);
    ctx.fillStyle = p.color;
    ctx.fillRect(-p.size / 2, -p.size / 4, p.size, p.size / 2);
    ctx.restore();
  }
  confetti.parts = confetti.parts.filter((p) => p.life > 0 && p.y < canvas.height + 40);
  if (confetti.parts.length) requestAnimationFrame(confettiFrame);
  else { confetti.running = false; ctx.clearRect(0, 0, canvas.width, canvas.height); }
}

// ---------------------------------------------------------------- devices (Settings)

const STATE_TEXT = {
  ready: ['Connected', 'ok'], streaming: ['Streaming', 'ok'], starting: ['Powering up…', 'busy'],
  connecting: ['Connecting…', 'busy'], switching: ['Switching…', 'busy'], searching: ['Not found', 'bad'],
  stalled: ['No pictures', 'warn'], error: ['Problem', 'bad'],
};
let devicesTimer = null;

function openDevices() {
  openSettings();
  requestAnimationFrame(() => $('devices').scrollIntoView({ behavior: 'smooth', block: 'start' }));
}

async function loadDevices() {
  clearTimeout(devicesTimer);
  if ($('settings').classList.contains('hidden')) return;
  try {
    const data = await api('/api/devices');
    renderDevice('arm', data);
    renderDevice('camera', data);
    renderUsbList(data);
  } catch { /* signed out or offline */ }
  devicesTimer = setTimeout(loadDevices, 2000);
}

// [group label or '', [[value, label], ...]]: every camera that could be used, grouped.
function cameraOptions(data) {
  const groves = data.serial.filter((p) => p.kind === 'camera');
  const others = data.serial.filter((p) => p.kind === 'other');
  return [
    ['', [['auto', 'Automatic (Grove by USB id)']]],
    ['Grove Vision AI V2', groves.map((p) => [`grove:${p.port}`, `Grove · ${p.port}${p.alias ? ` (${p.alias})` : ''}`])],
    ['Cameras', data.video.map((v) => [v.port,
      `${v.builtin ? 'Built-in' : 'USB'} camera · ${v.name.replace(/:.*$/, '')} · ${v.port}`])],
    ['Other USB serial (might be a Grove)', others.map((p) => [`grove:${p.port}`, `${p.name} · ${p.port}`])],
    ['Network', [['url', 'Stream URL…']]],
  ].filter(([, options]) => options.length);
}
function armOptions(data) {
  return [
    ['', [['auto', 'Automatic (UNO by USB id)']]],
    ['USB serial', data.serial.filter((p) => p.kind !== 'camera').map((p) => [p.port, `${p.name} · ${p.port}`])],
  ].filter(([, options]) => options.length);
}

function renderDevice(device, data) {
  const box = $(`dev-${device}`);
  const status = data.status[device];
  const choice = data.choices[device];
  const [stateText, dot] = status ? (STATE_TEXT[status.state] || [status.state, 'warn']) : ['Not running', 'bad'];
  const where = status?.port ? ` · ${status.port}` : '';
  const fps = device === 'camera' && status?.state === 'streaming' && status.fps ? ` · ${status.fps} fps` : '';
  const owner = app.user?.role === 'owner';

  // Rebuild only when something visible changed, so an open <select> is not reset under the finger.
  const key = JSON.stringify([status?.state, status?.port, status?.detail, fps, choice, owner,
    device === 'arm' ? armOptions(data) : cameraOptions(data)]);
  if (box.dataset.key === key) return;
  if (box.contains(document.activeElement) && document.activeElement.tagName !== 'BUTTON') return;
  box.dataset.key = key;

  const head = el('div', 'device-head');
  head.append(el('span', 'device-icon', device === 'arm' ? '🦾' : '📷'));
  const title = el('div', 'device-title');
  title.append(el('strong', '', device === 'arm' ? 'Arm' : 'Camera'),
    el('span', 'secondary small', choice === 'auto' ? 'Automatic' : `Pinned to ${choice}`));
  const badge = el('span', `device-state ${dot}`);
  badge.append(el('span', `dot ${dot}`), `${stateText}${where}${fps}`);
  head.append(title, badge);
  box.replaceChildren(head);
  if (status?.detail && !['ready', 'streaming'].includes(status.state)) box.append(el('p', 'device-detail', status.detail));
  if (!status) box.append(el('p', 'device-detail', device === 'arm' ? 'The serial bridge is not running (start pi.launch.py or real.launch.py).' : 'The camera node is not running.'));
  if (!owner) return;

  const row = el('div', 'device-row');
  const select = el('select');
  select.setAttribute('aria-label', `${device} source`);
  const groups = device === 'arm' ? armOptions(data) : cameraOptions(data);
  for (const [label, options] of groups) {
    const parent = label ? Object.assign(document.createElement('optgroup'), { label }) : select;
    for (const [value, text] of options) parent.append(new Option(text, value));
    if (label) select.append(parent);
  }
  const values = groups.flatMap(([, options]) => options.map(([v]) => v));
  const isUrl = device === 'camera' && /^(https?|rtsp):\/\//.test(choice);
  select.value = isUrl ? 'url' : values.includes(choice) ? choice : 'auto';
  const url = el('input', 'device-url hidden');
  Object.assign(url, { type: 'url', placeholder: 'http://192.168.1.50:8080/video', value: isUrl ? choice : '' });
  select.onchange = () => url.classList.toggle('hidden', select.value !== 'url');
  url.classList.toggle('hidden', select.value !== 'url');
  const use = el('button', 'btn tinted small-btn', 'Use');
  use.onclick = async () => {
    const source = select.value === 'url' ? url.value.trim() : select.value;
    if (device === 'arm' && source !== choice && !confirm('Switching the arm port restarts the arm (it stands up). Go ahead?')) return;
    try {
      await api('/api/devices/use', { device, source });
      toast(source === 'auto' ? `${nice(device)}: back to automatic.` : `${nice(device)}: using ${source}.`);
      box.dataset.key = '';
      loadDevices();
    } catch (err) { toast(err.message); }
  };
  const again = el('button', 'btn plain small-btn', 'Reconnect');
  again.onclick = async () => {
    if (device === 'arm' && !confirm('Reconnecting restarts the arm (it stands up). Go ahead?')) return;
    try { await api('/api/devices/reconnect', { device }); toast(`${nice(device)}: reconnecting…`); } catch (err) { toast(err.message); }
  };
  row.append(select, use, again);
  box.append(row, url);
}

function renderUsbList(data) {
  const rows = [...data.serial.map((p) => ({ ...p, type: 'serial' })), ...data.video.map((v) => ({ ...v, kind: 'video', type: 'video' }))];
  const key = JSON.stringify(rows);
  if ($('usb-list').dataset.key === key) return;
  $('usb-list').dataset.key = key;
  if (!rows.length) {
    $('usb-list').replaceChildren(el('div', 'cell secondary', 'No USB devices found. Check the cables and hub.'));
    return;
  }
  $('usb-list').replaceChildren(...rows.map((d) => {
    const row = el('div', 'cell usb-row');
    const text = el('span', 'usb-text');
    text.append(el('strong', '', d.name), el('span', 'secondary small',
      `${d.port}${d.alias ? ` (${d.alias})` : ''}${d.usb_id ? ` · ${d.usb_id}` : ''}`));
    const tag = d.used_by ? `${nice(d.used_by)}`
      : d.kind === 'video' ? (d.builtin ? 'Built-in camera' : 'USB camera')
        : { arm: 'Arm board', camera: 'Grove camera' }[d.kind] || 'Not used';
    row.append(text, el('span', `usb-tag ${d.used_by ? 'used' : ''}`, tag));
    return row;
  }));
}

// ---------------------------------------------------------------- settings

function buildSettings() {
  $('me-name').textContent = app.user.user;
  $('me-role').textContent = nice(app.user.role);
  document.querySelectorAll('.owner-only').forEach((node) => node.classList.toggle('hidden', app.user.role !== 'owner'));
  $('party').checked = store.get('party', 'true') === 'true';
}

$('session').onchange = () => loadReview();
$('party').onchange = (e) => { store.set('party', String(e.target.checked)); if (e.target.checked) celebrate(e.target, 40); };

async function openSettings() {
  $('settings').classList.remove('hidden');
  $('settings-dim').classList.remove('hidden');
  $('me-mode').textContent = { real: 'Real arm', sim: 'Simulation', offline: 'Not connected' }[app.state?.mode] || '—';
  for (const box of [$('dev-arm'), $('dev-camera'), $('usb-list')]) box.dataset.key = '';
  loadDevices();
  if (app.user.role === 'owner') {
    loadPeople();
    loadReview();
  }
}
function closeSettings() {
  $('settings').classList.add('hidden');
  $('settings-dim').classList.add('hidden');
  $('invite-code').classList.add('hidden');
}
$('open-settings').onclick = openSettings;
$('close-settings').onclick = closeSettings;
$('settings-dim').onclick = closeSettings;

wireSegmented($('theme'), (v) => { store.set('theme', v); applyAppearance(); });
$('solid').onchange = (e) => { store.set('solid', String(e.target.checked)); applyAppearance(); };
wireSegmented($('invite-role'));

$('logout').onclick = async () => {
  try { await api('/api/logout', {}); } catch { /* already gone */ }
  closeSettings();
  showAuth();
};
$('release-control').onclick = async () => {
  try { await api('/api/control/release', {}); toast('Others can control the arm now.'); } catch (err) { toast(err.message); }
};

async function loadPeople() {
  try {
    const { people } = await api('/api/people');
    $('people').replaceChildren(...people.map((p) => {
      const row = el('div', 'cell');
      const name = el('span');
      name.append(el('strong', '', p.name), ' ', el('span', 'person-role', nice(p.role)));
      row.append(name);
      if (p.name !== app.user.user) {
        const remove = el('button', 'btn plain destructive-text', 'Remove');
        remove.onclick = async () => {
          if (!confirm(`Remove ${p.name}? They will be signed out.`)) return;
          await api('/api/people/remove', { name: p.name }).catch((err) => toast(err.message));
          loadPeople();
        };
        row.append(remove);
      }
      return row;
    }));
  } catch { /* not owner */ }
}

$('invite').onclick = async () => {
  try {
    const { code, role, minutes } = await api('/api/people/code', { role: segmentedValue($('invite-role')) });
    const box = $('invite-code');
    box.textContent = code;
    box.append(el('small', '', `On the new device, open this page, tap "Have a pairing code?" and enter it. Joins as ${role}; valid ${minutes} minutes, once.`));
    box.classList.remove('hidden');
  } catch (err) { toast(err.message); }
};

async function loadReview() {
  const name = $('session').value;
  if (!name || app.user?.role !== 'owner') return;
  try {
    const { lines } = await api(`/api/review?session=${encodeURIComponent(name)}`);
    $('review').textContent = lines.join('\n');
  } catch (err) { $('review').textContent = err.message; }
}

const cameraKindLabel = (kind, button = false) => (button
  ? (app.cameraKind === 'mask' ? 'Back to the normal picture' : 'Show the colour mask')
  : { mask: 'Colour mask', detections: 'Detections' }[kind] || 'Camera');

function setCameraKind(kind) {
  app.cameraKind = kind;
  $('cam-mask').setAttribute('aria-pressed', String(kind === 'mask'));
  $('detections').checked = kind === 'detections';
  if (app.cameraBig) $('camera-full').src = streamUrl();
  else { $('camera-pip').src = streamUrl(); $('pip-label').textContent = cameraKindLabel(kind); }
  app.tableKey = '';
}
$('detections').onchange = (e) => setCameraKind(e.target.checked ? 'detections' : 'camera');
$('cam-mask').onclick = () => setCameraKind(app.cameraKind === 'mask' ? 'camera' : 'mask');

// ---------------------------------------------------------------- go

applyAppearance();
start();
