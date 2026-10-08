// Braccio dashboard: sign in, 3D arm from the URDF + live transforms, camera,
// sorting, manual moves and settings. Served by web_dashboard.py.

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
};

const COLOR_CSS = {
  red: '#ff3b30', orange: '#ff9500', yellow: '#ffcc00', green: '#34c759', cyan: '#32ade6',
  blue: '#007aff', purple: '#af52de', magenta: '#ff2d92', pink: '#ff6fae', white: '#f2f2f7', black: '#1c1c1e',
};
const colorCss = (name) => COLOR_CSS[String(name || '').replace(/\d+$/, '')] || '#8e8e93';
const nice = (name) => String(name).replace(/(\d+)$/, ' $1').replace(/^./, (c) => c.toUpperCase());

const STEPS = {
  look: ['DETECTING', 'TARGET_CONFIRMED', 'GO_HOME'],
  reach: ['MOVE_ABOVE_CUBE', 'VERIFY_CUBE', 'DESCEND'],
  grip: ['GRASP', 'LIFT'],
  check: ['VERIFY_GRASP', 'VERIFY_FAILED'],
  drop: ['MOVE_TO_BIN', 'LOWER', 'RELEASE', 'RETREAT', 'VERIFY_PLACEMENT'],
};
const ORDER = Object.keys(STEPS);

function describe(task) {
  const c = task.current || {};
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
    MOVE_TO_BIN: `Carrying the ${cubeText}`,
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
  $('auth').classList.add('hidden');
  $('stage').classList.remove('hidden');
  $('app').classList.remove('hidden');
  document.body.dataset.role = app.user.role;
  $('avatar').textContent = app.user.user.slice(0, 1);
  buildSort();
  buildMove();
  buildSettings();
  initView();
  loadScene().catch(() => setTimeout(loadScene, 2000));
  openEvents();
  setTab(store.get('tab', 'sort'));
  moveTabIndicator();
  reframe();
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

  // stage placeholder
  const waiting = !view.posed;
  $('stage-empty').classList.toggle('hidden', !waiting || app.cameraBig);
  $('stage-empty-text').textContent = s.mode === 'offline'
    ? 'The arm is not connected. Start real.launch.py or sim.launch.py.'
    : 'Loading the 3D model…';

  // status capsule
  const modeText = { real: 'Real arm', sim: 'Simulation', offline: 'Not connected' }[s.mode];
  const controlText = s.control ? (s.control.mine ? 'You have control' : `${s.control.user} has control`) : '';
  const sub = [modeText, controlText].filter(Boolean).join(' · ');
  if (s.mode === 'offline') setStatus('Arm offline', sub, 'bad');
  else if (s.task.running) setStatus(describe(s.task), sub, 'busy');
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
  for (const el of document.querySelectorAll('#panel button:not(#grabber):not(#take-control), #panel input')) {
    if (el.closest('#control-banner')) continue;
    el.disabled = blocked;
  }
  $('stop').disabled = !canControl() || s.mode === 'offline';
  $('stop').classList.toggle('armed', s.task.running);
  $('release-control').classList.toggle('hidden', !driving());

  // sorting progress
  const running = s.task.running;
  $('sort-setup').classList.toggle('hidden', running);
  $('sort-running').classList.toggle('hidden', !running);
  if (running) {
    if (!previous?.task.running) app.placed = [];
    $('run-title').textContent = describe(s.task);
    const colors = (s.task.colors || []).map((c) => (c === 'all' ? 'all colours' : nice(c).toLowerCase()));
    $('run-sub').textContent = `Sorting ${colors.join(', ')}${s.task.by ? ` · started by ${s.task.by}` : ''}`;
    const at = ORDER.findIndex((k) => STEPS[k].includes(s.task.state));
    for (const li of $('steps').children) {
      const i = ORDER.indexOf(li.dataset.step);
      li.className = at < 0 ? '' : i < at ? 'done' : i === at ? 'now' : '';
    }
    if (s.task.state === 'RELEASE' && app.lastState !== 'RELEASE') {
      app.placed.push(s.task.current?.bin || s.task.current?.cube);
      renderPlaced();
    }
  } else if (previous?.task.running && s.task.state === 'COMPLETE') {
    const n = app.placed.length;
    toast(n ? `Done — sorted ${n} cube${n === 1 ? '' : 's'}.` : 'Done — no cubes it could pick.');
  }
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
  if (hasCamera && !$('camera-pip').src && !app.cameraBig) $('camera-pip').src = streamUrl();

  if (!$('settings').classList.contains('hidden')) {
    $('raw').textContent = `now     ${JSON.stringify(s.servo)}\ntarget  ${JSON.stringify(s.target)}\nmode    ${s.mode}\nstate   ${s.task.state}`;
  }
}

const article = (device) => (/^[aeiou]/i.test(device || '') ? `an ${device}` : `a ${device || 'browser'}`);

function renderPlaced() {
  $('placed').replaceChildren(...app.placed.map((c) => {
    const s = document.createElement('span');
    s.className = 'swatch';
    s.title = nice(c || 'cube');
    s.style.background = colorCss(c);
    return s;
  }));
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
  if (!$('app').classList.contains('hidden')) stopEverything();
});

// ---------------------------------------------------------------- sort tab

function buildSort() {
  const chips = $('color-chips');
  chips.replaceChildren();
  const all = chip('All', null, true);
  all.dataset.all = '1';
  chips.append(all);
  for (const c of app.info.colors) chips.append(chip(nice(c), c, false));
  chips.onclick = (e) => {
    const b = e.target.closest('.chip');
    if (!b) return;
    if (b.dataset.all) app.selected.clear();
    else if (app.selected.has(b.dataset.color)) app.selected.delete(b.dataset.color);
    else app.selected.add(b.dataset.color);
    for (const el of chips.children) {
      el.setAttribute('aria-pressed', String(el.dataset.all ? app.selected.size === 0 : app.selected.has(el.dataset.color)));
    }
    updateStartLabel();
  };
  setSegmented($('drop-mode'), store.get('drop', 'auto'));
  updateStartLabel();
}

function chip(label, color, pressed) {
  const b = document.createElement('button');
  b.className = color ? 'chip' : 'chip plain-chip';
  b.setAttribute('aria-pressed', String(pressed));
  if (color) {
    b.dataset.color = color;
    const sw = document.createElement('span');
    sw.className = 'swatch';
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

wireSegmented($('drop-mode'), (v) => store.set('drop', v));
$('start-sort').onclick = async () => {
  $('sort-error').textContent = '';
  const advancedDrop = $('drop-advanced').value;
  const body = {
    colors: [...app.selected],
    drop: advancedDrop || segmentedValue($('drop-mode')),
    session: $('session').value || '',
    model: $('model').value || '',
  };
  if (!(await ensureControl())) return;
  try { await api('/api/task/start', body); } catch (err) { $('sort-error').textContent = err.message; }
};
$('stop-sort').onclick = () => drive('/api/task/stop', {});

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
    const row = document.createElement('div');
    row.className = 'joint';
    row.innerHTML = `<label for="joint-${i}">${JOINT_LABELS[i]}</label>
      <input type="range" id="joint-${i}" min="${lo}" max="${hi}" step="1" value="${Math.round((lo + hi) / 2)}">
      <output>–</output>`;
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

function setTab(tab) {
  app.tab = tab === 'move' ? 'move' : 'sort';
  store.set('tab', app.tab);
  $('tab-sort').classList.toggle('hidden', app.tab !== 'sort');
  $('tab-move').classList.toggle('hidden', app.tab !== 'move');
  for (const b of $('tabs').querySelectorAll('button')) b.setAttribute('aria-selected', String(b.dataset.tab === app.tab));
  $('panel').classList.remove('collapsed');
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

// ---------------------------------------------------------------- camera swap

function streamUrl() { return `/api/camera/${app.cameraKind}?t=${Date.now()}`; }

$('pip').onclick = () => {
  app.cameraBig = !app.cameraBig;
  const canvas = $('scene');
  if (app.cameraBig) {
    $('camera-pip').removeAttribute('src');
    $('camera-pip').classList.add('hidden');
    $('pip').append(canvas);
    $('camera-full').src = streamUrl();
    $('camera-full').classList.remove('hidden');
    $('pip-label').textContent = '3D';
    $('pip').setAttribute('aria-label', 'Show the 3D view');
  } else {
    $('camera-full').removeAttribute('src');
    $('camera-full').classList.add('hidden');
    $('stage').prepend(canvas);
    $('camera-pip').classList.remove('hidden');
    $('camera-pip').src = streamUrl();
    $('pip-label').textContent = app.cameraKind === 'detections' ? 'Detections' : 'Camera';
    $('pip').setAttribute('aria-label', 'Enlarge camera');
  }
  $('pip').append($('pip-label'));
  requestAnimationFrame(resize);
};

// ---------------------------------------------------------------- settings

function buildSettings() {
  $('me-name').textContent = app.user.user;
  $('me-role').textContent = nice(app.user.role);
  document.querySelectorAll('.owner-only').forEach((el) => el.classList.toggle('hidden', app.user.role !== 'owner'));
  const sessions = app.info.sessions || [];
  const select = $('session');
  select.replaceChildren(...sessions.map((s) => new Option(`${s.name} (${s.examples} examples)`, s.name)));
  if (app.info.default_session) select.value = app.info.default_session;
  fillModels();
}

function fillModels() {
  const session = (app.info.sessions || []).find((s) => s.name === $('session').value);
  const models = session ? [...session.models].reverse() : [];
  $('model').replaceChildren(new Option('Latest', ''), ...models.map((m) => new Option(m.replace('model_', 'Model '), m)));
}

$('session').onchange = () => { fillModels(); loadReview(); };

async function openSettings() {
  $('settings').classList.remove('hidden');
  $('settings-dim').classList.remove('hidden');
  $('me-mode').textContent = { real: 'Real arm', sim: 'Simulation', offline: 'Not connected' }[app.state?.mode] || '—';
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
      const row = document.createElement('div');
      row.className = 'cell';
      const name = document.createElement('span');
      name.innerHTML = `<strong></strong> <span class="person-role"></span>`;
      name.querySelector('strong').textContent = p.name;
      name.querySelector('.person-role').textContent = nice(p.role);
      row.append(name);
      if (p.name !== app.user.user) {
        const remove = document.createElement('button');
        remove.className = 'btn plain destructive-text';
        remove.textContent = 'Remove';
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
    const note = document.createElement('small');
    note.textContent = `On the new device, open this page, tap "Have a pairing code?" and enter it. Joins as ${role}; valid ${minutes} minutes, once.`;
    box.append(note);
    box.classList.remove('hidden');
  } catch (err) { toast(err.message); }
};

async function loadReview() {
  const name = $('session').value;
  if (!name || app.user.role !== 'owner') return;
  try {
    const { lines } = await api(`/api/review?session=${encodeURIComponent(name)}`);
    $('review').textContent = lines.join('\n');
  } catch (err) { $('review').textContent = err.message; }
}

$('detections').onchange = (e) => {
  app.cameraKind = e.target.checked ? 'detections' : 'camera';
  if (app.cameraBig) $('camera-full').src = streamUrl();
  else { $('camera-pip').src = streamUrl(); $('pip-label').textContent = e.target.checked ? 'Detections' : 'Camera'; }
};

setInterval(async () => {
  const advanced = document.querySelector('.advanced');
  if ($('settings').classList.contains('hidden') || !advanced?.open || app.user?.role !== 'owner') return;
  try { $('log').textContent = (await api('/api/log')).lines.join('\n'); } catch { /* ignore */ }
}, 2000);

// ---------------------------------------------------------------- go

applyAppearance();
start();
