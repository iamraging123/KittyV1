import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { OBJLoader }     from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader }     from 'three/addons/loaders/MTLLoader.js';

/* ---------- shared attitude + altitude state ---------- */
let roll = 0, pitch = 0, yaw = 0;
let altitude = 0, verticalVelocity = 0;

/* ---------- shared GPS state ---------- */
let gpsLat = 0, gpsLon = 0, gpsAlt = 0, gpsFix = 0;
let gpsHasFix = false;
let gpsPrev = null;            // last fixed sample, for ground-speed estimate
let gpsSpeed = 0;              // m/s over ground (derived)

/* ============================================
   THREE.JS SCENE
   ============================================ */
const scene    = new THREE.Scene();
scene.background = new THREE.Color(0x141414);

/* The renderer mounts inside a layout box (#viewport3d) and sizes to it,
   rather than filling the whole window like the original single-canvas page. */
const viewport3d = document.getElementById('viewport3d');
const vpW = () => viewport3d.clientWidth  || innerWidth;
const vpH = () => viewport3d.clientHeight || innerHeight;

const camera   = new THREE.PerspectiveCamera(60, vpW() / vpH(), 0.1, 100);
camera.position.set(3, 2, 3);
camera.lookAt(0, 0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(vpW(), vpH());
renderer.setPixelRatio(devicePixelRatio);
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.0;
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
viewport3d.appendChild(renderer.domElement);

/* Keep the camera + drawing buffer matched to the box, not the window. */
function sizeRenderer() {
  const w = vpW(), h = vpH();
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
  renderer.setSize(w, h);
}
if (window.ResizeObserver) new ResizeObserver(sizeRenderer).observe(viewport3d);

/* Orbit controls — right-click to rotate camera, scroll to zoom */
const controls = new OrbitControls(camera, renderer.domElement);
controls.mouseButtons = { LEFT: null, MIDDLE: THREE.MOUSE.DOLLY, RIGHT: THREE.MOUSE.ROTATE };
controls.enableDamping = true;
controls.dampingFactor = 0.1;

/* Rotation group — handles attitude rotation at the origin.
   Content group — sits inside, holds the model geometry.
   Offset shifts contentGroup so the rotation axis passes
   through a specific point on the model. */
const rotationGroup = new THREE.Group();
const contentGroup  = new THREE.Group();
rotationGroup.add(contentGroup);
scene.add(rotationGroup);

/* Default box */
const geometry = new THREE.BoxGeometry(1.0, 0.15, 1);
const boxMaterials = [
  new THREE.MeshStandardMaterial({ color: 0xff3030, metalness: 0.15, roughness: 0.4 }), // +X  red
  new THREE.MeshStandardMaterial({ color: 0xaa1818, metalness: 0.15, roughness: 0.4 }), // -X  dark red
  new THREE.MeshStandardMaterial({ color: 0x30ff50, metalness: 0.15, roughness: 0.4 }), // +Y  green
  new THREE.MeshStandardMaterial({ color: 0x18aa28, metalness: 0.15, roughness: 0.4 }), // -Y  dark green
  new THREE.MeshStandardMaterial({ color: 0x3050ff, metalness: 0.15, roughness: 0.4 }), // +Z  blue
  new THREE.MeshStandardMaterial({ color: 0x1818aa, metalness: 0.15, roughness: 0.4 }), // -Z  dark blue
];
const cube = new THREE.Mesh(geometry, boxMaterials);
cube.castShadow = true;
cube.receiveShadow = true;
contentGroup.add(cube);

scene.add(new THREE.GridHelper(6, 24, 0x555555, 0x2a2a2a));
scene.add(new THREE.AxesHelper(2));

/* Shadow-receiving ground plane */
const groundGeo = new THREE.PlaneGeometry(12, 12);
const groundMat = new THREE.ShadowMaterial({ opacity: 0.45 });
const ground = new THREE.Mesh(groundGeo, groundMat);
ground.rotation.x = -Math.PI / 2;
ground.position.y = -0.005;
ground.receiveShadow = true;
scene.add(ground);

/* Hemisphere light for ambient sky/ground colour variation */
const hemi = new THREE.HemisphereLight(0x6688cc, 0x443322, 0.7);
scene.add(hemi);

const dir = new THREE.DirectionalLight(0xfff0e0, 1.2);
dir.position.set(5, 10, 7);
dir.castShadow = true;
dir.shadow.mapSize.width  = 2048;
dir.shadow.mapSize.height = 2048;
dir.shadow.camera.near = 0.5;
dir.shadow.camera.far  = 30;
dir.shadow.camera.left   = -4;
dir.shadow.camera.right  =  4;
dir.shadow.camera.top    =  4;
dir.shadow.camera.bottom = -4;
dir.shadow.bias = -0.0005;
dir.shadow.radius = 3;
scene.add(dir);
const fill = new THREE.DirectionalLight(0xc0d0ff, 0.5);
fill.position.set(-3, -2, -5);
scene.add(fill);
const rim = new THREE.DirectionalLight(0xffffff, 0.35);
rim.position.set(-5, 3, 8);
scene.add(rim);

/* ---------- Procedural normal-map texture for surface detail ---------- */
function createSubtleNormalMap(size) {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = size;
  const ctx = canvas.getContext('2d');
  const img = ctx.createImageData(size, size);
  const d = img.data;
  for (let i = 0; i < size * size; i++) {
    /* Slight random perturbation around flat normal (128,128,255) */
    const jitter = 6;
    d[i * 4]     = 128 + (Math.random() - 0.5) * jitter; // R = X
    d[i * 4 + 1] = 128 + (Math.random() - 0.5) * jitter; // G = Y
    d[i * 4 + 2] = 255;                                   // B = Z
    d[i * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  const tex = new THREE.CanvasTexture(canvas);
  tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
  tex.repeat.set(4, 4);
  return tex;
}
const subtleNormal = createSubtleNormalMap(128);

window.addEventListener('resize', sizeRenderer);

/* ============================================
   STEP FILE LOADER  (occt-import-js, lazy)
   ============================================ */
/* ---------- IndexedDB helpers for model persistence ---------- */
function openModelDB() {
  return new Promise((resolve, reject) => {
    const req = indexedDB.open('StepFileStore', 2);
    req.onupgradeneeded = (e) => {
      const db = req.result;
      if (!db.objectStoreNames.contains('files')) db.createObjectStore('files');
    };
    req.onsuccess = () => resolve(req.result);
    req.onerror   = () => reject(req.error);
  });
}
async function dbPut(key, value) {
  const db = await openModelDB();
  return new Promise((resolve, reject) => {
    const tx = db.transaction('files', 'readwrite');
    tx.objectStore('files').put(value, key);
    tx.oncomplete = resolve;
    tx.onerror    = () => reject(tx.error);
  });
}
async function dbGet(key) {
  const db = await openModelDB();
  return new Promise((resolve, reject) => {
    const tx  = db.transaction('files', 'readonly');
    const req = tx.objectStore('files').get(key);
    req.onsuccess = () => resolve(req.result || null);
    req.onerror   = () => reject(req.error);
  });
}
/* Convenience wrappers */
async function saveStepBytes(bytes) { await dbPut('current', bytes); await dbPut('modelType', 'step'); }
async function loadStepBytes()      { return dbGet('current'); }
async function saveObjData(objText, mtlText) {
  await dbPut('objText', objText);
  await dbPut('mtlText', mtlText);
  await dbPut('modelType', 'obj');
}

let occt = null;

async function initOcct() {
  if (occt) return;
  await new Promise((resolve, reject) => {
    const s = document.createElement('script');
    s.src = 'https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/occt-import-js.js';
    s.onload = resolve;
    s.onerror = () => reject(new Error('Failed to load occt-import-js'));
    document.head.appendChild(s);
  });
  occt = await occtimportjs({
    locateFile: (name) => `https://cdn.jsdelivr.net/npm/occt-import-js@0.0.23/dist/${name}`
  });
}

async function loadStepFromBytes(buffer, save) {
  const loading = document.getElementById('loadingOverlay');
  loading.style.display = 'flex';

  try {
    await initOcct();

    const result = occt.ReadStepFile(buffer, null);

    clearContentGroup();

    /* Build meshes from STEP data */
    for (let i = 0; i < result.meshes.length; i++) {
      const md = result.meshes[i];
      const geo = new THREE.BufferGeometry();

      const posArr = md.attributes.position.array;
      geo.setAttribute('position',
        new THREE.Float32BufferAttribute(posArr, 3));

      if (md.attributes.normal) {
        geo.setAttribute('normal',
          new THREE.Float32BufferAttribute(md.attributes.normal.array, 3));
      } else {
        geo.computeVertexNormals();
      }

      const indexArr = new Uint32Array(md.index.array);
      geo.setIndex(new THREE.BufferAttribute(indexArr, 1));

      /* --- Per-face vertex colours from brep_faces --- */
      let hasPerFaceColor = false;
      if (md.brep_faces && md.brep_faces.length > 0) {
        const vertCount = posArr.length / 3;
        const colBuf = new Float32Array(vertCount * 3);
        /* Default: fill with mesh-level colour or neutral grey */
        const fallback = md.color
          ? new THREE.Color(md.color[0], md.color[1], md.color[2])
          : new THREE.Color(0x99aacc);
        for (let v = 0; v < vertCount; v++) {
          colBuf[v * 3]     = fallback.r;
          colBuf[v * 3 + 1] = fallback.g;
          colBuf[v * 3 + 2] = fallback.b;
        }

        for (const face of md.brep_faces) {
          if (!face.color) continue;
          hasPerFaceColor = true;
          const fc = new THREE.Color(face.color[0], face.color[1], face.color[2]);
          /* Paint every vertex referenced by this face's triangles */
          const triStart = face.first_tri_index;
          const triEnd   = triStart + face.triangle_count;
          for (let t = triStart; t < triEnd; t++) {
            for (let k = 0; k < 3; k++) {
              const vi = indexArr[t * 3 + k];
              colBuf[vi * 3]     = fc.r;
              colBuf[vi * 3 + 1] = fc.g;
              colBuf[vi * 3 + 2] = fc.b;
            }
          }
        }

        if (hasPerFaceColor) {
          geo.setAttribute('color', new THREE.Float32BufferAttribute(colBuf, 3));
        }
      }

      /* Mesh-level fallback colour */
      let meshColor;
      if (md.color) {
        meshColor = new THREE.Color(md.color[0], md.color[1], md.color[2]);
      } else {
        meshColor = new THREE.Color(0x99aacc);
      }

      const mat = new THREE.MeshStandardMaterial({
        color: hasPerFaceColor ? 0xffffff : meshColor,
        vertexColors: hasPerFaceColor,
        side: THREE.DoubleSide,
        metalness: 0.2,
        roughness: 0.45,
        normalMap: subtleNormal,
        normalScale: new THREE.Vector2(0.3, 0.3)
      });

      const mesh = new THREE.Mesh(geo, mat);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      contentGroup.add(mesh);
    }

    /* Center and scale to fit ~2 units */
    const box3 = new THREE.Box3().setFromObject(contentGroup);
    const center = box3.getCenter(new THREE.Vector3());
    const size = box3.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    const scale = 2.0 / maxDim;

    for (const child of contentGroup.children) {
      child.position.sub(center);
      child.scale.setScalar(scale);
    }

    if (save) await saveStepBytes(buffer);

  } catch (err) {
    alert('Failed to load STEP file: ' + err.message);
  } finally {
    loading.style.display = 'none';
  }
}

/* ============================================
   SHARED HELPERS
   ============================================ */
function clearContentGroup() {
  while (contentGroup.children.length > 0) {
    const child = contentGroup.children[0];
    contentGroup.remove(child);
    if (child.geometry) child.geometry.dispose();
    if (child.material) {
      if (Array.isArray(child.material)) child.material.forEach(m => m.dispose());
      else child.material.dispose();
    }
  }
}

/* ============================================
   OBJ FILE LOADER
   ============================================ */
async function loadObjFromText(text, mtlText, save) {
  const loading = document.getElementById('loadingOverlay');
  loading.style.display = 'flex';

  try {
    clearContentGroup();

    const loader = new OBJLoader();

    /* If an MTL was provided, parse it and attach materials */
    if (mtlText) {
      const mtlLoader = new MTLLoader();
      const materials = mtlLoader.parse(mtlText, '');
      materials.preload();
      loader.setMaterials(materials);
    }

    const obj = loader.parse(text);

    obj.traverse((child) => {
      if (!child.isMesh) return;
      child.castShadow = true;
      child.receiveShadow = true;

      /* Upgrade materials to MeshStandardMaterial, preserving colours */
      if (child.material) {
        const mats = Array.isArray(child.material) ? child.material : [child.material];
        for (let i = 0; i < mats.length; i++) {
          const m = mats[i];
          if (!m.isMeshStandardMaterial) {
            const std = new THREE.MeshStandardMaterial({
              color: m.color || 0x99aacc,
              map: m.map || null,
              side: THREE.DoubleSide,
              metalness: 0.2,
              roughness: 0.45,
              normalMap: subtleNormal,
              normalScale: new THREE.Vector2(0.3, 0.3)
            });
            if (m.vertexColors) std.vertexColors = true;
            m.dispose();
            mats[i] = std;
          } else {
            m.side = THREE.DoubleSide;
            if (!m.normalMap) {
              m.normalMap = subtleNormal;
              m.normalScale = new THREE.Vector2(0.3, 0.3);
            }
          }
        }
        child.material = mats.length === 1 ? mats[0] : mats;
      }

      if (!child.geometry.attributes.normal) child.geometry.computeVertexNormals();
    });

    /* Re-parent meshes into contentGroup */
    const meshes = [];
    obj.traverse((c) => { if (c.isMesh) meshes.push(c); });
    for (const m of meshes) contentGroup.add(m);

    /* Center and scale to fit ~2 units */
    const box3 = new THREE.Box3().setFromObject(contentGroup);
    const center = box3.getCenter(new THREE.Vector3());
    const size = box3.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    const s = 2.0 / maxDim;

    for (const child of contentGroup.children) {
      child.position.sub(center);
      child.scale.setScalar(s);
    }

    if (save) await saveObjData(text, mtlText);

  } catch (err) {
    alert('Failed to load OBJ file: ' + err.message);
  } finally {
    loading.style.display = 'none';
  }
}

/* ============================================
   FILE INPUT — routes by extension
   ============================================ */
const fileInput = document.getElementById('fileInput');

fileInput.addEventListener('change', async (e) => {
  const files = Array.from(e.target.files);
  if (files.length === 0) { e.target.value = ''; return; }

  const mtlFile  = files.find(f => /\.mtl$/i.test(f.name));
  const objFile  = files.find(f => /\.obj$/i.test(f.name));
  const stepFile = files.find(f => /\.(step|stp)$/i.test(f.name));

  const mtlText = mtlFile ? await mtlFile.text() : null;

  if (objFile) {
    await loadObjFromText(await objFile.text(), mtlText, true);
  } else if (stepFile) {
    const buffer = new Uint8Array(await stepFile.arrayBuffer());
    await loadStepFromBytes(buffer, true);
  }
  e.target.value = '';
});

/* Restore saved model on load */
dbGet('modelType').then(async (type) => {
  if (type === 'obj') {
    const objText = await dbGet('objText');
    const mtlText = await dbGet('mtlText');
    if (objText) await loadObjFromText(objText, mtlText, false);
  } else {
    const bytes = await loadStepBytes();
    if (bytes) await loadStepFromBytes(bytes, false);
  }
}).catch(() => {});

document.getElementById('loadModelBtn').addEventListener('click', () => fileInput.click());
document.getElementById('loadModelFloating').addEventListener('click', () => fileInput.click());

/* ============================================
   SERIAL PORT PANEL  (Web Serial API)
   ============================================ */
const portSelect       = document.getElementById('portSelect');
const portRequestBtn   = document.getElementById('portRequestBtn');
const portConnectBtn   = document.getElementById('portConnectBtn');
const portDisconnectBtn = document.getElementById('portDisconnectBtn');
const portStatusDot    = document.getElementById('portStatusDot');
const portStatusText   = document.getElementById('portStatusText');

let knownPorts = [];
let activePort = null;
let activeReader = null;
let readLoopRunning = false;

function portLabel(port, index) {
  const info = port.getInfo();
  if (info.usbVendorId) {
    return 'USB ' + info.usbVendorId.toString(16).toUpperCase() +
           ':' + (info.usbProductId || 0).toString(16).toUpperCase();
  }
  return 'Port ' + (index + 1);
}

async function refreshPortList() {
  knownPorts = await navigator.serial.getPorts();
  portSelect.innerHTML = '';
  if (knownPorts.length === 0) {
    const opt = document.createElement('option');
    opt.value = '';
    opt.textContent = 'No ports available';
    portSelect.appendChild(opt);
    portConnectBtn.disabled = true;
  } else {
    knownPorts.forEach((port, i) => {
      const opt = document.createElement('option');
      opt.value = i;
      opt.textContent = portLabel(port, i);
      portSelect.appendChild(opt);
    });
    portConnectBtn.disabled = !!activePort;
  }
}

function showMainUI() {
  document.getElementById('overlay').style.display = 'none';
  document.getElementById('hud').style.display = 'block';
  document.getElementById('loadModelFloating').style.display = 'block'; /* inside serialPortPanel */
  document.getElementById('offsetPanel').style.display = 'block';
  document.getElementById('yawOffsetPanel').style.display = 'block';
  document.getElementById('serialConsole').style.display = 'flex';
  document.getElementById('graphPanel').style.display = 'flex';
  document.getElementById('altGraphPanel').style.display = 'flex';
  document.getElementById('recordPanel').style.display = 'block';
  document.getElementById('lightPanel').style.display = 'block';
  document.getElementById('serialPortPanel').style.display = 'block';
  document.getElementById('gpsPanel').style.display = 'block';
  /* Layout containers reveal their panels; let the page settle, then nudge. */
  document.body.classList.add('connected');
  initGpsMap();
  /* The layout may now be scrollable / resized — let Leaflet re-measure. */
  if (gpsMap) setTimeout(() => gpsMap.invalidateSize(), 250);
}

/* Optional narrow-screen rail toggles (present only in layouts that use them). */
const railToggleLeftBtn  = document.getElementById('railToggleLeft');
const railToggleRightBtn = document.getElementById('railToggleRight');
if (railToggleLeftBtn) railToggleLeftBtn.addEventListener('click', () => {
  document.getElementById('railLeft')?.classList.toggle('open');
});
if (railToggleRightBtn) railToggleRightBtn.addEventListener('click', () => {
  document.getElementById('railRight')?.classList.toggle('open');
  if (gpsMap) setTimeout(() => gpsMap.invalidateSize(), 250);
});

function setConnectedState(label) {
  portStatusDot.className = 'status-dot connected';
  portStatusText.textContent = 'Connected to ' + label;
  portConnectBtn.disabled = true;
  portDisconnectBtn.disabled = false;
  portSelect.disabled = true;
}

function setDisconnectedState(msg) {
  portStatusDot.className = 'status-dot disconnected';
  portStatusText.textContent = msg || 'Not connected';
  portConnectBtn.disabled = knownPorts.length === 0;
  portDisconnectBtn.disabled = true;
  portSelect.disabled = false;
  activePort = null;
}

async function readSerialLoop(port) {
  readLoopRunning = true;
  const decoder = new TextDecoder();
  let buf = '';

  while (port.readable && readLoopRunning) {
    activeReader = port.readable.getReader();
    try {
      while (true) {
        const { value, done } = await activeReader.read();
        if (done) break;
        buf += decoder.decode(value, { stream: true });
        const lines = buf.split('\n');
        buf = lines.pop();

        for (const ln of lines) {
          const trimmed = ln.trim();
          if (trimmed.length > 0) appendConsoleLine(trimmed);
          const parts = trimmed.split(/\s+/);
          /* Accepted telemetry layouts:
             3 -> roll pitch yaw
             5 -> + altitude vz
             9 -> + gps_lat gps_lon gps_alt gps_fix */
          if (parts.length !== 3 && parts.length !== 5 && parts.length !== 9) continue;
          const r = parseFloat(parts[0]);
          const p = parseFloat(parts[1]);
          const y = parseFloat(parts[2]);
          if (isNaN(r) || isNaN(p) || isNaN(y)) continue;
          roll = r; pitch = p; yaw = y;
          if (parts.length >= 5) {
            const a  = parseFloat(parts[3]);
            const vz = parseFloat(parts[4]);
            if (!isNaN(a))  altitude = a;
            if (!isNaN(vz)) verticalVelocity = vz;
          }
          if (parts.length === 9) {
            const la = parseFloat(parts[5]);
            const lo = parseFloat(parts[6]);
            const ga = parseFloat(parts[7]);
            const fx = parseInt(parts[8], 10);
            updateGps(la, lo, ga, fx);
          }
        }
      }
    } catch (readErr) {
      if (readLoopRunning) console.warn('Serial read error:', readErr);
    } finally {
      activeReader.releaseLock();
      activeReader = null;
    }
  }
}

async function connectToPort(port, label) {
  try {
    await port.open({ baudRate: 115200 });
    await new Promise(r => setTimeout(r, 200));
    activePort = port;
    setConnectedState(label);
    showMainUI();
    readSerialLoop(port).then(() => {
      if (readLoopRunning) {
        setDisconnectedState('Port disconnected');
        activePort = null;
      }
    });
  } catch (err) {
    setDisconnectedState('Error: ' + err.message);
    portStatusText.textContent = 'Error: ' + err.message;
  }
}

/* "Connect Serial" overlay button — request port + connect immediately */
document.getElementById('connectBtn').addEventListener('click', async () => {
  try {
    const port = await navigator.serial.requestPort();
    await refreshPortList();
    const idx = knownPorts.indexOf(port);
    const label = idx >= 0 ? portLabel(port, idx) : 'Serial Port';
    showMainUI();
    await connectToPort(port, label);
  } catch (err) {
    document.getElementById('status').textContent = 'Error: ' + err.message;
  }
});

/* "Add Port" — request a new port and add it to the dropdown */
portRequestBtn.addEventListener('click', async () => {
  try {
    await navigator.serial.requestPort();
    await refreshPortList();
  } catch (err) {
    /* user cancelled the picker */
  }
});

/* "Connect" — connect to the selected port in the dropdown */
portConnectBtn.addEventListener('click', async () => {
  const idx = parseInt(portSelect.value);
  if (isNaN(idx) || !knownPorts[idx]) return;
  const port = knownPorts[idx];
  const label = portLabel(port, idx);
  await connectToPort(port, label);
});

/* "Disconnect" — close the active port */
portDisconnectBtn.addEventListener('click', async () => {
  readLoopRunning = false;
  try {
    if (activeReader) await activeReader.cancel();
    if (activePort) await activePort.close();
  } catch (e) { /* ignore close errors */ }
  setDisconnectedState('Disconnected');
  await refreshPortList();
});

/* Initial port list population */
refreshPortList();

/* ============================================
   SERIAL CONSOLE
   ============================================ */
const consoleLinesEl = document.getElementById('consoleLines');
const MAX_CONSOLE_LINES = 200;
let consoleLineCount = 0;

function appendConsoleLine(text) {
  const span = document.createElement('span');
  span.textContent = text + '\n';
  consoleLinesEl.appendChild(span);
  consoleLineCount++;
  if (consoleLineCount > MAX_CONSOLE_LINES) {
    consoleLinesEl.removeChild(consoleLinesEl.firstChild);
    consoleLineCount--;
  }
  consoleLinesEl.scrollTop = consoleLinesEl.scrollHeight;
}

document.getElementById('clearConsoleBtn').addEventListener('click', () => {
  consoleLinesEl.innerHTML = '';
  consoleLineCount = 0;
});

/* ============================================
   LIVE GPS MAP  (Leaflet)
   ============================================ */
let gpsMap = null;
let gpsMarker = null;
let gpsTrail = null;            // polyline of fixed positions
let gpsTrailPts = [];
let gpsAutoCenter = true;       // follow the rocket until the user pans away
const GPS_TRAIL_MAX = 1000;

const FIX_NAMES = {
  0: 'NO FIX', 1: 'DEAD RECKONING', 2: '2D FIX',
  3: '3D FIX', 4: 'GNSS+DR', 5: 'TIME ONLY'
};

/* ---------- Selectable basemaps (all key-free) ----------
   Satellite/hybrid use Esri World Imagery; streets use OSM; dark matches
   the app theme; topo shows terrain contours. */
function buildBasemaps() {
  const esriImagery = () => L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
    { maxZoom: 19, maxNativeZoom: 19,
      attribution: 'Tiles &copy; Esri, Maxar, Earthstar Geographics' });

  /* Transparent street/label overlay to lay over imagery for a "hybrid" view */
  const esriLabels = () => L.tileLayer(
    'https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
    { maxZoom: 19, attribution: '&copy; Esri' });

  const satellite = esriImagery();

  /* Hybrid = imagery + labels, grouped so the control treats them as one base */
  const hybrid = L.layerGroup([esriImagery(), esriLabels()]);

  const streets = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',
    { maxZoom: 19, attribution: '&copy; OpenStreetMap' });

  const dark = L.tileLayer(
    'https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png',
    { maxZoom: 20, attribution: '&copy; OpenStreetMap, &copy; CARTO',
      subdomains: 'abcd' });

  const topo = L.tileLayer('https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png',
    { maxZoom: 17, attribution: '&copy; OpenTopoMap (CC-BY-SA), &copy; OpenStreetMap' });

  return {
    'Satellite':        satellite,
    'Satellite+Labels': hybrid,
    'Streets':          streets,
    'Dark':             dark,
    'Topographic':      topo
  };
}

function initGpsMap() {
  if (gpsMap || typeof L === 'undefined') return;

  const basemaps = buildBasemaps();

  /* Restore the last-used basemap, defaulting to satellite. */
  const savedName = localStorage.getItem('gpsBasemap');
  const startName = (savedName && basemaps[savedName]) ? savedName : 'Satellite';

  gpsMap = L.map('gpsMap', {
    zoomControl: true,
    layers: [basemaps[startName]]
  }).setView([0, 0], 2);

  /* Basemap picker (collapses to a layers icon; expands on hover/tap). */
  L.control.layers(basemaps, null, { position: 'topright', collapsed: true }).addTo(gpsMap);
  gpsMap.on('baselayerchange', (e) => localStorage.setItem('gpsBasemap', e.name));

  /* Scale bar for distance reference. */
  L.control.scale({ position: 'bottomleft', imperial: true, metric: true }).addTo(gpsMap);

  gpsTrail  = L.polyline([], { color: '#d4a15a', weight: 2 }).addTo(gpsMap);
  gpsMarker = L.circleMarker([0, 0], {
    radius: 6, color: '#fff', weight: 2,
    fillColor: '#4caf50', fillOpacity: 0.9
  }).addTo(gpsMap);

  /* The panel was hidden at map-creation time, so Leaflet measured a 0-size
     container. Recompute once it's visible to avoid gray/partial tiles. */
  setTimeout(() => gpsMap.invalidateSize(), 200);

  /* Keep the map sized to its (resizable) layout box. */
  const mapEl = document.getElementById('gpsMap');
  if (window.ResizeObserver && mapEl) {
    new ResizeObserver(() => { if (gpsMap) gpsMap.invalidateSize(); }).observe(mapEl);
  }

  /* Manual pan disables auto-follow until the user clicks Recenter. */
  gpsMap.on('dragstart', () => { gpsAutoCenter = false; });
}

function haversine(lat1, lon1, lat2, lon2) {
  const R = 6371000; // m
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLon = (lon2 - lon1) * Math.PI / 180;
  const a = Math.sin(dLat / 2) ** 2 +
            Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) *
            Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(a));
}

function updateGps(lat, lon, alt, fix) {
  gpsFix = isNaN(fix) ? 0 : fix;

  /* A fix is usable only at 2D/3D and not the (0,0) null island the
     firmware reports before the first lock. */
  const valid = !isNaN(lat) && !isNaN(lon) && gpsFix >= 2 &&
                !(lat === 0 && lon === 0);

  const dot     = document.getElementById('gpsDot');
  const noFixEl = document.getElementById('gpsNoFix');
  document.getElementById('gpsFixLabel').textContent = FIX_NAMES[gpsFix] || 'UNKNOWN';

  if (!valid) {
    gpsHasFix = false;
    dot.className = 'gps-dot nofix';
    if (noFixEl) noFixEl.style.display = 'flex';
    document.getElementById('gpsLatVal').textContent   = '--';
    document.getElementById('gpsLonVal').textContent   = '--';
    document.getElementById('gpsAltVal').textContent   = '--';
    document.getElementById('gpsSpeedVal').textContent = '--';
    return;
  }

  /* Ground-speed estimate from successive fixes (distance / dt). */
  const now = performance.now();
  if (gpsPrev) {
    const dtSec = (now - gpsPrev.t) / 1000;
    if (dtSec > 0.05) gpsSpeed = haversine(gpsPrev.lat, gpsPrev.lon, lat, lon) / dtSec;
  }
  gpsPrev = { lat, lon, t: now };

  gpsLat = lat; gpsLon = lon; gpsAlt = alt; gpsHasFix = true;

  dot.className = 'gps-dot fix';
  if (noFixEl) noFixEl.style.display = 'none';
  document.getElementById('gpsLatVal').textContent   = lat.toFixed(6) + '°';
  document.getElementById('gpsLonVal').textContent   = lon.toFixed(6) + '°';
  document.getElementById('gpsAltVal').textContent   = isNaN(alt) ? '--' : alt.toFixed(1) + ' m';
  document.getElementById('gpsSpeedVal').textContent = gpsSpeed.toFixed(1) + ' m/s';

  if (!gpsMap) return;
  const ll = [lat, lon];
  gpsMarker.setLatLng(ll);
  gpsTrailPts.push(ll);
  if (gpsTrailPts.length > GPS_TRAIL_MAX) gpsTrailPts.shift();
  gpsTrail.setLatLngs(gpsTrailPts);

  if (gpsAutoCenter) {
    if (gpsMap.getZoom() < 15) gpsMap.setView(ll, 16);
    else gpsMap.panTo(ll, { animate: false });
  }
}

document.getElementById('gpsCenterBtn').addEventListener('click', () => {
  gpsAutoCenter = true;
  if (gpsMap && gpsHasFix) gpsMap.setView([gpsLat, gpsLon], 16);
});

/* ============================================
   MODEL OFFSET CONTROLS
   ============================================ */
const modelOffset = new THREE.Vector3(0, 0, 0);

function setupOffsetControl(sliderId, inputId, axis) {
  const slider = document.getElementById(sliderId);
  const numInput = document.getElementById(inputId);
  slider.addEventListener('input', () => {
    const v = parseFloat(slider.value);
    modelOffset[axis] = v;
    numInput.value = v.toFixed(2);
  });
  numInput.addEventListener('input', () => {
    const v = parseFloat(numInput.value);
    if (isNaN(v)) return;
    modelOffset[axis] = v;
    slider.value = Math.max(-10, Math.min(10, v));
  });
}
setupOffsetControl('offX', 'offXVal', 'x');
setupOffsetControl('offY', 'offYVal', 'y');
setupOffsetControl('offZ', 'offZVal', 'z');

document.getElementById('resetOffsetBtn').addEventListener('click', () => {
  modelOffset.set(0, 0, 0);
  for (const [id, valId] of [['offX','offXVal'],['offY','offYVal'],['offZ','offZVal']]) {
    document.getElementById(id).value = 0;
    document.getElementById(valId).value = '0.00';
  }
});

function applyOffsetToSliders() {
  const mapping = [['offX','offXVal','x'],['offY','offYVal','y'],['offZ','offZVal','z']];
  for (const [id, valId, axis] of mapping) {
    document.getElementById(id).value = modelOffset[axis];
    document.getElementById(valId).value = modelOffset[axis].toFixed(2);
  }
}

/* ---------- Pick Center mode ---------- */
let pickCenterMode = false;
const pickCenterBtn = document.getElementById('pickCenterBtn');
const raycaster = new THREE.Raycaster();
const pointerNDC = new THREE.Vector2();

function setPickMode(on) {
  pickCenterMode = on;
  renderer.domElement.style.cursor = on ? 'crosshair' : '';
  pickCenterBtn.classList.toggle('pick-active', on);
}

pickCenterBtn.addEventListener('click', () => setPickMode(!pickCenterMode));

renderer.domElement.addEventListener('click', (e) => {
  if (!pickCenterMode) return;

  /* Compute normalised device coordinates */
  const rect = renderer.domElement.getBoundingClientRect();
  pointerNDC.x =  ((e.clientX - rect.left) / rect.width)  * 2 - 1;
  pointerNDC.y = -((e.clientY - rect.top)  / rect.height) * 2 + 1;

  raycaster.setFromCamera(pointerNDC, camera);
  const hits = raycaster.intersectObjects(contentGroup.children, true);
  if (hits.length === 0) return;

  /* Convert hit point to contentGroup local space and negate */
  const localPt = contentGroup.worldToLocal(hits[0].point.clone());
  modelOffset.set(-localPt.x, -localPt.y, -localPt.z);
  applyOffsetToSliders();

  setPickMode(false);
});

/* ============================================
   ATTITUDE OFFSET CONTROLS
   ============================================ */
let rollOffset = 0, pitchOffset = 0, yawOffset = 0;

function setupAttOffsetControl(sliderId, valId, setter) {
  const slider   = document.getElementById(sliderId);
  const numInput = document.getElementById(valId);
  slider.addEventListener('input', () => {
    const v = parseFloat(slider.value);
    setter(v);
    numInput.value = v.toFixed(1);
  });
  numInput.addEventListener('input', () => {
    const v = parseFloat(numInput.value);
    if (isNaN(v)) return;
    setter(v);
    slider.value = Math.max(-180, Math.min(180, v));
  });
}
setupAttOffsetControl('rollOff',  'rollOffVal',  (v) => { rollOffset = v; });
setupAttOffsetControl('pitchOff', 'pitchOffVal', (v) => { pitchOffset = v; });
setupAttOffsetControl('yawOff',   'yawOffVal',   (v) => { yawOffset = v; });

document.getElementById('resetYawOffsetBtn').addEventListener('click', () => {
  rollOffset = 0; pitchOffset = 0; yawOffset = 0;
  for (const [id, valId] of [['rollOff','rollOffVal'],['pitchOff','pitchOffVal'],['yawOff','yawOffVal']]) {
    document.getElementById(id).value = 0;
    document.getElementById(valId).value = '0.0';
  }
});

/* ============================================
   LIGHTING CONTROLS
   ============================================ */
const LIGHT_DEFAULTS = { ambient: 0.7, key: 1.2, fill: 0.5, rim: 0.35, exposure: 1.0 };
const lightTargets = {
  ambient:  (v) => { hemi.intensity = v; },
  key:      (v) => { dir.intensity = v; },
  fill:     (v) => { fill.intensity = v; },
  rim:      (v) => { rim.intensity = v; },
  exposure: (v) => { renderer.toneMappingExposure = v; }
};

function setupLightControl(sliderId, valId, apply) {
  const slider   = document.getElementById(sliderId);
  const numInput = document.getElementById(valId);
  slider.addEventListener('input', () => {
    const v = parseFloat(slider.value);
    apply(v);
    numInput.value = v.toFixed(2);
  });
  numInput.addEventListener('input', () => {
    const v = parseFloat(numInput.value);
    if (isNaN(v)) return;
    apply(v);
    slider.value = Math.max(parseFloat(slider.min), Math.min(parseFloat(slider.max), v));
  });
}
setupLightControl('ltAmbient',  'ltAmbientVal',  lightTargets.ambient);
setupLightControl('ltKey',      'ltKeyVal',       lightTargets.key);
setupLightControl('ltFill',     'ltFillVal',      lightTargets.fill);
setupLightControl('ltRim',      'ltRimVal',       lightTargets.rim);
setupLightControl('ltExposure', 'ltExposureVal',  lightTargets.exposure);

document.getElementById('resetLightBtn').addEventListener('click', () => {
  const map = [
    ['ltAmbient','ltAmbientVal','ambient'],
    ['ltKey','ltKeyVal','key'],
    ['ltFill','ltFillVal','fill'],
    ['ltRim','ltRimVal','rim'],
    ['ltExposure','ltExposureVal','exposure']
  ];
  for (const [sId, vId, name] of map) {
    const d = LIGHT_DEFAULTS[name];
    lightTargets[name](d);
    document.getElementById(sId).value = d;
    document.getElementById(vId).value = d.toFixed(2);
  }
});

/* ============================================
   DATA RECORDING
   ============================================ */
let isRecording = false;
let recordData = [];
let recordStartTime = 0;
let lastRecordTime = 0;
let recordTimer = null;
const RECORD_INTERVAL = 100; /* ms between samples (~10/sec) */

const recToggleBtn  = document.getElementById('recToggleBtn');
const recExportBtn  = document.getElementById('recExportBtn');
const recStatusEl   = document.getElementById('recStatus');
const recDurationEl = document.getElementById('recDuration');

function stopRecording() {
  isRecording = false;
  if (recordTimer) { clearTimeout(recordTimer); recordTimer = null; }
  recToggleBtn.textContent = 'Start';
  recStatusEl.textContent = 'Stopped — ' + recordData.length + ' samples';
}

recToggleBtn.addEventListener('click', () => {
  if (isRecording) {
    stopRecording();
  } else {
    recordData = [];
    recordStartTime = performance.now();
    lastRecordTime = 0;
    isRecording = true;
    recToggleBtn.textContent = 'Stop';
    const dur = parseInt(recDurationEl.value);
    if (dur > 0) {
      recordTimer = setTimeout(stopRecording, dur * 1000);
    }
  }
});

function recordSample() {
  if (!isRecording) return;
  const now = performance.now();
  if (now - lastRecordTime < RECORD_INTERVAL) return;
  lastRecordTime = now;
  const t = ((now - recordStartTime) / 1000).toFixed(3);
  /* Log GPS only when there's a valid fix; otherwise leave it blank so the
     firmware's pre-lock (0,0) null island never ends up in the data. */
  const lat = gpsHasFix ? gpsLat : null;
  const lon = gpsHasFix ? gpsLon : null;
  recordData.push({ t, roll, pitch, yaw, altitude, lat, lon });
  const dur = parseInt(recDurationEl.value);
  if (dur > 0) {
    const elapsed = (now - recordStartTime) / 1000;
    recStatusEl.textContent = 'Recording… ' + recordData.length + ' samples (' + Math.min(elapsed, dur).toFixed(1) + '/' + dur + 's)';
  } else {
    recStatusEl.textContent = 'Recording… ' + recordData.length + ' samples';
  }
}

recExportBtn.addEventListener('click', () => {
  if (recordData.length === 0) { recStatusEl.textContent = 'No data to export'; return; }
  const wantRoll  = document.getElementById('recRoll').checked;
  const wantPitch = document.getElementById('recPitch').checked;
  const wantYaw   = document.getElementById('recYaw').checked;
  const wantAlt   = document.getElementById('recAlt').checked;
  const wantGps   = document.getElementById('recGps').checked;

  let header = 'Time(s)';
  if (wantRoll)  header += ',Roll';
  if (wantPitch) header += ',Pitch';
  if (wantYaw)   header += ',Yaw';
  if (wantAlt)   header += ',Altitude_m';
  if (wantGps)   header += ',Latitude,Longitude';

  const lines = [header];
  for (const d of recordData) {
    let row = d.t;
    if (wantRoll)  row += ',' + d.roll.toFixed(2);
    if (wantPitch) row += ',' + d.pitch.toFixed(2);
    if (wantYaw)   row += ',' + d.yaw.toFixed(2);
    if (wantAlt)   row += ',' + d.altitude.toFixed(2);
    if (wantGps)   row += ',' + (d.lat != null ? d.lat.toFixed(7) : '') +
                          ',' + (d.lon != null ? d.lon.toFixed(7) : '');
    lines.push(row);
  }

  const csv = lines.join('\n');
  const blob = new Blob([csv], { type: 'text/csv' });
  const url = URL.createObjectURL(blob);
  const now = new Date();
  const ts = now.getFullYear() + '-' +
    String(now.getMonth() + 1).padStart(2, '0') + '-' +
    String(now.getDate()).padStart(2, '0') + '_' +
    String(now.getHours()).padStart(2, '0') + '-' +
    String(now.getMinutes()).padStart(2, '0') + '-' +
    String(now.getSeconds()).padStart(2, '0');
  const a = document.createElement('a');
  a.href = url;
  a.download = 'attitude_recording_' + ts + '.csv';
  a.click();
  URL.revokeObjectURL(url);
  recStatusEl.textContent = 'Exported ' + recordData.length + ' samples';
});

/* ============================================
   ATTITUDE GRAPH
   ============================================ */
const graphCanvas = document.getElementById('graphCanvas');
const graphCtx = graphCanvas.getContext('2d');
const GRAPH_POINTS = 200;
const graphData = { roll: [], pitch: [], yaw: [] };
const altGraphData = [];
const altGraphCanvas = document.getElementById('altGraphCanvas');
const altGraphCtx = altGraphCanvas.getContext('2d');

function drawGraph() {
  const rect = graphCanvas.parentElement.getBoundingClientRect();
  const w = rect.width;
  const h = rect.height - graphCanvas.offsetTop;
  if (graphCanvas.width !== w * devicePixelRatio || graphCanvas.height !== h * devicePixelRatio) {
    graphCanvas.width  = w * devicePixelRatio;
    graphCanvas.height = h * devicePixelRatio;
    graphCanvas.style.width  = w + 'px';
    graphCanvas.style.height = h + 'px';
  }

  const ctx = graphCtx;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  ctx.clearRect(0, 0, w, h);

  /* Push new data */
  graphData.roll.push(roll);
  graphData.pitch.push(pitch);
  graphData.yaw.push(yaw);
  if (graphData.roll.length > GRAPH_POINTS) {
    graphData.roll.shift();
    graphData.pitch.shift();
    graphData.yaw.shift();
  }

  /* Auto-range from data */
  let min = Infinity, max = -Infinity;
  for (const key of ['roll', 'pitch', 'yaw']) {
    for (const v of graphData[key]) {
      if (v < min) min = v;
      if (v > max) max = v;
    }
  }
  const pad = Math.max((max - min) * 0.1, 5);
  min -= pad; max += pad;

  /* Grid lines */
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 0.5;
  const gridSteps = 5;
  for (let i = 0; i <= gridSteps; i++) {
    const y = (i / gridSteps) * h;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  /* Zero line */
  if (min < 0 && max > 0) {
    const zeroY = h - ((0 - min) / (max - min)) * h;
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, zeroY); ctx.lineTo(w, zeroY); ctx.stroke();
  }

  /* Y-axis labels */
  ctx.fillStyle = '#666';
  ctx.font = '10px Consolas, monospace';
  ctx.textBaseline = 'middle';
  for (let i = 0; i <= gridSteps; i++) {
    const val = max - (i / gridSteps) * (max - min);
    const y = (i / gridSteps) * h;
    ctx.fillText(val.toFixed(0) + '°', 2, y);
  }

  /* Draw lines */
  const colors = { roll: '#c27c6e', pitch: '#7aab85', yaw: '#8a9bae' };
  for (const key of ['roll', 'pitch', 'yaw']) {
    const arr = graphData[key];
    if (arr.length < 2) continue;
    ctx.strokeStyle = colors[key];
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < arr.length; i++) {
      const x = (i / (GRAPH_POINTS - 1)) * w;
      const y = h - ((arr[i] - min) / (max - min)) * h;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

function drawAltGraph() {
  const rect = altGraphCanvas.parentElement.getBoundingClientRect();
  const w = rect.width;
  const h = rect.height - altGraphCanvas.offsetTop;
  if (altGraphCanvas.width !== w * devicePixelRatio || altGraphCanvas.height !== h * devicePixelRatio) {
    altGraphCanvas.width  = w * devicePixelRatio;
    altGraphCanvas.height = h * devicePixelRatio;
    altGraphCanvas.style.width  = w + 'px';
    altGraphCanvas.style.height = h + 'px';
  }

  const ctx = altGraphCtx;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  ctx.clearRect(0, 0, w, h);

  altGraphData.push(altitude);
  if (altGraphData.length > GRAPH_POINTS) altGraphData.shift();

  let min = Infinity, max = -Infinity;
  for (const v of altGraphData) {
    if (v < min) min = v;
    if (v > max) max = v;
  }
  /* Floor the range so a flat signal on the pad still shows something sensible */
  if (!isFinite(min) || !isFinite(max)) { min = 0; max = 1; }
  if (max - min < 1) { const mid = (max + min) * 0.5; min = mid - 0.5; max = mid + 0.5; }
  const pad = (max - min) * 0.1;
  min -= pad; max += pad;

  /* Grid */
  ctx.strokeStyle = '#333';
  ctx.lineWidth = 0.5;
  const gridSteps = 5;
  for (let i = 0; i <= gridSteps; i++) {
    const y = (i / gridSteps) * h;
    ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
  }

  /* Zero line */
  if (min < 0 && max > 0) {
    const zeroY = h - ((0 - min) / (max - min)) * h;
    ctx.strokeStyle = '#555';
    ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(0, zeroY); ctx.lineTo(w, zeroY); ctx.stroke();
  }

  /* Y-axis labels */
  ctx.fillStyle = '#666';
  ctx.font = '10px Consolas, monospace';
  ctx.textBaseline = 'middle';
  for (let i = 0; i <= gridSteps; i++) {
    const val = max - (i / gridSteps) * (max - min);
    const y = (i / gridSteps) * h;
    ctx.fillText(val.toFixed(1) + ' m', 2, y);
  }

  /* Altitude line */
  if (altGraphData.length >= 2) {
    ctx.strokeStyle = '#d4a15a';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    for (let i = 0; i < altGraphData.length; i++) {
      const x = (i / (GRAPH_POINTS - 1)) * w;
      const y = h - ((altGraphData[i] - min) / (max - min)) * h;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
}

/* ============================================
   ANIMATION LOOP
   ============================================ */
const DEG2RAD = Math.PI / 180;
const attEuler    = new THREE.Euler(0, 0, 0, 'YZX');
const attQ        = new THREE.Quaternion();
const offsetEuler = new THREE.Euler(0, 0, 0, 'YZX');
const offsetQ     = new THREE.Quaternion();

function animate() {
  requestAnimationFrame(animate);

  controls.update();
  contentGroup.position.copy(modelOffset);

  /* Attitude quaternion from serial data */
  attEuler.set(roll * DEG2RAD, yaw * DEG2RAD, -pitch * DEG2RAD);
  attQ.setFromEuler(attEuler);

  /* Model rotational offset (pre-rotation applied to the model) */
  offsetEuler.set(rollOffset * DEG2RAD, yawOffset * DEG2RAD, -pitchOffset * DEG2RAD);
  offsetQ.setFromEuler(offsetEuler);

  /* Attitude first, then offset rotates the model within that frame */
  rotationGroup.quaternion.copy(attQ).multiply(offsetQ);

  document.getElementById('rollVal').textContent  = roll.toFixed(2).padStart(7);
  document.getElementById('pitchVal').textContent = pitch.toFixed(2).padStart(7);
  document.getElementById('yawVal').textContent   = yaw.toFixed(2).padStart(7);
  document.getElementById('altVal').textContent   = altitude.toFixed(2).padStart(7);
  document.getElementById('vzVal').textContent    = verticalVelocity.toFixed(2).padStart(7);

  recordSample();
  drawGraph();
  drawAltGraph();
  renderer.render(scene, camera);
}

animate();
