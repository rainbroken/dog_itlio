import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
import {proposedMapId, assertImportedMap, sourceDescription} from './import_state.js';

const $ = id => document.getElementById(id);
const viewport = $('viewport');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#0b111a');
const camera = new THREE.PerspectiveCamera(48, 1, .01, 5000);
camera.up.set(0, 0, 1);
camera.position.set(8, -10, 8);
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
viewport.append(renderer.domElement);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
const grid = new THREE.GridHelper(40, 80, '#294252', '#172735');
grid.rotation.x = Math.PI / 2;
grid.position.z = -.05;
scene.add(grid);
const objects = new THREE.Group();
scene.add(objects);
const raycaster = new THREE.Raycaster();
raycaster.params.Points.threshold = .085;
let data = null, surfaceObject = null, mapObject = null, selection = 'start';
let start = null, goal = null, routeObject = null, startMarker = null, goalMarker = null;
let busy = false, planning = false, epoch = 0;
const height = {low: {value: -1e6}, high: {value: 1e6}};

function status(message, error = false) {
  $('status').textContent = message;
  $('statusDot').classList.toggle('error', error);
}
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body),
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}
function dispose(object) {
  if (!object) return;
  object.traverse(child => {child.geometry?.dispose(); child.material?.dispose();});
  object.removeFromParent();
}
function pointMaterial(size, opacity) {
  return new THREE.ShaderMaterial({
    transparent: true, depthWrite: opacity > .5,
    uniforms: {low: height.low, high: height.high, size: {value: size}, alpha: {value: opacity}},
    vertexShader: `attribute vec3 color; varying vec3 vColor; varying float z;
      uniform float size; void main(){vColor=color;z=position.z;
      gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.0);gl_PointSize=size;}`,
    fragmentShader: `varying vec3 vColor; varying float z; uniform float low,high,alpha;
      void main(){if(z<low||z>high||distance(gl_PointCoord,vec2(.5))>.5)discard;
      gl_FragColor=vec4(vColor,alpha);}`,
  });
}
function makePoints(rows, isSurface) {
  const positions = [], colors = [];
  const color = new THREE.Color();
  const [low, high] = [data.bounds[0][2], data.bounds[1][2]];
  for (const row of rows) {
    positions.push(...row.slice(0, 3));
    const value = isSurface ? (row[3] * .19 + .42) % 1 : .56 - .4 * (row[2] - low) / Math.max(.1, high - low);
    color.setHSL(value, isSurface ? .65 : .25, isSurface ? .59 : .48);
    colors.push(color.r, color.g, color.b);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  const object = new THREE.Points(geometry, pointMaterial(isSurface ? 5 : 2, isSurface ? .9 : .45));
  objects.add(object);
  return object;
}
function fit() {
  if (!data) return;
  const box = new THREE.Box3(new THREE.Vector3(...data.bounds[0]), new THREE.Vector3(...data.bounds[1]));
  const center = box.getCenter(new THREE.Vector3());
  const size = Math.max(2, box.getSize(new THREE.Vector3()).length());
  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(.8, -1.1, .8).multiplyScalar(size));
  camera.far = Math.max(100, size * 20);
  camera.updateProjectionMatrix();
  controls.update();
}
function choose(which) {
  selection = which;
  $('pickStart').classList.toggle('active', which === 'start');
  $('pickGoal').classList.toggle('active', which === 'goal');
}
function clearRoute() {
  epoch++;
  dispose(routeObject); routeObject = null;
  $('pathLength').textContent = '—'; $('pathRise').textContent = '—';
}
function clearEndpoints() {
  clearRoute();
  dispose(startMarker); dispose(goalMarker);
  startMarker = goalMarker = start = goal = null;
  $('startText').textContent = $('goalText').textContent = '未选择';
  choose('start');
}
function marker(point, color) {
  const object = new THREE.Mesh(new THREE.SphereGeometry(.075, 16, 12),
    new THREE.MeshBasicMaterial({color, depthTest: false}));
  object.position.set(point[0], point[1], point[2] + .07);
  objects.add(object);
  return object;
}
async function refreshMaps() {
  const result = await api('/api/maps');
  $('maps').replaceChildren(new Option('选择已保存地图', ''), ...result.maps.map(n => new Option(n, n)));
  if (data) $('maps').value = data.map_id;
}
function clearDisplayedMap(message) {
  clearEndpoints(); dispose(mapObject); dispose(surfaceObject);
  mapObject = surfaceObject = data = null;
  $('mapTitle').textContent = '未显示地图';
  $('revision').textContent = 'Z ↑ · 单位 m';
  $('sourceInfo').textContent = '';
  $('nodeCount').textContent = $('componentCount').textContent = '—';
  $('empty').style.display = 'flex';
  $('empty').querySelector('h2').textContent = message;
  $('empty').querySelector('p').textContent = '导入或加载成功后才会显示地图';
}
async function displayMap(expected) {
  const next = await api('/api/map');
  assertImportedMap(next, expected);
  clearEndpoints(); dispose(mapObject); dispose(surfaceObject);
  data = next;
  mapObject = makePoints(data.cloud, false);
  surfaceObject = makePoints(data.surfaces, true);
  surfaceObject.visible = $('showSurfaces').checked;
  $('empty').style.display = 'none';
  $('mapTitle').textContent = data.map_id;
  $('sourceInfo').textContent = sourceDescription(data);
  $('revision').textContent = `版本 ${data.revision.slice(0, 8)} · Z ↑`;
  $('nodeCount').textContent = data.valid_surfaces.toLocaleString();
  $('componentCount').textContent = data.components;
  for (const id of ['minHeight', 'maxHeight']) {
    $(id).min = Math.floor(data.bounds[0][2] * 20) / 20 - .1;
    $(id).max = Math.ceil(data.bounds[1][2] * 20) / 20 + .1;
  }
  $('minHeight').value = $('minHeight').min;
  $('maxHeight').value = $('maxHeight').max;
  updateClipping(); fit(); await refreshMaps();
  status(`地图就绪：${data.valid_surfaces} 个可通行节点，${data.step_edges} 条台阶连接。请选择起点。`);
}
function updateClipping() {
  let low = Number($('minHeight').value), high = Number($('maxHeight').value);
  if (low > high) {high = low; $('maxHeight').value = high;}
  height.low.value = low; height.high.value = high;
  $('minLabel').textContent = low.toFixed(2) + ' m';
  $('maxLabel').textContent = high.toFixed(2) + ' m';
}
async function job(action) {
  if (busy) return;
  busy = true;
  for (const id of ['demo', 'import', 'load', 'plan']) $(id).disabled = true;
  clearDisplayedMap('正在生成或加载地图…');
  $('operationMessage').textContent = '';
  $('operationMessage').classList.remove('error-text');
  try {
    status('正在处理地图与导航连接，请稍候…');
    const {job: token} = await action();
    let completed;
    while (true) {
      const state = await api(`/api/jobs/${token}`);
      if (state.state === 'failed') throw new Error(state.error);
      if (state.state === 'complete') {completed = state; break;}
      await new Promise(resolve => setTimeout(resolve, 700));
    }
    await displayMap(completed);
    $('operationMessage').textContent = `当前已显示地图：${data.map_id}`;
  } catch (error) {
    clearDisplayedMap('导入或加载失败');
    $('operationMessage').textContent = error.message;
    $('operationMessage').classList.add('error-text');
    status(error.message, true);
  }
  finally {busy = false; for (const id of ['demo', 'import', 'load', 'plan']) $(id).disabled = false;}
}
async function plan() {
  if (busy || planning) return;
  if (!data || !start || !goal) return status('请先在可通行表面选择起点和目标。', true);
  clearRoute();
  const token = epoch;
  planning = true; $('plan').disabled = true;
  try {
    status('正在搜索跨层可通行路线…');
    const result = await api('/api/plan', {map_id: data.map_id, revision: data.revision, start, goal});
    if (token !== epoch) return;
    routeObject = new THREE.Group();
    for (const section of result.sections) {
      const points = result.points.slice(section.start_index, section.end_index + 1)
        .map(p => new THREE.Vector3(p[0], p[1], p[2] + .06));
      const curve = new THREE.CurvePath();
      for (let i = 1; i < points.length; i++) curve.add(new THREE.LineCurve3(points[i - 1], points[i]));
      routeObject.add(new THREE.Mesh(new THREE.TubeGeometry(curve, Math.max(2, points.length * 3), .017, 6, false),
        new THREE.MeshBasicMaterial({color: section.mode === 'stairs' ? '#ffbd69' : '#54dfc3', depthTest: false})));
    }
    objects.add(routeObject);
    dispose(startMarker); dispose(goalMarker);
    startMarker = marker(result.points[0], '#54dfc3');
    goalMarker = marker(result.points.at(-1), '#ffbd69');
    $('startText').textContent = result.points[0].map(v => v.toFixed(2)).join(', ');
    $('goalText').textContent = result.points.at(-1).map(v => v.toFixed(2)).join(', ');
    $('pathLength').textContent = result.length.toFixed(2) + ' m';
    $('pathRise').textContent = result.ascent.toFixed(2) + ' m';
    status(`规划完成 · 下降 ${result.descent.toFixed(2)} m · ${result.sections.filter(s => s.mode === 'stairs').length} 段台阶候选 · 仅预览`);
  } catch (error) {if (token === epoch) status(error.message, true);}
  finally {planning = false; $('plan').disabled = busy;}
}
$('demo').onclick = () => job(() => api('/api/demo', {}));
$('load').onclick = () => job(() => api('/api/load', {name: $('maps').value}));
$('import').onclick = () => job(async () => {
  const file = $('pcd').files[0];
  if (!file) throw new Error('请选择 PCD 文件');
  const config = JSON.parse($('config').value);
  const query = new URLSearchParams({name: $('mapName').value.trim(), filename: file.name, config: JSON.stringify(config)});
  const response = await fetch(`/api/import?${query}`, {method: 'POST', body: file});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error);
  return result;
});
$('pcd').onchange = () => {
  const file = $('pcd').files[0];
  if (!file) return;
  $('fileName').textContent = `${file.name} · ${(file.size / 1048576).toFixed(1)} MiB`;
  $('mapName').value = proposedMapId(file.name);
  $('operationMessage').classList.remove('error-text');
  $('operationMessage').textContent = `已选择 ${file.name}，尚未导入。点击“导入并生成导航图”；同名项目将自动编号。`;
};
$('pickStart').onclick = () => choose('start');
$('pickGoal').onclick = () => choose('goal');
$('clear').onclick = clearEndpoints;
$('plan').onclick = plan;
$('fit').onclick = fit;
$('minHeight').oninput = $('maxHeight').oninput = updateClipping;
$('showSurfaces').onchange = () => {if (surfaceObject) surfaceObject.visible = $('showSurfaces').checked;};
let down = null;
renderer.domElement.addEventListener('pointerdown', event => {down = [event.clientX, event.clientY, event.button];});
renderer.domElement.addEventListener('pointerup', event => {
  if (!down || down[2] !== 0 || Math.hypot(event.clientX - down[0], event.clientY - down[1]) > 5 || !surfaceObject || busy) return;
  const rect = renderer.domElement.getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2((event.clientX - rect.left) / rect.width * 2 - 1,
    -(event.clientY - rect.top) / rect.height * 2 + 1), camera);
  const hit = raycaster.intersectObject(surfaceObject).find(h => {
    const z = data.surfaces[h.index][2]; return z >= height.low.value && z <= height.high.value;
  });
  if (!hit) return status('此处没有可选择的通行表面。请旋转视角或调整高度裁剪。', true);
  const point = data.surfaces[hit.index].slice(0, 3);
  clearRoute();
  if (selection === 'start') {
    start = point; dispose(startMarker); startMarker = marker(point, '#54dfc3');
    $('startText').textContent = point.map(v => v.toFixed(2)).join(', ');
    choose('goal'); status('起点已选择，请点击目标所在楼层的地面。');
  } else {
    goal = point; dispose(goalMarker); goalMarker = marker(point, '#ffbd69');
    $('goalText').textContent = point.map(v => v.toFixed(2)).join(', ');
    plan();
  }
});
new ResizeObserver(() => {
  camera.aspect = viewport.clientWidth / viewport.clientHeight;
  camera.updateProjectionMatrix(); renderer.setSize(viewport.clientWidth, viewport.clientHeight);
}).observe(viewport);
renderer.setAnimationLoop(() => {controls.update(); renderer.render(scene, camera);});
refreshMaps().catch(error => status(error.message, true));
