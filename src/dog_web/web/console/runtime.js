import * as THREE from 'three';
import {OrbitControls} from '/vendor/OrbitControls.js';
import {proposedMapId, assertImportedMap} from '/import_state.js';
import {mountLocalization} from './localization.js';
import {createRobotModel} from './robot_model.js';
import {mountMotionSettings} from './motion_settings.js';
import {TeleopClient} from './teleop_client.js';
import {mountLiveClouds} from './cloud_view.js';
import {mountMapLibrary} from './map_library.js';
import {mountMapTaskNotice} from './map_task_notice.js';
import {mountTerrainReview} from './terrain_review.js';

const $ = id => document.getElementById(id);
// BPX MotionState telemetry values differ from SetMotion request constants.
const motionStates = {0: '趴下', 1: '站起中', 2: '被动阻尼', 3: '趴下中', 6: '站立/运动'};
const gaitStates = {0:'四足 Walk',3:'双足',4:'翻转',6:'WalkPhase',7:'姿态跟踪',8:'跑步',12:'跳跃'};
const state = {page: 'overview', map: null, start: null, goal: null, selection: null,
  epoch: 0, busy: false, robot: null, model:null, cloudSequence: -1,driving:false};
let toastTimer;
const mapTask=mountMapTaskNotice($('mapTaskNotice'));
function tell(message, error = false) {
  $('toast').textContent = message; $('toast').classList.add('visible');
  clearTimeout(toastTimer); toastTimer = setTimeout(() => $('toast').classList.remove('visible'), 6000);
  const row = document.createElement('div'); row.className = 'event';
  for (const [tag, text] of [['time', new Date().toLocaleTimeString('zh-CN')], ['span', error ? '错误' : '接口'], ['p', message]]) {
    const element = document.createElement(tag); element.textContent = text; row.append(element);
  }
  $('eventList').prepend(row);
  while ($('eventList').children.length > 50) $('eventList').lastChild.remove();
  $('eventCount').textContent = $('eventList').children.length;
}
async function api(path, body, raw = false) {
  for(let attempt=0;attempt<2;attempt++){
    const response = await fetch(path, {method: body === undefined ? 'GET' : 'POST',
      headers: body === undefined || raw ? {} : {'Content-Type': 'application/json'},
      body: body === undefined ? undefined : raw ? body : JSON.stringify(body),
      signal: AbortSignal.timeout(120000),cache:'no-store'});
    const content=await response.text();let result;
    try{result=JSON.parse(content);}
    catch{
      // A read can be retried after a truncated response. Never retry a command:
      // the robot may already have received it even if its reply was lost.
      if(body===undefined&&attempt===0){await new Promise(r=>setTimeout(r,100));continue;}
      throw new Error(`${path} 返回${content.trim()?'非 JSON':'空'}响应（HTTP ${response.status}），请检查 Web 服务与网络代理`);
    }
    if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
    if (result.success === false) throw new Error(result.message || '操作被后端拒绝');
    return result;
  }
}
function action(fn) { return async () => { try { await fn(); } catch (e) { tell(e.message, true); } }; }
function text(selector, value) { const el = document.querySelector(selector); if (el) el.textContent = value; }

// Replace every mock status before the first network request.
$('eventList').replaceChildren(); $('eventCount').textContent = '0';
text('.connection', '等待 ROS 状态'); text('.demo-tag', '本地 ROS 2 接口');
text('.sidebar-bottom', '机器人工作台 v0.2');
text('.sidebar-note strong', '未加载地图'); text('.sidebar-note > span:not(.mini-label)', '几何地图 / 地形规划');
text('.scene-caption strong', '等待真实几何点云'); text('.scene-caption small', 'map · Z ↑ · m');
text('.scene-badge', '未加载地图'); text('.posture', '姿态 —'); text('.battery', '电量 —');
document.querySelector('.route-track').textContent = '选择起点与目标后运行三维地表 A*';
document.querySelector('.floor-switch').hidden = true; // Floors must come from actual map heights, not assumed 1F/2F.
text('.footer-detail', '执行需启用导航与完成标定；实机楼梯能力待验证');
text('.bottom-bar b', '等待接口');
$('stop').textContent = '◉ 停止运动';
$('taskBadge').textContent = '待规划'; $('destination').textContent = '未选择目标';
$('startCoordinates').textContent = '未选择起点';
$('goalCoordinates').textContent = '未选择终点'; $('routeLength').textContent = '—';
$('goalLabel').hidden = true;
$('pickGoal').insertAdjacentHTML('beforebegin', '<button class="button" id="pickStart" title="重新选择规划起点">起点</button>');
document.querySelectorAll('.scene-metrics strong, .velocity-values strong, .attitude b').forEach(e => e.textContent = '—');
document.querySelectorAll('[data-health]').forEach(e => { e.querySelector('small').textContent = '等待数据'; e.querySelector('b').textContent = '—'; e.querySelector('i').className = 'dot amber'; });

// Z-up scene shared by live geometry and the selected offline planning project.
const palette=getComputedStyle(document.documentElement);
const scene = new THREE.Scene(); scene.background = new THREE.Color(palette.getPropertyValue('--scene-bg').trim()||'#000000');
const camera = new THREE.PerspectiveCamera(48, 1, .01, 5000); camera.up.set(0, 0, 1); camera.position.set(8, -10, 8);
const renderer = new THREE.WebGLRenderer({canvas: $('mapCanvas'), antialias: true});
renderer.setPixelRatio(1); // Keep WebGL work bounded while mapping and driving.
const controls = new OrbitControls(camera, renderer.domElement); controls.enableDamping = true;
const grid = new THREE.GridHelper(40, 80,palette.getPropertyValue('--grid-major').trim()||'#174b40',palette.getPropertyValue('--grid-minor').trim()||'#08251e'); grid.rotation.x = Math.PI / 2; grid.position.z = -.05; scene.add(grid);
const groups = {};
for (const name of ['cloud', 'surface', 'path', 'live', 'scan', 'markers']) { groups[name] = new THREE.Group(); scene.add(groups[name]); }
groups.initial=new THREE.Group();scene.add(groups.initial);
const localization=mountLocalization({api,tell,
  pick:()=>{state.selection='initial_pose';tell('请点选机器人当前脚下的地面，再设置地图朝向。');},
  preview:value=>{clear(groups.initial);if(!value)return;
    const arrow=new THREE.ArrowHelper(new THREE.Vector3(Math.cos(value.yaw),Math.sin(value.yaw),0),
      new THREE.Vector3(...value.point).add(new THREE.Vector3(0,0,.12)),.7,'#bd9aff',.2,.12);
    groups.initial.add(arrow);
  }});
scene.add(new THREE.HemisphereLight('#e7fff9','#15392f',2));
const modelLight=new THREE.DirectionalLight('#ffffff',2);modelLight.position.set(4,-3,6);scene.add(modelLight);
const modelView=createRobotModel({scene,onReady:bounds=>{
  if(state.page==='overview'&&!state.map&&!state.robot?.pose&&!bounds.isEmpty()){
    const sphere=bounds.getBoundingSphere(new THREE.Sphere());
    controls.target.copy(sphere.center);
    const distance=Math.max(1.2,sphere.radius/Math.sin(THREE.MathUtils.degToRad(camera.fov/2))*1.3);
    camera.position.copy(sphere.center).addScaledVector(new THREE.Vector3(.8,-1.1,.55).normalize(),distance);
    camera.updateProjectionMatrix();controls.update();
  }
}});
const robot=modelView.root;
function updateRobotModel() {
  modelView.update({...state.robot,...state.model},state.map,{page:state.page,
    frame:state.page==='mapping'?liveClouds.frame():'map'});
  $('locateRobot').disabled=!robot.userData.positioned;
  $('locateRobot').title=robot.userData.positioned?'将视角移到机器狗当前位置':robot.userData.readout;
  document.querySelector('[data-view="follow"]').disabled=!robot.userData.positioned;
}
const height = {value: 1e6};
function clear(group) {
  for (const child of [...group.children]) {child.geometry?.dispose(); child.material?.dispose(); group.remove(child);}
}
function points(rows, group, color, size) {
  clear(group);
  if (!rows.length) return;
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(rows.flatMap(p => p.slice(0, 3)), 3));
  const material = new THREE.ShaderMaterial({transparent: true,
    uniforms: {clip: height, color: {value: new THREE.Color(color)}, size: {value: size}},
    vertexShader: 'varying float z; uniform float size; void main(){z=position.z; gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.); gl_PointSize=size;}',
    fragmentShader: 'varying float z; uniform float clip; uniform vec3 color; void main(){if(z>clip)discard;gl_FragColor=vec4(color,.8);}'});
  group.add(new THREE.Points(geometry, material));
}
function drawSurfaces(region = null) {
  const rows = state.map?.surfaces || [];
  if (state.page !== 'terrain') {points(rows, groups.surface, '#bdd49b', 4);return;}
  clear(groups.surface);
  const positions = [], colors = [], color = new THREE.Color();
  for (const row of rows) {
    if (region != null && row[3] !== region) continue;
    positions.push(...row.slice(0,3));
    color.setHSL(((row[3] * .61803398875) % 1 + 1) % 1, .6, .58);colors.push(color.r,color.g,color.b);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  const material = new THREE.ShaderMaterial({uniforms:{clip:height},
    vertexShader:'attribute vec3 color; varying vec3 tint; varying float z; void main(){tint=color;z=position.z;gl_Position=projectionMatrix*modelViewMatrix*vec4(position,1.);gl_PointSize=5.;}',
    fragmentShader:'uniform float clip; varying vec3 tint; varying float z; void main(){if(z>clip)discard;gl_FragColor=vec4(tint,1.);}'});
  groups.surface.add(new THREE.Points(geometry,material));
}
function updateOverview() {
  $('overviewMap').textContent = state.map?.display_name || state.map?.map_id || '未加载地图';
  $('overviewRevision').textContent = state.map ? `版本 ${state.map.revision.slice(0,8)} · ${Number(state.map.points).toLocaleString()} 点` : '先选择地图项目';
  const loc = state.robot?.localization;
  const sameMap = state.map && loc?.map_path?.replace(/\/(?:localization|cloud)\.pcd$/,'').split('/').at(-1) === state.map.map_id;
  $('overviewLocalization').textContent = !state.map ? '待加载地图' : !loc?.fresh ? '定位状态未就绪' : !sameMap ? '定位地图不一致' : loc.valid ? '定位有效' : '待定位确认';
  $('overviewRoute').textContent = state.ascent == null ? '尚未规划' : $('routeLength').textContent;
  $('overviewTerrain').textContent = state.map ? `${state.map.components} 个连通区域` : '等待地图';
}
function fit(bounds) {
  if (!bounds) return;
  const box = new THREE.Box3(new THREE.Vector3(...bounds[0]), new THREE.Vector3(...bounds[1]));
  const center = box.getCenter(new THREE.Vector3()), size = Math.max(2, box.getSize(new THREE.Vector3()).length());
  controls.target.copy(center); camera.position.copy(center).add(new THREE.Vector3(.8, -1.1, .8).multiplyScalar(size));
  camera.far = Math.max(100, size * 20); camera.updateProjectionMatrix(); controls.update();
}
function clearRoute() { state.epoch++; state.ascent = null; clear(groups.path); $('routeLength').textContent = '—'; $('taskBadge').textContent = '待规划'; }
function clearMap() {
  clearRoute(); clear(groups.cloud); clear(groups.surface); clear(groups.markers);
  state.map = state.start = state.goal = null;
  $('startCoordinates').textContent = '未选择起点';
  $('goalCoordinates').textContent = '未选择终点';
  localization.update(state.robot,null);
  updateRobotModel();
  document.querySelectorAll('.scene-metrics strong').forEach(e => e.textContent = '—');
  text('.scene-badge', '未加载地图'); text('.sidebar-note strong', '未加载地图');
  $('destination').textContent = '未选择目标';
}
function markers() {
  clear(groups.markers);
  for (const [p, color] of [[state.start, '#54dfc3'], [state.goal, '#ffbd69']]) if (p) {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(.08), new THREE.MeshBasicMaterial({color, depthTest: false, transparent: true}));
    mesh.renderOrder = 20;
    mesh.position.set(p[0], p[1], p[2] + .07); groups.markers.add(mesh);
  }
  for (const [id, point, label] of [['startCoordinates', state.start, '起点'], ['goalCoordinates', state.goal, '终点']]) {
    $(id).textContent = point ? point.map((v, i) => `${'XYZ'[i]} ${v.toFixed(3)}`).join('  ') : `未选择${label}`;
  }
  $('destination').textContent = state.goal ? '地表目标' : '未选择目标';
}
function refreshWorkspace() {
  const mapping = state.page === 'mapping';
  for (const key of ['cloud','surface','path','markers','initial']) groups[key].visible = !mapping && (document.querySelector(`[data-layer="${key}"]`)?.checked ?? true);
  if (!mapping) groups.cloud.visible = document.querySelector('[data-layer="cloud"]').checked && document.querySelector('[data-render="cloud"]').classList.contains('active');
  if (state.page !== 'navigation') {groups.markers.visible = false;groups.initial.visible = false;}
  if (state.page === 'terrain') {groups.cloud.visible = false;groups.surface.visible = true;groups.path.visible = false;}
  text('.scene-caption .mini-label', mapping ? 'LIVE LIDAR / IMU' : 'SAVED MAP / ROUTE REVIEW');
  text('.scene-badge', mapping ? '实时采集 · odom / map' : state.map ? `${state.map.map_id} / ${state.map.revision.slice(0,8)}` : '未加载地图');
  text('.scene-caption strong', mapping ? '等待累计建图点云' : state.map?.display_name || state.map?.map_id || '从地图管理加载已有几何地图');
  text('.scene-caption small', mapping ? '实时采集坐标系 · 以点云元数据为准' : 'map · Z ↑ · m');
  const metrics = document.querySelectorAll('.scene-metrics strong');
  if (!mapping) [state.map?.points, state.map?.valid_surfaces, state.map?.components, state.ascent == null ? '—' : `${state.ascent.toFixed(2)} m`].forEach((value,i) => metrics[i].textContent = value ?? '—');
  liveClouds.visibility();
  updateOverview();
}
async function displayMap(expected) {
  const map = await api('/api/map'); assertImportedMap(map, expected); clearMap(); state.map = map;
  localization.update(state.robot,map);
  updateRobotModel();
  points(map.cloud, groups.cloud, '#7db8aa', 2); drawSurfaces();
  text('.scene-badge', `${map.map_id} / ${map.revision.slice(0, 8)}`);
  text('.scene-caption strong', map.display_name || map.source?.filename || map.map_id);
  text('.sidebar-note strong', map.display_name || map.map_id);
  const metrics = document.querySelectorAll('.scene-metrics strong');
  [map.points, map.valid_surfaces, map.components, '—'].forEach((v, i) => metrics[i].textContent = v);
  const slider = $('heightClip'); slider.min = map.bounds[0][2] - .1; slider.max = map.bounds[1][2] + .1;
  slider.value = slider.max; slider.oninput(); fit(map.bounds);
  state.selection = 'start';
  refreshWorkspace();
  return map;
}
function updateMapActions() {
  document.querySelectorAll('[data-map-task-action]').forEach(button=>{button.disabled=state.busy||button.dataset.unavailable==='true';});
}
async function job(operation,title='正在加载地图') {
  if (state.busy) throw new Error('已有地图任务进行中');
  state.busy = true;updateMapActions();mapTask.start(title);clearMap();
  try {
    const {job: id} = await operation();
    for (;;) {
      const result = await api(`/api/jobs/${id}`);
      if (result.state === 'failed') throw new Error(result.error);
      if (result.state === 'complete') {
        mapTask.loading();
        const map=await displayMap(result);
        mapTask.complete(map);tell(`地图 ${map.map_id} 已加载完成，请选择规划起点。`);break;
      }
      await new Promise(r => setTimeout(r, 700));
    }
  } catch(error) {mapTask.fail(error);throw error;}
  finally { state.busy = false;updateMapActions(); }
}
async function plan() {
  if (state.page !== 'navigation') throw new Error('请在定位与导航页面设置起终点并规划');
  if (!state.map || !state.start || !state.goal) throw new Error('请先加载地图并选择起点和目标');
  clearRoute(); const epoch = state.epoch;
  const result = await api('/api/plan', {map_id: state.map.map_id, revision: state.map.revision, start: state.start, goal: state.goal});
  if (epoch !== state.epoch) return;
  for (const section of result.sections) {
    const geometry = new THREE.BufferGeometry().setFromPoints(result.points.slice(section.start_index, section.end_index + 1).map(p => new THREE.Vector3(p[0], p[1], p[2] + .05)));
    const line = new THREE.Line(geometry, new THREE.LineBasicMaterial({color: section.mode === 'stairs' ? '#ffbd69' : '#54dfc3', depthTest: false, transparent: true}));
    line.renderOrder = 10; groups.path.add(line);
  }
  state.start = result.points[0]; state.goal = result.points.at(-1); markers();
  state.ascent = result.ascent;
  $('routeLength').textContent = `${result.length.toFixed(2)} m`; $('taskBadge').textContent = '规划完成 · 仅预览';
  document.querySelectorAll('.scene-metrics strong')[3].textContent = `${result.ascent.toFixed(2)} m`;
  tell(`规划完成：${result.sections.filter(s => s.mode === 'stairs').length} 段台阶候选；未发送运动指令。`);
  refreshWorkspace();
}
const raycaster = new THREE.Raycaster(); raycaster.params.Points.threshold = .12;
let down;
$('mapCanvas').addEventListener('pointerdown', e => down = [e.clientX, e.clientY, e.button]);
$('mapCanvas').addEventListener('pointerup', e => {
  if (state.page !== 'navigation' || !down || down[2] !== 0 || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5 || !state.selection || state.busy || !state.map) return;
  const rect = $('mapCanvas').getBoundingClientRect();
  raycaster.setFromCamera(new THREE.Vector2((e.clientX - rect.left) / rect.width * 2 - 1, -(e.clientY - rect.top) / rect.height * 2 + 1), camera);
  const hit = raycaster.intersectObjects(groups.surface.children).find(h => state.map.surfaces[h.index][2] <= height.value);
  if (!hit) return tell('请点击可见的可通行地表节点');
  if(state.selection==='initial_pose') {
    localization.choose(state.map.surfaces[hit.index].slice(0,3));state.selection=null;return;
  }
  clearRoute(); state[state.selection] = state.map.surfaces[hit.index].slice(0, 3);
  state.selection = state.selection === 'start' ? 'goal' : null; markers();
  if (state.start && state.goal) action(plan)();
});
new ResizeObserver(() => {
  const rect = $('mapCanvas').getBoundingClientRect(); if (!rect.width || !rect.height) return;
  camera.aspect = rect.width / rect.height; camera.updateProjectionMatrix(); renderer.setSize(rect.width, rect.height, false);
}).observe($('mapCanvas'));
let lastRender=0;
renderer.setAnimationLoop(time => {
  if(document.hidden||$('sceneWorkspace').hidden||time-lastRender<(state.driving?67:33))return;
  lastRender=time;
  if (state.view === 'follow' && robot.userData.positioned) {
    const delta=robot.position.clone().sub(controls.target).multiplyScalar(.1);
    camera.position.add(delta);controls.target.add(delta);
  }
  controls.update(); renderer.render(scene, camera);
});
$('heightClip').oninput = () => {height.value = Number($('heightClip').value); $('heightValue').textContent = height.value.toFixed(2) + ' m';};
$('resetView').onclick = () => fit((state.page === 'mapping' ? liveClouds.bounds() : state.map?.bounds) || [[-5, -5, 0], [5, 5, 3]]);
$('locateRobot').onclick = () => {
  if (!robot.userData.positioned) return tell(robot.userData.readout || '当前位姿未就绪',true);
  const offset=camera.position.clone().sub(controls.target);
  if(offset.length()<2||offset.length()>12)offset.set(3,-4,2.5);
  controls.target.copy(robot.position);camera.position.copy(robot.position).add(offset);controls.update();
};
$('zoomIn').onclick = () => {camera.position.sub(controls.target).multiplyScalar(.85).add(controls.target);};
$('zoomOut').onclick = () => {camera.position.sub(controls.target).multiplyScalar(1.15).add(controls.target);};
$('toggleLayers').onclick = () => {$('layerPopover').hidden = !$('layerPopover').hidden;};
document.querySelectorAll('[data-layer]').forEach(input => input.onchange = () => {
  const key = input.dataset.layer; (key === 'grid' ? grid : groups[key]).visible = input.checked;
  refreshWorkspace();
});
$('views').onclick = e => {
  const b = e.target.closest('button'); if (!b) return;
  state.view = b.dataset.view;
  $('views').querySelectorAll('button').forEach(x => x.classList.toggle('active', x === b));
  if (state.view === 'top') camera.position.copy(controls.target).add(new THREE.Vector3(0, -.001, 20));
  else $('resetView').click();
};
$('renderModes').onclick = e => {
  const b = e.target.closest('button'); if (!b) return;
  $('renderModes').querySelectorAll('button').forEach(x => x.classList.toggle('active', x === b));
  groups.cloud.visible = b.dataset.render === 'cloud'; groups.surface.visible = true;
  refreshWorkspace();
};
$('pickGoal').onclick = () => {state.selection = state.start ? 'goal' : 'start'; tell(`请在地表选择${state.selection === 'start' ? '起点' : '目标'}；地图坐标未与实时会话配准时只可手选。`);};
$('pickStart').onclick = () => {state.selection = 'start'; clearRoute(); tell('请在当前地图的可通行地表重新选择起点');};
$('plan').onclick = action(plan);
$('execute').onclick = action(async () => {
  if(!state.map||!state.goal)throw new Error('请先加载地图并选择目标');
  const result=await api('/api/navigation/start',{map_id:state.map.map_id,revision:state.map.revision,goal:state.goal});
  tell(result.message,!result.success);
});
$('pause').onclick = action(async () => {
  const paused=state.robot?.navigation?.task_state==='paused';
  const result=await api(`/api/navigation/${paused?'resume':'pause'}`,{});tell(result.message||'任务状态已更新',!result.success);
});
$('cancel').onclick = action(async () => {
  if(['waiting_inputs','following','paused','relocalizing','waiting_route','waiting_walk_state'].includes(state.robot?.navigation?.task_state)) {
    const result=await api('/api/navigation/cancel',{});tell(result.message||'已请求取消',!result.success);
  } else clearRoute();
});

const teleop = new TeleopClient({
  limits: () => state.robot?.limits,
  onError: error => tell(error.message, true),
  onChange: direction => {
    state.driving=Object.values(direction).some(Boolean);
    document.querySelectorAll('[data-drive]').forEach((button,i) => {
    const axis = ['x','x','y','y','yaw','yaw'][i];
    const pressed = direction[axis] === (i % 2 ? -1 : 1);
    button.classList.toggle('active',pressed); button.setAttribute('aria-pressed',String(pressed));
  });}
});
function release() {return teleop.stop();}
window.addEventListener('blur', release); document.addEventListener('visibilitychange', () => {if (document.hidden) release();});
window.addEventListener('pagehide', () => {
  if (teleop.token) navigator.sendBeacon('/api/control/release', new Blob([JSON.stringify({token: teleop.token})], {type: 'application/json'}));
  release();
});
const liftPointer = event => teleop.lift(`pointer:${event.pointerId}`);
window.addEventListener('pointerup', liftPointer); window.addEventListener('pointercancel', liftPointer);
$('stop').onclick = action(async () => {await release(); const result = await api('/api/control/stop', {}); tell(result.message);});
function controlPanel() {
  document.querySelectorAll('[data-motion]').forEach(b => b.onclick = action(async () => {
    if (!confirm(`向机器人发送“${b.textContent}”指令？请确认周围空间与机身支撑条件。`)) return;
    await release(); const {token} = await api('/api/control/acquire', {});
    try { const result = await api('/api/control/motion', {token, motion: Number(b.dataset.motion), confirm: true}); tell(`驱动回执：${result.message}；姿态完成以遥测为准。`); }
    finally { await api('/api/control/release', {token}).catch(() => {}); }
  }));
  document.addEventListener('keydown', e => {
    const editing = ['INPUT','TEXTAREA','SELECT'].includes(document.activeElement?.tagName) || document.activeElement?.isContentEditable;
    if (e.key === 'Escape' || (e.code === 'Space' && !editing)) {e.preventDefault(); release(); return;}
  });
  document.addEventListener('focusin', e => {
    if (['INPUT','TEXTAREA','SELECT'].includes(e.target.tagName) || e.target.isContentEditable) release();
  });
}
controlPanel();
const liveClouds=mountLiveClouds({groups,clip:height,getState:()=>state,fit,text,getFocus:()=>controls.target});
liveClouds.start();

function switchPage(key) {
  release(); state.page = key;
  document.querySelectorAll('[data-page]').forEach(b => b.classList.toggle('active', b.dataset.page === key));
  document.querySelectorAll('[data-page]').forEach(b => b.dataset.page === key ? b.setAttribute('aria-current','page') : b.removeAttribute('aria-current'));
  const content = ['maps','settings','diagnostics'].includes(key);
  $('sceneWorkspace').hidden = content; $('contentPage').hidden = !content;
  $('workspacePurpose').hidden = content;
  $('mainWorkspace').dataset.page = key;
  $('mainWorkspace').setAttribute('aria-label', document.querySelector(`[data-page="${key}"]`).textContent.trim());
  const mapping = key === 'mapping';
  const titles = {overview:'三维工作台', mapping:'SLAM 建图', navigation:'定位与导航', terrain:'地形与通行'};
  $('workspaceTitle').textContent = titles[key] || '';
  const purposes = {
    overview:['MISSION OVERVIEW','查看当前地图、定位与路线状态，从总览进入各项任务。','打开地图管理 ↗','maps','全景地图'],
    mapping:['01 CAPTURE → 02 SAVE → 03 BUILD','采集 LiDAR–IMU 几何点云，保存后在地图管理生成导航图。','查看已保存 PCD ↗','maps','实时几何采集'],
    navigation:['01 LOCALIZE → 02 TARGET → 03 PLAN','预览可手选起点；执行使用实时定位，并要求标定与安全仲裁就绪。','选择导航地图 ↗','maps','定位与路线'],
    terrain:['SURFACE CONNECTIVITY / HEIGHT','按连通区域检查三维地表与高度分布，核对当前地图构建约束。','进入路线验证 ↗','navigation','连通区域 · 颜色区分'],
  };
  const purpose = purposes[key] || ['','','','maps',''];
  [$('workspaceEyebrow').textContent,$('workspaceDescription').textContent,$('workspaceAction').textContent] = purpose;
  $('workspaceAction').onclick = () => switchPage(purpose[3]);
  $('sceneTitle').textContent = purpose[4];
  text('.panel-heading .subtle', mapping ? 'LIVE CAPTURE' : 'MAP REVIEW');
  document.querySelector('.task-card').hidden = key !== 'navigation';
  document.querySelector('.localization-card').hidden = key !== 'navigation';
  $('overviewSummary').hidden = key !== 'overview';
  $('taskTitle').textContent = '02 / 起终点与路线';
  $('liveScanCard').hidden = !mapping;
  $('renderModes').hidden = mapping || key === 'terrain';
  for (const input of document.querySelectorAll('[data-layer="surface"], [data-layer="path"]')) input.closest('label').hidden = mapping;
  $('heightClip').closest('label').hidden = mapping;
  $('layerPopover').hidden = true;
  document.querySelector('.scene-metrics').hidden = key !== 'navigation';
  document.querySelector('.scene-legend').hidden = mapping || key === 'terrain';
  document.querySelector('.robot-model-status').hidden = false;
  updateRobotModel();
  refreshWorkspace();
  drawSurfaces();
  height.value = mapping ? 1e6 : Number($('heightClip').value);
  if (!content) $('resetView').click();
  $('modeCard').hidden = !['mapping','terrain'].includes(key);
  if (key === 'mapping') {
    $('modeCard').innerHTML = '<div class="card-heading"><h2>采集会话</h2><span class="pill">纯 LIO</span></div><p id="mappingState">等待会话状态</p><div class="mapping-stage"><small>01 / 采集</small><p>确认 LiDAR 与 IMU 数据新鲜后启动。</p><div class="action-row"><button class="button primary" id="mappingStart">启动建图</button><button class="button" id="mappingStop">结束建图</button></div></div><div class="mapping-stage"><small>02 / 保存几何地图</small><p>保存 PCD 后，在地图管理中生成导航图。</p><button class="button" id="saveMap">保存几何 PCD</button></div><p class="settings-note">需要已有 MID-360 驱动输入。结束只管理网页启动的会话。此页只显示实时采集，不叠加已保存地图或规划路线；切回工作台保留原地图与起终点。</p>';
    for (const [id, operation] of [['mappingStart','start'],['mappingStop','stop'],['saveMap','save']]) $(id).onclick = action(async () => {
      $(id).disabled = true;
      try {if(operation==='start'){liveClouds.reset();height.value=1e6;}const result = await api(`/api/slam/${operation}`, {}); tell(result.message);} finally {if ($(id)) $(id).disabled = false;}
    });
  } else if (key === 'terrain') {
    mountTerrainReview($('modeCard'), {map:state.map,clip:height.value,onRegion:drawSurfaces,
      onClip:value=>{$('heightClip').value=value;$('heightClip').oninput();}});
    $('terrainNextConfig').innerHTML = '<span class="eyebrow">NEXT IMPORT</span><h2>下一次地图构建</h2><p class="settings-note">目标机器人长 0.66 m、宽 0.58 m。此配置仅用于下一次导入。</p><details><summary>编辑构建参数</summary><textarea id="terrainConfig" rows="8" aria-label="地形构建 JSON"></textarea><button class="button" id="applyConfig">应用到下一次导入</button><p class="settings-note">完整配置由后端在导入时校验，不修改当前地图。</p></details>';
    $('terrainConfig').value = JSON.stringify(terrainConfig, null, 2);
    $('applyConfig').onclick = action(async () => {const next = JSON.parse($('terrainConfig').value); if (!next || Array.isArray(next) || typeof next !== 'object') throw new Error('配置必须是 JSON 对象'); terrainConfig = next; tell('已应用，下次导入将由后端验证并构建。');});
  }
  if (key === 'maps') action(renderMaps)();
  if (key === 'diagnostics') {$('contentPage').innerHTML = '<h2>实时诊断</h2><pre id="diagnosticOutput" style="white-space:pre-wrap"></pre>'; updateTelemetry(state.robot);}
  if (key === 'settings') {
    mountMotionSettings($('contentPage'), {api, release, onChanged: limits => {
      if (state.robot) state.robot.limits = limits;
    }});
  }
  updateTelemetry(state.robot);
}
// Robot footprint; heights select local obstacles, not a full 3D body model.
// Existing projects keep their saved configuration until explicitly rebuilt.
let terrainConfig = {
  body_length: 0.66, body_width: 0.58, body_top: 0.5, resolution: 0.10,
  normal_radius: 0.10, normal_fallback_radius: 0.15, normal_thickness: 0.02, plane_distance: 0.015,
  support_distance: 0.12, connection_distance: 0.25, terrain_tolerance: 0.02,
};
async function renderMaps() {
  mountMapLibrary({root:$('contentPage'),api,isBusy:()=>state.busy,setBusy:value=>{state.busy=value;updateMapActions();},
    onError:error=>tell(error.message,true),notice:tell,getMapping:()=>state.robot?.mapping,
    load:async name=>{await job(()=>api('/api/load',{name}));switchPage('overview');},
    demo:async()=>{await job(()=>api('/api/demo',{}),'正在生成合成测试地图');switchPage('overview');},
    upload:{selected(file,input){if(file)input.value=proposedMapId(file.name);},async start(file,name){
      if(!file)throw Error('请选择 PCD 文件');if(file.size>128*1024*1024)throw Error('文件超过 128 MiB 上限');
      const params=new URLSearchParams({name:name.trim()||proposedMapId(file.name),filename:file.name,config:JSON.stringify(terrainConfig)});
      await job(()=>api(`/api/import?${params}`,file,true),'正在导入 PCD 并生成导航图');switchPage('overview');}},
    importSaved:async(file,name)=>{await job(()=>api('/api/import_saved',{id:file.id,...(name?{name}:{}),config:terrainConfig}),'正在生成导航图并加载');switchPage('overview');},
    unload:()=>{clearMap();liveClouds.reset();$('mapTaskNotice').hidden=true;},
    renamed(kind,id,name){if(kind==='map'&&state.map?.map_id===id){state.map.display_name=name;text('.sidebar-note strong',name);text('.scene-caption strong',name);}}
  });
}
document.querySelectorAll('[data-page]').forEach(b => b.onclick = () => switchPage(b.dataset.page));
document.querySelectorAll('[data-open-page]').forEach(b => b.onclick = () => switchPage(b.dataset.openPage));
function updateTelemetry(data) {
  state.robot = data; if (!data) return;
  updateOverview();
  localization.update(data,state.map);
  text('.connection', data.ros ? data.control_ready ? '机器人状态新鲜' : 'ROS 已连接 · 机器人未就绪' : 'ROS 桥接未启用');
  if(data.ros)document.querySelector('.connection').removeAttribute('title');
  document.querySelector('.robot-card .dot').className = `dot ${data.control_ready ? 'green' : 'amber'}`;
  document.querySelector('.motion-telemetry').title = data.velocity ? '机身里程计遥测；姿态以可用的 IMU / 位姿数据为准' : '等待新鲜遥测';
  text('.bottom-bar b', data.ros ? 'ROS 桥接在线' : '离线模式');
  $('controlOwner').textContent = !data.control_ready ? '等待机器人连接' : data.control_claimed ? '网页控制中' : '控制权空闲';
  $('controlOwner').title = '此控制权只协调网页客户端；不代表网页处于离线预览模式';
  const controlStatus = !data.ros ? 'ROS 接口未连接' : !data.streams?.control ?
    '未收到 /dog/status，请检查 bringup 配置中的 driver 和机器狗驱动。' :
    !data.streams.control.fresh ? '机器人状态已过期，请检查驱动与网络。' :
    !data.robot?.connected ? '驱动已发布状态，机器人网络尚未连接。' :
    !data.robot?.state_fresh ? '机器人已连接，等待有效状态数据。' : '机器人已连接，手动控制可用。';
  if ($('manualControlStatus')) $('manualControlStatus').textContent = controlStatus;
  if(state.driving&&teleop.metrics&&$('manualControlStatus'))$('manualControlStatus').textContent+=` 控制往返 ${teleop.metrics.roundTripMs.toFixed(0)} ms · 页面心跳间隔 ${teleop.metrics.uiGapMs.toFixed(0)} ms`;
  document.querySelectorAll('[data-motion], [data-drive]').forEach(button => {button.disabled = !data.control_ready;});
  text('.battery', data.battery == null ? '电量 —' : `电量 ${data.battery.toFixed(0)}%`);
  text('.posture', postureText(data.robot));
  const values = document.querySelectorAll('.velocity-values strong');
  values[0].textContent = data.velocity ? `${Math.hypot(data.velocity[0], data.velocity[1]).toFixed(2)} m/s` : '—';
  values[1].textContent = data.velocity ? `${data.velocity[2].toFixed(2)} rad/s` : '—';
  for (const button of document.querySelectorAll('[data-health]')) {
    const key = button.dataset.health, stream = data.streams?.[key];
    const ready = key === 'navigation' ? data.navigation_ready : key === 'localization' ? data.localization_ready : key === 'tf' ? data.tf_ready : key === 'control' ? data.control_ready : stream?.fresh;
    button.querySelector('i').className = `dot ${ready ? 'green' : 'amber'}`;
    button.querySelector('small').textContent = key==='localization' ? data.localization?.fresh ? data.localization.valid?'已确认定位':data.localization.state:'未收到定位状态' : key==='navigation' ? data.navigation?.fresh?data.navigation.phase||'等待任务':'未收到执行状态' : ready ? '数据新鲜' : stream ? '数据过期' : '未收到数据';
    button.querySelector('b').textContent = stream ? `${stream.hz.toFixed(1)} Hz` : ready ? '就绪' : '—';
  }
  const nav=data.navigation||{};
  const active=nav.fresh&&['waiting_inputs','following','paused','relocalizing','waiting_route','waiting_walk_state'].includes(nav.task_state);
  $('execute').disabled=!data.navigation_ready||active||!state.goal;
  $('execute').title=data.navigation_ready?'使用实时定位作为起点':'需要执行开关、标定、定位和安全仲裁就绪';
  $('pause').disabled=!active;$('pause').textContent=nav.task_state==='paused'?'继续任务':'暂停任务';
  $('cancel').textContent=active?'取消任务':'清除路线';
  updateRobotModel();
  liveClouds.visibility();
  const orientation = data.body_orientation || (data.tf_ready && data.pose?.quaternion);
  if (Array.isArray(orientation) && orientation.length === 4 && orientation.every(Number.isFinite) && Math.abs(new THREE.Quaternion(...orientation).length()-1)<.01) {
    const angles = new THREE.Euler().setFromQuaternion(new THREE.Quaternion(...orientation).normalize(), 'ZYX');
    document.querySelectorAll('.attitude b').forEach((b, i) => b.textContent = `${([angles.x, angles.y, angles.z][i] * 180 / Math.PI).toFixed(1)}°`);
  } else document.querySelectorAll('.attitude b').forEach(b => b.textContent = '—');
  if ($('mappingState')) $('mappingState').textContent = `${data.mapping?.stopping ? '退出保存中' : data.mapping?.owned ? '网页会话已启动' : '无网页会话'} / LIO ${data.streams?.lio?.fresh ? '数据新鲜' : '未就绪'}\n${data.mapping?.output_dir || ''}${data.mapping?.exit_code != null ? ` / 退出码 ${data.mapping.exit_code}` : ''}`;
  if ($('diagnosticOutput')) $('diagnosticOutput').textContent = JSON.stringify(data, null, 2);
}
async function poll() {
  try {updateTelemetry(await api('/api/robot'));}
  catch (e) {await release(); updateTelemetry({ros:false}); text('.connection', `接口离线：${e.message}`);document.querySelector('.connection').title=e.message;}
  setTimeout(poll, 500);
}
function postureText(robot){return `机身状态 ${motionStates[robot?.motion]??robot?.motion??'—'} / 步态 ${gaitStates[robot?.gait]??robot?.gait??'—'}`;}
// Small read-only snapshots keep joint motion responsive without fetching map
// clouds or the full diagnostic payload at the same rate. Never issue commands.
async function pollModel(){
  if(document.hidden){setTimeout(pollModel,250);return;}
  try{
    state.model=await api('/api/robot/model');
    updateRobotModel();
    text('.posture',postureText(state.model.robot));
  }catch{
    state.model={joints:null,body_orientation:null,pose:null,tf_ready:false};
    updateRobotModel();
  }
  setTimeout(pollModel,50);
}
document.querySelectorAll('[data-health]').forEach(button => button.onclick = () => {
  $('dialogTitle').textContent = button.textContent; $('dialogBody').textContent = JSON.stringify(state.robot, null, 2);
  $('detailDialog').showModal();
});
$('closeDialog').onclick = $('confirmDialog').onclick = () => $('detailDialog').close();
$('showEvents').onclick = () => {$('events').hidden = !$('events').hidden;};
$('collapseEvents').onclick = () => {$('events').hidden = true;};
setInterval(() => {$('clock').textContent = new Date().toLocaleTimeString('zh-CN');}, 1000);
switchPage('overview'); poll(); pollModel();
