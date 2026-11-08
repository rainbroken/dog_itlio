/* Local design prototype. All geometry and telemetry are synthetic; no ROS or network API. */
(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  const canvas = $('mapCanvas');
  const ctx = canvas.getContext('2d');
  const state = { page: 'overview', angle: -.65, tilt: .62, zoom: 1, panX: 0, panY: 0, view: 'free', floor: 'all', clip: 3.5, render: 'cloud', layers: { cloud: true, surface: true, path: true, grid: true }, goal: [6, 3, 2.4], path: true, picking: false, mapping: false };
  let width = 0, height = 0, toastTimer, framePending = false;
  let seed = 3107;
  const random = () => { seed = (seed * 1664525 + 1013904223) >>> 0; return seed / 4294967296; };
  const points = [], surfaces = [], walls = [];
  function floor(x0, y0, x1, y1, z, kind = 'floor') {
    surfaces.push({ x0, y0, x1, y1, z, kind });
    for (let x = x0; x <= x1; x += .14) for (let y = y0; y <= y1; y += .14) {
      if (random() > .21) points.push([x + (random() - .5) * .07, y + (random() - .5) * .07, z + random() * .025, kind, random()]);
    }
  }
  function wall(x0, y0, x1, y1, z0, z1) {
    walls.push([[x0, y0, z0], [x1, y1, z0], [x1, y1, z1], [x0, y0, z1]]);
    const length = Math.hypot(x1 - x0, y1 - y0), steps = Math.ceil(length / .12);
    for (let i = 0; i <= steps; i++) for (let z = z0; z <= z1; z += .13) {
      if (random() > .32) points.push([x0 + (x1 - x0) * i / steps + (random() - .5) * .06, y0 + (y1 - y0) * i / steps + (random() - .5) * .06, z + random() * .02, 'wall', random()]);
    }
  }
  floor(-7, -4, 2.5, 1.8, 0);
  floor(-7, 1.8, -3.8, 4, 0);
  floor(2.5, -4, 5, -1, 0);
  floor(3.2, 1.8, 7.5, 4.8, 2.4);
  floor(-.6, 4.2, 3.2, 4.8, 2.4);
  floor(-.6, -.5, .8, 1.8, 1.2);
  for (let i = 0; i < 8; i++) floor(.8 + i * .3, -.5, 1.1 + i * .3, 1.8, 1.2 - i * .15, 'stairs');
  for (let i = 0; i < 8; i++) floor(-.6, 1.8 + i * .3, .8, 2.1 + i * .3, 1.2 + i * .15, 'stairs');
  // Synthetic corridor boundaries and room partitions, deliberately not a real robot map.
  wall(-7, -4, 5, -4, 0, 1.7);
  wall(-7, -4, -7, 4, 0, 1.5);
  wall(-7, 4, -3.8, 4, 0, 1.15);
  wall(-3.8, 1.8, -3.8, 4, 0, 1.15);
  wall(-4, -4, -4, -1.8, 0, 1.5);
  wall(-4, -1.8, -2.5, -1.8, 0, 1.3);
  wall(0, -4, 0, -2.2, 0, 1.5);
  wall(2.5, -1, 5, -1, 0, 1.2);
  wall(5, -4, 5, -1, 0, 1.6);
  wall(.8, -.5, 3.2, -.5, 0, .8);
  wall(-.6, 4.8, 7.5, 4.8, 2.4, 3.35);
  wall(7.5, 1.8, 7.5, 4.8, 2.4, 3.35);
  wall(3.2, 1.8, 7.5, 1.8, 2.4, 2.8);
  // A small equipment block gives the floor point cloud a readable obstacle.
  wall(-5.8, 1, -4.9, 1, 0, .7); wall(-4.9, 1, -4.9, 1.8, 0, .7);
  const route = [[-4.7, -1, 0], [-2.1, -.4, 0], [2.1, -1.2, 0], [3.5, -.9, 0], [3.25, .8, 0], [3.05, .8, .15], [2.75, .8, .3], [2.45, .8, .45], [2.15, .8, .6], [1.85, .8, .75], [1.55, .8, .9], [1.25, .8, 1.05], [.95, .8, 1.2], [.1, 1.7, 1.2], [.1, 2.1, 1.35], [.1, 2.4, 1.5], [.1, 2.7, 1.65], [.1, 3, 1.8], [.1, 3.3, 1.95], [.1, 3.6, 2.1], [.1, 3.9, 2.25], [.1, 4.5, 2.4], [3.5, 4.5, 2.4], [6, 3, 2.4]];
  const robot = [-4.7, -1, 0];
  function project(p) {
    const c = Math.cos(state.angle), s = Math.sin(state.angle);
    const x = p[0] * c - p[1] * s, y = p[0] * s + p[1] * c;
    const scale = Math.min(width / 19.5, height / 13.5) * state.zoom;
    return [width * .48 + x * scale + state.panX, height * .63 + (y * Math.sin(state.tilt) - p[2] * Math.cos(state.tilt)) * scale + state.panY];
  }
  function visible(p) {
    return p[2] <= state.clip && (state.floor === 'all' || (state.floor === '1' ? p[2] < 1.25 : p[2] >= 1.2));
  }
  function line(a, b, color, size = 1, dash = []) {
    const pa = project(a), pb = project(b); ctx.strokeStyle = color; ctx.lineWidth = size; ctx.setLineDash(dash);
    ctx.beginPath(); ctx.moveTo(...pa); ctx.lineTo(...pb); ctx.stroke(); ctx.setLineDash([]);
  }
  function polygon(p, fill, stroke) {
    ctx.beginPath(); p.map(project).forEach((a, i) => i ? ctx.lineTo(...a) : ctx.moveTo(...a)); ctx.closePath();
    ctx.fillStyle = fill; ctx.fill(); if (stroke) { ctx.strokeStyle = stroke; ctx.lineWidth = .8; ctx.stroke(); }
  }
  function draw() {
    framePending = false;
    if (!width || !height) return;
    ctx.clearRect(0, 0, width, height);
    const glow = ctx.createRadialGradient(width * .52, height * .52, 20, width * .52, height * .52, width * .65);
    glow.addColorStop(0, '#143326'); glow.addColorStop(1, '#091b12'); ctx.fillStyle = glow; ctx.fillRect(0, 0, width, height);
    if (state.layers.grid) {
      for (let i = -15; i <= 15; i++) {
        line([i, -12, -.12], [i, 12, -.12], i % 5 ? '#24382e60' : '#3a51426a', .6);
        line([-15, i, -.12], [15, i, -.12], i % 5 ? '#24382e60' : '#3a51426a', .6);
      }
    }
    if (state.layers.surface) surfaces.forEach(f => {
      if (!visible([f.x0, f.y0, f.z])) return;
      const stairs = f.kind === 'stairs';
      polygon([[f.x0,f.y0,f.z],[f.x1,f.y0,f.z],[f.x1,f.y1,f.z],[f.x0,f.y1,f.z]], stairs ? '#b09b5733' : state.render === 'terrain' ? '#81ab7355' : '#4b8d7220', stairs ? '#bd9d6766' : '#7cba9b48');
    });
    if (state.layers.cloud) {
      const ordered = points.filter(visible).sort((a,b) => project(a)[1] - project(b)[1]);
      for (const p of ordered) {
        const a = project(p), wallPoint = p[3] === 'wall';
        ctx.fillStyle = p[3] === 'stairs' ? `rgba(206,175,110,${.3 + p[4] * .5})` : wallPoint ? `rgba(125,184,170,${.16 + p[4] * .48})` : `rgba(112,170,141,${.18 + p[4] * .42})`;
        const r = (wallPoint ? 1.15 : .9) * Math.min(1.5, state.zoom); ctx.fillRect(a[0], a[1], r, r);
      }
    }
    if (state.render === 'terrain') walls.forEach(w => {
      if (visible(w[0])) polygon(w, '#80a7920b', '#6e948133');
    });
    if (state.layers.path && state.path) {
      const r = [...route.slice(0, -1), state.goal];
      r.forEach((p,i) => {
        if (!i || !visible(p) || !visible(r[i-1])) return;
        line(r[i-1].map((v,j) => j === 2 ? v + .045 : v), p.map((v,j) => j === 2 ? v + .045 : v), '#a7c99125', 9);
        line(r[i-1].map((v,j) => j === 2 ? v + .05 : v), p.map((v,j) => j === 2 ? v + .05 : v), p[2] > 0 && p[2] < 2.4 ? '#dbc086' : '#c1daa1', 2);
        if (i % 2 === 0) { const q = project([p[0],p[1],p[2]+.05]); ctx.beginPath(); ctx.arc(...q, 2, 0, Math.PI*2); ctx.fillStyle='#e2edba'; ctx.fill(); }
      });
    }
    drawRobot();
    const goalOnScreen = visible(state.goal);
    if (goalOnScreen) {
      const g = project(state.goal); ctx.strokeStyle='#c7dca9'; ctx.lineWidth=1.4;
      ctx.beginPath(); ctx.ellipse(g[0],g[1],10,5,0,0,Math.PI*2); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(g[0],g[1]); ctx.lineTo(g[0],g[1]-29); ctx.stroke();
      ctx.fillStyle='#bdd49b'; ctx.beginPath(); ctx.moveTo(g[0],g[1]-29); ctx.lineTo(g[0]+12,g[1]-25); ctx.lineTo(g[0],g[1]-21); ctx.fill();
      $('goalLabel').style.left = `${Math.max(8,Math.min(width-145,g[0]+17))}px`;
      $('goalLabel').style.top = `${Math.max(60,Math.min(height-100,g[1]-41))}px`;
    }
    $('goalLabel').hidden = !goalOnScreen;
    const r = project(robot); ctx.fillStyle='#a5b7a3';ctx.font='9px "Microsoft YaHei", sans-serif';ctx.fillText('BPX-01',r[0]-18,r[1]+32);
    ctx.fillStyle='#668570';ctx.font='9px monospace';ctx.fillText('1F / +0.00 m',...project([-6,3.4,.05]));
    if (state.floor !== '1') ctx.fillText('2F / +2.40 m',...project([4.3,4.1,2.5]));
  }
  function drawRobot() {
    if (!visible(robot)) return;
    const [x,y,z] = robot;
    const r=project(robot); ctx.beginPath();ctx.ellipse(r[0],r[1],23,11,-.2,0,Math.PI*2);ctx.fillStyle='#93bd9722';ctx.fill();ctx.strokeStyle='#8faf7977';ctx.stroke();
    for (const dx of [-.36,.36]) for (const dy of [-.19,.19]) {
      line([x+dx,y+dy,z+.46],[x+dx+.1,y+dy*1.5,z+.23],'#b9c5b7',3);
      line([x+dx+.1,y+dy*1.5,z+.23],[x+dx-.1,y+dy*1.7,z+.03],'#7e9588',3);
    }
    const bottom=[[x-.5,y-.23,z+.38],[x+.5,y-.23,z+.38],[x+.5,y+.23,z+.38],[x-.5,y+.23,z+.38]];
    const top=bottom.map(p=>[p[0],p[1],p[2]+.19]);
    polygon([bottom[0],bottom[1],top[1],top[0]],'#6c8074','#a3b7a3');
    polygon([bottom[1],bottom[2],top[2],top[1]],'#485e51','#8aa28d');
    polygon(top,'#bfccc0','#e0e9d5');
    const sensor=project([x+.15,y,z+.7]);ctx.beginPath();ctx.ellipse(...sensor,4,3,0,0,Math.PI*2);ctx.fillStyle='#76a98e';ctx.fill();
    line([x,y,z+.02],[x+1,y,z+.02],'#b8cba0',1.5);
  }
  function invalidate() { if (!framePending) { framePending=true; requestAnimationFrame(draw); } }
  function resize() {
    const rect=canvas.getBoundingClientRect(); if (!rect.width) return;
    width=rect.width;height=rect.height;
    const dpr=Math.min(devicePixelRatio||1,2);canvas.width=Math.round(width*dpr);canvas.height=Math.round(height*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);invalidate();
  }
  new ResizeObserver(resize).observe(canvas);
  let drag=null;
  canvas.addEventListener('pointerdown',e=>{if(e.button!==0)return;drag={x:e.clientX,y:e.clientY,moved:false};canvas.setPointerCapture(e.pointerId);canvas.style.cursor='grabbing';});
  canvas.addEventListener('pointermove',e=>{
    if(!drag)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;
    if(Math.abs(dx)+Math.abs(dy)>2)drag.moved=true;
    if(e.shiftKey){state.panX+=dx;state.panY+=dy;}else{state.angle+=dx*.008;state.tilt=Math.max(.18,Math.min(1.55,state.tilt+dy*.005));}
    drag.x=e.clientX;drag.y=e.clientY;invalidate();
  });
  function endDrag(e){
    if(!drag)return;const wasClick=!drag.moved;drag=null;canvas.style.cursor=state.picking?'crosshair':'grab';
    if(wasClick&&state.picking&&e.type==='pointerup'){
      const rect=canvas.getBoundingClientRect(),mx=e.clientX-rect.left,my=e.clientY-rect.top;
      // Pick the nearest sampled surface in screen space, including its actual height.
      let best=null,distance=Infinity;
      for(const p of points){if(p[3]==='wall'||!visible(p))continue;const q=project(p),d=Math.hypot(mx-q[0],my-q[1]);if(d<distance){best=p;distance=d;}}
      if(best&&distance<22){state.goal=[best[0],best[1],Math.round(best[2]*20)/20];state.path=false;state.picking=false;canvas.style.cursor='grab';$('destination').textContent=state.goal[2]>1.8?'二层 · 自选目标':'地表 · 自选目标';$('goalCoordinates').textContent=`X ${state.goal[0].toFixed(2)}　Y ${state.goal[1].toFixed(2)}　Z ${state.goal[2].toFixed(2)} m`;$('goalLabel').textContent='◎ 自选目标 · '+state.goal[2].toFixed(2)+' m';$('routeLength').textContent='—';toast('目标已绑定到可见地表；请重新生成示意路线。');invalidate();}
      else toast('请点击可见的地表点；可先切换楼层或调整高度裁切。');
    }
  }
  canvas.addEventListener('pointerup',endDrag);canvas.addEventListener('pointercancel',endDrag);
  canvas.addEventListener('wheel',e=>{e.preventDefault();state.zoom=Math.max(.55,Math.min(2.8,state.zoom*Math.exp(-e.deltaY*.001)));invalidate();},{passive:false});
  function select(group, button){group.querySelectorAll('button').forEach(b=>b.classList.toggle('active',b===button));}
  $('views').addEventListener('click',e=>{const b=e.target.closest('button');if(!b)return;select($('views'),b);state.view=b.dataset.view;state.angle=state.view==='follow'?-.95:-.65;state.tilt=state.view==='top'?1.54:.62;state.panX=0;state.panY=0;if(state.view==='follow'){const q=project(robot);state.panX=width*.48-q[0];state.panY=height*.60-q[1];}invalidate();});
  $('renderModes').addEventListener('click',e=>{const b=e.target.closest('button');if(!b)return;select($('renderModes'),b);state.render=b.dataset.render;invalidate();});
  document.querySelector('.floor-switch').addEventListener('click',e=>{const b=e.target.closest('button');if(!b)return;select(document.querySelector('.floor-switch'),b);state.floor=b.dataset.floor;invalidate();});
  $('toggleLayers').onclick=()=>{$('layerPopover').hidden=!$('layerPopover').hidden;};
  document.querySelectorAll('[data-layer]').forEach(input=>input.onchange=()=>{state.layers[input.dataset.layer]=input.checked;invalidate();});
  $('heightClip').oninput=e=>{state.clip=Number(e.target.value);$('heightValue').textContent=state.clip.toFixed(1)+' m';invalidate();};
  $('resetView').onclick=()=>{state.angle=-.65;state.tilt=.62;state.panX=state.panY=0;state.zoom=1;state.view='free';select($('views'),$('views').firstElementChild);invalidate();};
  $('zoomIn').onclick=()=>{state.zoom=Math.min(2.8,state.zoom*1.15);invalidate();};
  $('zoomOut').onclick=()=>{state.zoom=Math.max(.55,state.zoom/1.15);invalidate();};
  function toast(message){$('toast').textContent=message;$('toast').classList.add('visible');clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),4200);}
  function log(message,type='演示'){
    const row=document.createElement('div');row.className='event';
    const t=document.createElement('time');t.textContent=new Date().toLocaleTimeString('zh-CN',{hour12:false});
    const label=document.createElement('span');label.className='event-type';label.textContent=type;
    const p=document.createElement('p');p.textContent=message;
    const small=document.createElement('small');small.textContent='本地演示';row.append(t,label,p,small);$('eventList').prepend(row);
    while($('eventList').children.length>20)$('eventList').lastElementChild.remove();
    $('eventCount').textContent=$('eventList').children.length;
  }
  function dialog(title,body){$('dialogTitle').textContent=title;$('dialogBody').innerHTML=body;$('detailDialog').showModal();}
  $('closeDialog').onclick=$('confirmDialog').onclick=()=>$('detailDialog').close();
  $('detailDialog').addEventListener('click',e=>{if(e.target===$('detailDialog')){const r=e.target.getBoundingClientRect();if(e.clientX<r.left||e.clientX>r.right||e.clientY<r.top||e.clientY>r.bottom)e.target.close();}});
  const health={
    control:['控制通道','未接入','未建立网页到机器人控制链路。连接、控制权和指令回执应分别展示；连接成功不能作为运动执行成功的依据。'],
    lidar:['LiDAR · MID-360','10 Hz · 模拟','最后数据间隔 24 ms。接入后依据点云时间戳、接收时刻和频率判断新鲜度。'],
    imu:['雷达 IMU','200 Hz · 模拟','此处代表雷达内置 IMU；机身 IMU 应独立显示，避免混淆数据来源。'],
    lio:['IteraLIO','运行中 · 模拟','展示纯 LiDAR–IMU 里程计状态。后续接入初始化、输出时效和配准诊断；LIO 运行不代表已有地图定位成功。'],
    tf:['TF 坐标链','变换完整 · 模拟','目标检查链为 map → odom → base_link → 传感器。实际发布者、外参方向及时间戳需接入时核对。'],
    localization:['地图定位','未接入','当前工作空间网页尚未接入已有地图定位。页面中的机器人位置为固定示意位置。'],
    navigation:['导航执行','未接入','已有离线三维规划预览；实时定位、动态避障、跟踪与运动执行仍需接入。']
  };
  document.querySelectorAll('[data-health]').forEach(b=>b.onclick=()=>{const h=health[b.dataset.health];dialog(h[0],`<p class="pill">${h[1]}</p><p>${h[2]}</p><p>本页面数据为设计演示，不反映设备实际状态。</p>`);});
  $('pickGoal').onclick=()=>{state.picking=!state.picking;canvas.style.cursor=state.picking?'crosshair':'grab';toast(state.picking?'点击地图地表设置目标；上下楼层可用左下角筛选。':'已退出选点。');};
  $('plan').onclick=()=>{state.path=true;$('taskBadge').textContent='规划预览';$('routeLength').textContent='18.6 m · 示例';log('已显示示意路线；未运行真实规划或碰撞检查。','规划');toast('已生成示意路线，真实规划算法未接入此设计页面。');invalidate();};
  $('cancel').onclick=()=>{state.path=false;$('taskBadge').textContent='待规划';$('routeLength').textContent='—';toast('已清除示意路线。');log('已清除示意路线。','任务');invalidate();};
  $('stop').onclick=()=>{state.path=false;state.mapping=false;$('taskBadge').textContent='已停止演示';$('routeLength').textContent='—';if(state.page==='mapping')renderModeCard();log('本地演示已停止；未向机器人发送停止指令。','停止');toast('演示已停止。控制通道未接入，机器人停车未确认。');invalidate();};
  function showEvents(open){$('events').hidden=!open;$('showEvents').setAttribute('aria-expanded',String(open));}
  $('showEvents').onclick=()=>showEvents($('events').hidden);
  $('collapseEvents').onclick=()=>showEvents(false);
  document.addEventListener('keydown',e=>{if(e.key==='Escape')showEvents(false);});
  const pages={overview:['机器人工作台','OVERVIEW','感知环境，理解地形，让每一步都有依据。'],mapping:['SLAM 建图','LIO MAPPING','从 LiDAR 与 IMU 出发，构建用于定位与导航的几何地图。'],maps:['地图管理','MAP LIBRARY','管理几何点云、地图版本与地形导航图。'],navigation:['定位与导航','LOCALIZATION / NAVIGATION','在三维地表上选择目标，检查跨层路线与执行条件。'],terrain:['地形与通行','TERRAIN ANALYSIS','查看地表连通关系、坡度与台阶候选。'],diagnostics:['设备与诊断','SYSTEM HEALTH','区分设备连接、数据新鲜度与功能就绪状态。'],settings:['参数设置','CONFIGURATION','为机器人尺寸、运动约束和页面显示建立清晰的参数边界。']};
  function renderModeCard(){
    $('modeCard').hidden=!['mapping','terrain'].includes(state.page);
    if(state.page==='mapping'){
      $('modeCard').innerHTML=`<div class="card-heading"><h2>建图会话</h2><span class="pill">本地演示</span></div><div class="mapping-progress">会话状态<strong>${state.mapping?'采集中':'待启动'}</strong>几何点云 · 纯 LiDAR–IMU</div><div class="action-row"><button class="button primary" id="mappingToggle">${state.mapping?'结束演示':'模拟建图'}</button><button class="button" id="saveMap">保存地图</button></div><p class="settings-note">此处只演示状态切换，不启动 IteraLIO。</p>`;
      $('mappingToggle').onclick=()=>{state.mapping=!state.mapping;renderModeCard();log(state.mapping?'建图会话进入采集演示。':'建图会话演示已结束。','建图');};
      $('saveMap').onclick=()=>{download('map-design-example.json',{demo:true,map_id:'building_a',unit:'m',z_up:true,geometry:'synthetic',note:'仅设计示例，不含真实 PCD 或导航缓存'});toast('已导出设计用地图元数据示例；未保存真实地图。');};
    }else if(state.page==='terrain'){
      $('modeCard').innerHTML='<div class="card-heading"><h2>地形分析</h2><span class="pill">几何候选</span></div><div class="setting-row"><span>普通坡度阈值</span><b>25°</b></div><div class="setting-row"><span>台阶高度上限</span><b>0.22 m</b></div><div class="setting-row"><span>机身包络半径</span><b>0.22 m</b></div><p class="settings-note">示例参数未经过目标机器狗能力标定。几何候选不等于可执行步态。</p>';
    }else $('modeCard').replaceChildren();
  }
  function download(name,data){const blob=new Blob([JSON.stringify(data,null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);}
  function renderContent(){
    const page=$('contentPage');
    if(state.page==='maps'){
      page.innerHTML='<div class="content-header"><p>3 个示例项目 · 仅展示地图管理流程</p><button class="button primary" id="importMap">＋ 导入 PCD</button><input id="pcdFile" type="file" accept=".pcd" hidden></div><div class="map-cards">'+[['园区 A · 多层环境','building_a','2 层 / 128,640 点','当前项目'],['室内走廊 · 一层','corridor_01','1 层 / 64,218 点','示例项目'],['楼梯测试区','stairs_lab','2 层 / 86,420 点','示例项目']].map((m,i)=>`<article class="map-card"><div class="map-art"><span class="pill">${m[3]}</span></div><div class="map-card-body"><h2>${m[0]}</h2><p>${m[1]} · ${m[2]}</p><p>几何地图 / Z ↑ / m / 模拟数据</p><button class="button ${i===0?'primary':''}" data-map="${i}">${i===0?'查看当前地图':'查看项目说明'}</button></div></article>`).join('')+'</div><p class="settings-note">导入在本原型中仅展示文件选择反馈，不读取或构建地图。真实地图可继续使用工作空间已有的离线规划页面。</p>';
      $('importMap').onclick=()=>$('pcdFile').click();$('pcdFile').onchange=e=>{const f=e.target.files[0];if(f){toast(`已选择 ${f.name}，此原型不解析 PCD。`);log(`已选择 PCD 文件：${f.name}；未导入地图。`,'地图');}};
      page.querySelectorAll('[data-map]').forEach(b=>b.onclick=()=>{if(b.dataset.map==='0')switchPage('overview');else dialog('示例地图项目','<p>此卡片用于展示地图库布局。当前三维场景仅提供“园区 A”合成地图，其他项目没有实际数据。</p>');});
    }else if(state.page==='diagnostics'){
      page.innerHTML='<div class="content-header"><p>以下为接口规划与模拟数据，尚未订阅真实 ROS 话题。</p><button id="exportDiagnostics" class="button">↓ 导出演示诊断</button></div><table class="diagnostic-table"><thead><tr><th>组件</th><th>状态</th><th>频率 / 延迟</th><th>数据来源或待接入项</th></tr></thead><tbody>'+[['机器人驱动','模拟在线','50 Hz','/dog/status'],['电池','76% · 模拟','—','/battery_state'],['LiDAR','模拟正常','10 Hz','LiDAR 点云 · 按 launch 核对'],['雷达 IMU','模拟正常','200 Hz','MID-360 IMU · 按 launch 核对'],['机身 IMU','未接入','—','/imu/data'],['IteraLIO','模拟运行','10 Hz','里程计与诊断 · 按 launch 核对'],['TF','模拟完整','24 ms','/tf /tf_static'],['地图定位','未接入','—','待定义状态接口'],['控制通道','未接入','—','/cmd_vel · 需控制权与回执机制'],['导航执行','未接入','—','待定义任务与反馈接口']].map(r=>`<tr><td>${r[0]}</td><td class="${r[1]==='未接入'?'warn-text':'ok-text'}">${r[1]}</td><td>${r[2]}</td><td><code>${r[3]}</code></td></tr>`).join('')+'</tbody></table>';
      $('exportDiagnostics').onclick=()=>download('diagnostics-demo.json',{demo:true,connected:false,components:health,note:'设计用示例，非运行时诊断'});
    }else if(state.page==='settings'){
      page.innerHTML='<div class="content-header"><p>修改仅作用于当前设计页面，不写入工作空间配置。</p><button id="saveSettings" class="button primary">保存页面偏好</button></div><div class="settings-grid"><section class="card"><div class="card-heading"><h2>地形与机器人 · 示例参数</h2><span class="subtle">未标定</span></div>'+[['机身包络半径 / m','radius',.22],['机身最低高度 / m','bottom',.30],['机身最高高度 / m','top',.85],['最大普通坡度 / °','slope',25],['台阶高度上限 / m','step',.22]].map(r=>`<label class="setting-row">${r[0]}<input id="setting-${r[1]}" data-pref="${r[1]}" type="number" min="0" step="0.01" value="${r[2]}"></label>`).join('')+'<p class="settings-note">调整不会触发地图重建或改变示意路线。实际参数需要机器人标定与真实数据验证。</p></section><section class="card"><div class="card-heading"><h2>页面显示</h2></div><label class="setting-row">默认视角<select data-pref="view"><option value="free">自由视角</option><option value="top">俯视</option></select></label><label class="setting-row">显示参考网格<input data-pref="grid" type="checkbox" checked></label><label class="setting-row">显示几何点云<input data-pref="cloud" type="checkbox" checked></label><label class="setting-row">显示规划路线<input data-pref="path" type="checkbox" checked></label><p class="settings-note">偏好保存在此浏览器，受浏览器本地文件存储策略影响。</p></section></div>';
      const controlFields=[['前后速度上限 / m/s','speedX',.30,.01],['横移速度上限 / m/s','speedY',.20,.01],['转向角速度上限 / rad/s','speedYaw',.40,.01],['线加速度上限 / m/s²','acceleration',.30,.01],['指令超时 / ms','commandTimeout',300,10]];
      page.querySelector('.settings-grid').insertAdjacentHTML('afterbegin',
        '<section class="card"><div class="card-heading"><h2>手动控制参数</h2><span class="pill">未接入</span></div>'+
        controlFields.map(([label,key,value,step])=>`<label class="setting-row">${label}<input data-pref="${key}" type="number" min="${step}" step="${step}" value="${value}"></label>`).join('')+
        '<div class="control-layout"><strong>键位说明</strong><div class="keyboard"><span></span><kbd>W</kbd><span></span><kbd>Q</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd><kbd>E</kbd></div><div class="keyboard-hint">W / S 前后 · A / D 横移 · Q / E 转向</div></div><p class="settings-note">仅保存设计用参数，不下发机器人。指令超时停车需要后端执行。</p></section>');
      try{const prefs=JSON.parse(localStorage.getItem('iteralio-design-prefs')||'{}');page.querySelectorAll('[data-pref]').forEach(el=>{if(Object.hasOwn(prefs,el.dataset.pref)){if(el.type==='checkbox')el.checked=prefs[el.dataset.pref];else el.value=prefs[el.dataset.pref];}});}catch{/* Local file storage may be restricted. */}
      $('saveSettings').onclick=()=>{
        const prefs={};let valid=true;page.querySelectorAll('[data-pref]').forEach(el=>{if(!el.checkValidity())valid=false;prefs[el.dataset.pref]=el.type==='checkbox'?el.checked:el.value;});
        if(!valid){toast('请按输入框要求填写有效数值；控制上限与超时必须大于 0。');return;}
        if(Number(prefs.bottom)>=Number(prefs.top)){toast('机身最低高度必须小于最高高度。');return;}
        for(const layer of ['grid','cloud','path']){state.layers[layer]=prefs[layer];document.querySelector(`[data-layer="${layer}"]`).checked=prefs[layer];}
        state.view=prefs.view;state.tilt=state.view==='top'?1.54:.62;select($('views'),$('views').querySelector(`[data-view="${state.view}"]`));
        try{localStorage.setItem('iteralio-design-prefs',JSON.stringify(prefs));toast('页面偏好已保存；ROS 参数未修改。');}catch{toast('显示偏好已应用；浏览器未允许本地存储。');}invalidate();
      };
    }
  }
  function switchPage(key){
    state.page=key;document.querySelectorAll('[data-page]').forEach(b=>{const active=b.dataset.page===key;b.classList.toggle('active',active);if(active)b.setAttribute('aria-current','page');else b.removeAttribute('aria-current');});
    $('mainWorkspace').setAttribute('aria-label',pages[key][0]);
    document.title=`${pages[key][0]} · IteraLIO`;
    const content=['maps','diagnostics','settings'].includes(key);$('sceneWorkspace').hidden=content;$('contentPage').hidden=!content;
    $('sceneTitle').textContent=key==='mapping'?'几何建图与轨迹':key==='terrain'?'多层地表与通行分析':key==='navigation'?'三维地图与路径':'三维场景与感知';
    if(key==='terrain'){state.render='terrain';select($('renderModes'),$('renderModes').lastElementChild);}else{state.render='cloud';select($('renderModes'),$('renderModes').firstElementChild);}
    renderModeCard();if(content)renderContent();else requestAnimationFrame(resize);
  }
  document.querySelectorAll('[data-page]').forEach(b=>b.onclick=()=>switchPage(b.dataset.page));
  document.querySelector('[data-page="overview"]').setAttribute('aria-current','page');
  function clock(){$('clock').textContent=new Date().toLocaleTimeString('zh-CN',{hour12:false});}
  clock();setInterval(clock,1000);resize();
})();
