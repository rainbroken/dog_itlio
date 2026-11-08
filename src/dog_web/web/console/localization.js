// UI adapter for the actual localizer. Submitting a seed is never treated as a
// successful localization; only the subsequently polled status can confirm it.
import {localizationMapStatus} from './robot_pose.js';
import {localizationFeedback} from './localization_feedback.js';

export function mountLocalization({api, tell, pick, preview}) {
  const card=document.createElement('section');card.className='card localization-card';
  card.innerHTML=`<details open><summary>01 / 确认定位</summary>
    <p id="localizationStatus" role="status">等待定位节点</p>
    <p id="globalLocalizationTime" class="settings-note" role="status" hidden></p>
    <p id="initialPosition">请先启动同一地图的定位；网页加载地图仅用于预览。未自动定位成功时可点选初值。</p>
    <div class="initial-fields"><label>地图朝向（°）<input id="initialYaw" type="number" min="-180" max="180" step="1" value="0"></label>
    <label>机身高度偏移（m）<input id="initialHeight" type="number" min="0.1" max="1.5" step="0.01" value="0.40"></label></div>
    <p class="settings-note">按住地面拖向机头方向，松开后提交。0° 朝 +X，90° 朝 +Y。</p>
    <div class="action-row"><button class="button" id="pickInitialPose">设置位置与朝向</button><button class="button primary" id="applyInitialPose">提交初值</button></div>
    <div class="action-row"><button class="button" id="localRelocalize">附近重定位</button><button class="button" id="globalRelocalize">全局重定位</button></div>
  </details>`;
  document.querySelector('.inspector').append(card);
  const get=id=>card.querySelector(`#${id}`);
  let map=null,robot=null,position=null,busy=false,revision=null,pending=null;
  function redraw(){preview(position?{point:position,yaw:Number(get('initialYaw').value)*Math.PI/180}:null);}
  function update(data,nextMap){
    robot=data;map=nextMap;
    const key=map?`${map.map_id}/${map.revision}`:null;
    if(key!==revision){revision=key;position=null;pending=null;redraw();get('initialPosition').textContent='人工初值：按住机器人脚下地面，拖向机头方向。';}
    const loc=robot?.localization;
    const status=localizationMapStatus(loc,map);
    const ready=status.state==='ready'||status.state==='unconfirmed';
    get('localizationStatus').textContent=localizationFeedback(loc);
    const timing=get('globalLocalizationTime'),elapsed=loc?.global_relocalization_elapsed_ms;
    timing.hidden=!Number.isFinite(elapsed)||loc.global_timing_state==='idle';
    timing.textContent=timing.hidden?'':`全局重定位${loc.global_timing_state==='succeeded'?'用时':loc.global_timing_state==='cancelled'?'已取消，用时':'已用时'} ${(elapsed/1000).toFixed(1)} 秒${loc.global_timing_state==='succeeded'?' · 已完成':''}`;
    const searching=loc?.fresh&&['initializing','global_relocalizing','relocalizing'].includes(loc.state);
    if(pending&&loc?.fresh&&!loc.valid)pending.observed=true;
    if(pending?.observed&&loc?.fresh&&loc.valid){tell('定位已确认');pending=null;}
    if(pending&&loc?.fresh&&loc.input_fresh!==false&&!loc.valid)
      get('localizationStatus').textContent+=` · 已等待 ${Math.floor((performance.now()-pending.since)/1000)} 秒`;
    if(ready&&busy)get('localizationStatus').textContent='正在提交定位请求…';
    else if(ready&&pending&&!pending.observed&&loc.input_fresh!==false)get('localizationStatus').textContent='请求已接收，等待定位节点开始处理';
    if(status.state!=='ready'&&status.state!=='unconfirmed')get('localizationStatus').textContent=status.reason;
    get('pickInitialPose').disabled=busy||!map;
    get('applyInitialPose').disabled=busy||!ready||!position;
    get('localRelocalize').disabled=busy||!ready||searching||!loc.has_prior;
    get('localRelocalize').title='以已有定位为中心搜索；修改位置和朝向请使用“设置位置与朝向”并提交初值';
    get('globalRelocalize').disabled=busy||!ready||searching||!loc.global_relocalization_enabled;
    get('globalRelocalize').title=loc?.global_relocalization_enabled?'优先检索历史关键帧并验证位置与朝向；无需点选初值':'需要在定位 YAML 中开启全局重定位';
  }
  async function request(endpoint,extra={}){
    if(!map)return tell('请先加载定位使用的地图',true);
    busy=true;update(robot,map);
    try{const result=await api(endpoint,{map_id:map.map_id,revision:map.revision,...extra});pending={since:performance.now()};tell(endpoint.endsWith('/initialize')?'初值已提交，等待连续配准确认':'重定位请求已接收，等待点云处理与定位确认');}
    catch(error){tell(error.message,true);}
    finally{busy=false;update(robot,map);}
  }
  get('pickInitialPose').onclick=()=>pick();
  get('applyInitialPose').onclick=()=>{
    const degrees=Number(get('initialYaw').value),height=Number(get('initialHeight').value);
    if(!Number.isFinite(degrees)||Math.abs(degrees)>180||!Number.isFinite(height)||height<.1||height>1.5)
      return tell('请填写有效朝向（-180～180°）和机身高度偏移（0.1～1.5 m）',true);
    request('/api/localization/initialize',{ground_position:position,yaw:degrees*Math.PI/180,body_height:height});
  };
  get('localRelocalize').onclick=()=>request('/api/localization/relocalize');
  get('globalRelocalize').onclick=()=>request('/api/localization/global_relocalize');
  get('initialYaw').oninput=redraw;
  update(null,null);
  return {update,choose(point,yaw){position=point.slice();if(Number.isFinite(yaw))get('initialYaw').value=(yaw*180/Math.PI).toFixed(1);get('initialPosition').textContent=`地面 XYZ：${point.map(v=>v.toFixed(2)).join(', ')} m`;redraw();update(robot,map);}};
}
