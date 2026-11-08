// UI adapter for the actual localizer. Submitting a seed is never treated as a
// successful localization; only the subsequently polled status can confirm it.
export function mountLocalization({api, tell, pick, preview}) {
  const card=document.createElement('section');card.className='card localization-card';
  card.innerHTML=`<details open><summary>01 / 确认定位</summary>
    <p id="localizationStatus" role="status">等待定位节点</p>
    <p id="initialPosition">加载同一地图后自动全局定位；未成功时可点选初值。</p>
    <div class="initial-fields"><label>地图朝向（°）<input id="initialYaw" type="number" min="-180" max="180" step="1" value="0"></label>
    <label>机身高度偏移（m）<input id="initialHeight" type="number" min="0.1" max="1.5" step="0.01" value="0.65"></label></div>
    <p class="settings-note">0° 朝地图 +X，90° 朝 +Y。高度偏移请按实际站姿校准。</p>
    <div class="action-row"><button class="button" id="pickInitialPose">点选位置</button><button class="button primary" id="applyInitialPose">提交初值</button></div>
    <div class="action-row"><button class="button" id="localRelocalize">附近重定位</button><button class="button" id="globalRelocalize">全局重定位</button></div>
  </details>`;
  document.querySelector('.inspector').append(card);
  const get=id=>card.querySelector(`#${id}`);
  let map=null,robot=null,position=null,busy=false,revision=null;
  const labels={waiting_initial_pose:'等待初始位姿',initializing:'初值配准确认中',tracking:'定位有效',
    degraded:'定位质量下降',lost:'定位失效',relocalizing:'附近重定位中',global_relocalizing:'全局搜索中'};
  function redraw(){preview(position?{point:position,yaw:Number(get('initialYaw').value)*Math.PI/180}:null);}
  function update(data,nextMap){
    robot=data;map=nextMap;
    const key=map?`${map.map_id}/${map.revision}`:null;
    if(key!==revision){revision=key;position=null;redraw();get('initialPosition').textContent='自动定位无需点选；人工初值请选择机器人脚下地面。';}
    const loc=robot?.localization;
    const sameMap=map&&loc?.map_path?.replace(/\/(?:localization|cloud)\.pcd$/,'').split('/').at(-1)===map.map_id;
    const ready=!!(map&&loc?.fresh&&loc?.map_ready&&sameMap);
    get('localizationStatus').textContent=loc?.fresh?`${loc.valid?'已确认定位':labels[loc.state]||loc.state||'等待定位'}${loc.reason&&loc.reason!=='accepted'?` · ${loc.reason}`:''}`:'定位节点未就绪或状态过期';
    if(loc?.fresh&&loc.localization_source==='historical_keyframes')get('localizationStatus').textContent+=' · 历史关键帧匹配';
    if(map&&loc?.fresh&&!sameMap)get('localizationStatus').textContent='网页地图与定位地图不同，请加载同一地图项目。';
    get('pickInitialPose').disabled=busy||!map;
    get('applyInitialPose').disabled=busy||!ready||!position;
    get('localRelocalize').disabled=busy||!ready||!loc.has_prior;
    get('globalRelocalize').disabled=busy||!ready||!loc.global_relocalization_enabled;
    get('globalRelocalize').title=loc?.global_relocalization_enabled?'优先检索历史关键帧并验证位置与朝向；无需点选初值':'需要在定位 YAML 中开启全局重定位';
  }
  async function request(endpoint,extra={}){
    if(!map)return tell('请先加载定位使用的地图',true);
    busy=true;update(robot,map);
    try{const result=await api(endpoint,{map_id:map.map_id,revision:map.revision,...extra});tell(result.message||'请求已提交，等待定位节点确认');}
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
  return {update,choose(point){position=point.slice();get('initialPosition').textContent=`地面 XYZ：${point.map(v=>v.toFixed(2)).join(', ')} m`;redraw();update(robot,map);}};
}
