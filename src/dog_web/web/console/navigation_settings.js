const fields=[['walk_speed','最大线速度',1,'m/s'],['yaw_rate','最大角速度',1,'rad/s'],
  ['acceleration','线加减速度',1,'m/s²'],['yaw_acceleration','角加减速度',2,'rad/s²'],
  ['stair_speed','楼梯线速度',1,'m/s'],['stair_yaw_rate','楼梯角速度',1,'rad/s']];
export async function mountNavigationSettings(root,{api,release}){
  const panel=document.createElement('section');panel.className='motion-settings motion-controls navigation-motion-controls';
  panel.innerHTML='<h2>自主导航速度与加速度</h2><div class="motion-sliders"></div><p class="settings-note">保存后重启导航服务生效。BPX 起步使用 0.21 避开 0.20 死区；楼梯速度不能超过普通速度。起停跨越死区时，指令变化不遵循连续加速度斜坡；实机响应仍需验证。</p><button class="button" disabled>保存导航参数</button><p class="settings-note" role="status">正在读取导航参数…</p>';
  root.append(panel);const inputs={},status=panel.querySelector('[role="status"]'),save=panel.querySelector('button');
  for(const [key,label,max,unit] of fields){
    const row=document.createElement('label');row.className='motion-slider';row.htmlFor='navigation-'+key;
    row.innerHTML=`<span>${label}</span><output>—</output><input id="navigation-${key}" type="range" aria-label="导航${label}（${unit}）" min="0.01" max="${max}" step="0.01" disabled>`;
    inputs[key]=row.querySelector('input');inputs[key].oninput=()=>row.querySelector('output').textContent=Number(inputs[key].value).toFixed(2)+' '+unit;panel.querySelector('.motion-sliders').append(row);
    if(!key.includes('acceleration'))inputs[key].min='.21';
  }
  const render=data=>{
    for(const [key] of fields){inputs[key].value=data.saved[key];inputs[key].oninput();}
    status.textContent=data.active===null?'执行节点未提供当前参数；显示已保存值或默认值，保存后需重启导航服务。':
      data.restart_required?'已保存，重启导航服务后生效。':'当前导航参数已生效。';
    if(data.active)status.textContent+=' 当前运行：'+fields.map(([key,label,,unit])=>`${label} ${Number(data.active[key]).toFixed(2)} ${unit}`).join('；');
  };
  try{const data=await api('/api/settings/navigation');if(!panel.isConnected)return;render(data);save.disabled=false;Object.values(inputs).forEach(input=>input.disabled=false);}
  catch(error){status.textContent='读取导航参数失败：'+error.message;}
  save.onclick=async()=>{
    const values=Object.fromEntries(fields.map(([key])=>[key,Number(inputs[key].value)]));
    if(values.stair_speed>values.walk_speed||values.stair_yaw_rate>values.yaw_rate){status.textContent='楼梯线速度、角速度不能超过对应的普通速度。';return;}
    save.disabled=true;Object.values(inputs).forEach(input=>input.disabled=true);
    try{await release();const data=await api('/api/settings/navigation',values);if(panel.isConnected)render(data);}
    catch(error){if(panel.isConnected)status.textContent='保存失败：'+error.message;}
    finally{save.disabled=false;Object.values(inputs).forEach(input=>input.disabled=false);}
  };
}
