export function mountMapTaskNotice(element) {
  const title=element.querySelector('[data-task-title]');
  const detail=element.querySelector('[data-task-detail]');
  const close=element.querySelector('button');
  close.onclick=()=>{if(element.dataset.state!=='running')element.hidden=true;};
  function show(state,heading,message){
    element.hidden=false;element.dataset.state=state;
    element.setAttribute('aria-busy',String(state==='running'));
    title.textContent=heading;detail.textContent=message;close.disabled=state==='running';
  }
  return {
    start(title){show('running',title,'正在处理，请稍候；完成后会在这里提醒，无需重复点击。');},
    loading(){show('running','导航图已就绪，正在加载显示','正在读取地图并更新三维场景…');},
    complete(map){show('complete','地图加载完成',`${map.map_id} · ${map.points} 个地图点 · ${map.valid_surfaces} 个可通行节点。进入定位与导航选择目标。`);},
    fail(error){show('failed','地图任务失败',error.message);}
  };
}
