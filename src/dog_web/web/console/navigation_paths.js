const activeStates=new Set(['waiting_inputs','following','paused','relocalizing','waiting_route','waiting_walk_state']);
const reasons={planning_budget_exceeded:'搜索达到时间预算，正在重试',route_expired:'路径已过期',
  route_ready:'等待同一帧路径',new_goal_pending:'正在接收新目标',waiting_inputs:'等待规划输入',
  scan_unavailable:'等待实时点云',start_blocked:'当前位置附近受障碍阻挡',goal_blocked:'终点附近受障碍阻挡',
  processing_or_data_expired:'规划耗时或输入过期',blocked_no_route:'实时障碍阻断路线',
  localization_unavailable:'等待有效定位',waiting_goal:'等待执行目标',cancelled:'已取消',
  map_mismatch:'地图不一致',goal_mismatch:'等待当前任务的路径',ros_unavailable:'ROS 未连接'};
export function consumePathLease(data,elapsed){
  if(!data?.valid)return data;
  if(!Number.isFinite(elapsed)||elapsed<0)return {valid:false,reason:'route_expired'};
  return {...data,valid_for_ms:data.valid_for_ms-elapsed};
}
export function navigationPathState(data,map,nav){
  if(!nav?.fresh||!activeStates.has(nav.task_state))return {valid:false,label:'局部路径：等待导航'};
  const fail=reason=>({valid:false,label:'局部路径：'+(reasons[reason]||reason||'等待规划')});
  if(!data?.valid)return fail(data?.reason);
  if(!data.task_id||data.task_id!==nav.task_id)return fail('goal_mismatch');
  if(!map||data.map_revision!==map.revision||data.map_project!==nav.map_project||
     data.map_project?.replace(/\/$/,'').split('/').at(-1)!==map.map_id||data.frame_id!=='map')return fail('map_mismatch');
  const validPoints=rows=>Array.isArray(rows)&&rows.length>0&&rows.length<=5000&&
    rows.every(p=>Array.isArray(p)&&p.length===3&&p.every(Number.isFinite));
  if(!validPoints(data.global_path)||!validPoints(data.local_path)||
     !Number.isFinite(data.valid_for_ms)||data.valid_for_ms<=0)return fail('route_expired');
  return {valid:true,label:`局部路径：${data.local_path.length} 点 · 实时更新`,
    global:data.global_path,local:data.local_path,sequence:data.sequence,lease:data.valid_for_ms};
}
