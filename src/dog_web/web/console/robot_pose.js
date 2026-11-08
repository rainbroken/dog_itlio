// Display policy only: the backend resolves map -> base_link and expires TF.
// A saved navigation project needs a matching, confirmed localization as well.
export function localizationMapStatus(loc, map) {
  if (!map) return {state:'no_map', reason:'请加载导航地图'};
  if (!loc?.fresh) return {state:'unavailable', reason:'定位节点未就绪或状态已过期'};
  if (!loc.map_ready) return {state:'not_loaded', reason:'定位地图尚未加载完成'};
  if (typeof loc.map_path !== 'string' || !loc.map_path.trim())
    return {state:'unknown_map', reason:'定位节点尚未报告地图路径'};
  const id = loc.map_path.replace(/\/+$/, '').replace(/\/(?:localization|cloud)\.pcd$/, '').split('/').at(-1);
  if (id !== map.map_id) return {state:'mismatch', reason:`定位地图与当前地图不一致（${id} / ${map.map_id}）`};
  return loc.valid ? {state:'ready'} : {state:'unconfirmed', reason:'地图定位未确认或已失效'};
}

export function robotPlacement(data, map, {page = 'overview', frame = 'map'} = {}) {
  const pose = data?.pose;
  const live = page === 'mapping';
  if (!live && page === 'navigation' && !map) return {reason:'请加载导航地图'};
  if (!live && map) {
    const status = localizationMapStatus(data?.localization, map);
    if (status.state !== 'ready') return {reason:status.reason};
  }
  if (!data?.tf_ready || !pose) return {reason:'等待新鲜的 map → base_link TF'};
  if (pose.frame !== frame) return {reason:`位姿与场景坐标系不一致（${pose.frame || '未知'} / ${frame}）`};
  if (!Number.isFinite(pose.age_ms) || Math.abs(pose.age_ms) >= 1000) return {reason:'机身 TF 已过期'};
  const p = pose.position, q = pose.quaternion;
  if (!Array.isArray(p) || p.length !== 3 || !p.every(Number.isFinite) ||
      !Array.isArray(q) || q.length !== 4 || !q.every(Number.isFinite) || Math.abs(Math.hypot(...q) - 1) >= .01)
    return {reason:'机身位姿数据无效'};
  return {pose, mode:live ? 'live' : map ? 'map' : 'live'};
}
