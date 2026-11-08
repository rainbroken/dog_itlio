const labels={waiting_initial_pose:'等待初始位姿',initializing:'初值配准确认中',tracking:'定位有效',
  degraded:'定位质量下降',lost:'定位失效',relocalizing:'附近重定位中',global_relocalizing:'全局搜索中'};
const reasons={global_relocalization_requested:'全局搜索请求已接收',relocalization_requested:'附近搜索请求已接收',
  global_time_budget_exceeded:'全局搜索超时，等待下一次尝试；也可设置人工初值',
  global_no_verified_candidate:'未找到通过验证的位置，等待重试；请尝试人工初值',
  ambiguous_global_relocalization:'存在多个相似位置，无法唯一确定；请设置人工初值',
  ambiguous_relocalization:'附近存在多个相似位置，请调整初值',
  insufficient_overlap_or_large_residual:'点云重叠不足或配准残差过大',
  global_requires_upright_gravity_frame:'机身倾角过大，无法进行全局搜索',
  global_hypothesis_budget_exceeded:'候选位置超出搜索预算，请设置人工初值',
  global_no_horizontal_surface_anchors:'地图缺少可搜索的地表位置',
  scan_too_small:'有效扫描点数不足',scan_gap:'点云输入中断',stale_data:'定位数据已过期',
  stale_scan:'扫描时间戳已过期',registration_result_too_old:'配准结果已过期',
  not_processed:'等待第一帧配准',not_initialized:'等待初始化'};
export function localizationFeedback(loc) {
  if(!loc?.fresh)return '定位节点未就绪或状态已过期';
  if(loc.input_fresh===false)return '等待新鲜的 LIO 去畸变点云；请检查 LiDAR 与 LIO 输入';
  if(loc.valid)return `已确认定位${loc.localization_source==='historical_keyframes'?' · 历史关键帧匹配':''}`;
  const label=labels[loc.state]||loc.state||'等待定位';
  if(loc.registration_running)return `${label} · 正在配准，请等待结果`;
  if(loc.reason==='accepted')return `${label} · 连续确认 ${loc.confirmations||0} 帧`;
  return `${label}${loc.reason?` · ${reasons[loc.reason]||loc.reason}`:''}`;
}
