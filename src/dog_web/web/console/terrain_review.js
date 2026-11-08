// Surface nodes carry component IDs from the saved navigation graph.
// These counts describe graph connectivity, not verified robot traversability.
export function surfaceRegions(surfaces = []) {
  const counts = new Map();
  for (const row of surfaces) counts.set(row[3], (counts.get(row[3]) || 0) + 1);
  return [...counts].sort((a, b) => b[1] - a[1] || a[0] - b[0]);
}

export function mountTerrainReview(root, {map, clip, onClip, onRegion}) {
  root.innerHTML = `<div class="terrain-review"><section><span class="eyebrow">CONNECTIVITY</span><h2>地表连通分析</h2><p class="settings-note">颜色区分导航图的连通区域。连通并不代表机器人已经验证可通过。</p><div class="terrain-facts"><div><small>可通行候选节点</small><strong data-fact="nodes">—</strong></div><div><small>连通区域</small><strong data-fact="regions">—</strong></div><div><small>地表高度范围 / m</small><strong data-fact="height">—</strong></div></div><label class="terrain-filter">查看连通区域<select aria-label="筛选地表连通区域" id="terrainRegion"></select></label><label class="terrain-filter">显示最高高度 <output id="terrainClipValue">—</output><input id="terrainClip" aria-label="地形最高显示高度" type="range" step="0.1"></label><p class="settings-note" data-region-info></p></section><section><span class="eyebrow">MAP CONSTRAINTS</span><h2>当前地图构建约束</h2><div class="terrain-constraints"></div><p class="settings-note">读取当前地图保存的参数。修改下一次导入配置不会重建当前地图；历史圆形包络不能等同于 0.66 × 0.58 m 的矩形碰撞校验。</p><details><summary>当前地图完整参数</summary><pre id="terrainParameters"></pre></details></section><section id="terrainNextConfig"></section></div>`;
  const get = selector => root.querySelector(selector);
  const surfaces = map?.surfaces || [], regions = surfaceRegions(surfaces);
  get('[data-fact="nodes"]').textContent = map ? map.valid_surfaces : '未加载地图';
  get('[data-fact="regions"]').textContent = map ? map.components : '—';
  let low = Infinity, high = -Infinity;
  for (const row of surfaces) {low = Math.min(low, row[2]); high = Math.max(high, row[2]);}
  get('[data-fact="height"]').textContent = surfaces.length ? `${low.toFixed(2)} ～ ${high.toFixed(2)}` : '—';
  const select = get('#terrainRegion');
  for (const [value, label] of [['all', '全部连通区域'], ...regions.map(([id, count]) => [String(id), `区域 ${id} · ${count} 节点`])]) {
    const option = document.createElement('option');option.value = value;option.textContent = label;select.append(option);
  }
  select.disabled = !surfaces.length;
  select.onchange = () => {
    const region = select.value === 'all' ? null : Number(select.value);
    onRegion(region);
    get('[data-region-info]').textContent = region == null ? '显示全部地表；不同区域之间没有导航图连接。' : `仅显示区域 ${region}；筛选只影响显示，不修改地图或规划约束。`;
  };
  get('[data-region-info]').textContent = '显示全部地表；不同区域之间没有导航图连接。';
  const slider = get('#terrainClip');slider.disabled = !surfaces.length;
  slider.min = map?.bounds?.[0]?.[2] ?? 0;slider.max = map?.bounds?.[1]?.[2] ?? 1;slider.value = clip;
  const showClip = () => {get('#terrainClipValue').textContent = `${Number(slider.value).toFixed(2)} m`;};
  slider.oninput = () => {showClip();onClip(Number(slider.value));};showClip();
  const config = map?.config || map?.parameters || {};
  const fields = [['网格分辨率', config.resolution, 'm'], ['坡度上限', config.max_slope_deg, '°'], ['台阶高度上限', config.max_step, 'm'], ['机身长度', config.body_length, 'm'], ['机身宽度', config.body_width, 'm']];
  if (config.body_length == null && config.body_radius != null) fields.push(['历史包络半径', config.body_radius, 'm']);
  for (const [label, value, unit] of fields) {
    const row = document.createElement('div');row.className = 'terrain-constraint';
    const name = document.createElement('span'), result = document.createElement('b');name.textContent = label;result.textContent = value == null ? '未记录' : `${value} ${unit}`;row.append(name, result);get('.terrain-constraints').append(row);
  }
  get('#terrainParameters').textContent = JSON.stringify(config, null, 2);
}
