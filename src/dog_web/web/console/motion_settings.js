// The backend owns persistence and validates the same limits used by teleoperation.
export async function mountMotionSettings(root, {api, release, onChanged}) {
  root.innerHTML = '<section class="motion-settings"><h2>机器人移动速度</h2><p class="settings-note">设置方向按钮和 W / S / A / D / Q / E 的速度。松开滑动条后自动保存，下次启动恢复上次配置。</p><div class="motion-sliders"></div><p class="settings-note" role="status" id="motionSaveStatus">正在读取已保存配置…</p><p class="settings-note">调整配置会停止当前网页控制。指令超时 350 ms；实际运动状态以机器人反馈为准。</p></section>';
  const status = root.querySelector('#motionSaveStatus');
  const list = root.querySelector('.motion-sliders');
  const fields = [['x','前进 / 后退',1,'m/s'], ['y','左右平移',.3,'m/s'], ['yaw','原地旋转',1,'rad/s']];
  let saved;
  const inputs = {};
  function render(values) {
    for (const [key,,,unit] of fields) {
      inputs[key].value = values[key];
      root.querySelector(`#motion-${key}-value`).textContent = `${Number(values[key]).toFixed(2)} ${unit}`;
    }
  }
  for (const [key,label,max,unit] of fields) {
    const card = document.createElement('label'); card.className = 'motion-slider';
    card.htmlFor = `motion-${key}`;
    card.innerHTML = `<span>${label}</span><output id="motion-${key}-value" for="motion-${key}">—</output><input id="motion-${key}" type="range" min="0" max="${max}" step="0.01" disabled><small>0.00 — ${max.toFixed(2)} ${unit}</small>`;
    list.append(card);
    const input = card.querySelector('input'); inputs[key] = input;
    input.oninput = () => {card.querySelector('output').textContent = `${Number(input.value).toFixed(2)} ${unit}`;};
    input.onchange = async () => {
      const values = Object.fromEntries(fields.map(([k]) => [k, Number(inputs[k].value)]));
      Object.values(inputs).forEach(element => element.disabled = true);
      status.textContent = '正在保存…';
      try {
        await release();
        saved = await api('/api/settings/motion', values);
        onChanged(saved);
        if (card.isConnected) {render(saved); status.textContent = '已保存；下次启动沿用此设置';}
      } catch (error) {
        if (card.isConnected) {render(saved); status.textContent = `保存失败：${error.message}`;}
      } finally {Object.values(inputs).forEach(element => element.disabled = false);}
    };
  }
  try {
    saved = await api('/api/settings/motion');
    if (!list.isConnected) return;
    render(saved); onChanged(saved);
    Object.values(inputs).forEach(element => element.disabled = false);
    status.textContent = '已读取上次配置';
  } catch (error) {if (status.isConnected) status.textContent = `读取失败：${error.message}`;}
}
