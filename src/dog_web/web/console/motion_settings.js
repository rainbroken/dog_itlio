// The backend owns persistence and validates the same limits used by teleoperation.
export async function mountMotionSettings(root, {api, release, onChanged}) {
  root.innerHTML = '<section class="motion-settings motion-controls" aria-busy="true"><h2>机器人移动速度</h2><div class="motion-sliders"></div><p class="settings-note" role="status" id="motionSaveStatus" hidden></p></section>';
  const status = root.querySelector('#motionSaveStatus');
  const list = root.querySelector('.motion-sliders');
  const panel=root.querySelector('.motion-controls');
  const fields = [['x','前后',1,'m/s'], ['y','左右',.3,'m/s'], ['yaw','旋转',1,'rad/s']];
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
    card.innerHTML = `<span>${label}</span><output id="motion-${key}-value" for="motion-${key}">—</output><input id="motion-${key}" aria-label="${label}限制速度（${unit}）" type="range" min="0" max="${max}" step="0.01" disabled>`;
    list.append(card);
    const input = card.querySelector('input'); inputs[key] = input;
    input.oninput = () => {card.querySelector('output').textContent = `${Number(input.value).toFixed(2)} ${unit}`;};
    input.onchange = async () => {
      const values = Object.fromEntries(fields.map(([k]) => [k, Number(inputs[k].value)]));
      Object.values(inputs).forEach(element => element.disabled = true);
      status.hidden=true;panel.setAttribute('aria-busy','true');
      try {
        await release();
        saved = await api('/api/settings/motion', values);
        onChanged(saved);
        if (card.isConnected) render(saved);
      } catch (error) {
        if (card.isConnected) {render(saved); status.textContent = `保存失败：${error.message}`;status.hidden=false;}
      } finally {panel.setAttribute('aria-busy','false');Object.values(inputs).forEach(element => element.disabled = false);}
    };
  }
  try {
    saved = await api('/api/settings/motion');
    if (!list.isConnected) return;
    render(saved); onChanged(saved);
    Object.values(inputs).forEach(element => element.disabled = false);
  } catch (error) {if (status.isConnected) {status.textContent = `读取失败：${error.message}`;status.hidden=false;}}
  finally{panel.setAttribute('aria-busy','false');}
}
