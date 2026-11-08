export function proposedMapId(filename) {
  return filename.replace(/\.pcd$/i, '').replace(/[^a-zA-Z0-9_-]/g, '_')
    .replace(/^[_-]+/, '').slice(0, 64) || 'map';
}

export function assertImportedMap(actual, expected) {
  if (expected && (actual.map_id !== expected.map_id || actual.revision !== expected.revision)) {
    throw new Error('当前地图已被另一请求切换，请重新加载所需地图。未显示其他地图代替本次结果。');
  }
}

export function sourceDescription(map) {
  const source = map.source || {};
  const name = source.filename || '历史地图';
  const fingerprint = source.upload_sha256 || source.cloud_sha256;
  return `${name} · ${Number(map.points).toLocaleString()} 点${fingerprint ? ` · SHA256 ${fingerprint.slice(0, 12)}` : ''}`;
}
