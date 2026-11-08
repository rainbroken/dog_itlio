// Packed preview only; the server retains the complete collision/planning map.
export function decodeMapPreview(metadata,buffer){
  const {cloud_count:cloudCount,surface_count:surfaceCount}=metadata;
  if(metadata.encoding!=='xyz-surface-f32le-v1'||!Number.isInteger(cloudCount)||cloudCount<0||cloudCount>150000||
    !Number.isInteger(surfaceCount)||surfaceCount<0||surfaceCount>100000||buffer.byteLength!==(cloudCount*3+surfaceCount*4)*4)
    throw new Error('地图预览数据不完整，请重新加载');
  const view=new DataView(buffer);let offset=0;
  const scalar=()=>{
    const value=view.getFloat32(offset,true);offset+=4;
    if(!Number.isFinite(value))throw new Error('地图预览坐标无效');return value;
  };
  const cloudPositions=new Float32Array(cloudCount*3);
  for(let i=0;i<cloudPositions.length;i++)cloudPositions[i]=scalar();
  const surfaces=Array.from({length:surfaceCount},()=>Array.from({length:4},scalar));
  return {...metadata,cloudPositions,surfaces};
}
export async function loadMapPreview(){
  const response=await fetch('/api/map?encoding=float32',{cache:'no-store',signal:AbortSignal.timeout(120000)});
  if(!response.ok){const body=await response.json();throw new Error(body.error||'地图预览加载失败');}
  return decodeMapPreview(JSON.parse(response.headers.get('X-Map-Metadata')),await response.arrayBuffer());
}
