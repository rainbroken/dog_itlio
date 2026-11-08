import * as THREE from 'three';
import {pclPalette} from './pcl_lut.js';
import {PreviewPolicy} from './preview_policy.js';
import {startMapStream} from './map_stream.js';

const palette=new THREE.DataTexture(pclPalette,256,1,THREE.RGBAFormat);
palette.minFilter=palette.magFilter=THREE.NearestFilter;palette.needsUpdate=true;
function makeCloud(clip,size=2){
  const material=new THREE.ShaderMaterial({uniforms:{palette:{value:palette},range:{value:new THREE.Vector2(0,1)},clip,size:{value:size}},
    vertexShader:'attribute float scalar; varying float value; varying float z; uniform float size; void main(){value=scalar;vec4 world=modelMatrix*vec4(position,1.);z=world.z;gl_Position=projectionMatrix*viewMatrix*world;gl_PointSize=size;}',
    fragmentShader:'uniform sampler2D palette;uniform vec2 range;uniform float clip;varying float value;varying float z;void main(){if(z>clip)discard;float t=clamp((value-range.x)/max(1e-9,range.y-range.x),0.,1.);gl_FragColor=texture2D(palette,vec2(t,.5));}'});
  return new THREE.Points(new THREE.BufferGeometry(),material);
}
function update(mesh,data,meta){
  const buffer=new THREE.InterleavedBuffer(data,4),geometry=new THREE.BufferGeometry();
  geometry.setAttribute('position',new THREE.InterleavedBufferAttribute(buffer,3,0));
  geometry.setAttribute('scalar',new THREE.InterleavedBufferAttribute(buffer,1,3));
  geometry.computeBoundingBox();mesh.geometry.dispose();mesh.geometry=geometry;
  mesh.material.uniforms.range.value.fromArray(meta.range);
}
export function mountLiveClouds({groups,clip,getState,fit,text,getFocus}){
  const accumulated=new THREE.Group(),overlay=makeCloud(clip,3),tileMeshes=new Map();
  groups.live.add(accumulated);groups.scan.add(overlay);
  const card=document.createElement('section');card.className='card live-scan-card';card.id='liveScanCard';
  card.innerHTML='<h2>当前雷达帧</h2><canvas aria-label="雷达当前采集点云"></canvas><p class="settings-note" id="scanStatus">等待 /interfaces/lidar</p><p class="settings-note">独立雷达坐标预览；有效定位后叠加到主场景。反射强度伪彩色。</p>';
  document.querySelector('.inspector').append(card);
  const canvas=card.querySelector('canvas'),renderer=new THREE.WebGLRenderer({canvas,antialias:false});
  renderer.setPixelRatio(1);renderer.setClearColor('#101924');
  const localScene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(50,1,.1,200);
  camera.up.set(0,0,1);const current=makeCloud({value:1e9});localScene.add(current);
  let mapMeta=null,scanMeta=null,first=true;
  function placement(mesh,meta){
    if(!meta?.transform){mesh.position.set(0,0,0);mesh.quaternion.identity();return false;}
    mesh.position.fromArray(meta.transform.position);mesh.quaternion.fromArray(meta.transform.quaternion);return true;
  }
  function visibility(){
    const state=getState(),loc=state.robot?.localization;
    const sameMap=loc?.map_path?.replace(/\/(?:cloud|localization)\.pcd$/,'').split('/').at(-1)===state.map?.map_id;
    const mapping=state.page==='mapping';
    const aligned=mapping||(state.map&&sameMap&&loc?.valid&&loc.fresh);
    const enabled=document.querySelector('[data-layer="cloud"]').checked;
    groups.live.visible=mapping&&enabled&&!!mapMeta;
    groups.scan.visible=enabled&&aligned&&!!scanMeta?.fresh&&performance.now()-scanMeta.updated<1500&&placement(overlay,scanMeta)&&document.querySelector('[data-layer="scan"]').checked;
    card.dataset.overlay=groups.scan.visible?'map':'sensor';
  }
  const mapStream=startMapStream({active:()=>getState().page==='mapping',driving:()=>getState().driving,
    focus(){accumulated.updateMatrixWorld(true);return accumulated.worldToLocal(getFocus().clone()).toArray();},
    remove(id){const mesh=tileMeshes.get(id);if(mesh){accumulated.remove(mesh);mesh.geometry.dispose();mesh.material.dispose();tileMeshes.delete(id);}},
    complete(id,data){let mesh=tileMeshes.get(id);if(!mesh){mesh=makeCloud(clip);tileMeshes.set(id,mesh);accumulated.add(mesh);}update(mesh,data,mapMeta);},
    metadata(meta){mapMeta=meta;placement(accumulated,meta);for(const mesh of tileMeshes.values())mesh.material.uniforms.range.value.fromArray(meta.range);visibility();},
    reset(){first=true;},
    status(meta){
      if(getState().page==='mapping'&&meta.waiting)text('.scene-caption strong','等待累计建图点云');
      if(getState().page==='mapping'&&meta.count!==undefined){
        // Fit once a useful initial batch has arrived, rather than the first tiny tile.
        if(first&&meta.loaded>=Math.min(meta.count,5000)&&meta.loaded){groups.live.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(accumulated);fit([box.min.toArray(),box.max.toArray()]);first=false;}
        text('.scene-caption strong',`实时累计建图 · 已同步 ${meta.loaded} / ${meta.count} 点${meta.loaded===meta.count?' · 同步完成':' · 正在补齐'}${meta.error?' · 传输重试中':''}`);
        text('.scene-caption small',`${meta.transform?'map':meta.frame} · Z ↑ · m · ${meta.field==='intensity'?'反射强度':'高度'}伪彩色`);
      }
    }});
  async function pollScan(){
    let sequence=-1;const policy=new PreviewPolicy('scan');
    for(;;){
      const options=policy.options(getState().driving),started=performance.now();let bytes=0;
      try{
        if(!document.hidden){
          const response=await fetch(`/api/cloud/scan?since=${sequence}&limit=${options.limit}`,{cache:'no-store',signal:AbortSignal.timeout(2500)});
          if(response.status!==204){
            if(response.status!==200&&response.status!==304)throw Error(`HTTP ${response.status}`);
            const meta=JSON.parse(response.headers.get('X-Cloud-Metadata'));
            if(!meta||!Number.isFinite(meta.count))throw Error('Invalid cloud metadata');
            meta.updated=performance.now();
            scanMeta=meta;
            if(response.status===200){
              const data=new Float32Array(await response.arrayBuffer());
              if(data.length!==meta.count*4)throw Error('Truncated cloud packet');
              bytes=data.byteLength;policy.completed(performance.now()-started);
              sequence=meta.cache_key||meta.sequence;
                update(overlay,data,meta);update(current,data,meta);
                if(meta.count){
                const box=current.geometry.boundingBox,center=box.getCenter(new THREE.Vector3());
                const radius=Math.min(60,Math.max(2,box.getSize(new THREE.Vector3()).length()*.65));
                camera.position.copy(center).add(new THREE.Vector3(.8,-1,.7).multiplyScalar(radius));camera.lookAt(center);
                }
            }
              text('#scanStatus',`${meta.fresh?'实时':'已过期'} · ${meta.frame} · ${meta.count} 点 · #${meta.sequence}`);
              card.dataset.sequence=String(meta.sequence);card.dataset.fresh=String(meta.fresh);
              if(card.getClientRects().length){const width=Math.max(100,canvas.clientWidth);camera.aspect=width/150;camera.updateProjectionMatrix();renderer.setSize(width,150,false);renderer.render(localScene,camera);}
            visibility();
          }else {scanMeta=null;text('#scanStatus','等待 /interfaces/lidar');groups.scan.visible=false;}
        }
      }catch(error){policy.failed();scanMeta=null;text('#scanStatus',`雷达预览离线：${error.message}`);groups.scan.visible=false;}
      await new Promise(resolve=>setTimeout(resolve,policy.delay(options,bytes,performance.now()-started)));
    }
  }
  return {start(){pollScan();},visibility,
    frame(){return mapMeta?.transform?'map':mapMeta?.frame || 'map';},
    bounds(){accumulated.updateMatrixWorld(true);const box=new THREE.Box3().setFromObject(accumulated);return box.isEmpty()?null:[box.min.toArray(),box.max.toArray()];},
    reset(){mapStream.clear();first=true;mapMeta=null;groups.live.visible=false;}};
}
