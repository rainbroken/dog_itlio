import {MapTileCache} from './map_tiles.js';
import {PreviewPolicy} from './preview_policy.js';

export function startMapStream({active,driving,complete,remove,metadata,status,reset,focus}){
  const cache=new MapTileCache({complete,remove}),policy=new PreviewPolicy('map');
  let meta=null,lastManifest=0,serial=0;
  const sleep=ms=>new Promise(r=>setTimeout(r,ms));
  const renderStatus=()=>{if(meta)status({...meta,loaded:cache.loaded});};
  async function run(){for(;;){
    if(document.hidden||!active()){await sleep(300);continue;}
    const options=policy.options(driving()),start=performance.now(),epoch=serial;let bytes=0,waiting=false;
    try{
      if(!lastManifest||start-lastManifest>2000){
        const r=await fetch('/api/cloud/manifest?since='+encodeURIComponent(cache.revision),{cache:'no-store',signal:AbortSignal.timeout(4000)});
        if(epoch!==serial)continue;
        if(r.status===204){cache.clear();meta=null;reset();status({waiting:true});await sleep(500);continue;}
        if(r.status===304)meta=JSON.parse(r.headers.get('X-Cloud-Metadata'));
        else{
          if(!r.ok)throw Error(`地图索引 HTTP ${r.status}`);
          const m=await r.json();if(epoch!==serial)continue;
          if(cache.epoch!==m.epoch)reset();cache.reconcile(m);meta=m;
        }
        lastManifest=performance.now();metadata(meta);renderStatus();
      }
      const tile=cache.next(focus?.());
      if(tile){
        const offset=tile.offset;
        const query=new URLSearchParams({epoch:cache.epoch,id:tile.id,version:tile.version,offset:String(offset),limit:String(options.limit)});
        const r=await fetch('/api/cloud/tile?'+query,{cache:'no-store',signal:AbortSignal.timeout(4000)});
        if(epoch!==serial)continue;
        if(r.status===409){lastManifest=0;await sleep(100);continue;}
        if(!r.ok)throw Error(`地图分块 HTTP ${r.status}`);
        const info=JSON.parse(r.headers.get('X-Cloud-Tile')),data=new Float32Array(await r.arrayBuffer());
        if(epoch!==serial)continue;
        bytes=data.byteLength;policy.completed(performance.now()-start);cache.accept(tile,info,data);renderStatus();
      }else waiting=true;
    }catch(error){policy.failed();status(meta?{...meta,loaded:cache.loaded,error:error.message}:{error:error.message});waiting=true;}
    await sleep(waiting?500:policy.delay(options,bytes,performance.now()-start));
  }}
  run();
  return {clear(){serial++;cache.clear();meta=null;lastManifest=0;}};
}
