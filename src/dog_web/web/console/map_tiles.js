// A block is replaced only after every slice of its advertised version arrives.
// Reconciliation handles removals and publisher/session changes explicitly.
export class MapTileCache {
  constructor({complete,remove}){this.complete=complete;this.remove=remove;this.tiles=new Map();this.epoch=null;this.revision='';this.cursor=0;}
  clear(){for(const id of this.tiles.keys())this.remove(id);this.tiles.clear();this.epoch=null;this.revision='';this.cursor=0;}
  reconcile(manifest){
    if(typeof manifest.epoch!=='string'||!Array.isArray(manifest.tiles))throw Error('Invalid map manifest');
    const ids=new Set();let total=0;
    for(const t of manifest.tiles){
      if(typeof t.id!=='string'||typeof t.version!=='string'||!Number.isInteger(t.count)||t.count<1||t.count>512||ids.has(t.id))throw Error('Invalid map tile');
      ids.add(t.id);total+=t.count;
    }
    if(total!==manifest.count||total>1500000)throw Error('Invalid map point total');
    if(this.epoch!==manifest.epoch)this.clear();
    this.epoch=manifest.epoch;this.revision=manifest.revision;
    for(const id of this.tiles.keys())if(!ids.has(id)){this.remove(id);this.tiles.delete(id);}
    for(const t of manifest.tiles){
      const previous=this.tiles.get(t.id);
      if(previous?.version===t.version&&previous.count===t.count)continue;
      this.tiles.set(t.id,{...t,offset:0,data:new Float32Array(t.count*4),done:false});
    }
  }
  next(focus){
    const pending=[...this.tiles.values()].filter(t=>!t.done);
    if(!pending.length)return null;
    // Prefer the viewed region; reserve every fourth selection for the rest
    // so a stream of nearby changes cannot permanently starve distant blocks.
    if(focus){const distance=t=>t.center.reduce((n,v,i)=>n+(v-focus[i])**2,0);pending.sort((a,b)=>distance(a)-distance(b));}
    const turn=this.cursor++;
    return pending[turn%4===3?Math.floor(turn/4)%pending.length:0];
  }
  accept(request,meta,data){
    const t=this.tiles.get(request.id);
    if(!t||this.epoch!==meta.epoch||t.version!==meta.version||request.version!==meta.version)return false;
    if(meta.id!==t.id||meta.offset!==t.offset||meta.total!==t.count||!Number.isInteger(meta.count)||meta.count<1||meta.count+t.offset>t.count||data.length!==meta.count*4)throw Error('Invalid map tile slice');
    t.data.set(data,t.offset*4);t.offset+=meta.count;
    if(t.offset===t.count){t.done=true;this.complete(t.id,t.data);}
    return true;
  }
  get loaded(){let n=0;for(const t of this.tiles.values())if(t.done)n+=t.count;return n;}
}
