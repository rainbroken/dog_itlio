import {test} from 'node:test';
import assert from 'node:assert/strict';
import {MapTileCache} from '../web/console/map_tiles.js';
const descriptor=(id,version,count=4)=>({id,version,count});
const manifest=(tiles,epoch='session')=>({tiles,epoch,revision:'revision',count:tiles.reduce((n,t)=>n+t.count,0)});
function setup(){const complete=[],remove=[];const cache=new MapTileCache({complete:(id,data)=>complete.push([id,[...data]]),remove:id=>remove.push(id)});return {cache,complete,remove};}
function accept(cache,t,count,offset=t.offset){return cache.accept(t,{epoch:cache.epoch,id:t.id,version:t.version,total:t.count,offset,count},new Float32Array(count*4).fill(offset));}
test('slices complete every point without dropping the map total; unchanged blocks are reused',()=>{
 const {cache,complete}=setup();cache.reconcile(manifest([descriptor('a','v1')]));const t=cache.next();
 accept(cache,t,2);assert.equal(cache.loaded,0);assert.equal(complete.length,0);
 cache.reconcile(manifest([descriptor('a','v1')]));assert.equal(cache.next().offset,2);
 accept(cache,t,2);assert.equal(cache.loaded,4);assert.equal(complete.length,1);assert.equal(cache.next(),null);
 cache.reconcile(manifest([descriptor('a','v1'),descriptor('b','v2')]));assert.equal(cache.next().id,'b');assert.equal(cache.loaded,4);
});
test('changed versions reject old data and removal or session reset clears old blocks',()=>{
 const {cache,complete,remove}=setup();cache.reconcile(manifest([descriptor('a','v1')]));const old=cache.next();accept(cache,old,2);
 cache.reconcile(manifest([descriptor('a','v2')]));assert.equal(accept(cache,old,2),false);assert.equal(cache.next().offset,0);
 accept(cache,cache.next(),4);assert.equal(complete.length,1);
 cache.reconcile(manifest([]));assert.deepEqual(remove,['a']);assert.equal(cache.loaded,0);
 cache.reconcile(manifest([descriptor('b','v1')]));cache.reconcile(manifest([descriptor('a','v1')],'new-session'));assert(remove.includes('b'));assert.equal(cache.next().id,'a');
});
test('malformed or duplicate slices never become visible',()=>{
 const {cache,complete}=setup();cache.reconcile(manifest([descriptor('a','v1')]));const t=cache.next();accept(cache,t,2);
 assert.throws(()=>accept(cache,t,2,0));assert.equal(complete.length,0);
 assert.throws(()=>cache.reconcile({...manifest([descriptor('a','v1')]),count:3}));
});
test('viewed blocks are prioritized while distant blocks retain a turn',()=>{
 const {cache}=setup();cache.reconcile(manifest([{...descriptor('far','1'),center:[100,0,0]},{...descriptor('near','1'),center:[0,0,0]}]));
 assert.equal(cache.next([0,0,0]).id,'near');
 let far=false;for(let i=0;i<12;i++)if(cache.next([0,0,0]).id==='far')far=true;
 assert(far);
});
