import {test} from 'node:test';
import assert from 'node:assert/strict';
import {decodeMapPreview} from '../web/console/map_preview.js';
const metadata={encoding:'xyz-surface-f32le-v1',cloud_count:1,surface_count:1,map_id:'a',revision:'r'};
function bytes(values){const buffer=new ArrayBuffer(values.length*4),view=new DataView(buffer);values.forEach((v,i)=>view.setFloat32(i*4,v,true));return buffer;}
test('packed preview preserves XYZ, component and map identity',()=>{
 const map=decodeMapPreview(metadata,bytes([1,2,-3,4,5,6,7]));
 assert.deepEqual([...map.cloudPositions],[1,2,-3]);assert.deepEqual(map.surfaces,[[4,5,6,7]]);assert.equal(map.revision,'r');
});
test('truncated, oversized and nonfinite geometry is rejected',()=>{
 assert.throws(()=>decodeMapPreview(metadata,bytes([1,2])));
 assert.throws(()=>decodeMapPreview({...metadata,cloud_count:150001},new ArrayBuffer(0)));
 assert.throws(()=>decodeMapPreview(metadata,bytes([NaN,2,3,4,5,6,7])));
});
