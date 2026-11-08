import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../web/console/preview_policy.js',import.meta.url),'utf8');
const {PreviewPolicy}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
test('slow downloads and failures reduce transmitted point count with a bounded floor',()=>{
 const p=new PreviewPolicy('map');assert.equal(p.options(false).limit,512);
 p.completed(1000);assert.equal(p.options(false).limit,256);
 for(let i=0;i<20;i++)p.failed();assert(p.options(true).limit>=32);
 for(let i=0;i<100;i++)p.completed(100);assert.equal(p.options(false).limit,512);
});
test('motion reduces preview traffic and byte budget delays large packets',()=>{
 for(const kind of ['map','scan']){const p=new PreviewPolicy(kind),idle=p.options(false),moving=p.options(true);
 assert(moving.limit<idle.limit);assert(moving.bytesPerSecond<idle.bytesPerSecond);
 const delay=p.delay(moving,moving.limit*16,100);
 assert(delay>=moving.period);assert((delay+100)*moving.bytesPerSecond/1000>=moving.limit*16);
 }
});
