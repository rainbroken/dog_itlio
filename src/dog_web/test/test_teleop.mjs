import {test,describe} from 'node:test';
import assert from 'node:assert/strict';
import {TeleopController} from '../web/console/teleop.js';
const tick = () => new Promise(resolve => setImmediate(resolve));
const deferred = () => {let resolve; const promise = new Promise(r => resolve=r); return {promise,resolve};};
function setup(override = {}) {
  const commands=[], errors=[];
  const controller = new TeleopController({interval:100000,
    limits:()=>({x:.53,y:.2,yaw:.4}),onError:error=>errors.push(error),
    api:async(path,body)=>{commands.push({path,...body});return {token:'lease'};},...override});
  return {controller,commands,errors};
}
describe('teleop',()=>{
test('combines three axes and partial release preserves the remaining keys',async()=>{
  const {controller:c,commands}=setup();
  try {
    c.press('key:w','w');c.press('key:a','a');c.press('key:q','q');await c.pump();
    assert.deepEqual(commands.at(-1),{path:'/api/control/velocity',token:'lease',x:.53,y:.2,yaw:.4});
    c.lift('key:a');await c.pump();assert.equal(commands.at(-1).y,0);assert.equal(commands.at(-1).x,.53);
    assert.equal(commands.filter(v=>v.path.endsWith('acquire')).length,1);
    assert.equal(commands.filter(v=>v.path.endsWith('release')).length,0);
    c.press('key:s','s');await c.pump();assert.equal(commands.at(-1).x,0);
    c.lift('key:s');await c.pump();assert.equal(commands.at(-1).x,.53);
    await c.stop();assert(commands.at(-1).path.endsWith('release'));
  } finally {await c.dispose();}
});
test('releasing a pointer does not release a keyboard key for the same axis',async()=>{
  const {controller:c,commands}=setup();
  try {c.press('key:w','w');c.press('pointer:1','w');await c.pump();c.lift('pointer:1');await c.pump();assert.equal(commands.at(-1).x,.53);}
  finally {await c.dispose();}
});
test('release during delayed acquisition never sends nonzero velocity',async()=>{
  const acquired=deferred(), commands=[];
  const {controller:c}=setup({api:async(path,body)=>{commands.push({path,...body});return path.endsWith('acquire')?acquired.promise:{};}});
  try {c.press('key:w','w');await tick();const stopped=c.stop();acquired.resolve({token:'late'});await stopped;
    assert.deepEqual(commands.map(v=>v.path),['/api/control/acquire','/api/control/release']);}
  finally {await c.dispose();}
});
test('pending requests serialize and coalesce intermediate input changes',async()=>{
  const delayed=deferred(),commands=[];let count=0;
  const {controller:c}=setup({api:async(path,body)=>{commands.push({path,...body});if(path.endsWith('velocity')&&count++===0)await delayed.promise;return {token:'lease'};}});
  try {
    c.press('key:w','w');await tick();c.press('key:a','a');c.press('key:q','q');c.lift('key:a');
    delayed.resolve();await c.pump();
    const velocities=commands.filter(v=>v.path.endsWith('velocity'));
    assert.equal(velocities.length,2);assert.equal(velocities[1].y,0);assert.equal(velocities[1].yaw,.4);
  } finally {await c.dispose();}
});
test('a command failure stops renewal instead of automatically restarting movement',async()=>{
  const commands=[];
  const {controller:c,errors}=setup({api:async(path,body)=>{commands.push({path,...body});if(path.endsWith('velocity'))throw Error('timeout');return {token:'lease'};}});
  try {c.press('key:w','w');await c.pump();assert.equal(errors.length,1);assert.equal(c.inputs.size,0);assert.equal(c.token,null);
    await c.pump();assert.equal(commands.filter(v=>v.path.endsWith('acquire')).length,1);}
  finally {await c.dispose();}
});
test('50 ms heartbeat renews held input without concurrent HTTP requests',async()=>{
  let outstanding=0, maximum=0, velocities=0;
  const {controller:c}=setup({interval:50,api:async(path)=>{
    maximum=Math.max(maximum,++outstanding);await new Promise(r=>setTimeout(r,10));outstanding--;
    if(path.endsWith('velocity'))velocities++;return {token:'lease'};
  }});
  try {c.press('key:w','w');await new Promise(r=>setTimeout(r,180));await c.stop();assert(velocities>=3);assert.equal(maximum,1);}
  finally {await c.dispose();}
});
});
