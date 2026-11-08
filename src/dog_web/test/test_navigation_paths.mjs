import {test} from 'node:test';
import assert from 'node:assert/strict';
import {navigationPathState,consumePathLease} from '../web/console/navigation_paths.js';
const map={map_id:'scene',revision:'r'};
const nav={fresh:true,task_state:'following',task_id:'task-a',map_project:'/maps/scene'};
const packet=()=>({valid:true,task_id:'task-a',frame_id:'map',map_revision:'r',map_project:'/maps/scene',sequence:2,
  global_path:[[0,0,0],[2,0,.1]],local_path:[[0,0,0],[.5,0,.02]],valid_for_ms:200});
test('execution global and local paths retain all three terrain coordinates',()=>{
  const r=navigationPathState(packet(),map,nav);
  assert(r.valid);assert.deepEqual(r.local[1],[.5,0,.02]);assert.equal(r.global.length,2);
});
test('cancel, completion and stale executor clear execution overlays',()=>{
  for(const task_state of ['cancelled','succeeded','aborted','idle'])assert(!navigationPathState(packet(),map,{...nav,task_state}).valid);
  assert(!navigationPathState(packet(),map,{...nav,fresh:false}).valid);
});
test('same revision in a different named map cannot reuse the path',()=>{
  assert(!navigationPathState(packet(),{...map,map_id:'another'},nav).valid);
  assert(!navigationPathState({...packet(),frame_id:'odom'},map,nav).valid);
});
test('expired, invalid, empty and nonfinite paths are hidden with a reason',()=>{
  for(const extra of [{valid_for_ms:0},{local_path:[]},{global_path:[[NaN,0,0]]}])assert(!navigationPathState({...packet(),...extra},map,nav).valid);
  const r=navigationPathState({valid:false,reason:'planning_budget_exceeded'},map,nav);
  assert.match(r.label,/时间预算/);
});
test('slow responses cannot extend leases or reattach a previous task path',()=>{
  assert.equal(navigationPathState(consumePathLease(packet(),150),map,nav).lease,50);
  assert(!navigationPathState(consumePathLease(packet(),250),map,nav).valid);
  assert(!navigationPathState(packet(),map,{...nav,task_id:'task-b'}).valid);
});
