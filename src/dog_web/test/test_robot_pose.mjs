import {test} from 'node:test';
import assert from 'node:assert/strict';
import {robotPlacement} from '../web/console/robot_pose.js';

const project={map_id:'saved-map'};
function telemetry() {return {tf_ready:true,pose:{position:[3,-2,.65],quaternion:[0,0,0,1],frame:'map',age_ms:30},
  localization:{map_path:'/maps/saved-map/cloud.pcd',fresh:true,valid:true}};}

test('mapping uses live base TF even while a different saved project remains loaded',()=>{
  const data=telemetry();data.localization={map_path:'/maps/other/cloud.pcd',fresh:false,valid:false};
  const result=robotPlacement(data,project,{page:'mapping'});
  assert.equal(result.pose,data.pose);assert.equal(result.mode,'live');
});
test('navigation requires a loaded project and a matching confirmed localization',()=>{
  const data=telemetry();
  assert.equal(robotPlacement(data,project,{page:'navigation'}).mode,'map');
  assert.equal(robotPlacement(data,null,{page:'navigation'}).pose,undefined);
  for(const patch of [{fresh:false},{valid:false},{map_path:'/maps/other/cloud.pcd'}]){
    assert.equal(robotPlacement({...data,localization:{...data.localization,...patch}},project,{page:'navigation'}).pose,undefined);
  }
});
test('stale TF, invalid numbers, wrong frames and missing TF never produce a current position',()=>{
  for(const page of ['mapping','navigation']){
    const data=telemetry();
    for(const patch of [{age_ms:1000},{age_ms:-1000},{age_ms:undefined},{position:[NaN,0,0]},
      {position:[1,2]},{quaternion:[0,0,0,0]},{quaternion:[Infinity,0,0,1]},{frame:'odom'}]){
      assert.equal(robotPlacement({...data,pose:{...data.pose,...patch}},project,{page}).pose,undefined);
    }
    assert.equal(robotPlacement({...data,tf_ready:false},project,{page}).pose,undefined);
    assert.equal(robotPlacement({tf_ready:false},project,{page}).pose,undefined);
  }
});
test('mapping cloud frame must match the robot TF frame',()=>{
  assert.equal(robotPlacement(telemetry(),project,{page:'mapping',frame:'odom'}).pose,undefined);
});
