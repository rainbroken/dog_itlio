import {test} from 'node:test';
import assert from 'node:assert/strict';
import {classifyWallPoints} from '../web/console/wall_geometry.js';

function plane(transform){
  const points=[];
  for(let i=-20;i<=20;i++)for(let j=-20;j<=20;j++)points.push(...transform(i*.05,j*.05));
  return new Float32Array(points);
}
const fraction=flags=>flags.reduce((n,v)=>n+v,0)/flags.length;
test('vertical planes remain visible at different headings and map translations',()=>{
  for(const yaw of [0,.43,Math.PI/2]){
    const p=plane((u,z)=>[20+u*Math.cos(yaw),-30+u*Math.sin(yaw),z-4]);
    assert(fraction(classifyWallPoints(p))>.85);
  }
});
test('floors, gentle slopes, separated stacked floors and thin poles are not walls',()=>{
  for(const slope of [0,.2,.5])assert.equal(fraction(classifyWallPoints(plane((x,y)=>[x,y,x*slope]))),0);
  const lower=plane((x,y)=>[x,y,0]),upper=plane((x,y)=>[x,y,2]);
  assert.equal(fraction(classifyWallPoints(new Float32Array([...lower,...upper]))),0);
  const pole=[];for(let z=0;z<3;z+=.02)pole.push(0,0,z);
  assert.equal(fraction(classifyWallPoints(new Float32Array(pole))),0);
  assert.equal(classifyWallPoints(new Float32Array()).length,0);
});
test('floor-wall junction retains high wall points without coloring the floor interior',()=>{
  const wall=plane((u,z)=>[u,0,z+1.5]),floor=plane((x,y)=>[x,y,0]);
  const flags=classifyWallPoints(new Float32Array([...wall,...floor]));
  assert(fraction(flags.slice(0,wall.length/3))>.85);
  for(let i=0;i<floor.length/3;i++)if(Math.abs(floor[i*3+1])>.5)assert.equal(flags[wall.length/3+i],0);
});
