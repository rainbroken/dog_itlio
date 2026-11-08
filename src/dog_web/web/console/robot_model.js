import * as THREE from 'three';
import URDFLoader from '/vendor/URDFLoader.js';
import {OBJLoader} from '/vendor/loaders/OBJLoader.js';
import {MTLLoader} from '/vendor/loaders/MTLLoader.js';
import {robotPlacement} from './robot_pose.js';

// URDF owns link transforms and joint axes. The driver supplies named joint
// angles; only a valid map pose may place the model in a loaded navigation map.
export function createRobotModel({scene, onReady}) {
  const root=new THREE.Group();scene.add(root);
  const heading=new THREE.ArrowHelper(new THREE.Vector3(1,0,0),new THREE.Vector3(0,0,.2),.55,'#ffbd69',.16,.09);
  heading.line.material.depthTest=heading.cone.material.depthTest=false;
  heading.renderOrder=30;root.add(heading);heading.visible=false;
  const status=document.createElement('span');status.className='robot-model-status';
  status.setAttribute('role','status');status.textContent='正在加载机器狗模型';
  document.querySelector('.scene').append(status);
  const manager=new THREE.LoadingManager(),loader=new URDFLoader(manager);
  const cache=new Map(),pending=new Map();let model=null,loaded=false,failed=false,seenJoints=false,lastData=null,lastMap=null,lastOptions={};
  loader.packages={dog_description:'/robot'};
  loader.loadMeshCb=(path,loadingManager,done)=>{
    if(cache.has(path)){done(cache.get(path).clone(true));return;}
    if(pending.has(path)){pending.get(path).push(done);return;}
    pending.set(path,[done]);
    const finish=object=>{
      if(object)cache.set(path,object);else failed=true;
      const callbacks=pending.get(path)||[];pending.delete(path);
      callbacks.forEach(callback=>callback(object?object.clone(true):null));
    };
    new MTLLoader(loadingManager).load(path.replace(/\.obj$/i,'.mtl'),materials=>{
      materials.preload();new OBJLoader(loadingManager).setMaterials(materials).load(path,finish,undefined,()=>finish(null));
    },undefined,()=>new OBJLoader(loadingManager).load(path,finish,undefined,()=>finish(null)));
  };
  manager.onLoad=()=>{
    loaded=!!model&&!failed;status.dataset.loaded=String(loaded);
    if(loaded){controller.update(lastData,lastMap,lastOptions);onReady(new THREE.Box3().setFromObject(root));}
  };
  loader.load('/robot/urdf/bpx.urdf',value=>{model=value;root.add(model);},undefined,()=>{failed=true;});
  const quaternion=value=>Array.isArray(value)&&value.length===4&&value.every(Number.isFinite)&&
    Math.abs(new THREE.Quaternion(...value).length()-1)<.01;
  const controller={root,update(data,map,options={}){
    lastData=data;lastMap=map;lastOptions=options;
    const joints=data?.joints;
    if(model&&Array.isArray(joints?.names)&&Array.isArray(joints.positions)){
      joints.names.forEach((name,index)=>{const value=joints.positions[index];
        if(model.joints[name]&&Number.isFinite(value)){model.setJointValue(name,value);seenJoints=true;}});
    }
    const placement=robotPlacement(data,map,options),pose=placement.pose;
    const positioned=!!pose;
    heading.visible=positioned;
    const preview=!map&&options.page!=='mapping'&&options.page!=='navigation';
    root.visible=positioned||preview;
    if(positioned){root.position.fromArray(pose.position);root.quaternion.fromArray(pose.quaternion).normalize();}
    else if(root.visible){
      root.position.set(0,0,0);root.quaternion.fromArray(quaternion(data?.body_orientation)?data.body_orientation:[0,0,0,1]).normalize();
      if(model){const bounds=new THREE.Box3().setFromObject(root);if(!bounds.isEmpty())root.position.z=-bounds.min.z;}
    }
    const mode=positioned?placement.mode:preview?'preview':'unlocalized';
    root.userData.mode=mode;root.userData.positioned=positioned;status.dataset.mode=mode;
    const yaw=positioned?new THREE.Euler().setFromQuaternion(root.quaternion,'ZYX').z*180/Math.PI:null;
    const readout=positioned?`${mode==='map'?'地图定位':options.page==='mapping'?'实时建图位姿':'实时 TF 位姿'} · ${pose.frame} · XYZ ${pose.position.map(v=>v.toFixed(2)).join(', ')} m · 朝向 ${yaw.toFixed(1)}°`:
      preview?'本体预览，未绑定地图':placement.reason;
    root.userData.readout=readout;
    const jointText=joints?'实时关节':seenJoints?'关节已过期，保留末帧':'参考关节姿态';
    status.textContent=`${readout} · ${failed?'模型加载失败':!loaded?'模型加载中':jointText}`;
  }};
  return controller;
}
