import {combinedDirection} from './teleop.js';

// DOM input and UI lease heartbeat live here. Repeated HTTP transmission lives
// in a worker so a short WebGL rendering stall cannot queue movement commands.
export class TeleopClient {
  constructor({limits,onError,onChange}) {
    Object.assign(this,{limits,onError,onChange}); this.inputs=new Map(); this.token=null;
    this.waiters=new Map(); this.serial=0; this.epoch=0; this.failed=false;
    this.worker=new Worker(new URL('./teleop_worker.js',import.meta.url),{type:'module'});
    this.worker.onmessage=({data})=>{
      if(data.type==='lease')this.token=data.token;
      if(data.type==='timing')this.metrics=data;
      if(data.type==='stopped'){this.waiters.get(data.id)?.();this.waiters.delete(data.id);}
      if(data.type==='error'){
        if(data.epoch===this.epoch){this.inputs.clear();this.onChange(combinedDirection(this.inputs));}
        this.onError(new Error(data.message));
      }
    };
    this.worker.onerror=()=>{
      this.failed=true;this.inputs.clear();this.onChange(combinedDirection(this.inputs));
      this.onError(new Error('控制发送线程异常，请刷新页面；后端将超时停止'));
      for(const resolve of this.waiters.values())resolve();this.waiters.clear();
    };
    this.timer=setInterval(()=>{if(this.inputs.size)this.send();},50);
  }
  send(){if(!this.failed)this.worker.postMessage({type:'input',epoch:this.epoch,entries:[...this.inputs],limits:this.limits()});}
  press(source,key){
    if(this.failed||this.inputs.get(source)===key)return;
    if(!this.inputs.size)this.epoch++;
    this.inputs.set(source,key);this.onChange(combinedDirection(this.inputs));this.send();
  }
  lift(source){if(this.inputs.delete(source)){this.onChange(combinedDirection(this.inputs));this.send();}}
  stop(){
    this.inputs.clear();this.onChange(combinedDirection(this.inputs));
    if(this.failed)return Promise.resolve();
    const id=++this.serial;
    return new Promise(resolve=>{this.waiters.set(id,resolve);this.worker.postMessage({type:'stop',id});});
  }
}
