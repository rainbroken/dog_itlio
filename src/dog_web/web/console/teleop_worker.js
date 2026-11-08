import {TeleopController} from './teleop.js';
import {ControlTransport} from './control_transport.js';

let limits, lastInput = 0, epoch = 0, blockedEpoch = -1,lastReport=0,maxUiGap=0;
const endpoint=new URL('/api/control/stream',self.location.href);endpoint.protocol=endpoint.protocol==='https:'?'wss:':'ws:';
const transport=new ControlTransport(endpoint.href);
function failed(message) {
  blockedEpoch = Math.max(blockedEpoch,epoch);
  self.postMessage({type:'error',message,epoch});
}
const controller = new TeleopController({
  limits: () => limits,
  onLease: token => self.postMessage({type:'lease',token}),
  onError: error => failed(error.message),
  api: async(path,body)=>{
    const result=await transport.request(path,body);
    if(performance.now()-lastReport>500){
      lastReport=performance.now();self.postMessage({type:'timing',roundTripMs:transport.rtt,uiGapMs:maxUiGap});maxUiGap=0;
    }
    return result;
  }
});
setInterval(() => {
  if (controller.inputs.size && performance.now() - lastInput > 250) {
    controller.stop();
    failed('页面输入心跳超时，已停止网页控制，请重新按键');
  }
},25);
self.onmessage = async ({data}) => {
  if (data.type === 'input') {
    if(data.epoch<=blockedEpoch)return;
    if(epoch===data.epoch&&lastInput)maxUiGap=Math.max(maxUiGap,performance.now()-lastInput);
    else maxUiGap=0;
    epoch=data.epoch;
    limits = data.limits; lastInput = performance.now(); controller.setInputs(data.entries);
  } else if (data.type === 'stop') {
    await controller.stop(); self.postMessage({type:'stopped',id:data.id});
  }
};
