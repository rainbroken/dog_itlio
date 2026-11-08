import {describe,test} from 'node:test';
import assert from 'node:assert/strict';
import {ControlTransport} from '../web/console/control_transport.js';

describe('control transport',()=>{
  function mock(){
    const sockets=[];
    class Socket {
      constructor(){this.readyState=0;this.sent=[];this.drop=false;this.reject=false;sockets.push(this);queueMicrotask(()=>{this.readyState=1;this.onopen?.();});}
      send(text){const request=JSON.parse(text);this.sent.push(request);if(this.drop)return;
        queueMicrotask(()=>this.onmessage?.({data:JSON.stringify({id:request.id,ok:!this.reject,result:{token:'lease',success:true},error:'expired'})}));}
      close(){this.readyState=3;queueMicrotask(()=>this.onclose?.());}
    }
    return {Socket,sockets};
  }
  test('reuses one persistent connection for all command replies',async()=>{
    const {Socket,sockets}=mock(),transport=new ControlTransport('ws://test/api/control/stream',{Socket});
    try{
      await transport.request('/api/control/acquire',{});
      for(let i=0;i<30;i++)await transport.request('/api/control/velocity',{token:'lease',x:.2,y:0,yaw:0});
      await transport.request('/api/control/release',{token:'lease'});
      assert.equal(sockets.length,1);assert.equal(sockets[0].sent.length,32);
    }finally{transport.disconnect();}
  });
  test('missing reply closes the stream and never retries an uncertain command',async()=>{
    const {Socket,sockets}=mock(),transport=new ControlTransport('ws://test/api/control/stream',{Socket,timeout:20});
    await transport.request('/api/control/acquire',{});sockets[0].drop=true;
    await assert.rejects(transport.request('/api/control/velocity',{token:'lease',x:.2}),/超时/);
    assert.equal(sockets[0].readyState,3);assert.equal(sockets[0].sent.length,2);
    await transport.request('/api/control/release',{token:'lease'});
    assert.equal(sockets.length,1);
  });
  test('server rejections propagate instead of claiming movement succeeded',async()=>{
    const {Socket,sockets}=mock(),transport=new ControlTransport('ws://test/api/control/stream',{Socket});
    await transport.request('/api/control/acquire',{});sockets[0].reject=true;
    await assert.rejects(transport.request('/api/control/velocity',{token:'lease',x:.2}),/expired/);
    transport.disconnect();
  });
});
