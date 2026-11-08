// Persistent, dedicated command connection. Map fetches cannot occupy its
// browser HTTP connection slot; commands are never retried after a lost reply.
export class ControlTransport {
  constructor(url,{Socket=WebSocket,timeout=500,connectTimeout=1500}={}) {
    Object.assign(this,{url,Socket,timeout,connectTimeout});this.socket=null;this.opening=null;this.pending=new Map();this.serial=0;this.rtt=0;
  }
  disconnect(error=new Error('控制连接已关闭')) {
    const socket=this.socket;this.socket=null;
    for(const request of this.pending.values()){clearTimeout(request.timer);request.reject(error);}this.pending.clear();
    if(socket)socket.close();
  }
  connect() {
    if(this.socket?.readyState===1)return Promise.resolve();
    if(this.opening)return this.opening;
    const socket=new this.Socket(this.url);this.socket=socket;
    this.opening=new Promise((resolve,reject)=>{
      const timer=setTimeout(()=>{reject(new Error('控制长连接建立超时'));this.disconnect();},this.connectTimeout);
      socket.onopen=()=>{clearTimeout(timer);resolve();};
      const failed=()=>{
        clearTimeout(timer);const error=new Error('控制长连接中断，请重新按键');reject(error);
        if(this.socket===socket)this.disconnect(error);
      };
      socket.onerror=failed;socket.onclose=failed;
      socket.onmessage=event=>{
        if(this.socket!==socket)return;
        let message;try{message=JSON.parse(event.data);}catch{this.disconnect(new Error('控制回执格式错误'));return;}
        const request=this.pending.get(message.id);if(!request)return;
        this.pending.delete(message.id);clearTimeout(request.timer);this.rtt=performance.now()-request.sent;
        if(message.ok)request.resolve(message.result);else request.reject(new Error(message.error||'控制指令被拒绝'));
      };
    }).finally(()=>{this.opening=null;});
    return this.opening;
  }
  async request(path,body) {
    if(path==='/api/control/release'&&this.socket?.readyState!==1)return {success:true};
    await this.connect();
    return new Promise((resolve,reject)=>{
      const id=++this.serial;
      const timer=setTimeout(()=>this.disconnect(new Error('控制回执超时，已关闭连接并停止续租')),this.timeout);
      this.pending.set(id,{resolve,reject,timer,sent:performance.now()});
      try{this.socket.send(JSON.stringify({id,path,body}));}
      catch(error){this.disconnect(error);}
    });
  }
}
