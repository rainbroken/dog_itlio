const gib=bytes=>(bytes/1073741824).toFixed(2)+' GiB';
export function mountResources(root){
  root.innerHTML='<section class="resources-page"><header><span class="eyebrow">HOST RESOURCES</span><h1>资源监控</h1><p>运行 Web 服务的主机 · 每秒刷新</p></header><p id="resourceStatus" role="status">正在读取资源占用…</p><div class="resource-summary"></div><h2>CPU 各核心</h2><div class="resource-cores"></div></section>';
  const page=root.firstElementChild,status=page.querySelector('#resourceStatus');
  const alive=()=>page.isConnected&&root.contains(page);
  function gauge(parent,id,label){
    const card=document.createElement('section');card.className='resource-gauge';card.dataset.resource=id;
    const heading=document.createElement('div'),title=document.createElement('span'),percent=document.createElement('output');
    title.textContent=label;percent.textContent='—';heading.append(title,percent);
    const bar=document.createElement('progress');bar.max=100;bar.setAttribute('aria-label',label+'占用率');
    const detail=document.createElement('small');card.append(heading,bar,detail);parent.append(card);
    return (value,text='')=>{
      if(Number.isFinite(value)){value=Math.max(0,Math.min(100,value));bar.value=value;percent.textContent=value.toFixed(1)+'%';}
      else{bar.removeAttribute('value');percent.textContent='采样中';}
      detail.textContent=text;
    };
  }
  const summary=page.querySelector('.resource-summary'),cores=page.querySelector('.resource-cores');
  const total=gauge(summary,'cpu','CPU 总占用'),memory=gauge(summary,'memory','内存'),swap=gauge(summary,'swap','交换空间');
  const cpuBars=new Map();
  async function poll(){
    if(!alive())return;
    try{
      if(!document.hidden){
        const response=await fetch('/api/system/resources',{cache:'no-store',signal:AbortSignal.timeout(2500)});
        if(!response.ok)throw new Error('HTTP '+response.status);
        const data=await response.json(),all=data.cpus.find(cpu=>cpu.id==='cpu');
        total(all.percent,`${data.cpus.length-1} 个逻辑核心`);
        memory(data.memory.percent,`${gib(data.memory.used_bytes)} / ${gib(data.memory.total_bytes)}`);
        swap(data.swap.percent,data.swap.total_bytes?`${gib(data.swap.used_bytes)} / ${gib(data.swap.total_bytes)}`:'未配置交换空间');
        const ids=new Set();
        for(const cpu of data.cpus){
          if(cpu.id==='cpu')continue;ids.add(cpu.id);
          if(!cpuBars.has(cpu.id))cpuBars.set(cpu.id,gauge(cores,cpu.id,'CPU '+cpu.id.slice(3)));
          cpuBars.get(cpu.id)(cpu.percent);
        }
        for(const id of cpuBars.keys())if(!ids.has(id)){cores.querySelector(`[data-resource="${id}"]`)?.remove();cpuBars.delete(id);}
        if(!alive())return;
        page.dataset.stale='false';status.textContent='更新于 '+new Date().toLocaleTimeString('zh-CN')+' · 内存占用按总量减可用量计算';
      }
    }catch(error){if(alive()){page.dataset.stale='true';status.textContent='资源数据暂不可用：'+error.message+'，已有读数为上次采样';}}
    if(alive())setTimeout(poll,1000);
  }
  poll();
}
