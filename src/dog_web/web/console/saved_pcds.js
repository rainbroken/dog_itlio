import {libraryCard} from './library_cards.js';
export function mountSavedPcds({root,api,importFile,onError,isBusy=()=>false,manage,onRendered=()=>{},getMapping=()=>null}){
  const section=document.createElement('section');section.className='saved-pcds';
  section.classList.add('library-section');
  section.innerHTML='<div class="content-header"><div><span class="library-eyebrow">SCAN ARCHIVE</span><h2>已保存的建图 PCD <span class="library-count" data-count>0</span></h2></div><button class="button" data-refresh>刷新列表</button></div><p class="library-hint">保存后的几何点云会自动出现在这里，可直接生成导航地图。</p><details class="library-storage"><summary>存储位置</summary><code data-root></code></details><div class="map-cards" data-files></div><p class="library-empty" data-no-match hidden>没有匹配的 PCD</p>';
  root.append(section);let loading=false;
  async function refresh(){
    if(loading||isBusy()||!section.isConnected)return;loading=true;
    try{
      const catalog=await api('/api/slam/pcds');if(!section.isConnected||isBusy())return;
      section.querySelector('[data-root]').textContent=catalog.root;
      const list=section.querySelector('[data-files]');list.replaceChildren();
      section.querySelector('[data-count]').textContent=catalog.files.length;
      for(const file of catalog.files){
        const mapping=getMapping();file.active=!!mapping?.owned&&`${catalog.root}/${file.id}`.startsWith(mapping.output_dir+'/');
        const card=libraryCard({item:file,kind:'pcd',busy:isBusy,primary:'生成导航图并加载',onRename:()=>manage?.('pcd','rename',file),onDelete:()=>manage?.('pcd','delete',file),onPrimary:async button=>{
          if(isBusy())return;
          button.disabled=true;button.textContent='正在生成并加载…';
          try{await importFile(file);}catch(error){onError(error);}
          finally{button.disabled=isBusy();button.textContent='生成导航图并加载';}
        }});list.append(card);
      }
      if(!catalog.files.length){const empty=document.createElement('p');empty.className='library-empty';empty.textContent='还没有保存的 PCD。完成建图后点击保存，即可在这里管理。';list.append(empty);}
      onRendered();
    }catch(error){if(section.isConnected)section.querySelector('[data-files]').textContent=`读取已保存 PCD 失败：${error.message}`;}
    finally{loading=false;}
  }
  section.querySelector('[data-refresh]').onclick=refresh;
  async function poll(){if(!section.isConnected)return;await refresh();if(section.isConnected)setTimeout(poll,5000);}
  poll();
  return {refresh};
}
