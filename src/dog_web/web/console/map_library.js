import {mountSavedPcds} from './saved_pcds.js';
import {libraryCard,itemTitle} from './library_cards.js';
import {libraryDialog} from './library_dialog.js';

export function mountMapLibrary({root,api,isBusy,setBusy,load,upload,demo,importSaved,unload,renamed,onError,notice,getMapping}){
  root.innerHTML=`<div class="library-page">
    <header class="library-header"><div><span class="library-eyebrow">MAP LIBRARY</span><h1>地图管理</h1><p>重命名、加载或删除地图，也可从保存的 PCD 生成导航图。</p></div><button class="button" id="refreshLibrary">刷新全部</button></header>
    <details class="library-import"><summary><div><h2>导入外部点云</h2><p>选择 PCD，生成可用于三维地形规划的地图项目。</p></div><span>展开导入 ▾</span></summary><div class="library-import-fields"><label class="library-upload">点云文件<input id="pcdFile" type="file" accept=".pcd"></label><label>地图编号（可选）<input id="mapName" placeholder="自动生成，仅限字母、数字、_、-"></label></div><div class="library-import-actions"><button id="importMap" class="button primary">导入并生成导航图</button><button id="demoMap" class="button">生成合成测试地图</button><span>合成地图仅用于规划测试</span></div></details>
    <div class="library-toolbar"><label>搜索地图<input id="librarySearch" type="search" placeholder="输入名称、地图编号或文件路径" autocomplete="off"></label><span>名称会在下次启动时保留</span></div>
    <section class="library-section" data-projects><div class="content-header"><div><span class="library-eyebrow">NAVIGATION MAPS</span><h2>导航地图 <span class="library-count" id="projectCount">0</span></h2></div><span class="library-hint">已生成地形导航图，可直接加载</span></div><div id="mapList" class="map-cards"></div><p class="library-empty" data-no-match hidden>没有匹配的导航地图</p></section>
    <div id="savedPcdLibrary"></div></div>`;
  const page=root.firstElementChild,alive=()=>page.isConnected&&root.contains(page);
  const $=id=>root.querySelector('#'+id);let saved;
  function filter(){
    if(!alive())return;
    const query=$('librarySearch').value.trim().toLocaleLowerCase();
    for(const section of root.querySelectorAll('.library-section')){
      const cards=[...section.querySelectorAll('[data-library-item]')];
      for(const card of cards)card.hidden=!!query&&!card.dataset.search.includes(query);
      section.querySelector('[data-no-match]').hidden=!cards.length||cards.some(card=>!card.hidden);
    }
  }
  async function projects(){
    const catalog=await api('/api/maps');if(!alive())return;
    $('projectCount').textContent=catalog.maps.length;const list=$('mapList');list.replaceChildren();
    for(const item of catalog.projects||catalog.maps.map(id=>({id}))){
      const card=libraryCard({item,kind:'map',busy:isBusy,primary:item.loaded?'重新加载':'加载地图',
        onPrimary:()=>run(()=>load(item.id)),onRename:()=>manage('map','rename',item),onDelete:()=>manage('map','delete',item),
        onUnload:()=>manage('map','unload',item)});list.append(card);
    }
    if(!catalog.maps.length){const empty=document.createElement('p');empty.className='library-empty';empty.textContent='还没有导航地图。从下方已保存的 PCD 生成，或导入外部文件。';list.append(empty);}filter();
  }
  async function run(action){if(isBusy())return;try{await action();}catch(error){onError(error);}}
  async function manage(kind,operation,item){
    if(isBusy())return;setBusy(true);
    try{
      const name=itemTitle(item,kind);
      if(operation==='rename'){
        const result=await libraryDialog({title:'重命名地图',message:'设置一个容易辨认的名称，地图编号和原始文件路径保持不变。',value:name});if(result===null)return;
        await api(kind==='map'?'/api/maps/rename':'/api/slam/pcds/rename',{id:item.id,name:result});renamed(kind,item.id,result);notice('名称已保存，下次启动会保留');
      }else if(operation==='delete'){
        const yes=await libraryDialog({title:'删除'+(kind==='map'?'导航地图':'建图 PCD'),message:`确定删除「${name}」？\n编号：${item.id}\n${kind==='map'?'对应原始建图 PCD 会保留。':'已生成的导航地图会保留。'}所选内容将移入本地回收目录。`,danger:true});if(!yes)return;
        await api(kind==='map'?'/api/maps/delete':'/api/slam/pcds/delete',{id:item.id,confirm:true});notice('已删除，文件已移入本地回收目录');
      }else{
        await api('/api/maps/unload',{id:item.id});unload(item.id);notice('地图已卸载，可以切换或删除');
      }
    }catch(error){onError(error);}
    finally{setBusy(false);if(alive()){await projects().catch(onError);await saved?.refresh();}}
  }
  for(const id of ['importMap','demoMap','refreshLibrary']){$(id).dataset.mapTaskAction='';$(id).disabled=isBusy();}
  $('pcdFile').onchange=()=>upload.selected($('pcdFile').files[0],$('mapName'));
  $('importMap').onclick=()=>run(()=>upload.start($('pcdFile').files[0],$('mapName').value));
  $('demoMap').onclick=()=>run(demo);$('librarySearch').oninput=filter;
  $('refreshLibrary').onclick=()=>run(async()=>{await projects();await saved.refresh();notice('地图列表已刷新');});
  saved=mountSavedPcds({root:$('savedPcdLibrary'),api,isBusy,manage,onRendered:filter,getMapping,onError,importFile:file=>importSaved(file,$('mapName').value.trim())});
  projects().catch(onError);
}
