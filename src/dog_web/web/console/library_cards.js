export const formatSize=bytes=>`${(Number(bytes||0)/1048576).toFixed(2)} MiB`;
export const formatTime=ms=>Number.isFinite(ms)?new Date(ms).toLocaleString('zh-CN',{hour12:false}):'时间未知';
export function itemTitle(item,kind){
  if(item.name&&item.name!==item.id)return item.name;
  if(kind==='pcd'){
    const match=item.id.match(/(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})/);
    if(match)return `建图 · ${match[1]}-${match[2]}-${match[3]} ${match[4]}:${match[5]}:${match[6]}`;
    return '建图 · '+item.id.split('/').slice(0,-1).join('/');
  }
  return item.id;
}
function element(tag,className,text){const el=document.createElement(tag);el.className=className;if(text!==undefined)el.textContent=text;return el;}
export function libraryCard({item,kind,busy,primary,onPrimary,onRename,onDelete,onUnload}){
  const card=element('article','card library-card');card.dataset.libraryItem='';card.dataset.kind=kind;card.dataset.id=item.id;
  card.dataset.search=[itemTitle(item,kind),item.id].join(' ').toLocaleLowerCase();
  const top=element('div','library-card-top'),icon=element('span','library-icon',kind==='map'?'▧':'◈'),badge=element('span','library-badge',kind==='map'?'导航地图':'几何 PCD');
  top.append(icon,badge);if(item.loaded)top.append(element('span','library-badge current','已加载'));if(item.active)top.append(element('span','library-badge current','建图中'));
  const title=element('h3','library-title',itemTitle(item,kind));title.title=title.textContent;
  const description=element('p','library-description',kind==='map'?`${Number(item.summary?.valid_surfaces||0).toLocaleString()} 个可通行节点 · ${formatSize(item.bytes)}`:`${formatSize(item.bytes)} · 原始几何点云`);
  const time=element('p','library-time',formatTime(item.modified_ms));
  const details=element('details','library-path'),summary=element('summary','',kind==='map'?'地图编号':'查看文件路径');
  details.append(summary,element('code','',item.id));
  const actions=element('div','library-card-actions');
  function button(text,style,callback){const b=element('button',`button ${style}`,text);b.type='button';b.dataset.mapTaskAction='';b.disabled=busy();b.onclick=()=>{if(!busy())callback(b);};return b;}
  const main=button(primary,'primary',onPrimary);main.dataset.libraryPrimary='';actions.append(main);
  const secondary=element('div','library-secondary');
  secondary.append(button('重命名','',onRename));
  if(item.loaded&&onUnload)secondary.append(button('卸载','',onUnload));
  const del=button('删除','library-danger',onDelete);del.dataset.delete='';
  if(item.loaded||item.active){del.disabled=true;del.dataset.unavailable='true';del.title=item.active?'请结束建图后再删除':'请先卸载或切换地图';}
  secondary.append(del);actions.append(secondary);
  card.append(top,title,description,time,details);
  if(item.error)card.append(element('p','library-warning',item.error));
  card.append(actions);return card;
}
