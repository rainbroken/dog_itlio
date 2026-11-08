export function libraryDialog({title,message,value,danger=false}){
  return new Promise(resolve=>{
    const dialog=document.createElement('dialog');dialog.className='library-dialog';
    const form=document.createElement('form');form.method='dialog';
    const heading=document.createElement('h2');heading.textContent=title;
    const description=document.createElement('p');description.textContent=message;
    const input=document.createElement('input');input.name='name';input.setAttribute('aria-label','地图名称');input.maxLength=80;input.required=true;input.value=value??'';
    const actions=document.createElement('div');actions.className='library-dialog-actions';
    const cancel=document.createElement('button');cancel.type='button';cancel.className='button';cancel.textContent='取消';
    const submit=document.createElement('button');submit.type='submit';submit.className=`button ${danger?'library-danger':'primary'}`;submit.textContent=danger?'确认删除':'保存名称';
    const done=result=>{dialog.close();dialog.remove();resolve(result);};cancel.onclick=()=>done(null);
    dialog.oncancel=e=>{e.preventDefault();done(null);};
    form.onsubmit=e=>{e.preventDefault();if(value!==undefined&&!input.value.trim()){input.reportValidity();return;}done(value===undefined?true:input.value.trim());};
    actions.append(cancel,submit);form.append(heading,description);if(value!==undefined)form.append(input);form.append(actions);dialog.append(form);document.body.append(dialog);dialog.showModal();
    (value===undefined?cancel:input).focus();if(value!==undefined)input.select();
  });
}
