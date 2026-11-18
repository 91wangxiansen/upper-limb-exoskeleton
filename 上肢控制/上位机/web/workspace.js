import {mountHandConsole} from '/hand-control/app.js';
import {setHandSnapshot} from '/upper_pose.js';

const el = id => document.getElementById(id);
let controller = null, selected = 'Arm';
function show(device) {
  selected = device;
  el('armWorkspace').hidden = device !== 'Arm';
  el('handWorkspace').hidden = device !== 'Hand';
  for (const item of ['Arm','Hand']) {
    el('show'+item).classList.toggle('active', item === device);
    el('show'+item).setAttribute('aria-pressed', String(item === device));
  }
  controller?.setVisible(device === 'Hand');
  window.scrollTo(0,0);
  window.dispatchEvent(new Event('resize'));
}
function updateHand(state) {
  setHandSnapshot(state);
  const live = state.connected;
  el('handPill').className = 'pill ' + (live ? state.fault ? 'fault' : 'online' : 'offline');
  el('handPill').querySelector('b').textContent = live ? '在线' : '离线';
  const count = state.fingers?.filter(f => live && f.online).length || 0;
  el('handPoseSummary').textContent = count
    ? `手部反馈 ${count}/4 · ${state.armed ? '驱动启用' : '驱动关闭'}` : '手部等待反馈';
}
for (const device of ['Arm','Hand']) el('show'+device).addEventListener('click',()=>show(device));
window.addEventListener('open-hand-control',()=>show('Hand'));
el('openHandConsole').addEventListener('click',()=>show('Hand'));

async function mount() {
  const response = await fetch('/hand-control/', {cache:'no-store'});
  if (!response.ok) throw Error('手部界面读取失败');
  const template = new DOMParser().parseFromString(await response.text(),'text/html');
  const host = el('handConsole'), root = host.attachShadow({mode:'open'});
  const style = document.createElement('link');
  style.rel = 'stylesheet'; style.href = '/hand-control/styles.css';
  const scope = document.createElement('style');
  scope.textContent = ':host{display:block;color:#e4ecf3;font:15px/1.5 Inter,"Microsoft YaHei",sans-serif}[hidden]{display:none!important}header{padding-top:16px}';
  const ready = new Promise((resolve,reject)=>{style.onload=resolve;style.onerror=()=>reject(Error('手部样式读取失败'));});
  root.append(style,scope,template.querySelector('header'),template.querySelector('main'));
  root.querySelector('.back-link').addEventListener('click',event=>{event.preventDefault();show('Arm');});
  await ready;
  controller = mountHandConsole(root,updateHand);
  controller.setVisible(selected === 'Hand');
  el('handConsoleMessage').hidden = true;
}
mount().catch(error=>{el('handConsoleMessage').textContent='手部界面加载失败 · '+error.message;});
