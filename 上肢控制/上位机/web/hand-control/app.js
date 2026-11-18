import {state,command,demo,client,connect,disconnect} from './transport.js';
import {alarmText} from './diagnostics.js';
import {startHeartbeat} from './heartbeat.js';
import {handView,setHandTarget} from '/upper_pose.js';
import {bindNumericBounds,setRange,validInputs} from '/input_bounds.js';

export function mountHandConsole(root, onState=()=>{}) {
  bindNumericBounds(root);
  const $=id=>root.getElementById(id), names=['食指','中指','无名指','小指'], colors=['#6ed7c9','#91b6ec','#f2bd82','#cc9ee5'];
  const phases={idle:'待机',moving:'移动',gripping:'抓握中',contact:'触阈保持',limit:'到达限位',held:'保持',fault:'故障'};
  const faults={feedback_lost:'舵机反馈中断',overtemperature:'温度过高',overload:'负载超限',controller_lost:'控制端心跳超时，请检查连接并保持网页前台',wrong_mode:'舵机模式不符',write_failed:'写指令失败',grip_timeout:'抓握超时',position_outside_limits:'位置超出校准行程',enable_failed:'启用失败',servo_alarm:'舵机返回报警，请检查状态'};
  let s=null,twin=null,target=handView().target,samples=[],configSig='',gestureSig='',busy=false,online=false,livePending=false,liveTimer=0,gen=0,idError='';
  let speedDirty=false,speedSig='';
  function notice(text){$('notice').textContent=text;}
  const poseParams=()=>Object.fromEntries(target.map((v,i)=>['q'+i,v]));
  function renderTarget(){target.forEach((q,i)=>setHandTarget(i,q));target.forEach((q,i)=>{$('s'+i).value=q;$('o'+i).value=q+'%';});twin?.update(s?.fingers.map(f=>online&&f.online?f.q:null)||[null,null,null,null],target);}
  function change(i,v){target[i]=Math.round(v);renderTarget();if($('live').checked){livePending=true;schedule();}}
  function schedule(){if(liveTimer)return;liveTimer=setTimeout(async()=>{liveTimer=0;if(!livePending||!$('live').checked)return;if(busy){schedule();return;}livePending=false;await act('pose',poseParams(),true);if(livePending)schedule();},130);}
  function cancelLive(invalidate=true){$('live').checked=false;livePending=false;clearTimeout(liveTimer);liveTimer=0;if(invalidate)gen++;}
  function selected(){return s?.gestures.find(g=>String(g.slot)===$('gesture').value);}
  async function act(op,data={},quiet=false){
    const ids=op==='config' ? Array.from({length:4},(_,i)=>cfgKeys.map(k=>k+i)).flat()
      : op==='set_id' ? ['oldID','newID'] : op==='speed' ? ['motionSpeed'] : [];
    if (!validInputs(ids.map($))) return;
    if (op==='set_id' && $('oldID').value===$('newID').value) {notice('原ID与新ID必须不同');return;}
    if (op==='config') {
      const configs=Array.from({length:4},(_,i)=>({id:Number($('id'+i).value),open:Number($('open'+i).value),close:Number($('close'+i).value)}));
      if (new Set(configs.map(c=>c.id)).size!==4 || configs.some(c=>Math.abs(c.open-c.close)<20)) {
        notice('四指ID必须互异，张开与闭合位置至少相差20');return;
      }
    }
    if(busy && !['stop','release'].includes(op))return;
    if(op==='scan'||op==='set_id')idError='';
    if(['stop','release','grip','arm','scan','set_id'].includes(op))cancelLive();
    const stamp=gen;busy=true;
    try{await command(op,data);if(op==='speed')speedDirty=false;await refresh();if(['stop','release','arm'].includes(op))target=s.fingers.map(f=>Math.round(f.q));renderTarget();if(!quiet&&stamp===gen)notice('已完成 · '+({pose:'发送目标',grip:'开始逐指抓握',arm:'启用控制',release:'关闭舵机驱动',stop:'停止并保持',save:'保存手势',delete:'删除手势',config:'保存校准',speed:'保存速度',scan:'搜索 ID',set_id:'修改 ID'}[op]||op));}
    catch(e){cancelLive();notice(e.message);if(op==='scan'||op==='set_id')idError=e.message;}
    finally{busy=false;if(s&&online)paint();}
  }
  for(let i=0;i<4;i++){
    const row=document.createElement('label');row.className='finger';row.innerHTML='<span>'+names[i]+'</span><input aria-label="'+names[i]+'闭合量" id="s'+i+'" type="range" min="0" max="100" value="0"><output id="o'+i+'">0%</output>';$('sliders').append(row);
    $('s'+i).oninput=e=>change(i,+e.target.value);
    const card=document.createElement('div');card.className='card';card.style.setProperty('--color',colors[i]);card.innerHTML='<b>'+names[i]+'</b><span id="fb'+i+'">等待反馈</span><span id="env'+i+'"></span>';$('cards').append(card);
  }
  const cfgKeys=['id','open','close','limit','speedSet'];
  $('cal').innerHTML='<div class="cfg"><small>手指</small><small>ID</small><small>张开</small><small>闭合</small><small>阈值</small><small>速度</small></div>'+names.map((n,i)=>'<div class="cfg"><span>'+n+'</span>'+cfgKeys.map(k=>'<input aria-label="'+n+k+'" type="number" id="'+k+i+'"'+(k==='speedSet'?' min="20" max="500"':'')+'>').join('')+'</div>').join('');
  for(let i=0;i<4;i++){
    setRange($('s'+i),0,100,true);
    for(const [k,lo,hi] of [['id',1,20],['open',0,1023],['close',0,1023],['limit',50,1000],['speedSet',20,500]])
      setRange($(k+i),lo,hi,true);
  }
  setRange($('oldID'),1,20,true);setRange($('newID'),1,20,true);
  setRange($('motionSpeed'),20,500,true);
  $('name').maxLength=48;
  $('name').title='名称允许1～48个UTF-8字节';
  $('name').addEventListener('input',()=>{
    let value=Array.from($('name').value);
    while(new TextEncoder().encode(value.join('')).length>48)value.pop();
    $('name').value=value.join('');
  });
  root.querySelectorAll('[data-tab]').forEach(b=>b.onclick=()=>{root.querySelectorAll('.pane').forEach(p=>p.hidden=p.id!==b.dataset.tab);root.querySelectorAll('[data-tab]').forEach(t=>t.classList.toggle('sel',t===b));});
  function paint(){
    const scope=s.owner===client,active=online&&s.armed&&scope;
    for (const k of ['stop','release','save','remove','saveConfig'])
      $(k).disabled=!online||busy||!!(s.owner&&!scope);
    $('saveConfig').disabled ||= !!s.armed;
    $('connectionPolicy').textContent=s.disconnectRelease===0&&s.leaseMs===0
      ?'网页断连不会关闭驱动；已接受的动作会继续执行，到达目标后保持。需要停止请点击“停止并保持”。'
      :'当前控制板仍启用心跳超时保护（'+((s.leaseMs??3000)/1000)+' 秒）；取消断连自动关闭驱动需要上传新版固件。';
    const alarmCount=s.fingers.filter(f=>f.alarm).length;
    $('status').textContent=(demo?'离线演示':'控制板')+' · '+(s.fault?(faults[s.fault]||s.fault):alarmCount?alarmCount+' 只舵机报警':s.gripping?'逐指抓握':s.armed?'已启用':'待机');
    $('arm').disabled=!online||!!s.armed||!s.ready||!!(s.owner&&!scope);
    const speedSupported=!!s.speedControl, moving=!!s.gripping||s.fingers.some(f=>['moving','gripping'].includes(f.phase));
    $('saveSpeed').disabled=!online||busy||!speedSupported||moving||!!(s.owner&&!scope);
    $('motionSpeed').disabled=!speedSupported;
    const speeds=s.fingers.map(f=>f.speedSet??100),newSpeedSig=JSON.stringify(speeds);
    if(newSpeedSig!==speedSig){speedSig=newSpeedSig;if(!speedDirty){$('motionSpeed').value=speeds[0];$('motionSpeedValue').value=speeds[0];}}
    $('speedCurrent').textContent=speedSupported?'已保存：'+speeds.join(' / ')+' 编码步/秒':'需要上传支持调速的新版固件';
    for(const k of ['apply','play','grip','live'])$(k).disabled=!active||!!s.gripping;
    const supportsID='scanDone' in s;
    for(const k of ['scanIDs','setID'])$(k).disabled=!online||busy||!!s.armed||!!s.gripping||!!(s.owner&&!scope)||!supportsID;
    $('foundIDs').textContent=!supportsID?'需要上传支持 ID 设置的新版固件':s.scanDone?'最近搜索到的 ID：'+(s.detectedIDs?.join('、')||'无响应'):'尚未搜索';
    $('idResult').textContent=idError||s.idMessage||'';
    if(!active||s.gripping)cancelLive(false);
    if(!s.armed && !s.ready && !$('notice').textContent.includes('失败') && $('notice').textContent==='等待反馈')notice('请在配置页核对四指 ID 和行程，并确认舵机在线。');
    s.fingers.forEach((f,i)=>{
      let diagnostic=f.online?(phases[f.phase]||f.phase)+' · 位置 '+f.p:'反馈不可用';
      if(f.alarm) diagnostic=(f.online?'在线':'已响应')+' · '+alarmText(f.alarm);
      if(f.readError===2) diagnostic+=' · 模式读取失败';
      $('fb'+i).textContent='ID '+f.id+' · '+diagnostic;
      $('env'+i).textContent=f.online?'负载 '+f.load+' / '+f.limit+' · '+Number(f.v).toFixed(1)+' V · '+f.temp+' °C':'读取失败次数 '+(f.failures??'—');
    });
    const cs=JSON.stringify(s.fingers.map(f=>cfgKeys.map(k=>f[k])));if(cs!==configSig){configSig=cs;s.fingers.forEach((f,i)=>cfgKeys.forEach(k=>$(k+i).value=f[k]??(k==='speedSet'?100:'')));$('verified').checked=s.fingers.every(f=>f.verified);}
    s.fingers.forEach((f,i)=>$('speedSet'+i).disabled=!speedSupported);
    const gs=JSON.stringify(s.gestures);if(gs!==gestureSig){gestureSig=gs;const old=$('gesture').value;$('gesture').replaceChildren(...s.gestures.map(g=>new Option(g.name,g.slot)));if(s.gestures.some(g=>String(g.slot)===old))$('gesture').value=old;}
    renderTarget();draw();
  }
  let poll=null;
  async function refresh(){
    if(poll)return poll;
    poll=(async()=>{try{s=await state();online=true;onState({connected:true,ageMs:0,...s});const t=performance.now();samples.push({t,f:s.fingers});samples=samples.filter(a=>t-a.t<30000);paint();}
    catch(e){online=false;onState({connected:false,error:e.message,fingers:[]});cancelLive();$('status').textContent='控制板离线';['apply','grip','play','arm','scanIDs','setID','saveSpeed','stop','release','save','remove','saveConfig'].forEach(k=>$(k).disabled=true);$('notice').textContent='未收到反馈 · '+e.message;for(let i=0;i<4;i++){$('fb'+i).textContent='反馈不可用';$('env'+i).textContent='';}renderTarget();throw e;}
    finally{poll=null;}})();return poll;
  }
  function draw(){
    const c=$('chart'),w=c.clientWidth,h=150,d=Math.min(devicePixelRatio,2);if(!w)return;c.width=w*d;c.height=h*d;const ctx=c.getContext('2d');ctx.scale(d,d);
    const key=$('metric').value,now=performance.now(),all=samples.flatMap(a=>a.f.filter(f=>f.online).map(f=>f[key]));
    const min=Math.min(0,...all),max=Math.max(1,...all,...(key==='load'?s.fingers.map(f=>f.limit):[]));
    const y=v=>123-(v-min)/(max-min)*112;ctx.font='11px monospace';ctx.fillStyle='#b6c8cf';
    for(let k=0;k<3;k++){const v=min+(max-min)*k/2;ctx.fillText(String(Math.round(v*10)/10),0,y(v)+4);ctx.strokeStyle='#344653';ctx.beginPath();ctx.moveTo(38,y(v));ctx.lineTo(w,y(v));ctx.stroke();}
    for(let i=0;i<4;i++){ctx.strokeStyle=colors[i];ctx.lineWidth=2;ctx.beginPath();let pen=false,prev=0;
      for(const a of samples){if(!a.f[i].online){pen=false;continue;}const x=38+(1-(now-a.t)/30000)*(w-43);if(!pen||a.t-prev>1200)ctx.moveTo(x,y(a.f[i][key]));else ctx.lineTo(x,y(a.f[i][key]));pen=true;prev=a.t;}ctx.stroke();}
    ctx.fillText('−30 s',38,145);ctx.fillText('现在',w-30,145);
  }
  $('metric').onchange=draw;
  $('arm').onclick=()=>act('arm');$('release').onclick=()=>act('release');$('stop').onclick=()=>act('stop');$('grip').onclick=()=>act('grip');$('apply').onclick=()=>act('pose',poseParams());
  $('save').onclick=()=>act('save',{name:$('name').value,...poseParams()});
  $('load').onclick=()=>{if(selected()){cancelLive();target=selected().q.slice();renderTarget();notice('已预览 · '+selected().name);}};
  $('play').onclick=()=>{if(selected()){target=selected().q.slice();renderTarget();act('pose',poseParams());}};
  $('remove').onclick=()=>{if(selected())act('delete',{slot:selected().slot});};
  $('saveConfig').onclick=()=>{if(!$('verified').checked){notice('请先核对安装方向和行程');return;}const data={};for(let i=0;i<4;i++)cfgKeys.forEach(k=>{if(k!=='speedSet'||s?.speedControl)data[(k==='speedSet'?'speed':k)+i]=$(k+i).value;});act('config',data);};
  $('motionSpeed').oninput=()=>{speedDirty=true;$('motionSpeedValue').value=$('motionSpeed').value;};
  $('saveSpeed').onclick=()=>act('speed',{speed:$('motionSpeed').value});
  $('scanIDs').onclick=()=>act('scan');
  $('setID').onclick=()=>act('set_id',{old_id:$('oldID').value,new_id:$('newID').value});
  $('resetView').onclick=()=>twin?.reset();$('live').onchange=()=>{if(!$('live').checked)cancelLive();};
  $('connectHand').onclick=async()=>{cancelLive();try{await connect();await refresh();notice('已连接手部');}catch(e){notice(e.message);}};
  $('disconnectHand').onclick=async()=>{cancelLive();try{await disconnect();await refresh();}catch(e){notice('已断开通信或反馈不可用 · '+e.message);}};
  startHeartbeat({send:()=>command('beat'),isActive:()=>s?.owner===client,
    onError:e=>{if(e.status===403||e.status===409){cancelLive();if(s?.armed)notice('控制心跳被拒绝：'+e.message+'，请检查状态后重新启用。');}}});
  async function loop(){try{if(!busy)await refresh();}catch{}setTimeout(loop,300);}
  loop();
  import('./twin.js').then(async({makeTwin})=>{twin=await makeTwin($('viewport'),change,()=>{});$('modelMsg').hidden=true;renderTarget();}).catch(e=>{$('modelMsg').textContent='3D 视图不可用，可使用右侧滑块 · '+e.message;});

  window.addEventListener('hand-target-change',()=>{target=handView().target;renderTarget();});
  return {setVisible(visible){if(!visible)cancelLive();else renderTarget();}};
}
