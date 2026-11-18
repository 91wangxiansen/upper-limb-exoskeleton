// Demo is explicitly selected with ?demo=1. Live failures never switch to simulation.
export const demo=new URLSearchParams(location.search).get('demo')==='1';
export const client=Array.from(crypto.getRandomValues(new Uint32Array(3))).join('-');
let epoch=0,token='',sim=null,last=performance.now();
function initSim(){
  const config=JSON.parse(localStorage.getItem('hand-demo-config')||'null');
  const fingers=[250,150,800,600].map((open,i)=>{
    const c=config?.[i]||{open,close:[600,650,300,200][i],id:1+i,limit:300};
    return {...c,speedSet:c.speedSet??100,p:c.open,q:0,target:0,load:0,speed:0,temp:28,v:7.3,phase:'idle',online:1,verified:1};
  });
  return {demo:true,speedControl:1,disconnectRelease:0,leaseMs:0,owner:'',ready:1,armed:0,gripping:0,fault:'',scanDone:0,detectedIDs:[],idMessage:'',fingers,gestures:JSON.parse(localStorage.getItem('hand-demo-poses')||'[]')};
}
function simulate(){
  if(!sim)sim=initSim();const now=performance.now(),dt=Math.min(.4,(now-last)/1000);last=now;
  sim.fingers.forEach((f,i)=>{
    const before=f.q;
    if(sim.armed && ['moving','gripping'].includes(f.phase)){
      f.q+=Math.sign(f.target-f.q)*Math.min(Math.abs(f.target-f.q),dt*f.speedSet*100/Math.max(20,Math.abs(f.close-f.open)));
      if(f.phase==='gripping' && f.q>=[35,60,78,100][i] && i<3){f.load=f.limit;f.phase='contact';f.target=f.q;}
      else if(Math.abs(f.q-f.target)<.01)f.phase=sim.gripping?'limit':'held';
    }
    f.p=Math.round(f.open+(f.close-f.open)*f.q/100);f.speed=Math.round((f.q-before)/Math.max(dt,.001)*(f.close-f.open)/100);
    if(f.phase!=='contact')f.load=Math.abs(f.speed)?45+i*9:0;
  });
  sim.gripping=Number(sim.fingers.some(f=>f.phase==='gripping'));return JSON.parse(JSON.stringify(sim));
}
async function request(path,options={}){
  const c=new AbortController(),timer=setTimeout(()=>c.abort(),2800);
  try{const r=await fetch(path,{cache:'no-store',...options,signal:c.signal});const j=await r.json();if(!r.ok){const error=Error(j.error||r.status);error.status=r.status;throw error;}return j;}
  finally{clearTimeout(timer);}
}
export async function state(){if(demo)return simulate();const j=await request('/api/state');token=j.token;epoch=j.epoch;return j;}
export async function command(op,params={}){
  if(!demo)return request('/api/command',{method:'POST',headers:{'X-Token':token,'X-Client':client},body:new URLSearchParams({op,epoch,...params})});
  simulate();if(op!=='release')sim.owner=client;
  if(op==='scan'){
    if(sim.armed)throw Error('请先释放');
    sim.scanDone=1;sim.detectedIDs=sim.fingers.map(f=>f.id);sim.idMessage='演示搜索完成，实际改号需连接真实单只舵机';
  }
  if(op==='set_id')throw Error('请连接真实控制板并只接一只舵机后改号');
  if(op==='speed'){
    if(sim.gripping||sim.fingers.some(f=>['moving','gripping'].includes(f.phase)))throw Error('请先停止并保持，再修改速度');
    const speed=Number(params.speed);
    if(!Number.isInteger(speed)||speed<20||speed>500)throw Error('速度范围为 20～500 编码步/秒');
    sim.fingers.forEach(f=>f.speedSet=speed);
    localStorage.setItem('hand-demo-config',JSON.stringify(sim.fingers.map(f=>Object.fromEntries(['id','open','close','limit','speedSet'].map(k=>[k,f[k]])))));
  }
  if(op==='arm')sim.armed=1;
  if(op==='release'){sim.armed=0;sim.owner='';}
  if(['release','stop'].includes(op)){sim.gripping=0;sim.fingers.forEach(f=>{f.target=f.q;f.phase=sim.armed?'held':'idle';});}
  if(op==='pose'||op==='grip'){
    if(!sim.armed||sim.gripping)throw Error('请先启用控制，抓握时先停止');
    sim.gripping=Number(op==='grip');sim.fingers.forEach((f,i)=>{f.target=op==='grip'?100:Number(params['q'+i]);f.phase=op==='grip'?'gripping':'moving';});
  }
  if(op==='save'){
    const name=params.name.trim();if(!name||new TextEncoder().encode(name).length>48)throw Error('名称需要 1–48 字节');
    let g=sim.gestures.find(g=>g.name===name);if(!g){const slot=Array.from({length:16},(_,i)=>i).find(i=>!sim.gestures.some(g=>g.slot===i));if(slot==null)throw Error('手势库已满');g={slot,name};sim.gestures.push(g);}
    g.q=sim.fingers.map((_,i)=>+params['q'+i]);localStorage.setItem('hand-demo-poses',JSON.stringify(sim.gestures));
  }
  if(op==='delete'){sim.gestures=sim.gestures.filter(g=>g.slot!=params.slot);localStorage.setItem('hand-demo-poses',JSON.stringify(sim.gestures));}
  if(op==='config'){
    if(sim.armed)throw Error('请先释放');
    const configs=sim.fingers.map((f,i)=>({...Object.fromEntries(['id','open','close','limit'].map(k=>[k,+params[k+i]])),speedSet:params['speed'+i]==null?f.speedSet:Number(params['speed'+i])}));
    if(new Set(configs.map(c=>c.id)).size!==4||configs.some(c=>c.id<1||c.id>20||Math.abs(c.open-c.close)<20||c.limit<50||c.limit>1000||!Number.isInteger(c.speedSet)||c.speedSet<20||c.speedSet>500))throw Error('请核对范围和 ID、速度');
    localStorage.setItem('hand-demo-config',JSON.stringify(configs));sim=initSim();
  }
}
