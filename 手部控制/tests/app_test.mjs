// Execute the real UI module with a minimal DOM and a simulated transport.
import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const elements=new Map();
class Element {
  value='';textContent='';checked=false;disabled=false;hidden=false;clientWidth=0;
  style={setProperty(){}};classList={toggle(){}};
  set innerHTML(html){for(const match of html.matchAll(/id="([^"]+)"/g))get(match[1]);}
  append(){} replaceChildren(){}
}
function get(id){if(!elements.has(id))elements.set(id,new Element());return elements.get(id);}
for(const match of fs.readFileSync(path.join(root,'web/index.html'),'utf8').matchAll(/id="([^"]+)"/g))get(match[1]);
get('notice').textContent='等待反馈';get('motionSpeed').value='100';
const model={speedControl:1,owner:'',ready:1,armed:0,gripping:0,fault:'',scanDone:0,gestures:[],
 fingers:[250,150,800,600].map((open,i)=>({id:i+1,open,close:[600,650,300,200][i],limit:300,speedSet:100,verified:1,online:1,p:open,q:0,phase:'idle',load:0,temp:28,v:6}))};
const commands=[];const timers=[];
let holdState=false,resumeState;
const context=vm.createContext({console,performance:{now:()=>1000},
 document:{hidden:false,getElementById:id=>{assert.ok(elements.has(id),'Unknown DOM id '+id);return elements.get(id);},createElement:()=>new Element(),querySelectorAll:()=>[]},
 setTimeout:fn=>{timers.push(fn);return timers.length;},clearTimeout(){},setInterval:fn=>{timers.push(fn);return timers.length;},clearInterval(){},
 devicePixelRatio:1,Option:class{},
 fixtureState:()=>holdState?new Promise(resolve=>{resumeState=()=>resolve(structuredClone(model));}):Promise.resolve(structuredClone(model)),fixtureCommand:async(op,params)=>{
  commands.push({op,params});if(op==='speed')model.fingers.forEach(f=>f.speedSet=Number(params.speed));
 }});
const modules=new Map();
async function load(name){
 if(modules.has(name))return modules.get(name);
 const code=name==='transport.js'?'export const demo=true,client="test-ui-client",state=globalThis.fixtureState,command=globalThis.fixtureCommand;':fs.readFileSync(path.join(root,'web',name),'utf8');
 const module=new vm.SourceTextModule(code,{context,importModuleDynamically:()=>{throw Error('3D loading is outside this UI test');}});
 modules.set(name,module);await module.link(spec=>load(spec.replace('./','')));return module;
}
await(await load('app.js')).evaluate();await new Promise(setImmediate);
assert.match(get('speedCurrent').textContent,/100 \/ 100 \/ 100 \/ 100/);
assert.equal(get('saveSpeed').disabled,false);
assert.match(get('connectionPolicy').textContent,/仍启用心跳超时保护/,'Old firmware must still show its actual timeout policy');
get('motionSpeed').value='300';get('motionSpeed').oninput();await get('saveSpeed').onclick();
assert.match(get('speedCurrent').textContent,/300 \/ 300 \/ 300 \/ 300/);
assert.equal(commands.find(c=>c.op==='speed').params.speed,'300');
get('verified').checked=true;get('saveConfig').onclick();await new Promise(setImmediate);
const config=commands.find(c=>c.op==='config').params;
assert.equal(config.speed0,300);assert.equal(config.speed3,300);
assert.ok(!('speedSet0' in config),'Config uses the firmware speed parameter names');
model.owner='test-ui-client';model.armed=1;model.disconnectRelease=0;model.leaseMs=0;
await timers.at(-1)(); // Poll once to learn that this page holds control.
assert.match(get('connectionPolicy').textContent,/网页断连不会关闭驱动/);
holdState=true;const slowPoll=timers.at(-1)();await Promise.resolve();
await timers[0]();
assert.ok(commands.some(c=>c.op==='beat'),'A pending state read must not block the real UI heartbeat');
holdState=false;resumeState();await slowPoll;
context.document.hidden=true;
const beatsBefore=commands.filter(c=>c.op==='beat').length;
await timers[0]();
assert.equal(commands.filter(c=>c.op==='beat').length,beatsBefore+1,'Background pages still send heartbeats');
console.log('PASS: real UI module renders speed settings and submits unified/per-finger speed commands.');
