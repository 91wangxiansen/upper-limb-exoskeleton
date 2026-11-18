import fs from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
const diagnostics=new vm.SourceTextModule(fs.readFileSync(path.join(root,'web/diagnostics.js'),'utf8'));
await diagnostics.link(()=>{throw Error('Unexpected import')});await diagnostics.evaluate();
const {alarmText}=diagnostics.namespace;
assert.equal(alarmText(0),'');
assert.match(alarmText(1),/电压异常.*0x01/);
assert.match(alarmText(5),/电压异常.*温度过高.*0x05/);
assert.match(alarmText(0x20),/负载过载.*0x20/);
assert.match(alarmText(0x80),/其他状态位异常.*0x80/);
const heartbeat=new vm.SourceTextModule(fs.readFileSync(path.join(root,'web/heartbeat.js'),'utf8'));
await heartbeat.link(()=>{throw Error('Unexpected import')});await heartbeat.evaluate();
let heartbeatTick,resolveBeat,beatCount=0,heartbeatActive=true,cancelled=false;
const stopHeartbeat=heartbeat.namespace.startHeartbeat({
 send:()=>{beatCount++;return new Promise(resolve=>{resolveBeat=resolve;});},
 isActive:()=>heartbeatActive,
 schedule:(fn,interval)=>{assert.equal(interval,600);heartbeatTick=fn;return 17;},
 cancel:id=>{assert.equal(id,17);cancelled=true;}
});
const pendingBeat=heartbeatTick();assert.equal(beatCount,1);
await heartbeatTick();assert.equal(beatCount,1,'A slow request must not overlap another heartbeat');
resolveBeat();await pendingBeat;
heartbeatActive=false;await heartbeatTick();assert.equal(beatCount,1);
heartbeatActive=true;const secondBeat=heartbeatTick();assert.equal(beatCount,2);resolveBeat();await secondBeat;
stopHeartbeat();assert.equal(cancelled,true);await heartbeatTick();assert.equal(beatCount,2);
let retryTick,errors=0,attempts=0;
heartbeat.namespace.startHeartbeat({send:async()=>{attempts++;throw Error('offline');},isActive:()=>true,
 onError:()=>errors++,schedule:fn=>{retryTick=fn;return 18;},cancel:()=>{}});
await retryTick();await retryTick();assert.equal(errors,2);assert.equal(attempts,2);
const source=fs.readFileSync(path.join(root,'web/transport.js'),'utf8');
const saved=new Map();let time=0;
const context=vm.createContext({URLSearchParams,TextEncoder,crypto:globalThis.crypto,performance:{now:()=>time},
  location:{search:'?demo=1'},localStorage:{getItem:k=>saved.get(k)??null,setItem:(k,v)=>saved.set(k,v)},setTimeout,clearTimeout,AbortController,console});
async function load(){const m=new vm.SourceTextModule(source,{context});await m.link(()=>{throw Error('Unexpected import')});await m.evaluate();return m.namespace;}
let api=await load();let s=await api.state();assert.equal(s.demo,true);
await api.command('arm');await api.command('grip');
for(let i=0;i<17;i++){time+=200;s=await api.state();}
assert.equal(s.fingers[0].phase,'contact');
assert.equal(s.fingers[1].phase,'contact');
assert.equal(s.fingers[2].phase,'gripping');
assert.ok(s.fingers[0].p>250);assert.ok(s.fingers[2].p<800);
await api.command('stop');const held=(await api.state()).fingers.map(f=>f.p);
time+=1000;assert.deepEqual(Array.from((await api.state()).fingers,f=>f.p),Array.from(held));
await api.command('save',{name:'剪刀手',q0:0,q1:0,q2:100,q3:100});
api=await load();s=await api.state();assert.equal(s.gestures[0].name,'剪刀手');
assert.deepEqual(Array.from(s.gestures[0].q),[0,0,100,100]);
await api.command('delete',{slot:0});assert.equal((await api.state()).gestures.length,0);
await api.command('release');await api.command('scan');
s=await api.state();assert.equal(s.scanDone,1);assert.deepEqual(Array.from(s.detectedIDs),[1,2,3,4]);
await assert.rejects(()=>api.command('set_id',{old_id:1,new_id:2}),/真实/);
await api.command('arm');await assert.rejects(()=>api.command('scan'),/释放/);
await api.command('release');await assert.rejects(()=>api.command('pose',{q0:0,q1:0,q2:0,q3:0}));
await api.command('speed',{speed:300});s=await api.state();
assert.ok(s.fingers.every(f=>f.speedSet===300));
api=await load();s=await api.state();assert.ok(s.fingers.every(f=>f.speedSet===300),'Speed survives demo reload');
await assert.rejects(()=>api.command('speed',{speed:0}),/20/);
await assert.rejects(()=>api.command('speed',{speed:501}),/500/);
await assert.rejects(()=>api.command('speed',{speed:100.5}),/20/);
await api.command('arm');await api.command('pose',{q0:100,q1:100,q2:100,q3:100});
await assert.rejects(()=>api.command('speed',{speed:200}),/停止/);
time+=200;const fastPosition=(await api.state()).fingers[0].p;
assert.ok(fastPosition>s.fingers[0].p,'The saved speed drives simulated movement');
await api.command('stop');await api.command('speed',{speed:200});
assert.ok((await api.state()).fingers.every(f=>f.speedSet===200));
await api.command('release');
const config={};(await api.state()).fingers.forEach((f,i)=>{for(const k of ['id','open','close','limit'])config[k+i]=f[k];config['speed'+i]=50+i*50;});
await api.command('config',config);assert.deepEqual(Array.from((await api.state()).fingers,f=>f.speedSet),[50,100,150,200]);
context.location.search='';context.fetch=async()=>{throw Error('offline');};
api=await load();await assert.rejects(()=>api.state(),/offline/);
const meta=JSON.parse(fs.readFileSync(path.join(root,'web/model.json')));
const binary=fs.readFileSync(path.join(root,'web/model.bin'));
assert.equal(meta.fingers.length,4);assert.equal(meta.parts.length,32);
for(const part of meta.parts){
 assert.equal(part.vertexOffset%4,0);
 assert.ok(part.vertexOffset+part.vertexCount*12<=binary.length);
 assert.ok(part.indexOffset+part.indexCount*2<=binary.length);
 for(let j=0;j<part.indexCount;j++)assert.ok(binary.readUInt16LE(part.indexOffset+j*2)<part.vertexCount);
}
console.log('PASS: speed limits/persistence/motion guard, independent heartbeats, contact, gestures, live failure, CAD bounds.');

