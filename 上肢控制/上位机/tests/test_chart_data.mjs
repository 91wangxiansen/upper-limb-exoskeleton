import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source=await readFile(new URL('../web/chart_data.js',import.meta.url),'utf8');
const {mergeHistory,smoothSeries,chartView}=await import('data:text/javascript;base64,'+Buffer.from(source).toString('base64'));
const sample=(start,end,value=0)=>Array.from({length:Math.round((end-start)*100)+1},(_,i)=>[start+i/100,value]);
const history=rows=>({shoulder:{velocity:rows},elbow:{velocity:rows.map(([t,v])=>[t,-v])}});

test('a 1200-point server buffer uses the complete plot width and actual duration',()=>{
  const view=chartView(history(sample(100,111.99,.05)),'velocity',112);
  assert.ok(view.span>11 && view.span<12);
  assert.equal(view.start,view.series.shoulder[0][0][0]);
  assert.equal(view.end,view.series.shoulder[0].at(-1)[0]);
});
test('overlapping snapshots accumulate 60 seconds with deduplication and expiry',()=>{
  let cache={};
  for(let end=12;end<=72;end++)cache=mergeHistory(cache,history(sample(end-12,end)),end);
  const rows=cache.shoulder.velocity;
  assert.ok(rows[0][0]>=12);
  assert.ok(rows.at(-1)[0]===72);
  assert.equal(new Set(rows.map(p=>p[0])).size,rows.length);
  const view=chartView(cache,'velocity',72);
  assert.ok(view.span>59 && view.span<=60);
});
test('filter suppresses holding noise, retains signed values and leaves raw rows intact',()=>{
  const rows=sample(0,8).map(([t],i)=>[t,.4+(i%2?.2:-.2)]);
  const before=JSON.stringify(rows);
  const filtered=smoothSeries(rows)[0].filter(([t])=>t>1);
  const rms=Math.sqrt(filtered.reduce((sum,[,v])=>sum+(v-.4)**2,0)/filtered.length);
  assert.ok(rms<.03);
  assert.equal(JSON.stringify(rows),before);
  const negative=smoothSeries(sample(0,2,-.8))[0];
  assert.ok(negative.every(p=>Math.abs(p[1]+.8)<1e-10));
});
test('missing samples and feedback gaps split lines and reset filter history',()=>{
  const segments=smoothSeries([[0,0],[.1,0],[.2,null],[.3,2],[.4,2],[3,-1],[3.1,-1]]);
  assert.equal(segments.length,3);
  assert.equal(segments[1][0][1],2);
  assert.equal(segments[2][0][1],-1);
});
test('real steps settle while tiny stationary noise is not expanded to a solid block',()=>{
  const rows=sample(0,4).map(([t])=>[t,t<1?0:1]);
  const filtered=smoothSeries(rows)[0];
  assert.ok(filtered.find(([t])=>t>=2.5)[1]>.97);
  const view=chartView(history(sample(10,12,.001)),'velocity',12);
  assert.ok(view.high-view.low>=.4);
});
test('empty, invalid, future and expired data never produce false feedback',()=>{
  const cache=mergeHistory({},history([[0,1],[NaN,2],[50,3],[1000,4]]),100);
  assert.deepEqual(cache.shoulder.velocity,[[50,3]]);
  assert.equal(chartView({},'velocity',100).empty,true);
  assert.equal(chartView(history([[0,1]]),'velocity',100).empty,true);
  assert.equal(chartView(history([[99,1]]),'velocity',105).stale,true);
});
