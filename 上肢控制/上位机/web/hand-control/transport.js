import * as legacyDemo from './demo_transport.js';
export const demo = legacyDemo.demo;
// This identifier is a UI ownership category. Device credentials stay in Python.
export const client = demo ? legacyDemo.client : 'unified-backend';
let revision = null;
let pending = false;
async function request(path, body) {
  const response = await fetch('/api/hand/' + path, {
    method:body === undefined ? 'GET' : 'POST', cache:'no-store',
    headers:{'Content-Type':'application/json'},
    body:body === undefined ? undefined : JSON.stringify(body),
    signal:AbortSignal.timeout(8000)
  });
  const data = await response.json();
  if (!response.ok) {
    const error = Error(data.error || '手部请求失败');
    error.status = response.status;
    throw error;
  }
  return body === undefined ? data : data.state;
}
function adapt(data) {
  if (!data.connected) throw Error(data.error || '请点击连接手部');
  revision = data.revision;
  return {...data, gestures:data.gestures || [],
    owner:data.ownership === 'local' ? client : data.ownership === 'other' ? 'other-controller' : ''};
}
export async function state() {
  if (demo) return legacyDemo.state();
  return adapt(await request('state'));
}
export async function connect() {
  if (demo) return legacyDemo.state();
  return adapt(await request('connect', {}));
}
export async function disconnect() {
  if (demo) return;
  revision = null;
  await request('disconnect', {});
}
export async function command(op, params={}) {
  if (demo) return legacyDemo.command(op, params);
  // The Python adapter maintains legacy board heartbeats independently of pages.
  if (op === 'beat') return;
  if (pending) throw Error('上一条指令仍在处理，请稍后重新操作');
  const body = {op, revision};
  if (op === 'pose' || op === 'save') body.targets = [0,1,2,3].map(i => Number(params['q'+i]));
  if (op === 'save') body.name = params.name;
  if (op === 'delete') body.slot = Number(params.slot);
  if (op === 'speed') body.speed = Number(params.speed);
  if (op === 'set_id') {
    body.old_id = Number(params.old_id);
    body.new_id = Number(params.new_id);
  }
  if (op === 'config') {
    body.confirmed = true;
    body.configs = [0,1,2,3].map(i => {
      const cfg = Object.fromEntries(['id','open','close','limit'].map(k => [k, Number(params[k+i])]));
      if (params['speed'+i] !== undefined) cfg.speedSet = Number(params['speed'+i]);
      return cfg;
    });
  }
  pending = true;
  try { return adapt(await request('command', body)); }
  finally { pending = false; }
}
