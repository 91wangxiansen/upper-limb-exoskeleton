import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const source = await readFile(new URL('../web/diagnostics.js', import.meta.url), 'utf8');
const {diagnosticView, signedMeter} = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const fresh = value => ({value, age_s:.1, fresh:true});
const fixture = () => ({
  state:{connected:true, motion_active:true, safety_latched:false},
  joint:{phase:'holding', feedback_age_s:.05, error_deg:.51, torque_nm:1.2,
    gravity_torque_nm:.2, load_bias_nm:1,
    sent_control:{age_s:.05, torque_limit_nm:3.5, estimated_total_nm:2.4,
      pd_estimate_nm:1.2, feedforward_nm:1.2, kp:12, kd:1.5},
    diagnostics:{run_mode:fresh(0), limit_torque_nm:fresh(5), limit_cur_a:fresh(6.5), iq_a:fresh(.11), vbus_v:fresh(24)}},
});
const viewOf = ({joint,state}) => diagnosticView(joint,state);

test('signed bars grow outwards from zero and clip to a finite display range', () => {
  assert.deepEqual(signedMeter(1,2), {left:50,width:25,valid:true,overflow:false});
  assert.deepEqual(signedMeter(-1,2), {left:25,width:25,valid:true,overflow:false});
  assert.deepEqual(signedMeter(-3,2), {left:0,width:50,valid:true,overflow:true});
  assert.equal(signedMeter(0,2).width,0);
  for (const value of [null,undefined,NaN,Infinity,'1']) assert.equal(signedMeter(value,2).valid,false);
  for (const limit of [null,undefined,NaN,Infinity,0,-1]) assert.equal(signedMeter(1,limit).valid,false);
});
test('fresh mode zero, feedback and actual sent estimates are shown without double counting PD', () => {
  const view = viewOf(fixture());
  assert.equal(view.mode,'运控模式');
  assert.equal(view.error,.51);
  assert.equal(view.iq,.11);
  assert.equal(view.voltage,24);
  assert.equal(view.limit,3.5);
  assert.equal(view.rows[1].value,2.4);
  assert.equal(view.rows[2].value,1.2);
  assert.equal(view.details.readback,5);
  assert.equal(view.details.kp,12);
});
test('real zero is not confused with missing data', () => {
  const f = fixture(); f.joint.torque_nm=0; f.joint.diagnostics.iq_a=fresh(0);
  const view=viewOf(f);
  assert.equal(view.rows[0].value,0); assert.equal(view.rows[0].valid,true); assert.equal(view.iq,0);
});
test('offline cached feedback, parameters and command values are hidden', () => {
  const f=fixture(); f.state.connected=false; const view=viewOf(f);
  assert.equal(view.phase,'离线'); assert.equal(view.error,null); assert.equal(view.iq,null);
  assert.equal(view.limit,null); assert.ok(view.rows.every(row=>row.value===null));
});
test('invalid or stale feedback ages cannot make old output look fresh', () => {
  for (const age of [null,undefined,-.1,.301,NaN,Infinity]) {
    const f=fixture(); f.joint.feedback_age_s=age; const view=viewOf(f);
    assert.equal(view.error,null); assert.ok(view.rows.every(row=>row.value===null));
    assert.equal(view.phase,'反馈过期'); assert.equal(view.phaseTone,'warn');
  }
});
test('parameter age and explicit freshness are checked independently', () => {
  for (const item of [{value:1,age_s:3.01,fresh:true},{value:1,age_s:null,fresh:true},
    {value:1,age_s:.1,fresh:false},fresh(NaN),fresh(Infinity)]) {
    const f=fixture(); f.joint.diagnostics.iq_a=item; assert.equal(viewOf(f).iq,null);
  }
});
test('stopped or missing/stale commands retain fresh actual feedback but not cached estimates', () => {
  for (const change of [f=>f.joint.phase='stopped', f=>f.state.motion_active=false,
    f=>f.joint.sent_control=null, f=>f.joint.sent_control.age_s=.301]) {
    const f=fixture(); change(f); const view=viewOf(f);
    assert.equal(view.rows[0].value,1.2); assert.equal(view.rows[1].value,null);
    assert.equal(view.rows[2].value,null); assert.equal(view.details.kp,null);
    assert.equal(view.details.gravity,null); assert.equal(view.limit,5);
  }
});
test('faults hide historical sent commands and display the safety state', () => {
  for (const change of [f=>f.state.safety_latched=true,f=>f.joint.fault_bits=1]) {
    const f=fixture(); change(f); const view=viewOf(f);
    assert.equal(view.phase,'安全停止'); assert.equal(view.phaseTone,'danger');
    assert.equal(view.rows[1].value,null);
  }
});
test('near-limit warning describes an estimate, not measured force or an added stop', () => {
  const f=fixture(); f.joint.sent_control.estimated_total_nm=-3.4;
  assert.equal(viewOf(f).commandText,'估算接近上限');
  assert.equal(viewOf(f).phase,'到位保持');
});
test('continuous control shows arrival only as an indicator and keeps sent diagnostics visible', () => {
  const f=fixture(); f.joint.phase='controlling';
  for (const arrived of [false,true]) {
    f.joint.on_target=arrived;
    const view=viewOf(f);
    assert.equal(view.phase, arrived ? '持续控制 · 已到位' : '持续控制 · 有偏差 / 未稳定');
    assert.equal(view.rows[1].value,2.4);
    assert.equal(view.rows[2].value,1.2);
    assert.equal(view.phaseTone,'ok');
  }
});
test('continuous-control arrival never overrides offline, stale or fault status', () => {
  for (const [change,expected] of [[f=>f.state.connected=false,'离线'],
    [f=>f.joint.feedback_age_s=.31,'反馈过期'],[f=>f.state.safety_latched=true,'安全停止']]) {
    const f=fixture(); f.joint.phase='controlling'; f.joint.on_target=true;
    change(f);
    assert.equal(viewOf(f).phase,expected);
    assert.equal(viewOf(f).rows[1].value,null);
  }
});
test('missing limits do not silently use configured defaults as a measurement range', () => {
  const f=fixture(); f.joint.sent_control.torque_limit_nm=null; f.joint.diagnostics.limit_torque_nm=fresh(null);
  const view=viewOf(f); assert.equal(view.limit,null);
  assert.equal(view.rows[0].value,1.2); assert.equal(view.rows[0].valid,false);
});
test('gravity editor is removed, both joint diagnostics are mounted, backend controls remain untouched', async () => {
  const html=await readFile(new URL('../web/index.html',import.meta.url),'utf8');
  const app=await readFile(new URL('../web/app.js',import.meta.url),'utf8');
  assert.equal((html.match(/data-field="diagnostics"/g)||[]).length,2);
  assert.doesNotMatch(html,/id="gravity(?:Calibrate|Disable|Status|Model)"/);
  assert.doesNotMatch(app,/ui\.gravity|\/api\/gravity/);
  assert.match(app,/diagnosticPanels\[key\]\(joint,\s*next\)/);
  assert.match(app,/next\.gravity\?\.enabled/);
});
