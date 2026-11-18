import {test} from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
const source = await readFile(new URL('../web/measured_pose.js',import.meta.url),'utf8');
const {measuredPoseView} = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const fixture = () => ({connected:true, joints:{
  shoulder:{verified:true,actual_deg:12.5,feedback_age_s:.01,target_deg:80,setpoint_deg:70},
  elbow:{verified:true,actual_deg:24,feedback_age_s:.02,target_deg:40,setpoint_deg:35},
}});

test('only actual angles drive pose, never target or trajectory reference', () => {
  const f=fixture();
  for (const phase of ['stopped','moving','controlling']) {
    f.control_phase=phase;
    f.joints.shoulder.target_deg=-40;
    f.joints.shoulder.setpoint_deg=60;
    const view=measuredPoseView(f);
    assert.equal(view.joints.shoulder.angle,12.5);
    assert.equal(view.joints.elbow.angle,24);
    assert.equal(view.status,'live');
  }
});
test('feedback updates both joints independently even when stopped or safety latched', () => {
  const f=fixture(); f.control_phase='stopped'; f.safety_latched=true;
  f.joints.shoulder.actual_deg=-30; f.joints.elbow.actual_deg=80;
  const view=measuredPoseView(f);
  assert.equal(view.joints.shoulder.angle,-30);
  assert.equal(view.joints.elbow.angle,80);
  assert.equal(view.status,'live');
});
test('elapsed browser time expires cached samples without another HTTP response', () => {
  const f=fixture();
  assert.equal(measuredPoseView(f,.27).status,'live');
  const view=measuredPoseView(f,.3);
  assert.equal(view.status,'stale');
  assert.equal(view.fresh,false);
  assert.equal(view.joints.shoulder.angle,12.5);
});
test('one stale joint makes the full arm non-real-time', () => {
  const f=fixture(); f.joints.elbow.feedback_age_s=.5;
  const view=measuredPoseView(f);
  assert.equal(view.joints.shoulder.fresh,true);
  assert.equal(view.joints.elbow.fresh,false);
  assert.equal(view.status,'stale');
});
test('offline and service failure cannot label cached feedback live', () => {
  const f=fixture(); f.connected=false;
  assert.equal(measuredPoseView(f).status,'offline');
  f.connected=true;
  const view=measuredPoseView(f,0,false);
  assert.equal(view.status,'unavailable');
  assert.equal(view.fresh,false);
});
test('invalid, missing or unverified telemetry never falls back to zero or target', () => {
  for (const patch of [{actual_deg:null},{actual_deg:NaN},{actual_deg:Infinity},{actual_deg:'20'},
    {verified:false},{feedback_age_s:null},{feedback_age_s:-1},{feedback_age_s:NaN},{feedback_age_s:Infinity}]) {
    const f=fixture(); Object.assign(f.joints.shoulder,patch);
    const view=measuredPoseView(f);
    assert.equal(view.joints.shoulder.angle,null);
    assert.equal(view.status,'waiting');
  }
});
test('a real zero remains a valid measured position', () => {
  const f=fixture(); f.joints.shoulder.actual_deg=0;
  assert.equal(measuredPoseView(f).joints.shoulder.angle,0);
  assert.equal(measuredPoseView(f).status,'live');
});
test('missing state and bad elapsed time never produce live posture', () => {
  assert.equal(measuredPoseView(null).fresh,false);
  for (const elapsed of [NaN,Infinity,-1,null]) assert.equal(measuredPoseView(fixture(),elapsed).fresh,false);
});
test('model integration separates transparent target from live actual and serializes 10Hz polling', async () => {
  const app=await readFile(new URL('../web/app.js',import.meta.url),'utf8');
  const html=await readFile(new URL('../web/index.html',import.meta.url),'utf8');
  assert.match(app,/const previewArm = makeKinematicArm\(true\)/);
  const targetScene=app.slice(app.indexOf('function updateTargetScene'),app.indexOf('function updateMeasuredScene'));
  assert.match(targetScene,/applyPose\(previewArm, preview.shoulder, preview.elbow\)/);
  assert.doesNotMatch(targetScene,/lastMeasuredPose|actual_deg|\/api\//);
  const setter=app.slice(app.indexOf('function setPreview'),app.indexOf('function targetWithinLimits'));
  assert.doesNotMatch(setter,/applyPose|\/api\//);
  assert.match(setter,/updateTargetScene\(\)/);
  assert.match(app,/applyPose\(measuredArm, lastMeasuredPose.shoulder, lastMeasuredPose.elbow\)/);
  assert.match(app,/worldToLocal.*matrixWorld\.clone\(\)\.invert\(\)/);
  assert.match(app,/if \(stateRequest\) return stateRequest/);
  assert.match(app,/setInterval\(\(\) => refreshState\(\), 100\)/);
  assert.match(app,/AbortSignal\.timeout\(2000\)/);
  assert.match(html,/id="poseStatus"/);
  assert.match(html,/透明：目标预览/);
  assert.match(app,/hit.object.userData.targetPose \? previewArm : measuredArm/);
  assert.match(app,/intersectObjects\(visibleDragLinks\(\), false\)/);
});
test('target visibility and hit testing do not depend on actual feedback being available', async () => {
  const app=await readFile(new URL('../web/app.js',import.meta.url),'utf8');
  const targetScene=app.slice(app.indexOf('function updateTargetScene'),app.indexOf('function updateMeasuredScene'));
  assert.match(targetScene,/previewArm.shoulder.visible = joints.every\(key => finite\(preview\[key\]\)\)/);
  assert.doesNotMatch(app,/armRoot.visible = false/);
  assert.match(app,/link.userData.targetPose \? previewArm.shoulder.visible : measuredArm.shoulder.visible/);
  const measuredScene=app.slice(app.indexOf('function updateMeasuredScene'),app.indexOf("ui.frontView.addEventListener"));
  assert.doesNotMatch(measuredScene,/applyPose\(previewArm|previewArm.shoulder.visible/);
});
