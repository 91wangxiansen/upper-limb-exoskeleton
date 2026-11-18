import {readFile} from 'node:fs/promises';
import assert from 'node:assert/strict';
import test from 'node:test';
const source = await readFile(new URL('../web/upper_pose.js', import.meta.url), 'utf8');
const pose = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));
const threeURL = new URL('../web/vendor/three.module.js', import.meta.url);
const THREE = await import(threeURL);
const modelSource = (await readFile(new URL('../web/hand_model.js', import.meta.url), 'utf8'))
  .replace("from 'three'", 'from ' + JSON.stringify(threeURL.href));
const {mountHand} = await import('data:text/javascript;base64,' + Buffer.from(modelSource).toString('base64'));

test('hand feedback expires independently and targets never become measurements', () => {
  pose.setHandTarget(0, 75);
  pose.setHandSnapshot({connected:true, ageMs:100, fingers:[{online:1,q:20},{online:0,q:90}]}, 1000);
  assert.deepEqual(pose.handView(1100).measured, [20,null,null,null]);
  assert.deepEqual(pose.handView(3100).measured, [null,null,null,null]);
  assert.equal(pose.handView(3100).target[0], 75);
  pose.setHandTarget(0, NaN);
  assert.equal(pose.handView().target[0], 75);
});

test('original CAD follows forearm transform and layers flex independently', async () => {
  const previous = globalThis.fetch;
  globalThis.fetch = async url => new Response(await readFile(new URL('../web' + url, import.meta.url)));
  try {
    const actual = new THREE.Group(), target = new THREE.Group();
    const model = await mountHand(actual, target);
    assert.equal(actual.children.length, 1);
    assert.equal(target.children.length, 1);
    const mount = actual.children[0], targetMount = target.children[0];
    const original = mount.getWorldPosition(new THREE.Vector3()).clone();
    actual.rotation.z = Math.PI/2;
    actual.updateMatrixWorld(true);
    const changed = mount.getWorldPosition(new THREE.Vector3());
    assert.ok(changed.distanceTo(original) > 1);
    assert.ok(Math.abs(changed.x - 2.35) < 1e-6);
    model.update({measured:[20,40,null,80], target:[90,70,50,30]});
    assert.equal(mount.children[0].rotation.x, -.2*1.03);
    assert.equal(targetMount.children[0].rotation.x, -.9*1.03);
    assert.equal(mount.children[2].visible, false);
    model.update({measured:[null,null,null,null], target:[0,0,0,0]});
    assert.equal(mount.visible, false);
    assert.equal(targetMount.visible, true);
    assert.ok(model.pickables.length > 0);
  } finally { globalThis.fetch = previous; }
});
