import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';
import { mountDiagnostics } from '/diagnostics.js?v=continuous-control';
import { measuredPoseView } from '/measured_pose.js?v=live-position';
import { mountHand } from '/hand_model.js';
import { handView, setHandTarget } from '/upper_pose.js';
import {bindNumericBounds,setRange,validInputs} from '/input_bounds.js';
import {chartView,mergeHistory} from '/chart_data.js';

const $ = id => document.getElementById(id);
const joints = ['shoulder', 'elbow'];
const colors = {shoulder: '#ffb84d', elbow: '#24d6e7'};
const ui = Object.fromEntries([
  'connectionPill', 'clock', 'stopButton', 'viewport', 'frontView', 'dragTip', 'poseStatus', 'portSelect',
  'refreshPorts', 'connectButton', 'discoverButton', 'resetSafetyButton',
  'shoulderPoseFooter', 'elbowPoseFooter', 'identityFooter', 'controlFooter', 'deviceSetup',
  'syncMoveButton', 'prepareAllButton', 'prepareGuide', 'statusText', 'eventLog', 'modal', 'modalTitle', 'modalText',
  'modalCancel', 'modalConfirm', 'toast',
  'loadStatus', 'loadForm', 'loadMode', 'loadModeHint', 'loadSave', 'loadRevert', 'loadSaveHint',
  'datasetStatus', 'datasetStart', 'datasetFinish', 'datasetDownload', 'datasetPath'
].map(id => [id, $(id)]));
const cards = Object.fromEntries(joints.map(key => [key, document.querySelector(`[data-joint="${key}"]`)]));
const field = (key, name) => cards[key].querySelector(`[data-field="${name}"]`);
const input = (key, name) => cards[key].querySelector(`[data-input="${name}"]`);
const diagnosticPanels = Object.fromEntries(joints.map(key => [key, mountDiagnostics(field(key, 'diagnostics'))]));
let state = null;
let stateReceivedAt = null;
let stateRequest = null;
let statePollForce = false;
let serviceAvailable = false;
const lastMeasuredPose = {shoulder:null, elbow:null};
let history = {shoulder: {}, elbow: {}};
let modalAction = null;
let toastTimer = null;
const configSignatures = {};
const configDirty = {shoulder:false, elbow:false};
const preview = {shoulder: null, elbow: null};
const loadInputs = [...document.querySelectorAll('[data-load]')];
bindNumericBounds(document);
for (const el of loadInputs) setRange(el,Number(el.min),Number(el.max));
let loadDirty = false;
let loadRevision = 0;
let loadSignature = null;

const finite = value => typeof value === 'number' && Number.isFinite(value);
const fmt = (value, digits = 1, suffix = '') => finite(value) ? `${value.toFixed(digits)}${suffix}` : '--';
const escapeHtml = value => String(value).replace(/[&<>"']/g, ch => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));

function toast(message, error = false) {
  clearTimeout(toastTimer);
  ui.toast.textContent = message;
  ui.toast.className = `toast show${error ? ' error' : ''}`;
  toastTimer = setTimeout(() => ui.toast.className = 'toast', 3500);
}

async function api(path, body = null, requestOptions = {}) {
  const options = body === null ? {...requestOptions} : {...requestOptions, method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)};
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok || data.ok === false) throw new Error(data.error || `HTTP ${response.status}`);
  return data;
}

function showConfirm(title, message, action, confirmText = '确认执行') {
  ui.modalTitle.textContent = title;
  ui.modalText.textContent = message;
  ui.modalConfirm.textContent = confirmText;
  modalAction = action;
  ui.modal.classList.remove('hidden');
}
function closeModal() { ui.modal.classList.add('hidden'); modalAction = null; }
ui.modalCancel.addEventListener('click', closeModal);
ui.modal.addEventListener('click', event => { if (event.target === ui.modal) closeModal(); });
ui.modalConfirm.addEventListener('click', async () => {
  const action = modalAction;
  closeModal();
  if (!action) return;
  try { await action(); await refreshState(); } catch (error) { toast(error.message, true); }
});

// Solid arm = measured feedback; transparent arm = independently editable target.
const scene = new THREE.Scene();
scene.fog = new THREE.FogExp2(0x071017, 0.05);
const camera = new THREE.PerspectiveCamera(38, 1, 0.1, 100);
camera.position.set(4.4, 2.8, 12);
const renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
renderer.outputColorSpace = THREE.SRGBColorSpace;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.15;
ui.viewport.prepend(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
orbit.enableDamping = true;
orbit.dampingFactor = 0.07;
orbit.enablePan = false;
orbit.minDistance = 5;
orbit.maxDistance = 16;
orbit.target.set(0, -.35, 0);
scene.add(new THREE.HemisphereLight(0xa9edff, 0x071017, 2.1));
const keyLight = new THREE.DirectionalLight(0x7fe8ff, 4.2);
keyLight.position.set(4, 7, 6); scene.add(keyLight);
const rimLight = new THREE.DirectionalLight(0xffb84d, 2.2);
rimLight.position.set(-5, 2, -4); scene.add(rimLight);
const grid = new THREE.GridHelper(14, 28, 0x1c7180, 0x183642);
grid.position.y = -3.1; grid.material.opacity = 0.3; grid.material.transparent = true; scene.add(grid);
const steel = new THREE.MeshStandardMaterial({color:0x91a8b2, metalness:0.76, roughness:0.28});
const dark = new THREE.MeshStandardMaterial({color:0x173442, metalness:0.7, roughness:0.3});
const shoulderAccent = new THREE.MeshStandardMaterial({color:0xffb84d, emissive:0x70400d, emissiveIntensity:0.45});
const elbowAccent = new THREE.MeshStandardMaterial({color:0x24d6e7, emissive:0x0a5260, emissiveIntensity:0.7});
const ghostMaterial = new THREE.MeshBasicMaterial({color:0xffd79b, transparent:true,
  opacity:.18, depthWrite:false, depthTest:true});

function disc(material, radius = 0.43) {
  const mesh = new THREE.Mesh(new THREE.CylinderGeometry(radius, radius, 0.36, 32), material);
  mesh.rotation.x = Math.PI / 2;
  return mesh;
}
function beam(length, radius, material) {
  return new THREE.Mesh(new THREE.CapsuleGeometry(radius, length - radius * 2, 8, 18), material);
}
const armRoot = new THREE.Group(); armRoot.rotation.y = -0.1; scene.add(armRoot);
const base = disc(dark); base.position.y = 2.65; armRoot.add(base);
const baseRing = new THREE.Mesh(new THREE.TorusGeometry(0.45, 0.04, 10, 40), shoulderAccent);
baseRing.position.y = 2.65; armRoot.add(baseRing);

function makeKinematicArm(target = false) {
  const shoulder = new THREE.Group(); shoulder.position.y = 2.65; armRoot.add(shoulder);
  const material = target ? ghostMaterial : steel;
  const upper = beam(2.65, target ? .30 : .28, material); upper.position.y = -1.325; shoulder.add(upper);
  if (!target) {
    for (const y of [-0.55, -1.05, -1.65, -2.15]) {
      const detail = new THREE.Mesh(new THREE.BoxGeometry(0.68, 0.055, 0.46), dark);
      detail.position.y = y; shoulder.add(detail);
    }
  }
  const elbow = new THREE.Group(); elbow.position.y = -2.65; shoulder.add(elbow);
  if (!target) {
    elbow.add(disc(dark));
    const ring = new THREE.Mesh(new THREE.TorusGeometry(0.45, 0.045, 10, 48), elbowAccent);
    ring.position.z = 0.22; elbow.add(ring);
  }
  const forearm = beam(2.35, target ? .26 : .24, material); forearm.position.y = -1.175; elbow.add(forearm);
  if (!target) {
    for (const y of [-0.72, -1.66]) {
      const cuff = new THREE.Mesh(new THREE.TorusGeometry(0.34, 0.06, 10, 36), dark);
      cuff.rotation.x = Math.PI / 2; cuff.position.y = y; elbow.add(cuff);
    }
  }
  upper.userData.dragJoint = 'shoulder';
  forearm.userData.dragJoint = 'elbow';
  for (const mesh of [upper, forearm]) {
    mesh.userData.targetPose = target;
    if (target) mesh.renderOrder = 2;
  }
  return {shoulder, elbow, upper, forearm};
}
const measuredArm = makeKinematicArm();
const previewArm = makeKinematicArm(true);
let handModel = null;
mountHand(measuredArm.elbow, previewArm.elbow).then(model => {
  handModel = model;
  document.getElementById('handModelStatus').textContent = 'CAD 已装配 · 指节角度为耦合近似';
}).catch(error => {
  document.getElementById('handModelStatus').textContent = error.message;
});
measuredArm.shoulder.visible = previewArm.shoulder.visible = base.visible = baseRing.visible = false;
const draggableLinks = [previewArm.upper, previewArm.forearm, measuredArm.upper, measuredArm.forearm];
const visibleDragLinks = () => draggableLinks.filter(link =>
  link.userData.targetPose ? previewArm.shoulder.visible : measuredArm.shoulder.visible);
function applyPose(arm, shoulderDeg, elbowDeg) {
  arm.shoulder.rotation.z = THREE.MathUtils.degToRad(shoulderDeg);
  arm.elbow.rotation.z = THREE.MathUtils.degToRad(elbowDeg);
}
function updateTargetScene() {
  previewArm.shoulder.visible = joints.every(key => finite(preview[key]));
  if (previewArm.shoulder.visible) applyPose(previewArm, preview.shoulder, preview.elbow);
}
function updateMeasuredScene() {
  const elapsed = stateReceivedAt === null ? Infinity : (performance.now()-stateReceivedAt)/1000;
  const view = measuredPoseView(state, elapsed, serviceAvailable);
  for (const key of joints) {
    const reading = view.joints[key];
    if (view.connected && reading.angle !== null) lastMeasuredPose[key] = reading.angle;
    const actual = reading.fresh ? fmt(reading.angle, 1, '°') : finite(lastMeasuredPose[key]) ? `${fmt(lastMeasuredPose[key],1,'°')}（旧）` : '--';
    const footer = `${actual} / ${fmt(preview[key],1,'°')}`;
    if (ui[`${key}PoseFooter`].textContent !== footer) ui[`${key}PoseFooter`].textContent = footer;
    const number = reading.fresh ? fmt(reading.angle) : '--';
    if (field(key,'actual').textContent !== number) field(key,'actual').textContent = number;
    const age = `${reading.fresh ? '反馈' : '非实时 · 反馈'} ${fmt(reading.age,2,'s')}`;
    if (field(key,'age').textContent !== age) field(key,'age').textContent = age;
  }
  measuredArm.shoulder.visible = base.visible = baseRing.visible = joints.every(key => finite(lastMeasuredPose[key]));
  if (measuredArm.shoulder.visible) applyPose(measuredArm, lastMeasuredPose.shoulder, lastMeasuredPose.elbow);
  if (ui.poseStatus.textContent !== view.text) ui.poseStatus.textContent = view.text;
  if (ui.viewport.dataset.poseStatus !== view.status) ui.viewport.dataset.poseStatus = view.status;
}
ui.frontView.addEventListener('click', () => {
  camera.position.set(4.4, 2.8, 12);
  orbit.target.set(0, -.35, 0);
  orbit.update();
});
function resizeScene() {
  const width = ui.viewport.clientWidth, height = ui.viewport.clientHeight;
  if (!width || !height) return;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height, false);
}
new ResizeObserver(resizeScene).observe(ui.viewport);
function animate() {
  updateMeasuredScene();
  if (handModel) handModel.update(handView());
  orbit.update();
  renderer.render(scene, camera);
  requestAnimationFrame(animate);
}
animate();

// Dragging the target hand only edits the preview. Sending is an explicit action.
let fingerDrag = null;
const handRay = new THREE.Raycaster();
renderer.domElement.addEventListener('pointerdown', event => {
  if (!handModel || event.button !== 0) return;
  const rect = renderer.domElement.getBoundingClientRect();
  handRay.setFromCamera({x:(event.clientX-rect.left)/rect.width*2-1,
    y:1-(event.clientY-rect.top)/rect.height*2}, camera);
  const meshes = handModel.navigationMeshes.filter(mesh => {
    for (let node=mesh; node; node=node.parent) if (!node.visible) return false;
    return true;
  });
  const hit = handRay.intersectObjects(meshes, false)[0];
  if (!hit || !hit.object.visible) return;
  const index = hit.object.userData.finger;
  fingerDrag = {index, x:event.clientX, y:event.clientY, start:handView().target[index], moved:false};
  orbit.enabled = false;
  renderer.domElement.setPointerCapture(event.pointerId);
  event.stopImmediatePropagation();
}, true);
renderer.domElement.addEventListener('pointermove', event => {
  if (!fingerDrag) return;
  fingerDrag.moved ||= Math.hypot(event.clientX-fingerDrag.x,event.clientY-fingerDrag.y)>6;
  if (fingerDrag.moved && fingerDrag.index >= 0) {
    setHandTarget(fingerDrag.index, fingerDrag.start + (event.clientY-fingerDrag.y)*.65);
    window.dispatchEvent(new Event('hand-target-change'));
  }
  event.stopImmediatePropagation();
}, true);
function finishFingerDrag(event) {
  if (!fingerDrag) return;
  const enter = !fingerDrag.moved && event.type === 'pointerup';
  fingerDrag = null; orbit.enabled = true;
  if (renderer.domElement.hasPointerCapture(event.pointerId)) renderer.domElement.releasePointerCapture(event.pointerId);
  event.stopImmediatePropagation();
  if (enter) window.dispatchEvent(new Event('open-hand-control'));
}
renderer.domElement.addEventListener('pointerup', finishFingerDrag, true);
renderer.domElement.addEventListener('pointercancel', finishFingerDrag, true);

function setPreview(key, raw, syncInputs = true) {
  const joint = state?.joints[key];
  if (!joint) return;
  const min = joint.limits.minimum_angle_deg, max = joint.limits.maximum_angle_deg;
  const value = Number(raw);
  if (!Number.isFinite(value)) return;
  preview[key] = Math.max(min, Math.min(max, value));
  if (syncInputs) {
    input(key, 'target').value = String(preview[key]);
    input(key, 'targetNumber').value = preview[key].toFixed(1);
  }
  field(key, 'target').textContent = `${preview[key].toFixed(1)}°`;
  field(key, 'target').value = `${preview[key].toFixed(1)}°`;
  updateTargetScene();
  updateMeasuredScene();
  if (!targetWithinLimits(key)) {
    cards[key].querySelector('[data-action="move"]').disabled = true;
    ui.syncMoveButton.disabled = true;
  }
}
function jointInputRanges(key, joint) {
  for (const [name,lo,hi] of [
    ['target',joint.limits.minimum_angle_deg,joint.limits.maximum_angle_deg],
    ['targetNumber',joint.limits.minimum_angle_deg,joint.limits.maximum_angle_deg],
    ['speed',.02,joint.limits.maximum_speed_rad_s], ['torque',.1,joint.limits.maximum_torque_nm],
    ['maxSpeed',.02,.30], ['maxTorque',.1,5], ['kp',0,20], ['kd',0,5]
  ]) setRange(input(key,name),lo,hi);
  const minimum=input(key,'minimum'), maximum=input(key,'maximum');
  minimum.dataset.exclusiveMax='true';maximum.dataset.exclusiveMin='true';
  setRange(minimum,null,maximum.value==='' ? joint.limits.maximum_angle_deg : Number(maximum.value));
  setRange(maximum,minimum.value==='' ? joint.limits.minimum_angle_deg : Number(minimum.value),null);
}

function targetWithinLimits(key, snapshot = state) {
  const joint = snapshot?.joints[key];
  return joint && finite(preview[key]) && preview[key] >= joint.limits.minimum_angle_deg && preview[key] <= joint.limits.maximum_angle_deg;
}

// Dragging either arm changes only the transparent target, never measured feedback.
const dragRaycaster = new THREE.Raycaster();
const dragPointer = new THREE.Vector2();
let modelDrag = null;
function rayFromPointer(event) {
  const rect = renderer.domElement.getBoundingClientRect();
  dragPointer.set((event.clientX - rect.left) / rect.width * 2 - 1, 1 - (event.clientY - rect.top) / rect.height * 2);
  dragRaycaster.setFromCamera(dragPointer, camera);
}
function pointerAngle(event, joint, plane, worldToLocal) {
  rayFromPointer(event);
  const world = new THREE.Vector3();
  if (!dragRaycaster.ray.intersectPlane(plane, world)) return null;
  const local = world.applyMatrix4(worldToLocal);
  const x = local.x, y = joint === 'shoulder' ? local.y - 2.65 : local.y + 2.65;
  if (x * x + y * y < 0.04) return null;
  return THREE.MathUtils.radToDeg(Math.atan2(x, -y));
}
function finishModelDrag(event) {
  if (!modelDrag || event.pointerId !== modelDrag.pointerId) return;
  modelDrag = null;
  orbit.enabled = true;
  ui.viewport.classList.remove('dragging');
  renderer.domElement.style.cursor = '';
  ui.dragTip.textContent = '拖动透明臂设目标 · 实体臂实时反馈\n点击控制按钮才执行';
  if (renderer.domElement.hasPointerCapture(event.pointerId)) renderer.domElement.releasePointerCapture(event.pointerId);
  event.stopImmediatePropagation();
}
renderer.domElement.addEventListener('pointerdown', event => {
  if (event.button !== 0 || !state) return;
  armRoot.updateWorldMatrix(true, true);
  rayFromPointer(event);
  const hit = dragRaycaster.intersectObjects(visibleDragLinks(), false)[0];
  if (!hit) return;
  const key = hit.object.userData.dragJoint;
  const dragArm = hit.object.userData.targetPose ? previewArm : measuredArm;
  const pivot = (key === 'shoulder' ? dragArm.shoulder : dragArm.elbow).getWorldPosition(new THREE.Vector3());
  const normal = new THREE.Vector3(0, 0, 1).transformDirection(armRoot.matrixWorld);
  const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, pivot);
  // Freeze the drag frame while measured feedback keeps moving the arm.
  const worldToLocal = (key === 'shoulder' ? armRoot : dragArm.shoulder).matrixWorld.clone().invert();
  const initialAngle = pointerAngle(event, key, plane, worldToLocal);
  if (initialAngle === null || !finite(preview[key])) return;
  modelDrag = {pointerId:event.pointerId, key, plane, worldToLocal, initialAngle, initialPreview:preview[key]};
  orbit.enabled = false;
  renderer.domElement.setPointerCapture(event.pointerId);
  ui.viewport.classList.add('dragging');
  event.preventDefault();
  event.stopImmediatePropagation();
}, true);
renderer.domElement.addEventListener('pointermove', event => {
  if (!modelDrag) {
    if (state) {
      rayFromPointer(event);
      renderer.domElement.style.cursor = dragRaycaster.intersectObjects(visibleDragLinks(), false).length ? 'crosshair' : '';
    }
    return;
  }
  if (event.pointerId !== modelDrag.pointerId) return;
  const angle = pointerAngle(event, modelDrag.key, modelDrag.plane, modelDrag.worldToLocal);
  if (angle !== null) {
    const delta = ((angle - modelDrag.initialAngle + 540) % 360) - 180;
    setPreview(modelDrag.key, modelDrag.initialPreview + delta);
    ui.dragTip.textContent = `${state.joints[modelDrag.key].display_name}目标 ${fmt(preview[modelDrag.key], 1, '°')}\n透明臂仅预览 · 实体臂仍为实际位置`;
  }
  event.preventDefault();
  event.stopImmediatePropagation();
}, true);
renderer.domElement.addEventListener('pointerup', finishModelDrag, true);
renderer.domElement.addEventListener('pointercancel', finishModelDrag, true);

function motionPayload(keys) {
  const payload = {targets_deg:{}, speeds_rad_s:{}, torque_limits_nm:{}};
  for (const key of keys) {
    payload.targets_deg[key] = preview[key];
    payload.speeds_rad_s[key] = Number(input(key, 'speed').value);
    payload.torque_limits_nm[key] = Number(input(key, 'torque').value);
  }
  return payload;
}
function confirmMove(keys) {
  if (!validInputs(keys.flatMap(key=>['targetNumber','speed','torque'].map(name=>input(key,name))))) return;
  if (!state?.connected) { toast('请先连接 CANable', true); return; }
  const invalid = keys.find(key => !targetWithinLimits(key));
  if (invalid) { toast(`${state.joints[invalid].display_name}目标超出软限位`, true); return; }
  const description = keys.map(key => {
    const name = state.joints[key].display_name;
    return `${name} ${fmt(preview[key], 1, '°')}，速度 ${input(key,'speed').value} rad/s，转矩限制 ${input(key,'torque').value} Nm`;
  }).join('；');
  // An explicit retarget click is sufficient within an already verified hold.
  // Preview/dragging still never sends motion; backend revalidates every request.
  if (state.motion_active) {
    api('/api/move', motionPayload(keys)).then(async result => {
      toast(`新目标已提交，预计 ${Number(result.result).toFixed(1)} 秒`);
      await refreshState();
    }).catch(error => toast(error.message, true));
    return;
  }
  showConfirm(
    keys.length === 2 ? '确认双关节协调运动' : '确认单关节运动',
    `${description}。${state.load_compensation?.enabled && state.load_compensation.strategy === 'tracking_support' ? `当前承托档：Kp=${state.load_compensation.position_kp}、Kd=${state.load_compensation.velocity_kd}、Ki=${state.load_compensation.ki_nm_per_rad_s}，出力强于原档；首次须低速小幅验证。` : ''}实际到位后持续保持，直到手动停止或触发保护。停止/断电会释放支撑力，请先支撑机构。`,
    async () => { const result = await api('/api/move', motionPayload(keys)); toast(`轨迹已启动，预计 ${Number(result.result).toFixed(1)} 秒`); }
  );
}

function collectSafetyConfig(key) {
  return {
    direction:Number(input(key,'direction').value),
    minimum_angle_deg:Number(input(key,'minimum').value),
    maximum_angle_deg:Number(input(key,'maximum').value),
    maximum_speed_rad_s:Number(input(key,'maxSpeed').value),
    maximum_torque_nm:Number(input(key,'maxTorque').value),
    motion_kp:Number(input(key,'kp').value),
    motion_kd:Number(input(key,'kd').value)
  };
}

function confirmPrepare(keys) {
  if (!validInputs(keys.flatMap(key=>['minimum','maximum','maxSpeed','maxTorque','kp','kd'].map(name=>input(key,name))))) return;
  if (!state?.connected) { toast('请先连接并识别双机', true); return; }
  const configs = Object.fromEntries(keys.map(key => [key, collectSafetyConfig(key)]));
  const description = keys.map(key => {
    const item = configs[key];
    return `${state.joints[key].display_name}：方向 ${item.direction > 0 ? '+1' : '-1'}，范围 ${item.minimum_angle_deg}°～${item.maximum_angle_deg}°，速度上限 ${item.maximum_speed_rad_s} rad/s，转矩上限 ${item.maximum_torque_nm} Nm，Kp ${item.motion_kp}，Kd ${item.motion_kd}`;
  }).join('；');
  showConfirm('确认方向、限位并启用',
    `${description}。请确认实际机械空间与正方向已核实。本操作会先停止双机、读取最新反馈并保存配置；不会立即运动。`,
    async () => {
      await api('/api/prepare', {configs, confirmed:true});
      for (const key of keys) configDirty[key] = false;
      toast('安全配置已确认，先以小角度、低速试运行');
      await refreshState(true);
    }, '确认并启用');
}

for (const key of joints) {
  input(key, 'target').addEventListener('input', event => setPreview(key, event.target.value));
  input(key, 'targetNumber').addEventListener('input', event => {
    if (event.target.value === '') return;
    setPreview(key, event.target.value);
    input(key, 'target').value = String(preview[key]);
  });
  input(key, 'targetNumber').addEventListener('change', event => setPreview(key, event.target.value));
  cards[key].querySelector('[data-action="move"]').addEventListener('click', () => confirmMove([key]));
  cards[key].querySelector('[data-action="zero"]').addEventListener('click', () => {
    showConfirm(`采集${state?.joints[key]?.display_name || key}软件零点`,
      '系统会先停止两台电机，再采用该关节停止后的新鲜位置反馈。采零后本关节验收会取消。',
      async () => { await api('/api/zero', {joint:key}); toast('零点已保存，请确认并启用'); }, '停止并采零');
  });
  cards[key].querySelector('[data-action="prepare"]').addEventListener('click', () => confirmPrepare([key]));
  for (const name of ['direction','minimum','maximum','maxSpeed','maxTorque','kp','kd']) {
    input(key, name).addEventListener('input', () => { configDirty[key] = true; if (state) renderState(state); });
  }
}
ui.prepareAllButton.addEventListener('click', () => confirmPrepare(joints));
ui.syncMoveButton.addEventListener('click', () => confirmMove(joints));
ui.stopButton.addEventListener('click', async () => {
  try {
    const result = await api('/api/stop', {});
    const failures = Object.entries(result.result).filter(([_key, value]) => value !== 'sent');
    toast(failures.length ? '停止发送结果异常，请检查实体急停与通信' : '两台电机均已发送停止帧', failures.length > 0);
    await refreshState();
  } catch (error) { toast(error.message, true); }
});
ui.refreshPorts.addEventListener('click', () => refreshState(true));
ui.connectButton.addEventListener('click', async () => {
  ui.connectButton.disabled = true;
  try {
    if (state?.connected) await api('/api/disconnect', {});
    else await api('/api/connect', {port:ui.portSelect.value});
    await refreshState(true);
  } catch (error) { toast(error.message, true); }
  finally { ui.connectButton.disabled = false; }
});
ui.discoverButton.addEventListener('click', async () => {
  try { await api('/api/discover', {}); await refreshState(); toast('两台电机 UID 识别完成'); }
  catch (error) { toast(error.message, true); }
});
ui.resetSafetyButton.addEventListener('click', async () => {
  try { await api('/api/reset-safety', {}); await refreshState(); toast('安全锁存已复位'); }
  catch (error) { toast(error.message, true); }
});

for (const [button, path] of [[ui.datasetStart, '/api/dataset/start'], [ui.datasetFinish, '/api/dataset/finish']]) {
  button.addEventListener('click', async () => {
    button.disabled = true;
    try { renderState((await api(path, {})).state); }
    catch (error) { toast(error.message, true); }
    finally { await refreshState(); }
  });
}

function fillLoadForm(load) {
  ui.loadMode.value = load.enabled ? load.strategy : 'off';
  for (const el of loadInputs) el.value = String(load[el.dataset.load]);
  loadSignature = JSON.stringify(load);
}
for (const el of [ui.loadMode, ...loadInputs]) {
  el.addEventListener('input', () => {
    loadDirty = true;
    loadRevision++;
    if (state) renderState(state);
  });
}
ui.loadRevert.addEventListener('click', () => {
  if (!state?.load_compensation) return;
  loadDirty = false;
  loadRevision++;
  fillLoadForm(state.load_compensation);
  renderState(state);
});
ui.loadForm.addEventListener('submit', event => {
  event.preventDefault();
  if (ui.loadSave.disabled || !validInputs(loadInputs) || !ui.loadForm.reportValidity()) return;
  const enabled = ui.loadMode.value !== 'off';
  // Off preserves the stored strategy and its tuning for the next re-enable.
  const parameters = {strategy: enabled ? ui.loadMode.value : state.load_compensation.strategy};
  for (const el of loadInputs) parameters[el.dataset.load] = Number(el.value);
  const revision = loadRevision;
  const body = {enabled, confirmed:enabled, parameters};
  const gains = parameters.strategy === 'tracking_support' && enabled
    ? `Kp=${parameters.position_kp}、Kd=${parameters.velocity_kd}`
    : '使用上方两关节各自的 Kp/Kd';
  showConfirm('保存运控模式与参数（不会运动）',
    `${ui.loadMode.selectedOptions[0].textContent}；${gains}。Ki=${parameters.ki_nm_per_rad_s}，附加补偿上限 ±${parameters.maximum_bias_nm} Nm，变化率 ${parameters.maximum_rate_nm_s} Nm/s。空载重力模型、标定、目标角度及总转矩上限保持不变。仅保存设置，下次手动运动时生效。`,
    async () => {
      await api('/api/load-compensation', body);
      if (revision === loadRevision) loadDirty = false;
      toast('运控模式与参数已保存，不会自动运动');
    }, '确认保存');
});

function renderState(next, force = false) {
  const first = !state;
  const previousSafety = state?.safety_latched || joints.some(key => state?.joints[key]?.fault_bits);
  state = next;
  const dataset = next.dataset;
  const rows = Object.values(dataset?.counts || {}).reduce((sum,value) => sum+value, 0);
  ui.datasetStatus.textContent = !dataset?.supported ? '记录功能暂不可用' : dataset.error ? `记录异常：${dataset.error}` :
    `${({idle:'未开始记录',starting:'正在准备',recording:'正在记录',finalizing:'正在整理',finished:'记录已完成',error:'记录失败'})[dataset.state] || '未知状态'} · ${dataset.counts?.feedback || 0} 帧反馈 / ${rows} 条`;
  ui.datasetStart.disabled = !dataset?.supported || !next.connected || dataset.busy;
  ui.datasetFinish.disabled = !dataset?.recording;
  const download = dataset?.download_ready && /^[0-9a-f]{32}$/.test(dataset.session_id);
  ui.datasetDownload.setAttribute('aria-disabled', String(!download));
  if (download) ui.datasetDownload.href = `/api/dataset/download/${dataset.session_id}`;
  else ui.datasetDownload.removeAttribute('href');
  ui.datasetPath.textContent = dataset?.directory ? `本地保存：${dataset.directory}` : '';
  ui.datasetPath.title = dataset?.directory || '';
  const allVerified = joints.every(key => next.joints[key].verified);
  const supportsHold = next.version === '2.3.3';
  const phaseText = {stopped:'已停止',moving:'轨迹运动中',controlling:'持续位置控制',settling:'到位确认中',holding:'到位保持',recovering:'扰动恢复中',limited:'小误差告警 · 未到位'};
  const canRetarget = !next.motion_active || ['controlling','holding','limited'].includes(next.control_phase);
  const readyToMove = {};
  const anyFault = joints.some(key => next.joints[key].fault_bits);
  const pill = ui.connectionPill;
  if (next.safety_latched || anyFault) { pill.className='pill fault'; pill.querySelector('b').textContent='安全停止'; }
  else if (next.connected && allVerified && joints.every(key => next.joints[key].commissioned && !configDirty[key])) { pill.className='pill online'; pill.querySelector('b').textContent='双机可用'; }
  else if (next.connected && allVerified) { pill.className='pill waiting'; pill.querySelector('b').textContent='待确认设置'; }
  else if (next.connected) { pill.className='pill waiting'; pill.querySelector('b').textContent='识别未完成'; }
  else { pill.className='pill offline'; pill.querySelector('b').textContent='离线'; }
  ui.connectButton.textContent = next.connected ? '断开并停止双机' : '连接并识别';
  ui.discoverButton.disabled = !next.connected;
  ui.resetSafetyButton.disabled = !next.connected || !next.safety_latched;
  ui.statusText.textContent = next.safety_latched ? `安全锁存：${next.safety_reason}` : next.status;
  const notices = joints.filter(key => next.joints[key].notice).map(key => `${next.joints[key].display_name}：${next.joints[key].notice}`);
  if (!next.safety_latched && notices.length) ui.statusText.textContent = notices.join('；');
  ui.identityFooter.textContent = joints.map(key => `${next.joints[key].motor_can_id} ${next.joints[key].verified ? '通过' : '未验证'}`).join(' · ');
  ui.controlFooter.textContent = next.motion_active ? `${phaseText[next.control_phase] || '控制中'} · 安全监控中` : allVerified && joints.every(key => next.joints[key].commissioned && !configDirty[key]) ? '已就绪，电机停止' : '待确认安全设置';
  const load = next.load_compensation;
  ui.loadStatus.textContent = load?.enabled ? `已保存：${load.strategy === 'tracking_support' ? `承托档 · Kp=${load.position_kp}，Kd=${load.velocity_kd}，Ki=${load.ki_nm_per_rad_s}` : '静态补偿'} · 附加力矩 ≤${load.maximum_bias_nm} Nm，变化率 ≤${load.maximum_rate_nm_s} Nm/s` : '已保存：原 PD 控制 · 负载补偿关闭，承托参数仍保留';
  if (load && !loadDirty && (force || first || loadSignature !== JSON.stringify(load))) fillLoadForm(load);
  const deadband = loadInputs.find(el=>el.dataset.load==='deadband_deg');
  const tolerance = next.motion_policy?.arrival_tolerance_deg ?? 1.5;
  if (ui.loadMode.value!=='off' && tolerance<=1) deadband.dataset.exclusiveMax='true';
  else delete deadband.dataset.exclusiveMax;
  setRange(deadband,.05,ui.loadMode.value==='off' ? 1 : Math.min(1,tolerance));
  ui.loadModeHint.textContent = ui.loadMode.value === 'tracking_support'
    ? '承托档使用下方 Kp/Kd；按轨迹当前角度误差学习补偿，换目标保留承托。'
    : '此模式使用上方两关节各自的 Kp/Kd。下方承托档 Kp/Kd 仅保存，不参与当前控制。';
  const saveBlocked = next.motion_active ? '请先支撑机构并停止双机，再保存' : !next.connected ? '请先连接双机再保存' : !allVerified ? '请先完成双机身份识别' : next.safety_latched || anyFault ? '请先排除故障并复位安全锁存' : '';
  ui.loadSave.disabled = !supportsHold || !loadDirty || !!saveBlocked;
  ui.loadSave.title = saveBlocked || (loadDirty ? '仅保存设置，下次手动运动时生效' : '没有未保存的修改');
  ui.loadRevert.disabled = !loadDirty;
  ui.loadSaveHint.textContent = `${loadDirty ? '有未保存的修改；当前控制仍按已保存参数。' : '已载入现有参数。'}${saveBlocked || '修改后点保存，不会自动运动。'}`;
  const missingZero = joints.filter(key => !finite(next.joints[key].software_zero_rad));
  ui.prepareGuide.textContent = !next.connected ? '第 1 步：连接并识别双机。' : !allVerified ? '请先完成两台电机的 UID 识别。' : missingZero.length ? `请先为${missingZero.map(key => next.joints[key].display_name).join('、')}采零。` : '核对两关节的方向和安全范围，然后点下方按钮一次启用。';
  ui.prepareAllButton.disabled = !next.connected || !allVerified || missingZero.length > 0 || next.safety_latched || anyFault;
  ui.prepareAllButton.title = ui.prepareAllButton.disabled ? ui.prepareGuide.textContent : '停止双机、检查最新反馈并保存安全配置';
  ui.prepareAllButton.textContent = joints.every(key => next.joints[key].commissioned && !configDirty[key]) ? '重新确认双关节设置' : '确认并启用双关节';
  // Expose recovery controls when a fault first arrives; polling must not reset a user's folds.
  if ((next.safety_latched || anyFault) && !previousSafety) ui.deviceSetup.open = true;
  ui.deviceSetup.dataset.attention = String(next.connected && (!allVerified || missingZero.length > 0 || next.safety_latched || anyFault || joints.some(key => !next.joints[key].commissioned || configDirty[key])));
  const ports = [...new Set([next.port, ...next.ports].filter(Boolean))];
  if (force || first || ports.join('|') !== [...ui.portSelect.options].map(item => item.value).join('|')) {
    const selected = ui.portSelect.value;
    ui.portSelect.replaceChildren(...ports.map(port => new Option(port, port)));
    ui.portSelect.value = ports.includes(selected) ? selected : next.port;
  }

  for (const key of joints) {
    const joint = next.joints[key];
    const card = cards[key];
    const signature = JSON.stringify([joint.direction, joint.limits, joint.control, joint.limits_confirmed]);
    const refreshInputs = (first || force || configSignatures[key] !== signature) && !configDirty[key];
    if (refreshInputs) {
      input(key,'direction').value = String(joint.direction);
      input(key,'minimum').value = String(joint.limits.minimum_angle_deg);
      input(key,'maximum').value = String(joint.limits.maximum_angle_deg);
      input(key,'maxSpeed').value = String(joint.limits.maximum_speed_rad_s);
      input(key,'maxTorque').value = String(joint.limits.maximum_torque_nm);
      input(key,'kp').value = String(joint.control.motion_kp);
      input(key,'kd').value = String(joint.control.motion_kd);
      input(key,'speed').value = String(joint.control.default_speed_rad_s);
      input(key,'torque').value = String(joint.control.default_torque_limit_nm);
      if (preview[key] === null || force) preview[key] = 0;
      configSignatures[key] = signature;
    }
    jointInputRanges(key,joint);
    setPreview(key, preview[key], refreshInputs);
    field(key,'identity').textContent = joint.verified ? `UID ${joint.observed_uid} ✓` : `期望 UID ${joint.expected_uid} · 未验证`;
    field(key,'commission').textContent = configDirty[key] ? '设置待确认' : joint.phase && joint.phase !== 'stopped' ? phaseText[joint.phase] : joint.commissioned ? '已就绪' : '待确认设置';
    field(key,'commission').className = joint.commissioned && !configDirty[key] ? 'joint-phase ready' : 'joint-phase';
    field(key,'zero').textContent = finite(joint.software_zero_rad) ? `${joint.software_zero_rad.toFixed(5)} rad` : '未设置';
    card.querySelector('[data-action="zero"]').disabled = !next.connected || !allVerified;
    card.querySelector('[data-action="prepare"]').disabled = !next.connected || !allVerified || !finite(joint.software_zero_rad) || next.safety_latched || anyFault;
    const canMove = supportsHold && canRetarget && next.connected && allVerified && joint.commissioned && joint.limits_confirmed && finite(joint.software_zero_rad) && !configDirty[key] && !next.safety_latched && !anyFault && targetWithinLimits(key, next);
    readyToMove[key] = canMove;
    card.querySelector('[data-action="move"]').disabled = !canMove || next.gravity?.enabled || (next.motion_active && joint.phase === 'stopped');
    field(key,'readiness').textContent = !next.connected ? '先连接并识别双机' : !allVerified ? '等待两台电机身份验证' : !finite(joint.software_zero_rad) ? '先将关节置于参考姿态并采零' : next.safety_latched || anyFault ? '先排除故障并复位安全锁存' : configDirty[key] ? '设置已修改，请重新确认并启用' : !targetWithinLimits(key, next) ? '目标超出软限位，请拖回允许范围' : joint.commissioned && joint.limits_confirmed ? '已就绪：选择目标后可运动' : '核对安全范围后点击“确认并启用”';
    if (canMove && next.gravity?.enabled) field(key,'readiness').textContent = '补偿已开启：请使用下方双关节按钮；另一关节可保持当前角度';
    else if (next.motion_active && !canRetarget) field(key,'readiness').textContent = `${phaseText[next.control_phase]}；规划轨迹结束后可换目标，随时可停止`;
    else if (canMove && next.control_phase === 'controlling') field(key,'readiness').textContent = '持续控制中：可直接换目标；到位状态仅供显示';
    else if (canMove && next.control_phase === 'limited') field(key,'readiness').textContent = '未达到到位容差：输出仍受限；稳定后可换目标，未解除任何故障保护';
    else if (canMove && next.control_phase === 'holding') field(key,'readiness').textContent = '保持中：修改目标后可再次运动，不会先释放电机';
    field(key,'readiness').dataset.needed = String(!canMove || !!next.gravity?.enabled || next.control_phase === 'limited');
    card.querySelector('[data-action="prepare"]').title = field(key,'readiness').textContent;
    card.querySelector('[data-action="move"]').title = next.gravity?.enabled ? '补偿需要双关节反馈，请使用双关节运动按钮' : !canRetarget ? '规划轨迹结束后可换目标；无需等待到位，随时可以停止' : canMove ? '持续控制目标角度；轨迹结束后可直接换目标' : field(key,'readiness').textContent;
    diagnosticPanels[key](joint, next);
  }
  ui.syncMoveButton.disabled = joints.some(key => !readyToMove[key] || (next.motion_active && next.joints[key].phase === 'stopped'));
  updateMeasuredScene();
  ui.eventLog.innerHTML = next.logs.slice().reverse().map(item => `<div><time>${escapeHtml(item.time)}</time><i class="${escapeHtml(item.level)}"></i><span>${escapeHtml(item.message)}</span></div>`).join('') || '<div><time>--</time><i></i><span>暂无事件</span></div>';
}

function refreshState(force = false) {
  statePollForce ||= force;
  // Never overlap polls or let a late, older response overwrite a newer pose.
  if (stateRequest) return stateRequest;
  stateRequest = (async () => {
    try {
      const next = await api('/api/state', null, {signal:AbortSignal.timeout(2000)});
      stateReceivedAt = performance.now();
      serviceAvailable = true;
      const syncInputs = statePollForce;
      statePollForce = false;
      renderState(next, syncInputs);
    } catch (error) {
      serviceAvailable = false;
      ui.statusText.textContent = `数字孪生服务连接异常：${error.message}`;
      updateMeasuredScene();
    } finally { stateRequest = null; }
  })();
  return stateRequest;
}

const chartSpecs = [
  {id:'positionChart', metric:'angle'}, {id:'velocityChart', metric:'velocity'},
  {id:'torqueChart', metric:'torque'}, {id:'temperatureChart', metric:'temperature'}
];
function drawChart(spec) {
  const canvas = $(spec.id), rect = canvas.getBoundingClientRect(), dpr = Math.min(window.devicePixelRatio || 1, 2);
  if (!rect.width || !rect.height) return;
  canvas.width = rect.width * dpr; canvas.height = rect.height * dpr;
  const ctx = canvas.getContext('2d'); ctx.scale(dpr, dpr);
  const w = rect.width, h = rect.height, p = {l:42,r:9,t:12,b:18};
  ctx.clearRect(0,0,w,h);
  ctx.strokeStyle = 'rgba(143,181,201,.12)';
  for (let i=0;i<4;i++) { const y=p.t+(h-p.t-p.b)*i/3; ctx.beginPath(); ctx.moveTo(p.l,y); ctx.lineTo(w-p.r,y); ctx.stroke(); }
  const view=chartView(history,spec.metric,Date.now()/1000);
  if (view.empty) { ctx.fillStyle='#607784'; ctx.font='10px Microsoft YaHei'; ctx.fillText('等待双路反馈',p.l+8,h/2); return; }
  const {low,high,start,span}=view;
  const axisDigits = Math.min(5, Math.max(1, 1-Math.floor(Math.log10(high-low))));
  ctx.fillStyle='#607784'; ctx.font='9px Consolas'; ctx.textAlign='right';
  ctx.fillText(high.toFixed(axisDigits),p.l-5,p.t+4); ctx.fillText(low.toFixed(axisDigits),p.l-5,h-p.b);
  ctx.save();ctx.beginPath();ctx.rect(p.l,p.t,w-p.l-p.r,h-p.t-p.b);ctx.clip();
  for (const key of joints) {
    ctx.strokeStyle=colors[key]; ctx.fillStyle=colors[key]; ctx.lineWidth=1.6;
    for (const segment of view.series[key]) {
      ctx.beginPath();
      for (let i=0;i<segment.length;i++) {
        const [stamp,value]=segment[i];
        const x=p.l+(stamp-start)/span*(w-p.l-p.r), y=h-p.b-(value-low)/(high-low)*(h-p.t-p.b);
        if (!i) ctx.moveTo(x,y); else ctx.lineTo(x,y);
        if (segment.length===1) {ctx.arc(x,y,2,0,Math.PI*2);ctx.fill();}
      }
      ctx.stroke();
    }
  }
  ctx.restore();ctx.fillStyle='#607784';
  ctx.textAlign='left';ctx.fillText(`-${span.toFixed(span<10?1:0)}s`,p.l,h-5);
  ctx.textAlign='right';ctx.fillText(view.stale?'末次反馈':'最新',w-p.r,h-5);
}
let historyPending=false;
async function refreshHistory() {
  if (historyPending) return;
  historyPending=true;
  try { history=mergeHistory(history,await api('/api/history'),Date.now()/1000); }
  catch (_error) { /* Keep the last received history, expiring it by timestamp below. */ }
  finally { historyPending=false;chartSpecs.forEach(drawChart); }
}
new ResizeObserver(() => chartSpecs.forEach(drawChart)).observe(document.querySelector('.charts'));
setInterval(() => { ui.clock.textContent = new Date().toLocaleTimeString('zh-CN',{hour12:false}); }, 500);
setInterval(() => refreshState(), 100);
setInterval(refreshHistory, 700);
refreshState(true);
refreshHistory();
