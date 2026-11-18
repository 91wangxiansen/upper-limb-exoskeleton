import * as THREE from 'three';

// Both layers share the original CAD and the standalone hand's coupled pivots.
export async function mountHand(measuredParent, targetParent) {
  const responses = await Promise.all([fetch('/hand-model/model.json'), fetch('/hand-model/model.bin')]);
  if (responses.some(r => !r.ok)) throw Error('手部 CAD 读取失败');
  const meta = await responses[0].json(), data = await responses[1].arrayBuffer();
  const geometries = meta.parts.map(part => {
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(
      new Float32Array(data, part.vertexOffset, part.vertexCount * 3).slice(), 3));
    geometry.setIndex(new THREE.BufferAttribute(new Uint16Array(data, part.indexOffset, part.indexCount).slice(), 1));
    geometry.computeVertexNormals();
    return geometry;
  });
  const colors = [0x79d5c8, 0x91b6ec, 0xe8bb83, 0xc6a3e8];
  const pickables = [];
  const navigationMeshes = [];
  const layers = [measuredParent, targetParent].map((parent, layer) => {
    const mount = new THREE.Group();
    mount.position.y = -2.35;
    mount.rotation.z = Math.PI;
    mount.scale.setScalar(.009);
    parent.add(mount);
    const rig = meta.fingers.map(({mcp, pip}) => {
      const base = new THREE.Group(), tip = new THREE.Group();
      base.position.fromArray(mcp);
      tip.position.fromArray(pip).sub(base.position);
      base.add(tip); mount.add(base);
      return {base, tip};
    });
    meta.parts.forEach((part, i) => {
      const f = part.finger;
      const material = layer
        ? new THREE.MeshBasicMaterial({color:0xffd79b, transparent:true, opacity:.16,
          depthWrite:false})
        : new THREE.MeshStandardMaterial({color:f < 0 ? 0x78949d : colors[f], roughness:.5, metalness:.35});
      const mesh = new THREE.Mesh(geometries[i], material);
      mesh.userData.finger = f;
      navigationMeshes.push(mesh);
      if (f < 0) { mount.add(mesh); return; }
      const distal = part.kind === 'distal';
      mesh.position.fromArray(meta.fingers[f][distal ? 'pip' : 'mcp']).multiplyScalar(-1);
      rig[f][distal ? 'tip' : 'base'].add(mesh);
      if (layer) {
        mesh.userData.finger = f;
        pickables.push(mesh);
      }
    });
    return {mount, rig};
  });
  return {pickables, navigationMeshes, update(view) {
    [view.measured, view.target].forEach((pose, layer) => {
      layers[layer].rig.forEach(({base, tip}, i) => {
        base.visible = pose[i] !== null;
        const q = (pose[i] ?? 0) / 100;
        base.rotation.x = -q * 1.03;
        tip.rotation.x = -q * 1.22;
      });
      // Hide measured palm when no live hand feedback is available.
      layers[layer].mount.visible = pose.some(value => value !== null);
    });
  }};
}
