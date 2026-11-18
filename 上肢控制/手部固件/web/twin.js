import * as THREE from 'three';
import {OrbitControls} from './vendor/OrbitControls.js';
export async function makeTwin(el,onDrag,onFinish){
  const [meta,data]=await Promise.all([fetch('model.json').then(r=>r.json()),fetch('model.bin').then(r=>r.arrayBuffer())]);
  const scene=new THREE.Scene(), camera=new THREE.PerspectiveCamera(37,1,1,2500);
  const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));el.append(renderer.domElement);
  scene.add(new THREE.HemisphereLight(0xe0f5ff,0x59646b,2.3));
  const light=new THREE.DirectionalLight(0xffffff,2.8);light.position.set(-130,200,300);scene.add(light);
  const orbit=new OrbitControls(camera,renderer.domElement);orbit.enablePan=false;orbit.minDistance=140;orbit.maxDistance=850;
  function reset(){camera.position.set(110,100,310);orbit.target.set(0,34,0);orbit.update();}
  reset();const pickables=[],rigs=[];
  const colors=[0x6ed7c9,0x91b6ec,0xf2bd82,0xcc9ee5];
  for(let layer=0;layer<2;layer++){
    const rig=meta.fingers.map(({mcp,pip})=>{
      const base=new THREE.Group(),tip=new THREE.Group();base.position.fromArray(mcp);
      tip.position.fromArray(pip).sub(base.position);base.add(tip);scene.add(base);return {base,tip};
    });rigs.push(rig);
    for(const part of meta.parts){
      const f=part.finger;if(layer && f<0)continue;
      const geometry=new THREE.BufferGeometry();
      geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(data,part.vertexOffset,part.vertexCount*3).slice(),3));
      geometry.setIndex(new THREE.BufferAttribute(new Uint16Array(data,part.indexOffset,part.indexCount).slice(),1));
      geometry.computeVertexNormals();
      const mat=layer?new THREE.MeshBasicMaterial({color:0xffda86,transparent:true,opacity:.17,depthWrite:false}):
        new THREE.MeshStandardMaterial({color:f<0?0x889fac:colors[f],roughness:.65,metalness:.18});
      const mesh=new THREE.Mesh(geometry,mat);mesh.userData.f=f;
      if(f<0){scene.add(mesh);continue;}
      const distal=part.kind==='distal',pivot=meta.fingers[f][distal?'pip':'mcp'];
      mesh.position.fromArray(pivot).multiplyScalar(-1);rig[f][distal?'tip':'base'].add(mesh);pickables.push(mesh);
    }
  }
  const ray=new THREE.Raycaster();let drag=null;let targets=[0,0,0,0];
  const canvas=renderer.domElement;
  canvas.addEventListener('pointerdown',e=>{
    const b=canvas.getBoundingClientRect();ray.setFromCamera({x:(e.clientX-b.left)/b.width*2-1,y:1-(e.clientY-b.top)/b.height*2},camera);
    const hit=ray.intersectObjects(pickables)[0];if(!hit)return;
    e.stopImmediatePropagation();const f=hit.object.userData.f;drag={f,y:e.clientY,start:targets[f]};
    orbit.enabled=false;canvas.setPointerCapture(e.pointerId);
  },true);
  canvas.addEventListener('pointermove',e=>{if(drag)onDrag(drag.f,Math.max(0,Math.min(100,Math.round(drag.start+(e.clientY-drag.y)*.65))));});
  function finish(){if(drag){drag=null;orbit.enabled=true;onFinish();}}
  canvas.addEventListener('pointerup',finish);canvas.addEventListener('pointercancel',()=>{drag=null;orbit.enabled=true;});
  const resize=new ResizeObserver(()=>{const w=el.clientWidth,h=el.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();});resize.observe(el);
  renderer.setAnimationLoop(()=>renderer.render(scene,camera));
  return {reset,update(measured,target){targets=target.slice();[measured,target].forEach((pose,layer)=>rigs[layer].forEach(({base,tip},i)=>{
    base.visible=pose[i]!=null && (!layer || measured[i]==null || Math.abs(measured[i]-target[i])>1);
    const q=(pose[i]??0)/100;base.rotation.x=-q*1.03;tip.rotation.x=-q*1.22;
  }));}};
}
