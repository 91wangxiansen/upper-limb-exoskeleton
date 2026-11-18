// Run against mocked HTTP routes. No serial ports or real boards are accessed.
const {chromium} = require('playwright');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const {execFileSync} = require('node:child_process');
const root = path.resolve(__dirname, '../web');
const armState = JSON.parse(execFileSync('python', ['-c',
  'import json; from digital_twin_server import TWIN; print(json.dumps(TWIN.snapshot()))'], {
  cwd:path.resolve(root, '..'), env:{...process.env, PYTHONPATH:path.resolve(root, '../.deps')}, encoding:'utf8'
}));
armState.connected = true;
armState.motion_active = true;
armState.control_phase = 'holding';
for (const [key, angle] of [['shoulder',20], ['elbow',45]]) {
  Object.assign(armState.joints[key], {verified:true, commissioned:true, actual_deg:angle,
    target_deg:angle, feedback_age_s:.01, phase:'holding'});
}

(async () => {
  const browser = await chromium.launch({
    executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
    headless:true
  });
  try {
    const page = await browser.newPage({viewport:{width:1440, height:960}});
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    let revision = 1, armed = false, connected = false, ownership = 'free';
    const sent = [];
    const armSent = [];
    let gestures = [], speed = 350, scanDone = 0;
    function state() {
      return {connected, monitoring:connected, armed, ready:true, gripping:false,
        ownership, revision, ageMs:10, speedControl:1, disconnectRelease:0, leaseMs:0,
        gestures, scanDone, detectedIDs:scanDone ? [1,2,3,4] : [], fingers: connected ? [1,2,3,4].map(id => ({
          id, online:1, phase:'held', p:500, q:25, load:-210, limit:300,
          speedSet:speed, open:500, close:900, verified:1, v:6, temp:28, alarm:0
        })) : []};
    }
    await page.route('http://127.0.0.1:8765/**', async route => {
      const url = new URL(route.request().url());
      if (url.pathname.startsWith('/api/hand/')) {
        const endpoint = url.pathname.split('/').pop();
        if (endpoint === 'connect') connected = true;
        if (endpoint === 'disconnect') connected = false;
        if (endpoint === 'command') {
          const body = route.request().postDataJSON();
          sent.push(body);
          assert.equal(body.revision, revision);
          revision++;
          if (body.op === 'arm') { armed = true; ownership = 'local'; }
          if (body.op === 'release') { armed = false; ownership = 'free'; }
          if (body.op === 'speed') speed = body.speed;
          if (body.op === 'save') gestures = [{name:body.name, q:body.targets, slot:0}];
          if (body.op === 'delete') gestures = [];
          if (body.op === 'scan') scanDone = 1;
        }
        return route.fulfill({json:endpoint === 'state' ? state() : {ok:true, state:state()}});
      }
      if (url.pathname === '/api/state') return route.fulfill({json:armState});
      if (url.pathname === '/api/history') return route.fulfill({json:{}});
      if (url.pathname === '/api/move') {
        const body = route.request().postDataJSON();
        armSent.push(body);
        for (const [key, value] of Object.entries(body.targets_deg)) armState.joints[key].actual_deg = value;
        return route.fulfill({json:{ok:true, result:1, state:armState}});
      }
      const file = path.join(root, url.pathname === '/' ? 'index.html' : url.pathname === '/hand-control/' ? 'hand-control/index.html' : url.pathname.slice(1));
      return route.fulfill({path:file});
    });
    await page.goto('http://127.0.0.1:8765/');
    await page.waitForFunction(() => document.querySelector('#viewport canvas'));
    await page.waitForFunction(() => document.querySelector('#handModelStatus').textContent.includes('已装配'));
    await page.locator('[data-joint="shoulder"] [data-input="targetNumber"]').fill('25');
    await page.locator('[data-joint="shoulder"] [data-input="targetNumber"]').dispatchEvent('input');
    await page.locator('[data-joint="elbow"] [data-input="targetNumber"]').fill('45');
    await page.locator('[data-joint="elbow"] [data-input="targetNumber"]').dispatchEvent('input');
    await page.click('#syncMoveButton');
    await page.waitForTimeout(200);
    assert.equal(armSent.length, 1);
    assert.equal(armSent[0].targets_deg.shoulder, 25);
    assert.equal(sent.length, 0);
    const consoleUI=page.locator('#handConsole');
    const waitHand=(selector,text)=>page.waitForFunction(({selector,text})=>document.getElementById('handConsole').shadowRoot?.querySelector(selector)?.textContent.includes(text),{selector,text});
    await page.click('#showHand');
    await consoleUI.locator('#connectHand').click();
    await page.waitForFunction(()=>!document.getElementById('handConsole').shadowRoot?.getElementById('arm').disabled);
    await consoleUI.locator('#arm').click();
    await page.waitForFunction(()=>!document.getElementById('handConsole').shadowRoot?.getElementById('apply').disabled);
    assert.equal(new URL(page.url()).pathname,'/');
    assert.equal(await page.locator('#armWorkspace').isVisible(),false);
    for(let i=0;i<4;i++)await consoleUI.locator('#s'+i).fill(String(i*30));
    await consoleUI.locator('#apply').click();
    await waitHand('#notice','发送目标');
    assert.deepEqual(sent.at(-1).targets,[0,30,60,90]);
    await consoleUI.locator('.back-link').click();
    assert.equal(await page.locator('#armWorkspace').isVisible(),true);
    const point = await page.evaluate(async () => {
      const T = await import('/vendor/three.module.js');
      const m = await fetch('/hand-model/model.json').then(r=>r.json());
      const data = await fetch('/hand-model/model.bin').then(r=>r.arrayBuffer());
      const part = m.parts.find(p=>p.finger===1 && p.kind==='proximal');
      const values = new Float32Array(data,part.vertexOffset,part.vertexCount*3);
      const box = new T.Box3().setFromArray(values);
      const p = box.getCenter(new T.Vector3()); p.z=box.max.z;
      const pivot = new T.Vector3().fromArray(m.fingers[1].mcp);
      p.sub(pivot).applyAxisAngle(new T.Vector3(1,0,0),-.25*1.03).add(pivot);
      p.multiplyScalar(.009).applyAxisAngle(new T.Vector3(0,0,1),Math.PI);
      p.y-=2.35;
      p.applyAxisAngle(new T.Vector3(0,0,1),Math.PI/4);p.y-=2.65;
      p.applyAxisAngle(new T.Vector3(0,0,1),25*Math.PI/180);p.y+=2.65;
      p.applyAxisAngle(new T.Vector3(0,1,0),-.1);
      const rect=document.querySelector('#viewport canvas').getBoundingClientRect();
      const camera=new T.PerspectiveCamera(38,rect.width/rect.height,.1,100);
      camera.position.set(4.4,2.8,12);camera.lookAt(0,-.35,0);camera.updateMatrixWorld(true);
      p.project(camera);
      return {x:rect.left+(p.x+1)/2*rect.width,y:rect.top+(1-p.y)/2*rect.height};
    });
    await page.mouse.move(point.x,point.y);await page.mouse.down();
    await page.mouse.move(point.x+15,point.y+15);await page.mouse.up();
    assert.equal(await page.locator('#armWorkspace').isVisible(),true);
    const beforeEntry=sent.length;
    await page.mouse.click(point.x,point.y);
    await consoleUI.locator('#sliders').waitFor({state:'visible'});
    assert.equal(sent.length,beforeEntry);
    assert.equal(new URL(page.url()).pathname,'/');
    await consoleUI.locator('#live').check();
    await Promise.all([page.waitForResponse(r=>r.url().endsWith('/api/hand/command')&&r.request().postDataJSON().op==='pose'),consoleUI.locator('#s0').fill('55')]);
    await consoleUI.locator('#live').uncheck();
    assert.equal(sent.at(-1).targets[0],55);
    await consoleUI.locator('#live').check();
    const beforeLeavingLive=sent.length;
    await page.evaluate(()=>{
      const root=document.getElementById('handConsole').shadowRoot;
      const slider=root.getElementById('s0');slider.value='56';slider.dispatchEvent(new Event('input',{bubbles:true}));
      root.querySelector('.back-link').click();
    });
    await page.waitForTimeout(250);
    assert.equal(sent.length,beforeLeavingLive);
    await page.click('#showHand');
    assert.equal(await consoleUI.locator('#live').isChecked(),false);
    await consoleUI.locator('#motionSpeed').fill('400');await consoleUI.locator('#saveSpeed').click();
    await waitHand('#speedCurrent','400');
    await consoleUI.locator('[data-tab="gestures"]').click();await consoleUI.locator('#name').fill('测试握姿');
    await consoleUI.locator('#save').click();
    await page.waitForFunction(()=>document.getElementById('handConsole').shadowRoot.getElementById('gesture').options.length===1);
    await consoleUI.locator('#load').click();await consoleUI.locator('#play').click();
    await page.waitForTimeout(200);await consoleUI.locator('#remove').click();
    await page.waitForFunction(()=>document.getElementById('handConsole').shadowRoot.getElementById('gesture').options.length===0);
    await consoleUI.locator('#release').click();await waitHand('#status','待机');
    await consoleUI.locator('[data-tab="setup"]').click();
    for(const [id,value,expected] of [['id0','99','20'],['open0','-5','0'],['limit0','9999','1000'],['speedSet0','999','500']]){
      const input=consoleUI.locator('#'+id),previous=await input.inputValue();
      await input.fill(value);assert.equal(await input.inputValue(),expected);await input.fill(previous);
    }
    await consoleUI.locator('#scanIDs').click();await waitHand('#foundIDs','1、2');
    await consoleUI.locator('#newID').fill('5');await consoleUI.locator('#setID').click();await waitHand('#notice','修改 ID');
    const beforeInvalid=sent.length;
    await consoleUI.locator('#id1').fill('1');await consoleUI.locator('#saveConfig').click();
    await waitHand('#notice','ID必须互异');assert.equal(sent.length,beforeInvalid);
    await consoleUI.locator('#id1').fill('2');await consoleUI.locator('#close0').fill('501');
    await consoleUI.locator('#saveConfig').click();await waitHand('#notice','至少相差20');assert.equal(sent.length,beforeInvalid);
    await consoleUI.locator('#close0').fill('900');await consoleUI.locator('#saveConfig').click();await waitHand('#notice','保存校准');
    assert.equal(sent.at(-1).op,'config');assert.equal(armSent.length,1);
    await consoleUI.locator('[data-tab="joints"]').click();
    const beforeSwitch=sent.length;
    if(process.env.HAND_UI_SCREENSHOT)await page.screenshot({path:process.env.HAND_UI_SCREENSHOT.replace('.png','-integrated.png'),fullPage:true});
    await page.setViewportSize({width:390,height:844});
    assert.equal(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth),true);
    await consoleUI.locator('.back-link').click();assert.equal(sent.length,beforeSwitch);
    await page.setViewportSize({width:1440,height:960});
    await page.locator('[data-joint="shoulder"] .joint-setup').evaluate(el=>el.open=true);
    for(const [name,value,expected] of [['speed','999','0.3'],['torque','999','5'],['kp','999','20'],['kd','-1','0']]){
      const input=page.locator('[data-joint="shoulder"] [data-input="'+name+'"]');
      await input.fill(value);assert.equal(await input.inputValue(),expected);
    }
    const min=page.locator('[data-joint="shoulder"] [data-input="minimum"]'),previous=await min.inputValue();
    const max=await page.locator('[data-joint="shoulder"] [data-input="maximum"]').inputValue();
    await min.fill(max);assert.equal(await min.inputValue(),previous);
    await min.fill('');await min.blur();assert.equal(await min.inputValue(),previous);
    await page.locator('.motion-settings-panel').evaluate(el=>el.open=true);
    await page.locator('.motion-settings-panel details').evaluateAll(items=>items.forEach(el=>el.open=true));
    for(const [name,maximum] of [['position_kp',20],['velocity_kd',5],['ki_nm_per_rad_s',2],['maximum_bias_nm',2.5],['maximum_rate_nm_s',.4],['deadband_deg',1],['maximum_error_deg',15],['tracking_error_rate_rad_s',.15]]){
      const input=page.locator('[data-load="'+name+'"]');
      await input.fill('999');assert.equal(Number(await input.inputValue()),maximum);
    }
    armState.joints.shoulder.limits.maximum_speed_rad_s=.15;
    await page.waitForFunction(()=>document.querySelector('[data-joint="shoulder"] [data-input="speed"]').max==='0.15');
    const speedInput=page.locator('[data-joint="shoulder"] [data-input="speed"]');
    await speedInput.fill('999');assert.equal(await speedInput.inputValue(),'0.15');
    await page.click('#showHand');ownership='other';await waitHand('#status','待机');
    await page.waitForFunction(()=>document.getElementById('handConsole').shadowRoot.getElementById('release').disabled);
    assert.equal(await consoleUI.locator('#release').isDisabled(),true);
    await page.reload();await page.click('#showHand');await consoleUI.locator('#sliders').waitFor({state:'visible'});
    assert.equal(sent.length,beforeSwitch);
    assert.deepEqual(errors, []);
    console.log('Unified arm then hand, persistent CAD, desktop/mobile, ownership and refresh: PASS');
  } finally {
    await browser.close();
  }
})().catch(error => { console.error(error); process.exitCode=1; });
