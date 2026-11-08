// Execute the shipped dashboard script with a fake HTTP/DOM boundary; no ROS or motion.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../rover_navigation/mission_controls.html'), 'utf8');
const elements = new Map(), listeners = {}, requests = [];
function element(id) {
  if (!elements.has(id)) elements.set(id, {id, tagName:'DIV', value:'', disabled:false,
    classList:{remove(){}, toggle(){}}, replaceChildren(){}, setAttribute(){}, removeAttribute(){}});
  return elements.get(id);
}
for (const match of html.matchAll(/<input\b([^>]+)>/g)) {
  const attributes = Object.fromEntries([...match[1].matchAll(/([\w-]+)="([^"]*)"/g)].map(m => [m[1],m[2]]));
  if (attributes.id) Object.assign(element(attributes.id), attributes, {tagName:'INPUT'});
}
const pad = ['ArrowUp','ArrowLeft','ArrowRight'].map(key => ({...element(key),dataset:{key}}));
const status = {session:'fake-scene',status:{mode:'manual',ready:true,age_seconds:0,command:[0,0]},
  mission_status:{mission:'idle'}, mapping:{clearance_confirmed:true}, scene:{available:true,phase:'ready'},
  manual_speed_limits:{linear:{min:.01,max:.5,step:.01,default:.2},angular:{min:.05,max:.6,step:.05,default:.3}}};
const context = vm.createContext({console, performance:{now:()=>100}, AbortSignal:{timeout:()=>undefined},
  Blob, URL, queueMicrotask, setInterval:()=>0, setTimeout:()=>0,
  navigator:{sendBeacon:(route)=>{requests.push({route});return true;}},
  window:{addEventListener(){}},
  document:{hidden:false,getElementById:element,querySelectorAll:()=>pad,
    addEventListener:(kind,callback)=>{listeners[kind]=callback;},createElement:()=>element('new'),createElementNS:()=>element('svg-new')},
  fetch:async(route,options={})=>{
    const data=options.body ? JSON.parse(options.body) : undefined;
    requests.push({route,data});
    if(route==='/camera.jpg')return {ok:false};
    return {ok:true,json:async()=>route==='/status'?status:{success:true,session:'control-token',message:'ok'}};
  }});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const run=code=>vm.runInContext(code,context);
const flush=async()=>{for(let i=0;i<10;i++)await Promise.resolve();};
const latestDrive=()=>requests.filter(r=>r.route==='/drive').at(-1)?.data;
async function main(){
  await flush();
  assert.equal(element('linear-speed-value').textContent,'0.20 m/s');
  assert.equal(element('angular-speed-value').textContent,'0.30 rad/s');
  // Selecting speeds while disarmed cannot enable control or send motion.
  element('linear-speed-up').onclick();
  await flush();
  assert.equal(element('linear-speed-value').textContent,'0.21 m/s');
  assert.equal(requests.filter(r=>r.route==='/drive'||r.route==='/manual').length,0);
  element('speed-reset').onclick();
  await run("act('manual')"); await flush();
  run("held('ArrowUp',true)"); await flush();
  assert.equal(latestDrive().linear,.2);
  const selectedToken=latestDrive().session;
  // Changing a speed releases a held direction and preserves the selected mode.
  element('linear-speed').value='.5';element('linear-speed').oninput();await flush();
  assert.equal(latestDrive().linear,0);
  assert.equal(latestDrive().session,selectedToken);
  assert.equal(element('linear-speed-up').disabled,true);
  listeners.keydown({key:'w',code:'KeyW',repeat:true,target:{tagName:'BODY'},preventDefault(){}});
  await flush();assert.equal(latestDrive().linear,0);
  listeners.keyup({key:'w',target:{tagName:'BODY'},preventDefault(){}});
  await flush();
  listeners.keydown({key:'w',code:'KeyW',repeat:false,target:{tagName:'BODY'},preventDefault(){}});
  await flush();assert.equal(latestDrive().linear,.5);
  run('release()');await flush();
  // Range overflow and malformed values cannot enter a control payload.
  run("setSpeed('linear',999)");await flush();
  assert.equal(element('linear-speed-value').textContent,'0.50 m/s');
  run("setSpeed('linear',NaN)");await flush();
  assert.equal(element('linear-speed-value').textContent,'0.50 m/s');
  run("setSpeed('linear',-1)");await flush();
  assert.equal(element('linear-speed-value').textContent,'0.01 m/s');
  assert.equal(element('linear-speed-down').disabled,true);
  element('angular-speed').value='.6';element('angular-speed').oninput();await flush();
  run("held('ArrowLeft',true)");await flush();assert.equal(latestDrive().angular,.6);
  run('release()');await flush();
  run("held('ArrowRight',true)");await flush();assert.equal(latestDrive().angular,-.6);
  run('release()');await flush();
  // Arrow keys used on the slider must not drive the rover.
  listeners.keydown({key:'ArrowUp',code:'ArrowUp',repeat:false,target:{tagName:'INPUT'},preventDefault(){}});
  await flush();assert.equal(latestDrive().linear,0);
  element('speed-reset').onclick();await flush();
  assert.equal(element('linear-speed-value').textContent,'0.20 m/s');
  assert.equal(element('angular-speed-value').textContent,'0.30 rad/s');
  assert.equal(latestDrive().linear,0);assert.equal(latestDrive().angular,0);
  // Stop still wins when focus is inside a speed setting.
  listeners.keydown({key:' ',code:'Space',target:{tagName:'INPUT'},preventDefault(){}});
  await flush();assert.ok(requests.some(r=>r.route==='/stop'));
  const before=requests.filter(r=>r.route==='/drive').length;
  run("setSpeed('linear',.4);clearSceneView()");await flush();
  assert.equal(element('linear-speed-value').textContent,'0.20 m/s');
  assert.equal(requests.filter(r=>r.route==='/drive').length,before);
  // A session-specific qualification cap clamps the old cached/default speed
  // before manual enable, including after the page clears for another scene.
  status.manual_speed_limits.linear.max=.05;
  status.manual_speed_limits.angular.max=.3;
  await run('refresh()');await flush();
  assert.equal(element('linear-speed-value').textContent,'0.05 m/s');
  await run("act('manual')");await flush();
  run("held('ArrowUp',true)");await flush();
  assert.equal(latestDrive().linear,.05);
  run('release();clearSceneView()');await flush();
  await run('refresh()');await flush();
  assert.equal(element('linear-speed-value').textContent,'0.05 m/s');
  run("setSpeed('linear',.2)");await flush();
  assert.equal(element('linear-speed-value').textContent,'0.05 m/s');
  assert.ok(requests.filter(r=>r.route==='/drive').every(r=>r.data.linear>=0&&r.data.linear<=.5&&Math.abs(r.data.angular)<=.6));
  status.manual_speed_limits.linear.max=.2;status.manual_speed_limits.linear.default=.05;
  await run('refresh()');await flush();
  assert.equal(element('linear-speed-value').textContent,'0.05 m/s');
  run("setSpeed('linear',.2)");await flush();
  assert.equal(element('linear-speed-value').textContent,'0.20 m/s');
  run('clearSceneView()');await flush();
  assert.equal(element('linear-speed-value').textContent,'0.05 m/s');
  console.log('Manual speed controls passed: defaults, limits, held-input release, fresh press, focus, Stop, restart.');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
