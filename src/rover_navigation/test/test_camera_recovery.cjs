// Exercise the shipped camera loop: outages, decode failures, recovery and scene changes.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const html=fs.readFileSync(path.join(__dirname,'../rover_navigation/mission_controls.html'),'utf8');
const nodes=new Map(),revoked=[],posts=[];let now=100,nextUrl=0,response={ok:false},decode=()=>Promise.resolve();
function element(id){if(!nodes.has(id))nodes.set(id,{id,tagName:'DIV',value:0,min:.01,max:.5,step:.01,
  classList:{remove(){},toggle(){}},setAttribute(name,value){this[name]=value;},removeAttribute(name){delete this[name];},replaceChildren(){}});return nodes.get(id);}
const state={session:'scene-one',status:{mode:'stopped',ready:false,age_seconds:0,command:[0,0]},scene:{phase:'ready'},mission_status:{mission:'idle'}};
const context=vm.createContext({console,performance:{now:()=>now},AbortSignal:{timeout:()=>undefined},
  setTimeout:()=>1,clearTimeout(){},setInterval(){},Blob,queueMicrotask,
  URL:{createObjectURL:()=>`blob:frame-${++nextUrl}`,revokeObjectURL:url=>revoked.push(url)},
  Image:class{decode(){return decode();}},navigator:{sendBeacon:()=>true},window:{addEventListener(){}},
  document:{getElementById:element,querySelectorAll:()=>[],addEventListener(){},createElementNS:()=>element('svg')},
  fetch:async(route,options={})=>{if(options.method==='POST')posts.push(route);if(route==='/camera.jpg')return typeof response==='function'?response():response;return {ok:true,json:async()=>state};}});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const run=source=>vm.runInContext(source,context);
const flush=async()=>{for(let i=0;i<12;i++)await Promise.resolve();};
const frame=()=>({ok:true,blob:async()=>new Blob(['jpeg']),headers:{get:()=> 'true'}});
async function main(){
  await flush();
  assert.equal(element('camera').src,undefined);
  response=frame();await run('camera()');
  assert.equal(element('camera').src,'blob:frame-1');
  assert.equal(element('camera')['data-stale'],'false');
  assert.equal(element('camera-state').textContent,'Live detections matched to this image');
  response={ok:false};now=1600;await run('camera()');
  assert.equal(element('camera').src,'blob:frame-1','HTTP failure must preserve the last decoded image');
  assert.equal(element('camera')['data-stale'],'true');
  assert.match(element('camera-state').textContent,/last image 1.5 s old/);
  response=frame();decode=()=>Promise.reject(Error('bad JPEG'));await run('camera()');
  assert.equal(element('camera').src,'blob:frame-1');
  assert.ok(revoked.includes('blob:frame-2'),'failed frames cannot leak blob URLs');
  let finishDecode;decode=()=>new Promise(resolve=>{finishDecode=resolve;});
  const waiting=run('camera()');await flush();
  assert.equal(element('camera').src,'blob:frame-1','decode new frames before replacing visible pixels');
  finishDecode();await waiting;
  assert.equal(element('camera').src,'blob:frame-3');
  assert.ok(revoked.includes('blob:frame-1'));
  decode=()=>Promise.resolve();let finishFetch;
  response=()=>new Promise(resolve=>{finishFetch=resolve;});
  const late=run('camera()');await flush();run('clearSceneView()');
  finishFetch(frame());await late;
  assert.equal(element('camera').src,undefined,'late old-scene frames cannot return after Restart');
  assert.ok(revoked.includes('blob:frame-4'));
  assert.deepEqual(posts,[],'camera recovery never selects a mode or sends motion');
  console.log('Camera recovery passed: retain last frame, label stale, decode failure, retry, restart fence, no motion.');
}
main().catch(error=>{console.error(error);process.exitCode=1;});
