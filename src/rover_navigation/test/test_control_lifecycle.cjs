// Run the shipped page against delayed HTTP responses; never contact a rover.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const html=fs.readFileSync(path.join(__dirname,'../rover_navigation/mission_controls.html'),'utf8');
const elements=new Map(),requests=[],listeners={};
function element(id){if(!elements.has(id))elements.set(id,{id,tagName:'DIV',value:'',disabled:false,hidden:false,
  classList:{remove(){},toggle(){}},replaceChildren(){},setAttribute(){},removeAttribute(){}});return elements.get(id);}
for(const m of html.matchAll(/<input\b([^>]+)>/g)){const a=Object.fromEntries([...m[1].matchAll(/([\w-]+)="([^"]*)"/g)].map(x=>[x[1],x[2]]));if(a.id)Object.assign(element(a.id),a,{tagName:'INPUT'});}
const fresh=()=>({session:'scene',status:{mode:'manual',ready:true,age_seconds:0,command:[0,0]},mission_status:{mission:'idle'},mapping:{clearance_confirmed:true},scene:{available:true,phase:'ready'}});
let nextStatus=null,nextManual=null,clock=100;
const deferred=()=>{let resolve,reject;const promise=new Promise((a,b)=>{resolve=a;reject=b;});return{promise,resolve,reject};};
const response=data=>({ok:true,json:async()=>data});
const context=vm.createContext({console,performance:{now:()=>clock},AbortSignal:{timeout:()=>undefined},Blob,URL,queueMicrotask,setInterval:()=>0,setTimeout:()=>0,
  navigator:{sendBeacon:route=>{requests.push(route);return true;}},window:{addEventListener(){}},
  document:{hidden:false,getElementById:element,querySelectorAll:()=>[],addEventListener:(k,v)=>listeners[k]=v,createElement:()=>element('new'),createElementNS:()=>element('svg')},
  fetch:async(route,options={})=>{requests.push(route);if(route==='/camera.jpg')return{ok:false};if(route==='/status'){const pending=nextStatus;nextStatus=null;return pending?pending.promise:response(fresh());}if(route==='/manual'&&nextManual){const pending=nextManual;nextManual=null;return pending.promise;}return response({success:true,session:'token',message:'ok'});}});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const run=code=>vm.runInContext(code,context),flush=async()=>{for(let i=0;i<12;i++)await Promise.resolve();};
async function main(){
  await flush();
  const old=nextStatus=deferred();const pending=run('refresh()');await flush();
  await run("act('manual')");await flush();assert.equal(run('token'),'token');
  old.resolve(response({...fresh(),status:{...fresh().status,mode:'stopped'}}));await pending;await flush();
  assert.equal(run('token'),'token','A status request begun before Enable must not revoke the new selection');
  const delayed=nextStatus=deferred();const older=run('refresh()');await flush();await run('refresh()');
  delayed.resolve(response({...fresh(),status:{...fresh().status,ready:false}}));await older;
  assert.equal(run('token'),'token','An older response must not overwrite a newer healthy status');
  const failed=nextStatus=deferred();const failedRequest=run('refresh()');await flush();
  await run("act('manual')");await flush();failed.reject(Error('Old request timed out'));await failedRequest;
  assert.equal(run('token'),'token','Old request errors must not disarm a later selection');
  run("held('ArrowUp',true)");await flush();clock=450;await run('refresh()');await run('sendDrive()');assert.equal(run('keys.size'),0,'A scheduling pause must release held directions');assert.equal(run('token'),'token');
  const before=requests.length;run('setCameraExpanded(true);setCameraExpanded(false)');await flush();
  assert.ok(!requests.slice(before).some(p=>['/manual','/stop','/start','/restart','/new-scene','/release'].includes(p)));
  const fault=nextStatus=deferred();const current=run('refresh()');fault.resolve(response({...fresh(),status:{...fresh().status,ready:false}}));await current;
  assert.equal(run('token'),null,'A current readiness failure must still disarm');
  const late=nextManual=deferred();const selecting=run("act('manual')");await flush();await run("act('stop')");
  late.resolve(response({success:true,session:'late-token'}));await selecting;await flush();assert.equal(run('token'),null,'Stop must fence a late Enable response');
  const beforeDiagnostics=requests.length;
  run("diagnostics({status:{fault:{reason:'unsafe_or_unobserved_ground',detail:{causes:['unknown_terrain']}}},mapping:{terrain_evidence:{enabled:true,pending_cells:4,max_supporting_views:2,required_views:3,weak_hazard_cells:8,protected_cells:3,cleared_cells:1}}})");
  assert.match(element('rejection').textContent,/full predicted motion sweep/);
  assert.match(element('terrain-evidence-health').textContent,/2\/3/);
  assert.match(element('terrain-evidence-health').textContent,/3 protected hazards stay blocked/);
  assert.match(element('terrain-evidence-health').textContent,/clearing never releases a safety stop/);
  assert.equal(requests.length,beforeDiagnostics,'Diagnostics must not send control requests');
  console.log('Control lifecycle passed: delayed status/error, response order, current fault, late Enable, camera expansion.');
}
main().catch(e=>{console.error(e);process.exitCode=1;});
