// Execute BOTH shipped scripts as classic scripts in the same page. No rover I/O.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const base=fs.readFileSync(path.join(__dirname,'../rover_navigation/mission_controls.html'),'utf8');
const panel=fs.readFileSync(path.join(__dirname,'../rover_navigation/demo_panel.html'),'utf8');
const html=base.replace('</script></html>','</script>'+panel+'</html>');
const flush=async()=>{for(let i=0;i<50;i++)await Promise.resolve();};
async function exercise(secure){
 const elements=new Map(),requests=[],timers=[];let rejectStart=false;
 function element(id){if(!elements.has(id))elements.set(id,{id,value:'',disabled:id==='demo-explore',checked:false,
  classList:{remove(){},toggle(){}},before(){},append(){},replaceChildren(){},setAttribute(){},removeAttribute(){},addEventListener(){}});return elements.get(id);}
 for(const m of html.matchAll(/<input\b([^>]+)>/g)){const a=Object.fromEntries([...m[1].matchAll(/([\w-]+)="([^"]*)"/g)].map(x=>[x[1],x[2]]));if(a.id)Object.assign(element(a.id),a,{tagName:'INPUT'});}
 const demo={session_id:'scene',mission_id:'one',epoch:0,age_seconds:0,phase:'ready',reason:'',elapsed_seconds:0,
  remaining_seconds:720,actual_distance_m:0,trails:[],policy:{},frontier_visits:[],navigation_health:{healthy:true,missing:[]}};
 const status={session:'scene',status:{mode:'stopped',ready:true,age_seconds:0,command:[0,0]},
  demo_status:demo,mission_status:{mission:'idle'},mapping:{clearance_confirmed:true},scene:{available:true,phase:'ready'}};
 const response=(value,ok=true)=>({ok,json:async()=>value});
 const crypto={getRandomValues:array=>{for(let i=0;i<array.length;i++)array[i]=i+17;return array;}};
 if(secure)crypto.randomUUID=()=> 'an-explicitly-owned-operator-token';
 const context=vm.createContext({console,crypto,Uint8Array,performance:{now:()=>100},AbortSignal:{timeout:()=>undefined},Blob,URL,
  setInterval:(fn,ms)=>timers.push({fn,ms}),setTimeout:()=>0,clearTimeout(){},navigator:{sendBeacon:()=>true},
  window:{addEventListener(){}},document:{hidden:false,getElementById:element,querySelector:()=>element('heading'),querySelectorAll:()=>[],addEventListener(){},createElement:()=>element('new'),createElementNS:()=>element('svg')},
  fetch:async(route,options={})=>{requests.push({route,...options});if(route==='/camera.jpg')return{ok:false};
   if(route==='/status')return response(status);
   if(route==='/demo/start-exploration'&&rejectStart)return response({success:false,message:'Inspect the rejected footprint'},false);
   return response({success:true,message:'Accepted'});}});
 for(const match of html.matchAll(/<script>([\s\S]*?)<\/script>/g))vm.runInContext(match[1],context);
 await flush();assert.equal(element('demo-explore').disabled,false,'The full page must enable Start after healthy status');
 demo.navigation_health={healthy:false,missing:['estimated_odometry']};vm.runInContext('window.demoUpdate(state)',context);
 assert.equal(element('demo-explore').disabled,true);assert.match(element('demo-message').textContent,/estimated_odometry/);
 demo.navigation_health={healthy:true,missing:[]};vm.runInContext('window.demoUpdate(state)',context);
 assert.equal(element('demo-explore').disabled,false);
 assert.ok(!requests.some(r=>r.route.startsWith('/demo/')),'Opening the page never authorizes autonomy');
 await element('demo-explore').onclick();await flush();
 const start=requests.find(r=>r.route==='/demo/start-exploration');assert.ok(start,'Click must reach the real shared POST helper');
 assert.equal(start.method,'POST');const body=JSON.parse(start.body);
 assert.equal(body.session_id,'scene');assert.equal(body.mission_id,'one');assert.ok(body.client_token.length>=16);
 demo.phase='exploring';await timers.find(t=>t.ms===200).fn();await flush();
 assert.equal(requests.at(-1).route,'/demo/heartbeat');
 demo.phase='ready';rejectStart=true;await element('demo-explore').onclick();await flush();
 assert.match(element('demo-message').textContent,/Inspect the rejected footprint/,'A failed command must stay visible after status refresh');
}
(async()=>{await exercise(true);await exercise(false);console.log('Integrated demo page passed: shared status, enabled Start, real POST helper, secure/insecure context, explicit heartbeat, visible rejection.');})().catch(e=>{console.error(e);process.exitCode=1;});
