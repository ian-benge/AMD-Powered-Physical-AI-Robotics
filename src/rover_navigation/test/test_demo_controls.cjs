// Execute the shipped autonomous panel; no connection to any rover.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../rover_navigation/demo_panel.html'),'utf8');
const elements=new Map(),timers=[],requests=[];let now=100;
function element(id){if(!elements.has(id))elements.set(id,{disabled:false,classList:{toggle(){},remove(){}},before(){},addEventListener(){},replaceChildren(){},setAttribute(){},append(){}});return elements.get(id);}
const status={session_id:'fresh',mission_id:'one',epoch:0,age_seconds:0,phase:'ready',reason:'',elapsed_seconds:0,remaining_seconds:720,actual_distance_m:0,trails:[],policy:{},navigation_health:{healthy:true,missing:[]}};
const context=vm.createContext({console,crypto:{randomUUID:()=> 'operator-owned-token'},performance:{now:()=>now},
  document:{getElementById:element,querySelector:()=>element('heading'),createElement:()=>element('new'),createElementNS:()=>element('svg')},
  window:{addEventListener(){}},state:{demo_status:status,status:{ready:true,age_seconds:0},mapping:{clearance_confirmed:true}},lastStatus:100,
  setInterval:fn=>timers.push(fn),refresh(){},act:async action=>requests.push({action}),
  post:async(action,data)=>{requests.push({action,data});return{success:true};}});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const run=code=>vm.runInContext(code,context),flush=async()=>{for(let i=0;i<12;i++)await Promise.resolve();};
(async()=>{
 await timers[0]();assert.equal(requests.length,0,'An observer must not keep autonomy armed');
 await element('demo-explore').onclick();await flush();assert.equal(requests[0].action,'demo/start-exploration');
 status.phase='exploring';await timers[0]();await flush();
 assert.equal(requests.at(-1).action,'demo/heartbeat');assert.equal(requests.at(-1).data.sequence,1);
 await timers[0]();assert.equal(requests.at(-1).data.sequence,2);
 now=1000;const before=requests.length;await timers[0]();assert.equal(requests.length,before,'Stale browser status must not renew heartbeat');
 now=100;status.phase='awaiting_target';run('window.demoUpdate(state)');
 context.state.platform_temperature={tctl_c:81.5,age_seconds:.5};run('window.demoUpdate(state)');
 assert.match(element('demo-temperature').textContent,/81\.5/);
 context.state.platform_temperature=null;run('window.demoUpdate(state)');
 assert.match(element('demo-temperature').textContent,/native temperature terminal/,'Unavailable readings must not freeze a prior temperature');
 assert.equal(element('demo-point').disabled,false);await timers[0]();assert.equal(requests.length,before,'Waiting at home must not depart automatically');
 status.phase='preview_ready';status.preview={id:'preview-one'};status.preview_remaining_seconds=0;run('window.demoUpdate(state)');
 assert.equal(element('demo-approve').disabled,true,'Expired previews must be disabled');
 status.phase='stopped';run('window.demoUpdate(state)');status.phase='exploring';await timers[0]();assert.equal(requests.length,before,'Stopped missions never rearm automatically');
 console.log('Demo panel passed: observer isolation, explicit owner, increasing heartbeat, stale status, target wait, expired approval, Stop.');
})().catch(e=>{console.error(e);process.exitCode=1;});
