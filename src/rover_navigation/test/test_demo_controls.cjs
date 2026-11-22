// Execute the shipped autonomous panel; no connection to any rover.
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const html=fs.readFileSync(path.join(__dirname,'../rover_navigation/demo_panel.html'),'utf8');
const elements=new Map(),timers=[],requests=[],timeouts=[];let now=100,finishApproval,stallHeartbeat=false;
function element(id){if(!elements.has(id))elements.set(id,{disabled:false,classList:{toggle(){},remove(){}},before(){},addEventListener(){},replaceChildren(){},setAttribute(){},append(){}});return elements.get(id);}
const status={session_id:'fresh',mission_id:'one',epoch:0,age_seconds:0,phase:'ready',reason:'',elapsed_seconds:0,remaining_seconds:720,actual_distance_m:0,trails:[],policy:{},navigation_health:{healthy:true,missing:[]}};
const context=vm.createContext({console,crypto:{randomUUID:()=> 'operator-owned-token'},performance:{now:()=>now},
  AbortSignal:{timeout(ms){const controller=new AbortController();timeouts.push({ms,controller});return controller.signal;}},
  document:{getElementById:element,querySelector:()=>element('heading'),createElement:()=>element('new'),createElementNS:()=>element('svg')},
  window:{addEventListener(){}},state:{demo_status:status,status:{ready:true,age_seconds:0},mapping:{clearance_confirmed:true}},lastStatus:100,
  setInterval:fn=>timers.push(fn),refresh(){},act:async action=>requests.push({action}),
  post:async(action,data,options)=>{requests.push({action,data});
   if(action==='demo/start-sample')return new Promise(resolve=>finishApproval=resolve);
   if(action==='demo/heartbeat'&&stallHeartbeat)return new Promise((_,reject)=>options.signal.addEventListener('abort',()=>reject(Error('Timed out'))));
   return{success:true,mission_id:data?.mission_id};}});
vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],context);
const run=code=>vm.runInContext(code,context),flush=async()=>{for(let i=0;i<12;i++)await Promise.resolve();};
(async()=>{
 await timers[0]();assert.equal(requests.length,0,'An observer must not keep autonomy armed');
 run('window.demoUpdate(state)');
 assert.equal(element('demo-explore').disabled,true);
 context.state.mapping.publication_latency_p95_seconds=.12;
 run('window.demoUpdate(state)');
 assert.equal(element('demo-explore').disabled,false);
 await element('demo-explore').onclick();await flush();assert.equal(requests[0].action,'demo/start-exploration');
 status.phase='exploring';await timers[0]();await flush();
 status.mapping_remaining_seconds=1450;run('window.demoUpdate(state)');
 assert.match(element('demo-time').textContent,/1450 s mapping left/);
 assert.equal(requests.at(-1).action,'demo/heartbeat');assert.equal(requests.at(-1).data.sequence,1);
 await timers[0]();assert.equal(requests.at(-1).data.sequence,2);
 stallHeartbeat=true;now=250;const pending=timers[0]();const waiting=requests.length;
 await timers[0]();assert.equal(requests.length,waiting,'Only one heartbeat may be in flight');
 assert.equal(timeouts.at(-1).ms,250);timeouts.at(-1).controller.abort();await pending;
 stallHeartbeat=false;now=550;await timers[0]();assert.equal(requests.length,waiting+1,'A timed-out reply must allow the next heartbeat');
 now=850;await timers[0]();assert.equal(requests.length,waiting+2,'A current receipt bridges a delayed full dashboard update');
 now=1000;status.epoch=1;await timers[0]();assert.equal(requests.length,waiting+2,'A receipt from another epoch cannot bridge stale status');
 status.epoch=0;now=1300;await timers[0]();const before=requests.length;
 now=1601;await timers[0]();assert.equal(requests.length,before,'Persistent dashboard status loss must stop heartbeat');
 context.lastStatus=now;status.phase='awaiting_target';run('window.demoUpdate(state)');
 context.state.platform_temperature={tctl_c:88.5,age_seconds:.5,qualification_stop_c:89,native_stop_c:90};
 run('window.demoUpdate(state)');
 assert.equal(element('demo-temperature').textContent,'88.5°C · qualification stop 89°C · native stop 90°C');
 context.state.platform_temperature={tctl_c:91.5,age_seconds:.5,qualification_stop_c:92,native_stop_c:92};
 run('window.demoUpdate(state)');
 assert.equal(element('demo-temperature').textContent,'91.5°C · stop 92°C');
 context.state.platform_temperature=null;run('window.demoUpdate(state)');
 assert.match(element('demo-temperature').textContent,/native temperature terminal/,'Unavailable readings must not freeze a prior temperature');
 assert.equal(element('demo-point').disabled,false);await timers[0]();assert.equal(requests.length,before,'Waiting at home must not depart automatically');
 status.phase='preview_ready';status.preview={id:'preview-one'};status.preview_remaining_seconds=0;run('window.demoUpdate(state)');
 assert.equal(element('demo-approve').disabled,true,'Expired previews must be disabled');
 status.preview_remaining_seconds=20;run('window.demoUpdate(state)');
 const approval=element('demo-approve').onclick();await flush();
 assert.equal(requests.at(-1).action,'demo/start-sample');
 await timers[0]();assert.equal(requests.at(-1).action,'demo/heartbeat','The owner renews contact while approval is pending');
 assert.ok(requests.at(-1).data.sequence>2);
 finishApproval({success:true});await approval;
 const afterApproval=requests.length;
 status.phase='stopped';run('window.demoUpdate(state)');status.phase='exploring';await timers[0]();assert.equal(requests.length,afterApproval,'Stopped missions never rearm automatically');
 console.log('Demo panel passed: bounded heartbeat retry, delayed dashboard update, stale epoch and status, sample approval, and Stop.');
})().catch(e=>{console.error(e);process.exitCode=1;});
