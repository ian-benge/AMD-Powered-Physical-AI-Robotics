// Execute BOTH shipped scripts as classic scripts in the same page. No rover I/O.
const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const base=fs.readFileSync(path.join(__dirname,'../rover_navigation/mission_controls.html'),'utf8');
const panel=fs.readFileSync(path.join(__dirname,'../rover_navigation/demo_panel.html'),'utf8');
const html=base.replace('</script></html>','</script>'+panel+'</html>');
const flush=async()=>{for(let i=0;i<50;i++)await Promise.resolve();};
async function exercise(secure){
 const elements=new Map(),requests=[],timers=[],draws=[];
 let rejectStart=false,rejectReturn=false,rejectStatus=false,nextElement=0,now=100,decode,delayStatus;
 function element(id){if(!elements.has(id))elements.set(id,{id,value:'',disabled:id==='demo-explore',checked:false,
  attributes:{},events:{},children:[],preceding:[],classList:{remove(){},toggle(){}},
  before(item){this.preceding.push(item);},
  append(...items){this.children.push(...items);},replaceChildren(...items){this.children=items;},
  setAttribute(k,v){this.attributes[k]=v;},removeAttribute(k){delete this.attributes[k];},
  addEventListener(k,fn){this.events[k]=fn;},remove(){this.removed=true;}});return elements.get(id);}
 for(const m of html.matchAll(/<input\b([^>]+)>/g)){const a=Object.fromEntries([...m[1].matchAll(/([\w-]+)="([^"]*)"/g)].map(x=>[x[1],x[2]]));if(a.id)Object.assign(element(a.id),a,{tagName:'INPUT'});}
 const demo={session_id:'scene',mission_id:'one',epoch:0,age_seconds:0,phase:'ready',reason:'',elapsed_seconds:0,
  remaining_seconds:720,actual_distance_m:0,trails:[],policy:{},frontier_visits:[],navigation_health:{healthy:true,missing:[]}};
 const status={session:'scene',status:{mode:'stopped',ready:true,age_seconds:0,command:[0,0]},
  demo_status:demo,mission_status:{mission:'idle'},mapping:{clearance_confirmed:true,publication_latency_p95_seconds:.12},
  scene:{available:true,phase:'ready'}};
 const response=(value,ok=true)=>({ok,json:async()=>value});
 const canvas={drawImage(image){draws.push(image.src);},clearRect(){draws.push('reset');}};
 element('terrain').getContext=()=>canvas;
 const crypto={getRandomValues:array=>{for(let i=0;i<array.length;i++)array[i]=i+17;return array;}};
 if(secure)crypto.randomUUID=()=> 'an-explicitly-owned-operator-token';
 const context=vm.createContext({console,crypto,Uint8Array,performance:{now:()=>now},AbortSignal:{timeout:()=>undefined},Blob,URL,
  setInterval:(fn,ms)=>timers.push({fn,ms}),setTimeout:()=>0,clearTimeout(){},navigator:{sendBeacon:()=>true},
  Image:class{decode(){return new Promise((resolve,reject)=>decode={resolve,reject});}},
  window:{addEventListener(){}},document:{hidden:false,getElementById:element,querySelector:()=>element('heading'),querySelectorAll:()=>[],addEventListener(){},createElement:()=>element('new'+nextElement++),createElementNS:()=>element('svg'+nextElement++)},
  fetch:async(route,options={})=>{requests.push({route,...options});if(route==='/camera.jpg')return{ok:false};
   if(route==='/status')return delayStatus?delayStatus.then(()=>response(status,!rejectStatus)):response(status,!rejectStatus);
   if(route.startsWith('/map.png?'))return {ok:true,blob:async()=>new Blob(['test frame'])};
   if(route==='/demo/start-exploration'&&rejectStart)return response({success:false,message:'Inspect the rejected footprint'},false);
   if(route==='/demo/return-now'&&rejectReturn)return response({success:false,message:'Return check unavailable. Try again.'},false);
   return response({success:true,message:'Accepted',mission_id:demo.mission_id});}});
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
 const pulse=timers.find(t=>t.ms===150).fn;
 demo.phase='exploring';await pulse();await flush();
 assert.equal(requests.at(-1).route,'/demo/heartbeat');
 let releaseStatus;delayStatus=new Promise(resolve=>releaseStatus=resolve);
 const slowStatus=vm.runInContext('refresh()',context);
 const statusCount=requests.filter(r=>r.route==='/status').length;
 timers.find(t=>t.ms===250).fn();await flush();
 assert.equal(requests.filter(r=>r.route==='/status').length,statusCount,'Periodic status requests must not overlap');
 releaseStatus();await slowStatus;delayStatus=null;
 status.map={width:80,height:80,resolution:.05,origin:[-2,-2],stamp:1,view_box:[1,1,23,24]};
 status.sample_sites={schema_version:2,ready:true,age_seconds:.1,
  objects:[{track_id:'0:1',class_name:'rock',state:'confirmed',map_position:[1,0,.06],confirmation_view_count:3}],
  tentative_objects:[{track_id:'0:2',class_name:'mudstone_slab',state:'tentative',map_position:[0,1,0],confirmation_view_count:1}]};
 status.soil_sites={schema_version:1,ready:true,age_seconds:.1,sites:[
  {track_id:'0:1',class_name:'pale_soil_patch',state:'confirmed',map_position:[-1,0,0],confirmation_view_count:3}]};
 await vm.runInContext('refresh()',context);
 await flush();
 const map=element('map');
 assert.equal(map.attributes.viewBox,'0 0 32 32');
 decode.resolve();await flush();
 assert.equal(draws.length,1);
 now+=1001;status.map.stamp=2;status.map.view_box=[2,2,23,24];
 await vm.runInContext('refresh()',context);
 await flush();
 assert.equal(map.attributes.viewBox,'0 0 32 32','Small coverage changes should stay within the same crop');
 assert.equal(draws.length,1,'The last image stays visible during a decode');
 decode.reject(new Error('bad image'));await flush();
 assert.equal(draws.length,1,'An invalid frame must leave the canvas in place');
 now+=1001;status.map.stamp=3;status.map.view_box=[1,1,44,24];
 await vm.runInContext('refresh()',context);
 await flush();
 decode.resolve();await flush();
 assert.equal(draws.length,2);
 assert.equal(map.attributes.viewBox,'0 0 64 32','A larger coverage region expands the crop');
 status.map.view_box=[1,1,23,24];
 await vm.runInContext('refresh()',context);
 assert.equal(map.attributes.viewBox,'0 0 64 32','A smaller observed region must not zoom the map back in');
 const full=element('demo-full-map');
 full.checked=true;full.onchange();
 assert.equal(map.attributes.viewBox,'0 0 80 80');
 full.checked=false;full.onchange();
 assert.equal(map.attributes.viewBox,'0 0 64 32');
 demo.trails=[
  {phase:'exploring',segment:1,pose:[0,0]},{phase:'exploring',segment:1,pose:[.1,0]},
  {phase:'exploration_reposition',segment:2,pose:[.1,0]},{phase:'exploration_reposition',segment:2,pose:[.1,.1]},
  {phase:'exploring',segment:3,pose:[.1,.1]},{phase:'exploring',segment:3,pose:[.2,.1]},
  {phase:'exploring',segment:4,pose:[.25,.1]},{phase:'exploring',segment:4,pose:[.3,.1]},
 ];
 await vm.runInContext('refresh()',context);
 const traces=element('markers').children[0].children.filter(p=>p.attributes['data-demo-route']==='executed');
 assert.equal(traces.length,4,'A reposition and a pause separate otherwise similar trail segments');
 assert.ok(traces.every(p=>p.attributes.points.split(' ').length===2),'No straight bridge is drawn between segments');
 const markers=element('object-markers').children;
 assert.equal(markers.length,2,'Raised and soil IDs occupy separate marker namespaces');
 assert.equal(markers[0].children[0].attributes.fill,'#60a5fa');
 assert.equal(markers[1].children[0].attributes.fill,'#e8ba94');
 markers[0].events.pointerenter();
 assert.match(element('map-tooltip').textContent,/Rock #0:1.*confirmed/);
 await vm.runInContext('refresh()',context);
 assert.equal(element('object-markers').children[0],markers[0],'Telemetry refresh preserves hover and focus');
 assert.equal(element('map-tooltip').hidden,false);
 rejectStatus=true;
 await vm.runInContext('refresh()',context);
 assert.equal(element('object-markers').children[0],markers[0],'A missed status response must keep visible markers');
 rejectStatus=false;
 markers[0].events.pointerleave();
 assert.equal(element('map-tooltip').hidden,true);
 const click={stopPropagation(){}};
 markers[0].events.click(click);
 assert.ok(!requests.some(r=>r.route==='/demo/select-target'),'An active mapping drive cannot accept a sample selection');
 demo.phase='exploration_paused';demo.resume_ready=true;demo.distinct_route_available=true;
 demo.auto_recovery=true;
 await vm.runInContext('refresh()',context);
 assert.equal(element('demo-resume').disabled,true);
 await pulse();await flush();
 assert.equal(requests.at(-1).route,'/demo/heartbeat','An automatic replan continues the owner lease');
 assert.ok(!requests.some(r=>r.route==='/demo/resume-exploration'));
 demo.reason='Navigation failed: Nav2 aborted, error 105';
 await vm.runInContext('refresh()',context);
 assert.match(element('demo-message').textContent,/error 105/);
 demo.phase='exploration_reposition';demo.auto_recovery=false;demo.reason='';
 demo.frontier=[-.8,-.3,0];demo.frontier_kind='clearance';
 demo.outward_path=[[0,0,0],demo.frontier];
 await vm.runInContext('refresh()',context);
 const recovery=element('markers').children[0].children.find(p=>p.attributes['aria-label']==='Recovery waypoint');
 assert.ok(recovery?.children.some(p=>p.textContent==='F'),'The checked recovery waypoint has an F marker');
 assert.equal(element('demo-message').textContent,'Following a checked recovery route.');
 demo.phase='exploring';demo.frontier=[.8,.3,0];demo.frontier_kind='frontier';
 await vm.runInContext('refresh()',context);
 const waypoint=element('markers').children[0].children.find(p=>p.attributes['aria-label']==='Frontier waypoint');
 assert.ok(waypoint?.children.some(p=>p.textContent==='F'));
 assert.equal(element('demo-message').textContent,'Exploration resumed.');
 demo.phase='exploration_paused';demo.frontier=null;demo.outward_path=[];
 demo.auto_recovery=false;
 await vm.runInContext('refresh()',context);
 assert.equal(element('demo-resume').disabled,false);
 assert.ok(!requests.some(r=>r.route==='/demo/resume-exploration'));
 await element('demo-resume').onclick();await flush();
 const resume=requests.find(r=>r.route==='/demo/resume-exploration');
 assert.equal(JSON.parse(resume.body).client_token,body.client_token);
 assert.equal(element('demo-alternate-route').disabled,false);
 await element('demo-alternate-route').onclick();await flush();
 const alternate=requests.find(r=>r.route==='/demo/alternate-exploration');
 assert.equal(JSON.parse(alternate.body).client_token,body.client_token);
 demo.phase='contact_paused';demo.contact_ready=false;
 await vm.runInContext('refresh()',context);
 assert.equal(element('demo-retreat').disabled,true);
 demo.contact_ready=true;
 await vm.runInContext('refresh()',context);
 assert.equal(element('demo-retreat').disabled,false);
 assert.ok(!requests.some(r=>r.route==='/demo/retreat-and-replan'));
 await element('demo-retreat').onclick();await flush();
 const retreat=requests.find(r=>r.route==='/demo/retreat-and-replan');
 assert.equal(JSON.parse(retreat.body).client_token,body.client_token);
 demo.phase='awaiting_target';
 await vm.runInContext('refresh()',context);
 markers[0].events.click(click);
 await flush();
 const selection=requests.find(r=>r.route==='/demo/select-target');
 assert.equal(JSON.parse(selection.body).target_id,'object:0:1');
 assert.ok(!requests.some(r=>r.route==='/demo/start-sample'),'Selecting a rock cannot authorize motion');
 element('tentative-toggle').checked=true;
 element('tentative-toggle').onchange();
 const slab=element('object-markers').children[2];
 assert.equal(slab.children[0].attributes.stroke,'#c4a4ff');
 assert.equal(slab.attributes['data-selectable'],'false');
 demo.target={class_name:'pale_soil_patch',track_id:'0:1'};
 demo.target_map=[-1,0];
 demo.preview={stopping_pose:[.2,0,Math.PI]};
 demo.phase='preview_ready';demo.preview_remaining_seconds=20;
 await vm.runInContext('refresh()',context);
 assert.match(element('demo-target').textContent,/stop 1.20 m from target/);
 await element('demo-alternate-sample').onclick();await flush();
 assert.ok(requests.some(r=>r.route==='/demo/alternate-sample'));
 assert.ok(!requests.some(r=>r.route==='/demo/start-sample'),'Regenerating a preview cannot authorize motion');
 const handoff=element('demo-sample-handoff'),sampleReturn=element('demo-sample-return'),returnInfo=element('demo-sample-return-info');
 assert.ok(element('stop').preceding.includes(handoff),'The sample action belongs in the sticky operator bar');
 demo.phase='sample_outbound';demo.sample_return_ready=true;
 await vm.runInContext('refresh()',context);
 assert.equal(handoff.hidden,false);
 assert.equal(sampleReturn.textContent,'Return home');
 assert.equal(sampleReturn.disabled,true);
 assert.match(returnInfo.textContent,/available after collection/);
 demo.phase='collecting';
 await vm.runInContext('refresh()',context);
 assert.equal(handoff.hidden,false,'The return control stays visible while collecting');
 assert.equal(sampleReturn.disabled,true);
 assert.match(returnInfo.textContent,/held stationary/);
 await sampleReturn.onclick();
 assert.ok(!requests.some(r=>r.route==='/demo/return-now'),'Collection does not authorize departure');
 demo.phase='awaiting_sample_return';
 await vm.runInContext('refresh()',context);
 assert.equal(sampleReturn.textContent,'Return without a sample');
 demo.sample={simulated:true,delivered:false};
 demo.sample_return_ready=false;
 await vm.runInContext('refresh()',context);
 assert.equal(sampleReturn.textContent,'Return home');
 assert.equal(sampleReturn.disabled,true);
 assert.match(returnInfo.textContent,/stationary rover and healthy navigation/);
 demo.sample_return_ready=true;demo.age_seconds=1.1;
 await vm.runInContext('refresh()',context);
 assert.equal(sampleReturn.disabled,true,'A stale result cannot authorize return');
 assert.match(returnInfo.textContent,/fresh rover status/);
 demo.age_seconds=0;
 await vm.runInContext('refresh()',context);
 assert.equal(sampleReturn.disabled,false);
 assert.match(returnInfo.textContent,/Sample collected/);
 assert.ok(!requests.some(r=>r.route==='/demo/return-now'),'The rover waits for explicit return');
 rejectReturn=true;await sampleReturn.onclick();await flush();
 assert.match(returnInfo.textContent,/Return check unavailable/,'The toolbar retains a refused return after refresh');
 rejectReturn=false;await sampleReturn.onclick();await flush();
 const returning=requests.filter(r=>r.route==='/demo/return-now').at(-1);
 assert.equal(JSON.parse(returning.body).client_token,body.client_token);
 demo.phase='sample_backing';await vm.runInContext('refresh()',context);
 assert.equal(handoff.hidden,true,'The operator sees the active reverse phase after accepting return');
 demo.phase='ready';rejectStart=true;await element('demo-explore').onclick();await flush();
 assert.match(element('demo-message').textContent,/Inspect the rejected footprint/,'A failed command must stay visible after status refresh');
 now+=1001;status.map.stamp=4;
 await vm.runInContext('refresh()',context);await flush();
 const staleDecode=decode;
 vm.runInContext('clearSceneView()',context);
 staleDecode.resolve();await flush();
 assert.equal(draws.length,3,'An old decode cannot paint after a scene reset');
 assert.equal(draws.at(-1),'reset');
}
(async()=>{await exercise(true);await exercise(false);console.log('Integrated demo page passed: explicit resume, persistent raster and markers, stable viewport.');})().catch(e=>{console.error(e);process.exitCode=1;});
