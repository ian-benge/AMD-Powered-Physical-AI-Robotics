#!/usr/bin/env node
/** Windows OBS recording + read-only mission synchronization. Never sends heartbeats or controls. */
import {createHash,randomBytes} from 'node:crypto';
import {mkdir,writeFile,open} from 'node:fs/promises';
import {resolve} from 'node:path';
import {parseArgs} from 'node:util';

const {values}=parseArgs({options:{output:{type:'string'},url:{type:'string',default:'http://127.0.0.1:8874'},
  port:{type:'string',default:'4455'},minutes:{type:'string',default:'35'},preflight:{type:'boolean'}}});
if(process.platform!=='win32')throw Error('Encoding must run on Windows');
if(!values.output)throw Error('Choose a new --output evidence directory');
const url=new URL(values.url),minutes=Number(values.minutes),port=Number(values.port);
if(!['localhost','127.0.0.1'].includes(url.hostname)||url.protocol!=='http:'||!Number.isInteger(port)||port<1||port>65535||!(minutes>0&&minutes<=45))throw Error('Use loopback HTTP, a valid OBS port and <=45 minutes');
const folder=resolve(values.output);await mkdir(folder);const log=await open(resolve(folder,'recording.jsonl'),'wx');
const pending=new Map();let identResolve,identReject,ended=false,owned=false,stopRequested=false;
const identified=new Promise((ok,no)=>{identResolve=ok;identReject=no;});
function event(kind,data={}){return log.write(JSON.stringify({utc:new Date().toISOString(),windows_monotonic_ms:performance.now(),kind,...data})+'\n');}
const ws=new WebSocket(`ws://127.0.0.1:${port}`);
function send(op,d){ws.send(JSON.stringify({op,d}));}
function rpc(requestType,requestData={}){return new Promise((ok,no)=>{
 const requestId=randomBytes(12).toString('hex');const timer=setTimeout(()=>{pending.delete(requestId);no(Error('OBS request timed out: '+requestType));},5000);
 pending.set(requestId,{ok,no,timer});send(6,{requestType,requestId,requestData});
});}
ws.addEventListener('error',()=>identReject(Error('Enable the OBS WebSocket server on loopback first')));
ws.addEventListener('close',()=>{ended=true;for(const p of pending.values()){clearTimeout(p.timer);p.no(Error('OBS disconnected'));}identReject(Error('OBS disconnected'));});
ws.addEventListener('message',({data})=>{
 const {op,d}=JSON.parse(String(data));
 if(op===0){let authentication;
  if(d.authentication){const password=process.env.OBS_WEBSOCKET_PASSWORD;if(!password){identReject(Error('Set OBS_WEBSOCKET_PASSWORD for the local OBS server'));return;}
   const secret=createHash('sha256').update(password+d.authentication.salt).digest('base64');
   authentication=createHash('sha256').update(secret+d.authentication.challenge).digest('base64');}
  send(1,{rpcVersion:1,eventSubscriptions:0,...authentication?{authentication}:{}});
 }else if(op===2)identResolve();
 else if(op===7){const p=pending.get(d.requestId);if(!p)return;clearTimeout(p.timer);pending.delete(d.requestId);
  d.requestStatus.result?p.ok(d.responseData):p.no(Error(d.requestStatus.comment||'OBS request failed'));}
});
const timeout=setTimeout(()=>identReject(Error('OBS handshake timed out')),7000);
process.on('SIGINT',()=>{stopRequested=true;});process.on('SIGTERM',()=>{stopRequested=true;});
process.stdin.setEncoding('utf8');process.stdin.on('data',text=>{void event('operator_video_marker',{text:text.trim().slice(0,500)});});
try{
 await identified;clearTimeout(timeout);
 const version=await rpc('GetVersion'),scene=await rpc('GetCurrentProgramScene'),items=await rpc('GetSceneItemList',{sceneName:scene.currentProgramSceneName});
 const inputs=[];for(const item of items.sceneItems)if(item.sceneItemEnabled&&item.sourceType==='OBS_SOURCE_TYPE_INPUT'){
  const input=await rpc('GetInputSettings',{inputName:item.sourceName});inputs.push({name:item.sourceName,kind:input.inputKind,settings:input.inputSettings});}
 if(!inputs.some(i=>/capture/.test(i.kind)))throw Error('Current OBS scene needs an enabled browser/window/display capture source; inspect its preview');
 const status=await fetch(new URL('/status',url),{signal:AbortSignal.timeout(2000)}).then(r=>{if(!r.ok)throw Error('Dashboard not ready');return r.json();});
 if(!status.demo_status?.mission_id||status.demo_status.phase!=='ready')throw Error('Start recording before exploration in a fresh ready mission');
 await writeFile(resolve(folder,'preflight.json'),JSON.stringify({version,scene,inputs,mission:status.demo_status,encoder_location:'Windows',operator_must_verify_preview:true},null,2));
 if(values.preflight){console.log('OBS and mission preflight passed. Inspect OBS preview and make a short playback test before acceptance.');}
 else{
  const current=await rpc('GetRecordStatus');if(current.outputActive)throw Error('A recording already exists; do not take ownership of another recording');
  await rpc('StartRecord');owned=true;await event('recording_started',{scene:scene.currentProgramSceneName});
  console.log('Recording on Windows. Leave this terminal open; Ctrl+C finalizes the original. Type markers here for exterior clips.');
  const until=performance.now()+minutes*60000;let lastMission=status.demo_status.mission_id;
  while(!stopRequested&&!ended&&performance.now()<until){const start=performance.now();
   try{const s=await fetch(new URL('/status',url),{signal:AbortSignal.timeout(1000)}).then(r=>r.json());
    await event('mission_snapshot',{http_round_trip_ms:performance.now()-start,status:s});
    if(s.demo_status?.mission_id!==lastMission){await event('mission_identity_changed',{previous:lastMission,current:s.demo_status?.mission_id});lastMission=s.demo_status?.mission_id;}
   }catch(e){await event('status_unavailable',{error:e.message});}
   await new Promise(ok=>setTimeout(ok,Math.max(0,500-(performance.now()-start))));
  }
 }
}finally{
 clearTimeout(timeout);
 try{for(const name of ['status','map.png']){const r=await fetch(new URL('/'+name,url),{signal:AbortSignal.timeout(2000)});if(r.ok)await writeFile(resolve(folder,name==='status'?'final-status.json':name),Buffer.from(await r.arrayBuffer()));}}catch(e){await event('final_snapshot_unavailable',{error:e.message});}
 if(owned&&!ended){try{const result=await rpc('StopRecord');await event('recording_finalized',result);console.log('Original recording:',result.outputPath);}catch(e){await event('recording_finalize_failed',{error:e.message});}}
 await log.sync();await log.close();ws.close();process.stdin.pause();
}
