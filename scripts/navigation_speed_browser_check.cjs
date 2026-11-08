// Read-only rover acceptance in a separate Chrome profile when the in-app browser is unavailable.
// Speed widgets are exercised while disarmed. This script never confirms clearance or enables motion.
const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),net=require('node:net');
const {spawn}=require('node:child_process');
const delay=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function main(){
  const output=process.argv[2];
  if(!output)throw Error('Usage: node scripts/navigation_speed_browser_check.cjs /new/output-prefix');
  for(const extension of ['.json','.png'])if(fs.existsSync(output+extension))throw Error('Evidence already exists: '+output+extension);
  fs.mkdirSync(path.dirname(output),{recursive:true});
  const listener=net.createServer();await new Promise(resolve=>listener.listen(0,'127.0.0.1',resolve));
  const port=listener.address().port;await new Promise(resolve=>listener.close(resolve));
  const profile=fs.mkdtempSync(path.join(os.tmpdir(),'rover-speed-browser-'));
  const chrome=spawn('C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe',[
    '--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check',
    '--disable-background-networking','--remote-debugging-address=127.0.0.1',
    `--remote-debugging-port=${port}`,`--user-data-dir=${profile}`,'about:blank'],{windowsHide:true,stdio:'ignore'});
  let ws,identifier=0;const pending=new Map();
  function call(method,params={}){return new Promise((resolve,reject)=>{
    const id=++identifier,timeout=setTimeout(()=>{pending.delete(id);reject(Error('Browser timeout: '+method));},10000);
    pending.set(id,{resolve:value=>{clearTimeout(timeout);resolve(value);},reject});ws.send(JSON.stringify({id,method,params}));
  });}
  try{
    let version;for(let i=0;i<80;i++){try{version=await(await fetch(`http://127.0.0.1:${port}/json/version`)).json();break;}catch{await delay(100);}}
    if(!version)throw Error('Chrome did not start');
    const target=await(await fetch(`http://127.0.0.1:${port}/json/new?about:blank`,{method:'PUT'})).json();
    ws=new WebSocket(target.webSocketDebuggerUrl);await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
    ws.onmessage=event=>{const message=JSON.parse(event.data);if(!message.id)return;const item=pending.get(message.id);if(!item)return;pending.delete(message.id);message.error?item.reject(Error(message.error.message)):item.resolve(message.result);};
    const evaluate=async expression=>{const result=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(result.exceptionDetails)throw Error(JSON.stringify(result.exceptionDetails));return result.result.value;};
    await call('Page.enable');await call('Emulation.setDeviceMetricsOverride',{width:1440,height:1800,deviceScaleFactor:1,mobile:false});
    await call('Page.navigate',{url:'http://127.0.0.1:8774'});
    const deadline=Date.now()+90000;
    while(Date.now()<deadline){if(await evaluate("typeof state!=='undefined' && state.status?.age_seconds<.6 && document.getElementById('camera').naturalWidth>0"))break;await delay(500);}
    assert.ok(await evaluate("typeof state!=='undefined' && state.status?.age_seconds<.6 && document.getElementById('camera').naturalWidth>0"),'Fresh rover status and camera required');
    const snapshot=()=>evaluate("({session:state.session,guard:state.status,scene:state.scene,mapping:state.mapping,limits:state.manual_speed_limits,camera:el('camera-state').textContent,drive:el('linear-speed-value').textContent,turn:el('angular-speed-value').textContent,linearMax:el('linear-speed').max,angularMax:el('angular-speed').max,token})");
    const before=await snapshot();assert.equal(before.guard.mode,'stopped');assert.equal(before.token,null);
    assert.equal(before.drive,'0.20 m/s');assert.equal(before.turn,'0.30 rad/s');
    assert.equal(Number(before.linearMax),.5);assert.equal(Number(before.angularMax),.6);
    await evaluate("window.speedCheckPosts=[];const originalFetch=fetch;window.fetch=(url,options={})=>{if(options.method==='POST')speedCheckPosts.push(url);return originalFetch(url,options);};");
    await evaluate("el('linear-speed-up').click()");
    assert.equal(await evaluate("el('linear-speed-value').textContent"),'0.21 m/s');
    await evaluate("el('linear-speed-down').click()");
    assert.equal(await evaluate("el('linear-speed-value').textContent"),'0.20 m/s');
    await evaluate("el('linear-speed').value='.5';el('linear-speed').dispatchEvent(new Event('input',{bubbles:true}));el('angular-speed').value='.6';el('angular-speed').dispatchEvent(new Event('input',{bubbles:true}));");
    assert.equal(await evaluate("el('linear-speed-value').textContent"),'0.50 m/s');
    assert.equal(await evaluate("el('angular-speed-value').textContent"),'0.60 rad/s');
    assert.ok(await evaluate("el('linear-speed-up').disabled && el('angular-speed-up').disabled"));
    await evaluate("el('speed-reset').click()");await delay(500);
    const after=await snapshot();assert.equal(after.drive,'0.20 m/s');assert.equal(after.turn,'0.30 rad/s');
    assert.equal(after.token,null);assert.equal(after.guard.mode,'stopped');assert.deepEqual(after.guard.command,[0,0]);
    const posts=await evaluate('speedCheckPosts');assert.deepEqual(posts,[],'Speed selection while stopped must send no control request');
    const screenshot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true});
    fs.writeFileSync(output+'.png',Buffer.from(screenshot.data,'base64'),{flag:'wx'});
    const report={passed:true,motionRequested:false,checks:['default speeds','increment/decrement','upper limits','reset','selection sends no commands','fresh camera','stopped rover'],before,after,posts};
    fs.writeFileSync(output+'.json',JSON.stringify(report,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify({passed:true,session:after.session,camera:after.camera,drive:after.drive,turn:after.turn,guard:after.guard.mode,screenshot:output+'.png'}));
  }finally{if(ws)ws.close();chrome.kill();}
}
main().catch(error=>{console.error(error);process.exitCode=1;});
