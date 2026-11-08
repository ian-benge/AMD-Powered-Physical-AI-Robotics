// Standalone Chrome fallback when the in-app browser connection cannot start.
// A separate temporary profile keeps the user's existing browser untouched.
const fs = require('node:fs');
const path = require('node:path');
const net = require('node:net');
const {spawn} = require('node:child_process');
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

async function main() {
  const output = process.argv[2];
  if (!output) throw Error('Usage: node navigation_browser_check.cjs /new/output-prefix [--restart] [--drive | --turn-right]');
  const restart = process.argv.includes('--restart');
  const turn = process.argv.includes('--turn-right');
  const drive = process.argv.includes('--drive') || turn;
  if (turn && process.argv.includes('--drive')) throw Error('Choose a forward or turn check');
  fs.mkdirSync(path.dirname(output), {recursive:true});
  const listener = net.createServer();
  await new Promise(resolve => listener.listen(0, '127.0.0.1', resolve));
  const port = listener.address().port;
  await new Promise(resolve => listener.close(resolve));
  const profile = fs.mkdtempSync(path.join(require('node:os').tmpdir(), 'rover-browser-'));
  const chrome = spawn('C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe', [
    '--headless=new', '--disable-gpu', '--no-first-run', '--no-default-browser-check',
    '--disable-background-networking', '--remote-debugging-address=127.0.0.1',
    `--remote-debugging-port=${port}`, `--user-data-dir=${profile}`, 'about:blank'
  ], {windowsHide:true, stdio:'ignore'});
  let ws;
  const pending = new Map(); let identifier = 0;
  function call(method, params={}) {
    return new Promise((resolve,reject) => {
      const id = ++identifier;
      const timeout = setTimeout(()=>{pending.delete(id);reject(Error('Browser command timed out: '+method));},10000);
      pending.set(id, {resolve:result=>{clearTimeout(timeout);resolve(result);},reject});
      ws.send(JSON.stringify({id,method,params}));
    });
  }
  try {
    let version;
    for(let i=0;i<100;i++) {
      try {version=await (await fetch(`http://127.0.0.1:${port}/json/version`)).json();break;} catch(e) {await delay(100);}
    }
    if(!version) throw Error('Standalone Chrome did not start');
    const target = await (await fetch(`http://127.0.0.1:${port}/json/new?about:blank`, {method:'PUT'})).json();
    ws = new WebSocket(target.webSocketDebuggerUrl);
    await new Promise((resolve,reject)=>{ws.addEventListener('open',resolve,{once:true});ws.addEventListener('error',reject,{once:true});});
    ws.addEventListener('message',event=>{const m=JSON.parse(event.data);const p=pending.get(m.id);if(p){pending.delete(m.id);m.error?p.reject(Error(m.error.message)):p.resolve(m.result);}});
    await call('Page.enable');
    await call('Emulation.setDeviceMetricsOverride',{width:1280,height:1100,deviceScaleFactor:1,mobile:false});
    await call('Page.navigate',{url:'http://127.0.0.1:8874/'});
    async function evaluate(expression) {
      const result=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
      if(result.exceptionDetails) throw Error(result.exceptionDetails.text);
      return result.result.value;
    }
    async function until(expression, timeout=12000) {
      const end=Date.now()+timeout;
      while(Date.now()<end){if(await evaluate(expression))return;await delay(100);}
      const state=await evaluate("({guard:document.getElementById('guard')?.textContent,reply:document.getElementById('reply')?.textContent,manual:document.getElementById('manual')?.textContent,disabled:document.getElementById('manual')?.disabled})");
      throw Error('UI condition not reached: '+expression+'; '+JSON.stringify(state));
    }
    await until("!!document.getElementById('guard') && document.getElementById('guard').textContent !== 'Waiting for rover status'");
    await evaluate(`(()=>{window.checkTrace=[];const record=v=>{checkTrace.push({wall:Date.now(),...v});if(checkTrace.length>300)checkTrace.shift();};const rawPost=post;post=async(...args)=>{const started=performance.now();try{const r=await rawPost(...args);record({request:args[0],ms:performance.now()-started,success:r.success});return r;}catch(e){record({request:args[0],ms:performance.now()-started,error:e.message});throw e;}};const rawDisarm=disarm;disarm=()=>{if(token)record({disarm:true,guard:state.status,reply:el('reply').textContent,statusAge:performance.now()-lastStatus,stack:new Error().stack});return rawDisarm();};})()`);
    let restartEvidence;
    if(restart) {
      await until("!document.getElementById('restart').disabled");
      const previous=await evaluate("state.session");
      const box=await evaluate("(()=>{const r=document.getElementById('restart').getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()");
      await call('Input.dispatchMouseEvent',{type:'mousePressed',button:'left',clickCount:1,...box});
      await call('Input.dispatchMouseEvent',{type:'mouseReleased',button:'left',clickCount:1,...box});
      await until(`!restarting && state.session && state.session!==${JSON.stringify(previous)} && state.mapping?.waiting_for_confirmation`,150000);
      const fresh=await evaluate("fetch('/status').then(r=>r.json())");
      if(fresh.map?.known_cells!==0 || fresh.sample_sites?.stored_count!==0 || fresh.status.mode!=='stopped') throw Error('Restart did not leave an empty, stopped scene');
      restartEvidence={previousSession:previous,session:fresh.session,knownCells:fresh.map.known_cells,storedCandidates:fresh.sample_sites.stored_count,mode:fresh.status.mode};
      if(drive) {
        const confirm=await evaluate("(()=>{const r=document.getElementById('confirm').getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()");
        await call('Input.dispatchMouseEvent',{type:'mousePressed',button:'left',clickCount:1,...confirm});
        await call('Input.dispatchMouseEvent',{type:'mouseReleased',button:'left',clickCount:1,...confirm});
      }
    }
    if(!restart || drive) await until(`document.getElementById('guard')?.textContent.includes('ready') || (${!drive} && state.mapping?.waiting_for_confirmation)`);
    await until("document.getElementById('camera')?.naturalWidth > 0");
    await until("document.getElementById('terrain')?.getAttribute('href')?.startsWith('/map.png')");
    await evaluate("new Promise((resolve,reject)=>{const image=new Image();image.onload=()=>requestAnimationFrame(()=>requestAnimationFrame(()=>resolve(true)));image.onerror=()=>reject(Error('Terrain image did not load'));image.src=document.getElementById('terrain').getAttribute('href');})");
    const snapshot = () => evaluate("({heading:document.querySelector('h1').textContent, guard:document.getElementById('guard').textContent, camera:document.getElementById('camera-state').textContent, detector:document.getElementById('detector').textContent, mapping:document.getElementById('mapping').textContent, manualDisabled:document.getElementById('manual').disabled, sites:document.querySelectorAll('#sites li').length})");
    async function selectManual() {
      const box=await evaluate("(()=>{const r=document.getElementById('manual').getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2};})()");
      await call('Input.dispatchMouseEvent',{type:'mousePressed',button:'left',clickCount:1,...box});
      await call('Input.dispatchMouseEvent',{type:'mouseReleased',button:'left',clickCount:1,...box});
      await until("document.getElementById('guard').textContent.startsWith('manual') && document.getElementById('manual').textContent==='Manual drive enabled'");
    }
    async function press(key, code, type) {
      await call('Input.dispatchKeyEvent',{type,key,code:key===' '?'Space':key,windowsVirtualKeyCode:code,nativeVirtualKeyCode:code});
    }
    async function idle() {
      await until("document.getElementById('guard').textContent.startsWith('manual') && document.getElementById('command').textContent==='Command: 0.00 m/s, 0.00 rad/s'");
      await delay(1000); // Remain selected for longer than the unchanged command lease.
      const state=await snapshot();
      if(!state.guard.startsWith('manual') || !state.manualDisabled) throw Error('Manual selection did not persist at rest');
      return state;
    }
    const report={before:await snapshot(),driveRequested:drive,restart:restartEvidence};
    if(process.argv.includes('--tracking')) {
      await until("state.sample_sites?.schema_version===2 && state.sample_sites.tentative_objects.length>0 && !!document.getElementById('tentative-toggle')");
      if(await evaluate("document.getElementById('tentative-toggle').checked || document.querySelectorAll('[data-track-state=tentative],[data-track-state=unverified]').length>0")) throw Error('Tentative candidates are visible by default');
      report.trackingBefore=await evaluate("({confirmed:state.sample_sites.objects.length,pending:state.sample_sites.tentative_objects.length,text:el('mapping').textContent})");
      await evaluate("document.getElementById('tentative-toggle').click()");
      await until("document.querySelectorAll('[data-track-state]').length===state.sample_sites.objects.length+state.sample_sites.tentative_objects.length");
      report.trackingShown=await evaluate("({markers:document.querySelectorAll('[data-track-state]').length,list:el('sites').textContent})");
      await evaluate("document.getElementById('tentative-toggle').click()");
      await until("document.querySelectorAll('[data-track-state=tentative],[data-track-state=unverified]').length===0");
      report.trackingHiddenAgain=true;
    }
    if(drive) {
      // Preserve the historical bounded trial speeds despite adjustable UI defaults.
      await evaluate("if(document.getElementById('linear-speed')){setSpeed('linear',.05);setSpeed('angular',.3);}");
      await selectManual();
      const key = turn ? 'ArrowRight' : 'ArrowUp', code = turn ? 39 : 38;
      await press(key,code,'keyDown');
      await delay(turn ? 3000 : 750);
      report.held=await snapshot();
      report.heldCommand=await evaluate("document.getElementById('command').textContent");
      await press(key,code,'keyUp');
      report.idleAfterFirst = await idle();
      report.repeatedCommands=[];
      for(const [nextKey,nextCode] of [['ArrowLeft',37],['ArrowRight',39]]) {
        await press(nextKey,nextCode,'keyDown');
        await until(`document.getElementById('command').textContent==='Command: 0.00 m/s, ${nextKey==='ArrowLeft'?'0.30':'-0.30'} rad/s'`);
        report.repeatedCommands.push({held:await snapshot(),command:await evaluate("document.getElementById('command').textContent")});
        await press(nextKey,nextCode,'keyUp');
        report.repeatedCommands.at(-1).released=await idle();
      }
      await press(' ',32,'keyDown');
      await press(' ',32,'keyUp');
      await until("document.getElementById('guard').textContent.includes('operator_stop')");
      report.explicitStop=await snapshot();
      // A dead browser connection while moving must still expire the guard's
      // command lease. Restoring connectivity must not reselect manual mode.
      await selectManual();
      await press('ArrowUp',38,'keyDown');
      await until("document.getElementById('command').textContent==='Command: 0.05 m/s, 0.00 rad/s'");
      await call('Network.enable');
      await call('Network.emulateNetworkConditions',{offline:true,latency:0,downloadThroughput:0,uploadThroughput:0});
      await delay(1000);
      await call('Network.emulateNetworkConditions',{offline:false,latency:0,downloadThroughput:-1,uploadThroughput:-1});
      await press('ArrowUp',38,'keyUp');
      await until("document.getElementById('guard').textContent.includes('command_timeout')");
      report.connectionLoss=await snapshot();
      // Explicit test-operator selection followed by Stop; send no drive input.
      await selectManual();
      await press(' ',32,'keyDown');
      await press(' ',32,'keyUp');
      await until("document.getElementById('guard').textContent.includes('operator_stop')");
    }
    report.after=await snapshot();
    if(process.argv.includes('--coverage')) {
      await until("!!state.motion_overlay?.coverage && !!document.getElementById('coverage-toggle')");
      await evaluate("document.getElementById('coverage-toggle').click()");
      await until("document.querySelectorAll('[data-coverage]').length>0 && !document.getElementById('coverage-legend').hidden");
      report.coverage=await evaluate("({text:document.getElementById('coverage-health').textContent,bands:[...document.querySelectorAll('[data-coverage]')].map(p=>p.dataset.coverage),source:state.mapping.coverage.source})");
      if(drive) {
        await evaluate("document.getElementById('coverage-toggle').focus()");
        const generation=await evaluate("state.status.stop_generation");
        await press(' ',32,'keyDown');await press(' ',32,'keyUp');
        await until(`state.status.stop_generation>${generation} && state.status.mode==='stopped'`);
        report.coverage.spaceStopWithCheckboxFocused=true;
      }
    }
    if(process.argv.includes('--diagnostics')) {
      await until("!!document.getElementById('diagnostic-toggle') && !!state.motion_overlay?.footprint");
      await evaluate("document.getElementById('diagnostic-toggle').click()");
      await until("document.getElementById('terrain').getAttribute('href')?.startsWith('/diagnostics.png') && document.querySelectorAll('#markers polygon').length>=12");
      const check=await evaluate("fetch('/diagnostics.png').then(async r=>({status:r.status,bytes:(await r.arrayBuffer()).byteLength}))");
      if(check.status!==200 || check.bytes<100) throw Error('Diagnostic terrain image unavailable');
      report.diagnostics=await evaluate("({depth:document.getElementById('depth-health').textContent,localization:document.getElementById('pose-health').textContent,area:document.getElementById('map-health').textContent,rejection:document.getElementById('rejection').textContent,footprints:document.querySelectorAll('#markers polygon').length,legend:!document.getElementById('diagnostic-legend').hidden})");
      report.diagnostics.image=check;
    }
    report.controlRoundTrip=await evaluate("(()=>{const values=(window.checkTrace||[]).filter(r=>r.request==='drive'&&r.success).map(r=>r.ms).sort((a,b)=>a-b);return {count:values.length,p95_ms:values.length?values[Math.floor((values.length-1)*.95)]:null,max_ms:values.at(-1)??null};})()");
    const screenshot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true});
    fs.writeFileSync(output+'.png',Buffer.from(screenshot.data,'base64'),{flag:'wx'});
    fs.writeFileSync(output+'.json',JSON.stringify(report,null,2)+'\n',{flag:'wx'});
    console.log(JSON.stringify(report,null,2));
  } catch(error) {
    if(ws && ws.readyState===1) {
      try {
        const state=await call('Runtime.evaluate',{expression:"({session:state.session,guard:state.status,reply:el('reply').textContent,trace:window.checkTrace})",returnByValue:true});
        fs.writeFileSync(output+'-failure.json',JSON.stringify({error:error.message,state:state.result.value},null,2)+'\n',{flag:'wx'});
        const screenshot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true});
        fs.writeFileSync(output+'-failure.png',Buffer.from(screenshot.data,'base64'),{flag:'wx'});
        if(drive) await call('Runtime.evaluate',{expression:"fetch('/stop',{method:'POST'}).then(r=>r.json())",awaitPromise:true});
      } catch(e) {console.error('Failure capture: '+e.message);}
    }
    throw error;
  } finally {
    if(ws && ws.readyState===1) {try{ws.send(JSON.stringify({id:++identifier,method:'Browser.close'}));}catch(e){}ws.close();}
    for(const p of pending.values())p.reject(Error('Browser closed'));
    chrome.kill();
  }
}
main().catch(error=>{console.error(error);process.exitCode=1;});
