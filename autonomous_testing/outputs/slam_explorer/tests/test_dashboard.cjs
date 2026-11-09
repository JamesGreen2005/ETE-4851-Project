// Exercise the actual dashboard script without ROS or camera hardware.

const assert = require('node:assert/strict');

const fs = require('node:fs');

const path = require('node:path');

const vm = require('node:vm');

const {spawnSync} = require('node:child_process');

const source = path.join(__dirname, '../robot_explorer/dashboard.py');

const extraction = spawnSync('python', ['-c',

  "import ast,sys; t=ast.parse(open(sys.argv[1],encoding='utf8').read()); print(next(ast.literal_eval(n.value) for n in t.body if isinstance(n,ast.Assign) and any(isinstance(x,ast.Name) and x.id=='PAGE' for x in n.targets)))", source], {encoding:'utf8',env:{...process.env,PYTHONIOENCODING:'utf-8'}});

assert.equal(extraction.status, 0, extraction.stderr);

const html = extraction.stdout;

const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

const events = {}, intervals = [], calls = [], revoked = [];

let now = 100, serial = 0, rearFailure = false, holdRear = false, releaseRear;

const elements = Object.fromEntries([...html.matchAll(/id="([^"]+)"/g)].map(m => [m[1], {

  hidden:true, style:{}, textContent:'', classList:{add(){},remove(){}},

  removeAttribute(name){delete this[name];}

}]));

const document = {hidden:false, getElementById:id=>elements[id],

  querySelector:selector=>elements[selector.slice(1)], addEventListener:(name,fn)=>events[name]=fn};

const context = {document, console, location:{hash:'#token=test'}, URLSearchParams, AbortSignal,

  performance:{now:()=>now}, setInterval:(fn,ms)=>intervals.push({fn,ms}),

  URL:{createObjectURL:()=>`blob:${++serial}`,revokeObjectURL:url=>revoked.push(url)},

  fetch:async(route,options)=>{

    calls.push({route,options});

    if(route==='/camera/rear.jpg'&&holdRear) await new Promise(resolve=>{releaseRear=resolve;});

    const ok=!(route==='/camera/rear.jpg'&&rearFailure);

    return {ok,blob:async()=>({}),text:async()=>'No fresh image',json:async()=>({ok:true})};

  }

};

vm.createContext(context);

vm.runInContext(script,context);

const run=code=>vm.runInContext(code,context);

const settle=()=>new Promise(resolve=>setImmediate(resolve));

(async()=>{

  run(`renderSensors({drive_age_s:.1,drive:{sensors:{fresh:true,age_s:.05,ina_shunt_ohms:.1,motors:[{motor:1,current_a:.25,voltage_v:24,power_w:6,count:123,counts_per_s:80,duty_pct:50,current_status:'ok'}]},sensor_ages:{lidar:.1},fields:{H:'90',VALID:'1'},rear_coverage:{known_bins:18,required_bins:20}},exploration:{enabled:false,reason:'Reverse path deviation 0.160 m exceeds 0.150 m'}})`);

  assert.equal(elements.m1current.textContent,'0.250');

  assert.equal(elements.m1count.textContent,'123');

  assert.equal(elements.m2current.textContent,'—');

  assert.match(elements.stopReason.textContent,/0.160 m/);

  assert.match(elements.sensorSummary.textContent,/18\/20/);

  run(`renderSensors({drive_age_s:2,drive:{sensors:{fresh:true,motors:[{motor:1,current_a:.25}]}}})`);

  assert.equal(elements.m1current.textContent,'—');

  assert.match(elements.sensorFreshness.textContent,/stale/);

  await run("Promise.all([camera('front'),camera('rear')])");

  assert.equal(elements.camera.hidden,false);

  assert.equal(elements.rearCamera.hidden,false);

  assert.notEqual(elements.camera.src,elements.rearCamera.src);

  for(const request of calls) assert.equal(request.options.headers['X-Control-Token'],'test');

  const frontUrl=elements.camera.src;

  rearFailure=true;

  await run("camera('rear')");

  assert.equal(elements.rearCamera.hidden,true);

  assert.equal(elements.camera.src,frontUrl);

  assert.match(elements.rearStatus.textContent,/No fresh rear/);

  rearFailure=false;

  await run("camera('rear')");

  assert.equal(elements.mirrorRear,undefined);

  assert.ok(html.indexOf('id="camera"')<html.indexOf('Live motor and sensor readings'));

  assert.ok(html.indexOf('id="diagnostics"')>html.indexOf('id="map"'));

  assert.ok(!html.includes('Rear video is for viewing'));

  holdRear=true;

  const pending=run("camera('rear')");

  const count=calls.length;

  await run("camera('rear')");

  assert.equal(calls.length,count,'one in-flight request per camera');

  await intervals.find(i=>i.ms===250).fn();

  assert.equal(calls.at(-1).route,'/heartbeat','heartbeat proceeds while rear is blocked');

  now+=1400;

  intervals.find(i=>i.ms===350).fn();

  assert.equal(elements.rearCamera.hidden,true,'watchdog clears blocked rear feed');

  releaseRear(); await pending; await settle();

  holdRear=false;

  document.hidden=true;

  events.visibilitychange();

  assert.equal(elements.camera.hidden,true);

  assert.equal(elements.rearCamera.hidden,true);

  const hiddenCount=calls.length;

  await run("camera('rear')");

  assert.equal(calls.length,hiddenCount);

  document.hidden=false;

  await run("camera('rear')");

  assert.equal(elements.rearCamera.hidden,false);

  elements.rearCamera.onerror();

  assert.equal(elements.rearCamera.hidden,true);

  assert.ok(revoked.length>0);

  let dots=0;
  const drawing={clearRect(){dots=0},beginPath(){},arc(){},stroke(){},moveTo(){},lineTo(){},closePath(){},fill(){},fillRect(){dots++}};
  elements.lidar.width=560;elements.lidar.height=420;elements.lidar.getContext=()=>drawing;
  run('renderLidar({fresh:true,points:[[1,0],[0,1],[5,0]]})');
  assert.equal(dots,2);assert.match(elements.lidarStatus.textContent,/Live/);
  run('renderLidar({fresh:false,points:[[1,0]]})');
  assert.equal(dots,0);assert.match(elements.lidarStatus.textContent,/No fresh scan/);
  const resetCount=calls.filter(c=>c.route==='/reset').length;

  await elements.reset.onclick();

  assert.equal(calls.filter(c=>c.route==='/reset').length,resetCount+1);

  assert.deepEqual(JSON.parse(calls.at(-1).options.body),{reset:true});

  assert.equal(calls.at(-1).options.headers['X-Control-Token'],'test');

  run('renderSensors({exploration:{resetting:true}})');

  assert.equal(elements.start.disabled,true,'start blocked during reset');

  run('renderSensors({exploration:{resetting:false,cancelling_actions:1}})');

  assert.equal(elements.start.disabled,true,'start blocked during cancellation');

  run('renderSensors({exploration:{resetting:false,cancelling_actions:0}})');

  assert.equal(elements.start.disabled,false,'start available after reset');

  await elements.stop.onclick();

  assert.deepEqual(JSON.parse(calls.at(-1).options.body),{enable:false});

  console.log('PASS dashboard: reset button, start gating, authenticated reset, stop,  independent feeds, authentication, rear failure/recovery, layout, request bounds, stale view clearing, heartbeat, hidden-tab clearing, decode failure.');

})().catch(error=>{console.error(error);process.exitCode=1;});

