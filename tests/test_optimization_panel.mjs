import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';

const source=readFileSync(new URL('../frontend/js/optimization.js',import.meta.url),'utf8');
const {setupOptimization}=await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const tick=()=>new Promise(resolve=>setImmediate(resolve));

function fixture(capabilities,{candidates=false}={}) {
  const ids=['optimization-status','optimization-start','optimization-cancel','optimization-method',
    'optimization-apply','optimization-artifacts','optimization-json','optimization-csv',
    'optimization-budget','optimization-seed','optimization-timing-enabled','optimization-timing',
    'optimization-environment','optimization-refresh','ai-optimization-metrics'];
  const nodes=Object.fromEntries(ids.map(id=>[id,{disabled:true,hidden:false,textContent:'',value:''}]));
  if(candidates)for(const id of ['optimization-candidates','optimization-candidates-body','optimization-best-download','optimization-recommendation','optimization-viewing'])nodes[id]={disabled:false,hidden:true,textContent:'',innerHTML:'',addEventListener:(name,fn)=>{nodes[id][name]=fn;}};
  nodes['optimization-method'].options=['bo','random','tpe'].map(value=>({value,disabled:false}));
  nodes['optimization-method'].value='bo';nodes['optimization-budget'].value='8';nodes['optimization-seed'].value='7';
  const events={},calls=[];
  globalThis.document={getElementById:id=>nodes[id],addEventListener:(name,fn)=>{events[name]=fn;}};
  let current=null,selected=null;
  globalThis.fetch=async(path,options)=>{
    calls.push({path,body:options?.body?JSON.parse(options.body):null});
    let data;
    if(path.endsWith('capabilities'))data=capabilities;
    else if(path==='/api/optimization')data={id:'job'};
    else if(path.endsWith('/result'))data={id:'selected',report:{optimization_selection:{candidate_id:path.split('/')[5],label:'Evaluación 2',N_blocks:8,recommended:path.includes('/best/')}}};
    else data={status:'COMPLETED',phase:'DONE',completed:5,budget:8,
      baseline:{N_blocks:10,M_microblocks:4},best:{N_blocks:8,M_microblocks:2,xy_error_mm:0,C_error_deg:0,coverage_complete:true},
      summary:{unique_configurations:5,adaptive_evaluations:1,outcome:'REDUCED',note:'Referencia conservada'}};
    if(candidates&&path==='/api/optimization/job')data.candidates=[
      {id:'constructor',label:'Constructor fijo',parameters:{candidate_window:4},recommended:true,valid:true,eligible:true,N_blocks:8,M_microblocks:2,arc_microblocks:0},
      {id:'evaluation-2',label:'Evaluación 2',parameters:{candidate_window:8},recommended:false,valid:true,eligible:true,N_blocks:8,M_microblocks:2,arc_microblocks:0,acquisition:'EI'},
      {id:'evaluation-3',label:'Evaluación 3',parameters:{candidate_window:2},recommended:false,valid:false,eligible:false,N_blocks:null,M_microblocks:null}];
    return {ok:true,json:async()=>data};
  };
  setupOptimization({getRequest:()=>current,onSelected:async result=>{selected=result;}});
  return {nodes,calls,events,setRequest:value=>{current=value;},get selected(){return selected;}};
}

let f=fixture({enabled:true,algorithms:{bo:true,tpe:true,random:true}});
await tick();assert.equal(f.nodes['optimization-start'].disabled,true);
f.setRequest({source:'G1 X10 F600',config:{arc_fit:{reference_model:'POLYLINE_C'}}});
f.events['xyzc:conversion-changed']();assert.equal(f.nodes['optimization-start'].disabled,false);
await f.nodes['optimization-start'].onclick();
assert.equal(f.calls.find(c=>c.path==='/api/optimization').body.config.arc_fit.reference_model,'POLYLINE_C');
assert.match(f.nodes['optimization-status'].textContent,/1 propuestas adaptativas/);
assert.equal(f.nodes['optimization-apply'].disabled,false);
await f.nodes['optimization-apply'].onclick();assert.equal(f.selected.id,'selected');

f=fixture({enabled:true,algorithms:{bo:false,tpe:false,random:true},python_executable:'wrong-python'});
await tick();assert.equal(f.nodes['optimization-method'].value,'random');
assert.equal(f.nodes['optimization-method'].options[0].disabled,true);
assert.match(f.nodes['optimization-environment'].textContent,/wrong-python/);

f=fixture({enabled:false,disabled_reason:'PUBLIC_ACCESS',algorithms:{bo:true,tpe:true,random:true}});
await tick();f.setRequest({source:'G1 X10',config:{}});f.events['xyzc:conversion-changed']();
assert.equal(f.nodes['optimization-start'].disabled,true);
assert.match(f.nodes['optimization-status'].textContent,/pública/);

f=fixture({enabled:true,algorithms:{bo:true,tpe:true,random:true}},{candidates:true});
await tick();f.setRequest({source:'G1 X10 F600',config:{arc_fit:{reference_model:'POLYLINE_C'}}});f.events['xyzc:conversion-changed']();
await f.nodes['optimization-start'].onclick();
assert.match(f.nodes['optimization-candidates-body'].innerHTML,/★ Recomendada/);
assert.match(f.nodes['optimization-candidates-body'].innerHTML,/Sin salida válida/);
assert.match(f.nodes['optimization-best-download'].href,/candidate\/best\/gcode$/);
await f.nodes['optimization-apply'].onclick();assert.ok(f.calls.some(c=>c.path.endsWith('/candidate/best/result')));
f.nodes['optimization-candidates-body'].click({target:{closest:()=>({dataset:{candidate:'evaluation-2'},disabled:false})}});
await tick();assert.ok(f.calls.some(c=>c.path.endsWith('/candidate/evaluation-2/result')));
assert.match(f.nodes['optimization-viewing'].textContent,/Evaluación 2/);
console.log('PASS: conversión predeterminada, consulta BO, selección, diagnóstico de dependencias y bloqueo público');
