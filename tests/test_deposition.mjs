import assert from 'node:assert/strict';
import {buildDeposition,beadVertices,depositedRange,isDepositing,layerHeights} from '../frontend/js/deposition.js';
import {buildPlaybackTrack,playbackAtCursor,primitivePose} from '../frontend/js/simulation.js';

const p=(id,type,start,end,extra={})=>({id,type,start_xyz:start,end_xyz:end,start_c:0,end_c:0,
  length_mm:Math.hypot(...end.map((v,i)=>v-start[i])),feed:1200,process_state:'DEPOSIT',sample_id:Number(id.slice(1)),layer_id:1,stroke_id:1,metadata:{},...extra});
const primitives=[p('P1','TRAVEL',[0,0,20],[50,0,20],{process_state:'TRAVEL'}),
  p('P2','ARC',[50,0,20],[0,50,20],{center_xyz:[0,0,20],radius:50,start_angle_rad:0,sweep_rad:Math.PI/2,start_c:180,end_c:270,length_mm:Math.PI*25}),
  p('P3','Z_ONLY',[0,50,20],[0,50,38],{start_c:270,end_c:270,layer_id:2}),
  p('P4','ARC',[0,50,38],[-50,0,38],{center_xyz:[0,0,38],radius:50,start_angle_rad:Math.PI/2,sweep_rad:Math.PI/2,start_c:270,end_c:360,layer_id:2,length_mm:Math.PI*25})];
const sim={machine_primitives:primitives,samples:primitives.map(p=>({sample_id:p.sample_id,X:p.end_xyz[0],Y:p.end_xyz[1],Z:p.end_xyz[2],C_deg:p.end_c,emit:true,
  motion_type:p.type==='TRAVEL'?'RAPID':'LINEAR',deposition_active:p.process_state==='DEPOSIT',feed_mm_min:1200,layer_id:p.layer_id,stroke_id:p.stroke_id})),
  audit:{config:{nozzle:{length_mm:35,width_mm:15},z_conversion:{enabled:true,first_layer_z_mm:20,subsequent_layer_height_mm:18}}}};
const before=JSON.stringify(sim),track=buildPlaybackTrack(sim.samples,0,()=>true,primitives),data=buildDeposition(sim,track);
assert.equal(layerHeights(sim).get(1),20);assert.equal(layerHeights(sim).get(2),18);
assert.equal(isDepositing(primitives[0]),false);assert.equal(isDepositing(primitives[2]),false);
assert.equal(depositedRange(data,playbackAtCursor(track,0)).complete,0);
const z1=beadVertices(data.pieces.find(p=>p.primitive.layer_id===1)).filter((_,i)=>i%3===2);
const z2=beadVertices(data.pieces.find(p=>p.primitive.layer_id===2)).filter((_,i)=>i%3===2);
assert.equal(Math.min(...z1),0);assert.equal(Math.max(...z1),20);
assert.equal(Math.min(...z2),20);assert.equal(Math.max(...z2),38);
const halfway=depositedRange(data,playbackAtCursor(track,.5));
assert.ok(halfway.complete>0&&halfway.complete<data.pieces.length);
const riseA=depositedRange(data,playbackAtCursor(track,1.2)),riseB=depositedRange(data,playbackAtCursor(track,1.8));
assert.equal(riseA.complete,riseB.complete);assert.equal(riseA.partial,null);assert.equal(riseB.partial,null);
assert.equal(data.pieces[riseA.complete].primitive.layer_id,2);
const end=depositedRange(data,playbackAtCursor(track,3));assert.equal(end.complete,data.pieces.length);
assert.ok(buildDeposition(sim,track,p=>p.layer_id===2).pieces.every(p=>p.height===18));
assert.throws(()=>buildDeposition(sim,track,()=>true,{maxSegments:1}),/Demasiadas/);
assert.equal(JSON.stringify(sim),before);
const originalZ=structuredClone(sim);originalZ.audit.config.z_conversion.enabled=false;
originalZ.machine_primitives[3].start_xyz[2]=45;originalZ.machine_primitives[3].end_xyz[2]=45;
assert.equal(layerHeights(originalZ).get(2),25);
const straight=p('P5','LINE',[0,0,20],[10,0,20],{start_c:90,end_c:90});
const vertices=beadVertices({primitive:straight,t0:0,t1:1,height:20,profile:{length_mm:35,width_mm:15},shift:0});
const ys=vertices.filter((_,i)=>i%3===1);assert.equal(Math.max(...ys)-Math.min(...ys),35);
const approach=p('P6','TRAVEL',[0,0,0],[10,10,20],{generated_reason:'INITIAL_XY_APPROACH'});
assert.equal(primitivePose(approach,.2).Z,20);assert.equal(approach.start_xyz[2],0);
console.log('PASS: espesor 20/18 mm, avance progresivo, G2/G3, subidas sin material, filtros, límites, boquilla y plan inmutable');
