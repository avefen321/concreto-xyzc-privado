// Nominal material visualization only. The immutable machine plan owns XYZ/C.
import {primitivePose} from './simulation.js?v=2.2';

export function layerHeights(sim) {
  const cfg=sim.audit?.config?.z_conversion||{};
  const printed=(sim.machine_primitives||[]).filter(p=>p.process_state==='DEPOSIT'&&['LINE','ARC'].includes(p.type));
  const zs=[...new Set(printed.map(p=>p.start_xyz[2]))].sort((a,b)=>a-b);
  const first=Number(cfg.first_layer_z_mm)||20,next=Number(cfg.subsequent_layer_height_mm)||18;
  const heights=new Map();
  for(const p of printed) {
    if(heights.has(p.layer_id))continue;
    const index=zs.indexOf(p.start_xyz[2]);
    const inferred=index>0?zs[index]-zs[index-1]:zs[0]>0?zs[0]:first;
    heights.set(p.layer_id,cfg.enabled===false?inferred:index===0?first:next);
  }
  return heights;
}

export function isDepositing(p) {
  return p.process_state==='DEPOSIT'&&['LINE','ARC'].includes(p.type)&&Math.hypot(
    p.end_xyz[0]-p.start_xyz[0],p.end_xyz[1]-p.start_xyz[1],p.type==='ARC'?p.radius*p.sweep_rad:0)>1e-9;
}

function section(p,t,height,profile,shift) {
  const q=primitivePose(p,t);
  const angle=p.type==='ARC'?p.start_angle_rad+p.sweep_rad*t+Math.sign(p.sweep_rad)*Math.PI/2:
    Math.atan2(p.end_xyz[1]-p.start_xyz[1],p.end_xyz[0]-p.start_xyz[0]);
  const nx=-Math.sin(angle),ny=Math.cos(angle),c=(q.C_deg-shift)*Math.PI/180;
  // Width is the projection of the rotating rectangular mouth onto the
  // cross-path normal; the nozzle's long dimension is not always the width.
  const half=(profile.length_mm??35)/2*Math.abs(Math.cos(c)*nx+Math.sin(c)*ny)+
    (profile.width_mm??15)/2*Math.abs(-Math.sin(c)*nx+Math.cos(c)*ny);
  return [[q.X+nx*half,q.Y+ny*half,q.Z-height],[q.X-nx*half,q.Y-ny*half,q.Z-height],
          [q.X-nx*half,q.Y-ny*half,q.Z],[q.X+nx*half,q.Y+ny*half,q.Z]];
}

export function beadVertices(piece,end=piece.t1) {
  const corners=[...section(piece.primitive,piece.t0,piece.height,piece.profile,piece.shift),
    ...section(piece.primitive,end,piece.height,piece.profile,piece.shift)];
  const values=[];
  for(const [a,b,c,d] of [[0,4,5,1],[1,5,6,2],[2,6,7,3],[3,7,4,0],[0,1,2,3],[4,7,6,5]])
    for(const i of [a,b,c,a,c,d])values.push(...corners[i]);
  return values;
}

export function buildDeposition(sim,track,include=()=>true,{maxSegments=80000,stepMM=4}={}) {
  const profile=sim.audit?.config?.nozzle||{},heights=layerHeights(sim);
  const samples=new Map(sim.samples.map(s=>[s.sample_id,s]));
  const entries=new Map(track.entries.map((e,i)=>[e.primitive?.id,i]));
  const primitives=(sim.machine_primitives||[]).filter(p=>isDepositing(p)&&include(p)&&entries.has(p.id));
  if(primitives.length>maxSegments)throw Error('Demasiadas primitivas para el volumen 3D. Filtre una capa o use la vista de trayectorias.');
  const total=primitives.reduce((sum,p)=>sum+p.length_mm,0);
  const step=Math.max(stepMM,total/Math.max(1,maxSegments-primitives.length));
  const pieces=[],values=[],colors=[];let left=maxSegments;
  primitives.forEach((p,i)=>{
    const desired=Math.max(1,Math.ceil(p.length_mm/step),p.type==='ARC'?Math.ceil(Math.abs(p.sweep_rad)/.06):1);
    const count=Math.min(desired,left-(primitives.length-i-1));left-=count;
    const sample=samples.get(p.sample_id),cordon=sample?.cordon_role==='CORDON_PRIME';
    const color=cordon?[.52,.40,.65]:(Number(p.layer_id)||0)%2?[.57,.59,.59]:[.49,.52,.53];
    for(let k=0;k<count;k++) {
      const piece={primitive:p,t0:k/count,t1:(k+1)/count,height:heights.get(p.layer_id)||18,
        profile,shift:p.metadata?.c_reference_shift_deg??sample?.c_reference_shift_deg??0,
        entryIndex:entries.get(p.id),color};
      pieces.push(piece);values.push(...beadVertices(piece));
      for(let v=0;v<36;v++)colors.push(...color);
    }
  });
  return {pieces,positions:new Float32Array(values),colors:new Float32Array(colors),heights,
    heightSource:sim.audit?.config?.z_conversion?.enabled===false?'PROGRAM_Z':'CONFIGURED',
    firstHeight:heights.values().next().value||Number(sim.audit?.config?.z_conversion?.first_layer_z_mm)||20,
    nextHeight:Number(sim.audit?.config?.z_conversion?.subsequent_layer_height_mm)||18};
}

export function depositedRange(data,head) {
  if(!head)return {complete:0,partial:null};
  let low=0,high=data.pieces.length;
  while(low<high){const mid=(low+high)>>1,p=data.pieces[mid];
    if(p.entryIndex<head.entryIndex||(p.entryIndex===head.entryIndex&&p.t1<=head.fraction+1e-10))low=mid+1;else high=mid;}
  const next=data.pieces[low];
  return {complete:low,partial:next&&next.entryIndex===head.entryIndex&&head.fraction>next.t0?{piece:next,end:head.fraction}:null};
}
