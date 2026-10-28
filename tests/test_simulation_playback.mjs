import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';

// Pure playback helpers can be exercised without a browser or a DOM mock.
const source = readFileSync(new URL('../frontend/js/simulation.js', import.meta.url), 'utf8');
const {buildPlaybackTrack, playbackAtCursor, playbackCursorAtTime, primitivePose, nozzleFootprint} = await import(
  `data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
const sample = (X, extra = {}) => ({X,Y:0,Z:20,C_deg:0,feed_mm_min:1200,
  emit:true,motion_type:'LINEAR',layer_id:1,stroke_id:1,...extra});

const footprint = nozzleFootprint(sample(100));
assert.deepEqual(footprint, [[82.5,-7.5,20],[117.5,-7.5,20],[117.5,7.5,20],[82.5,7.5,20]]);
for (const angle of [0, 45, 90, 180, 359]) {
  const corners = nozzleFootprint(sample(100, {C_deg:angle}), {length_mm:35,width_mm:15});
  assert.ok(Math.abs(Math.hypot(...corners[1].map((v,i)=>v-corners[0][i]))-35)<1e-9);
  assert.ok(Math.abs(Math.hypot(...corners[2].map((v,i)=>v-corners[1][i]))-15)<1e-9);
}
const referenced = nozzleFootprint(sample(0,{C_deg:60,c_reference_shift_deg:-90}), {length_mm:35,width_mm:15});
const expectedReferenced = nozzleFootprint(sample(0,{C_deg:150}), {length_mm:35,width_mm:15});
for(let i=0;i<4;i++)for(let j=0;j<3;j++)
  assert.ok(Math.abs(referenced[i][j]-expectedReferenced[i][j])<1e-9,'Viewer applies physical C reference shift');

// Subdividing a 100 mm line must not change its ideal duration or the position at 2.5 s.
for (const blocks of [1,10,1000]) {
  const samples = [sample(0,{emit:false,motion_type:'START'})];
  for(let i=1;i<=blocks;i++) samples.push(sample(100*i/blocks));
  const before = JSON.stringify(samples);
  const track = buildPlaybackTrack(samples);
  assert.ok(Math.abs(track.totalSeconds-5)<1e-9);
  const head = playbackAtCursor(track,playbackCursorAtTime(track,2.5));
  assert.ok(Math.abs(head.pose.X-50)<1e-9);
  assert.equal(JSON.stringify(samples),before,'Playback must not mutate the final plan');
  assert.equal(track.estimatedCount,0);
}

const changingFeed = buildPlaybackTrack([sample(0,{emit:false,motion_type:'START'}),
  sample(100),sample(200,{feed_mm_min:600,C_deg:450})]);
assert.equal(changingFeed.totalSeconds,15);
const mid = playbackAtCursor(changingFeed,playbackCursorAtTime(changingFeed,10));
assert.equal(mid.pose.X,150);
assert.equal(mid.pose.C_deg,225,'Unwrapped C interpolates continuously');
assert.equal(mid.entry.sample.X,200,'The inspector retains the real block destination');
assert.equal(playbackCursorAtTime(changingFeed,100),changingFeed.entries.length-1);

const references = buildPlaybackTrack([sample(0,{emit:false,motion_type:'START'}),
  sample(100),sample(100,{emit:false,motion_type:'LINEAR'}),sample(200)]);
assert.equal(references.totalSeconds,10,'Metadata must not add animation time');
assert.equal(references.entries.length,3);

const mixed = [sample(0,{emit:false,motion_type:'START'}),sample(100),
  sample(0,{Y:100,layer_id:2,stroke_id:2,motion_type:'RAPID'}),
  sample(0,{Y:100,layer_id:2,stroke_id:2,emit:false,motion_type:'START'}),
  sample(100,{Y:100,layer_id:2,stroke_id:2})];
const layer = buildPlaybackTrack(mixed,0,s=>s.layer_id===1);
assert.equal(layer.totalSeconds,5,'Filtering must not connect unrelated layers');
assert.equal(layer.entries.length,2);
const deposition = buildPlaybackTrack(mixed,3);
assert.equal(deposition.totalSeconds,5,'Deposition start skips earlier travels');
assert.equal(deposition.entries[0].sample.Y,100);

const unknown = buildPlaybackTrack([sample(0,{emit:false,motion_type:'START'}),
  sample(100,{motion_type:'RAPID'}),sample(200,{feed_mm_min:null}),
  sample(200,{motion_type:'C_ONLY',C_deg:90})]);
assert.equal(unknown.estimatedCount,2);
assert.equal(unknown.totalSeconds,8.5,'C-only G94 feed is interpreted as degrees per minute');
assert.equal(playbackAtCursor(unknown,playbackCursorAtTime(unknown,6.25)).pose.C_deg,45);
assert.equal(buildPlaybackTrack([]).entries.length,0);
assert.equal(playbackAtCursor(buildPlaybackTrack([]),0),null);
console.log('PASS: duration/density independence, feed, interpolation, immutable plan, filters, references, unknown timings and endpoints');

const fixture=JSON.parse(readFileSync(new URL('./native_arc_pose_fixture.json',import.meta.url),'utf8'));
for(const item of fixture.expected){
  const p=primitivePose(fixture.primitive,item.t);
  for(const [i,key] of ['X','Y','Z','C_deg'].entries())
    assert.ok(Math.abs(p[key]-item.pose[i])<1e-9,`Backend/viewer mismatch ${key} at ${item.t}`);
  for(let i=0;i<2;i++) assert.ok(Math.abs(p.tangent_xy[i]-item.tangent_xy[i])<1e-9);
}
const arc=fixture.primitive;
const arcSamples=[sample(arc.start_xyz[0],{Y:arc.start_xyz[1],C_deg:arc.start_c,emit:false,motion_type:'START'}),
  sample(arc.end_xyz[0],{Y:arc.end_xyz[1],Z:arc.end_xyz[2],C_deg:arc.end_c,sample_id:arc.sample_id,feed_mm_min:arc.feed})];
const nativeTrack=buildPlaybackTrack(arcSamples,0,()=>true,[arc]);
assert.ok(Math.abs(nativeTrack.totalSeconds-60*arc.length_mm/arc.feed)<1e-9);
const nativeMid=playbackAtCursor(nativeTrack,playbackCursorAtTime(nativeTrack,nativeTrack.totalSeconds/2));
const expectedMid=primitivePose(arc,.5);
for(const key of ['X','Y','Z','C_deg']) assert.ok(Math.abs(nativeMid.pose[key]-expectedMid[key])<1e-9);
console.log('PASS: native arc poses match backend at five fractions, true arc duration and simultaneous C');
