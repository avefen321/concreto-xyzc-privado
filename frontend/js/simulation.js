// Nominal nozzle footprint in model millimetres, not deposited material.
export function nozzleFootprint(sample, profile = {}) {
  const angle = ((sample.C_deg || 0) - (sample.c_reference_shift_deg || 0)) * Math.PI / 180;
  const halfLength = (profile.length_mm ?? 35) / 2;
  const halfWidth = (profile.width_mm ?? 15) / 2;
  const ux = Math.cos(angle), uy = Math.sin(angle);
  return [[-1,-1],[1,-1],[1,1],[-1,1]].map(([a,b]) => [
    sample.X + a * halfLength * ux - b * halfWidth * uy,
    sample.Y + a * halfLength * uy + b * halfWidth * ux,
    sample.Z,
  ]);
}

// Display timing only. The backend samples and exported motion plan are immutable.
export function buildPlaybackTrack(samples, startIndex = 0, include = () => true, primitives = []) {
  const entries = [];
  const bySample = new Map(primitives.map(p => [p.sample_id, p]));
  let previous = null, previousIndex = -1, totalSeconds = 0, estimatedCount = 0;
  for (let index = 0; index < samples.length; index++) {
    const sample = samples[index];
    const reference = sample.motion_type === 'START' || index === 0;
    if (!sample.emit && !reference) continue;
    const primitive = bySample.get(sample.sample_id);
    const from = primitive ? {...sample, X:primitive.start_xyz[0], Y:primitive.start_xyz[1],
      Z:primitive.start_xyz[2], C_deg:primitive.start_c} : reference ? sample : previous || sample;
    if (index >= startIndex && include(sample)) {
      // START is a position reference, not a commanded move to interpolate.
      const distance = primitive?.length_mm ?? Math.hypot(sample.X - from.X, sample.Y - from.Y, sample.Z - from.Z);
      const rotation = Math.abs(sample.C_deg - from.C_deg);
      const rapid = ['RAPID', 'PREP_RAPID'].includes(sample.motion_type);
      const moving = !reference && (distance > 1e-9 || rotation > 1e-9);
      const cOnly = sample.motion_type === 'C_ONLY' || primitive?.type === 'C_ONLY';
      const knownLinearFeed = !rapid && distance > 1e-9 && Number.isFinite(sample.feed_mm_min) && sample.feed_mm_min > 0;
      const knownRotationFeed = cOnly && rotation > 1e-9 && Number.isFinite(sample.feed_mm_min) && sample.feed_mm_min > 0;
      const knownFeed = knownLinearFeed || knownRotationFeed;
      const estimated = moving && !knownFeed;
      // A pure rotary G94 move uses degrees/minute; linear moves use mm/minute.
      const duration = !moving ? 0 : knownRotationFeed ? rotation * 60 / sample.feed_mm_min
        : knownLinearFeed ? distance * 60 / sample.feed_mm_min : Math.max(.25, distance / 50, rotation / 90);
      if (!entries.length) entries.push({index: previousIndex >= startIndex ? previousIndex : index,
        sample: reference ? sample : from, from, duration: 0, startTime: 0, endTime: 0, estimated: false});
      if (sample !== entries[0].sample || entries.length > 1) {
        entries.push({index, sample, from, primitive, duration, startTime: totalSeconds,
          endTime: totalSeconds + duration, estimated});
        totalSeconds += duration;
        if (estimated) estimatedCount++;
      }
    }
    previous = sample;
    previousIndex = index;
  }
  return {entries, totalSeconds, estimatedCount};
}

export function playbackAtCursor(track, cursor) {
  if (!track.entries.length) return null;
  const position = Math.max(0, Math.min(track.entries.length - 1, cursor));
  const entryIndex = Math.ceil(position);
  const entry = track.entries[entryIndex];
  const fraction = entryIndex ? position - entryIndex + 1 : 1;
  const pose = {...entry.sample};
  for (const key of ['X', 'Y', 'Z', 'C_deg'])
    pose[key] = entry.from[key] + (entry.sample[key] - entry.from[key]) * fraction;
  if (entry.primitive?.type === 'ARC') Object.assign(pose, primitivePose(entry.primitive,fraction));
  const dx = entry.sample.X - entry.from.X, dy = entry.sample.Y - entry.from.Y;
  const length = Math.hypot(dx, dy);
  if (!entry.primitive && length > 1e-9 && fraction < 1) pose.tangent_xy = [dx / length, dy / length];
  return {entry, entryIndex, fraction, pose, cursor: position,
    seconds: entry.startTime + entry.duration * fraction};
}

// Only interpolate the geometry and C supplied by the backend. No tangent,
// fillet or unwrap algorithm is reconstructed in the viewer.
export function primitivePose(p,t) {
  const xyz=p.start_xyz.map((a,i)=>a+(p.end_xyz[i]-a)*t);
  if(p.type==='ARC' && t>0 && t<1){
    const angle=p.start_angle_rad+p.sweep_rad*t;
    xyz[0]=p.center_xyz[0]+p.radius*Math.cos(angle);
    xyz[1]=p.center_xyz[1]+p.radius*Math.sin(angle);
  }
  const pose={X:xyz[0],Y:xyz[1],Z:xyz[2],C_deg:p.start_c+(p.end_c-p.start_c)*t};
  if(p.type==='ARC' && p.tangent_start_xy){
    const a=p.sweep_rad*t, [x,y]=p.tangent_start_xy;
    pose.tangent_xy=[x*Math.cos(a)-y*Math.sin(a),x*Math.sin(a)+y*Math.cos(a)];
  }
  return pose;
}

function primitivePoints(p,end=1) {
  const count=p.type==='ARC'?Math.max(2,Math.ceil(Math.abs(p.sweep_rad)*end/.04)):1;
  return Array.from({length:count+1},(_,i)=>{const q=primitivePose(p,end*i/count);return [q.X,q.Y,q.Z]});
}

export function playbackCursorAtTime(track, seconds) {
  if (!track.entries.length || seconds <= 0) return 0;
  if (seconds >= track.totalSeconds) return track.entries.length - 1;
  let lo = 1, hi = track.entries.length - 1;
  while (lo < hi) {
    const mid = Math.floor((lo + hi) / 2);
    if (track.entries[mid].endTime <= seconds) lo = mid + 1; else hi = mid;
  }
  const entry = track.entries[lo];
  return lo - 1 + (entry.duration ? (seconds - entry.startTime) / entry.duration : 1);
}

export function createSimulator(state, fmt) {
  const $ = id => document.getElementById(id);
  const camera = {yaw: Math.PI / 4, elevation: Math.atan(1 / Math.sqrt(2)), zoom: 1, panX: 0, panY: 0, focus: null};
  let drag = null;
  let interactionMode = 'orbit';
  let hitTargets = [];
  let playback = {entries: [], totalSeconds: 0, estimatedCount: 0};
  let playbackCursor = 0;
  let lastFrameTime = null;

  function pausePlayback() {
    state.playing = false;
    cancelAnimationFrame(state.timer);
    state.timer = null;
    lastFrameTime = null;
    $('play-button').textContent = '▶';
    $('play-button').setAttribute('aria-label', 'Reproducir');
  }

  function refreshPlayback() {
    pausePlayback();
    const sim = state.simulation;
    if (!sim) {playback = {entries: [], totalSeconds: 0, estimatedCount: 0}; return;}
    const layer = $('layer-select').value, stroke = $('stroke-select').value;
    const include = s => (layer === 'all' || String(s.layer_id) === layer) &&
                         (stroke === 'all' || String(s.stroke_id) === stroke);
    const startMode=$('start-mode').value;
    const start = startMode==='program' ? sim.program_start_index
      :startMode==='deposition' ? sim.deposition_start_index
      :(sim.initial_orientation_start_index ?? sim.deposition_start_index);
    playback = buildPlaybackTrack(sim.samples, start, include, sim.machine_primitives || []);
    playbackCursor = 0;
    $('timeline').max = Math.max(0, playback.entries.length - 1);
    $('timeline').value = 0;
    $('timeline').step = $('play-mode').value === 'blocks' ? '1' : '0.001';
    $('play-button').disabled = playback.entries.length < 2;
    draw(sim);
  }

  function animate(timestamp) {
    if (!state.playing) return;
    if (state.page !== 'simulacion' || !state.simulation) {pausePlayback(); return;}
    const elapsed = lastFrameTime == null ? 0 : Math.max(0, (timestamp - lastFrameTime) / 1000);
    lastFrameTime = timestamp;
    const speed = Number($('play-speed').value);
    if ($('play-mode').value === 'continuous') {
      const current = playbackAtCursor(playback, playbackCursor);
      playbackCursor = playbackCursorAtTime(playback, (current?.seconds || 0) + elapsed * speed);
    } else {
      // Two blocks per second at 1x, independent of frame rate.
      playbackCursor = Math.min(playback.entries.length - 1, playbackCursor + elapsed * speed * 2);
    }
    $('timeline').value = $('play-mode').value === 'blocks' ? Math.floor(playbackCursor) : playbackCursor;
    draw(state.simulation);
    if (playbackCursor >= playback.entries.length - 1) pausePlayback();
    else state.timer = requestAnimationFrame(animate);
  }

  function resetCamera(kind = 'iso') {
    camera.yaw = kind === 'side' ? Math.PI / 2 : kind === 'top' || kind === 'front' ? 0 : Math.PI / 4;
    camera.elevation = kind === 'top' ? Math.PI / 2 : kind === 'front' || kind === 'side' ? 0 : Math.atan(1 / Math.sqrt(2));
    camera.zoom = 1;
    camera.panX = 0;
    camera.panY = 0;
    camera.focus = null;
    draw(state.simulation);
  }

  function fitCamera() {
    camera.zoom = 1;
    camera.panX = 0;
    camera.panY = 0;
    camera.focus = null;
    draw(state.simulation);
  }

  function setView(view) {
    state.view = view;
    $('view-2d').classList.toggle('selected', view === '2d');
    $('view-3d').classList.toggle('selected', view === '3d');
    $('camera-controls').classList.toggle('hidden', view !== '3d');
    $('sim-canvas').parentElement.classList.toggle('canvas-3d', view === '3d');
    draw(state.simulation);
  }

  function fillSelectors(simulation) {
    if (!simulation) return;
    const strokes = simulation.original_strokes;
    const layers = [...new Set(strokes.map(s => s.layer_id).filter(id => id != null))];
    $('layer-select').innerHTML = '<option value="all">Todas</option>' +
      layers.map(id => `<option value="${id}">Capa ${id}</option>`).join('');
    $('stroke-select').innerHTML = '<option value="all">Todos</option>' +
      strokes.map(s => `<option value="${s.stroke_id}">Trazo ${s.stroke_id}${s.cordon ? ' · cordón' : s.deposition ? ' · impresión' : ' · travel'}</option>`).join('');
    $('sample-id').max = simulation.samples.length;
    camera.focus = null; camera.zoom = 1;
    setStart();
  }

  function setStart() {
    const sim = state.simulation;
    if (!sim) return;
    refreshPlayback();
  }

  function resize(canvas) {
    const box = canvas.getBoundingClientRect();
    const ratio = devicePixelRatio || 1;
    canvas.width = Math.max(1, Math.round(box.width * ratio));
    canvas.height = Math.max(1, Math.round(box.height * ratio));
    const context = canvas.getContext('2d');
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    return {ctx: context, w: box.width, h: box.height};
  }

  function grid(ctx, w, h) {
    ctx.strokeStyle = '#252d31';
    ctx.lineWidth = 1;
    for (let x = 30; x < w; x += 60) {ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, h); ctx.stroke();}
    for (let y = 30; y < h; y += 60) {ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();}
  }

  function projection(bounds, w, h) {
    const center = camera.focus || [(bounds[0] + bounds[1]) / 2, (bounds[2] + bounds[3]) / 2, (bounds[4] + bounds[5]) / 2];
    if (state.view === '2d') {
      const scale = Math.min((w - 90) / Math.max(1, bounds[1] - bounds[0]),
                             (h - 90) / Math.max(1, bounds[3] - bounds[2])) * camera.zoom;
      return point => [w / 2 + (point[0] - center[0]) * scale,
                       h / 2 - (point[1] - center[1]) * scale];
    }
    const cy = Math.cos(camera.yaw), sy = Math.sin(camera.yaw);
    const ce = Math.cos(camera.elevation), se = Math.sin(camera.elevation);
    const rotate = point => {
      const x = point[0] - center[0], y = point[1] - center[1], z = point[2] - center[2];
      return [cy * x - sy * y, (sy * x + cy * y) * se + z * ce];
    };
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const x of [bounds[0], bounds[1]])
      for (const y of [bounds[2], bounds[3]])
        for (const z of [bounds[4], bounds[5]]) {
          const point = rotate([x, y, z]);
          minX = Math.min(minX, point[0]); maxX = Math.max(maxX, point[0]);
          minY = Math.min(minY, point[1]); maxY = Math.max(maxY, point[1]);
        }
    const scale = Math.min((w - 90) / Math.max(1, maxX - minX),
                           (h - 90) / Math.max(1, maxY - minY)) * camera.zoom;
    return point => {
      const q = rotate(point);
      return [w / 2 + q[0] * scale + camera.panX,
              h / 2 - q[1] * scale + camera.panY];
    };
  }

  function drawPath(ctx, points, project, color, width, dashed = false) {
    if (points.length < 2) return;
    ctx.beginPath();
    points.forEach((point, index) => {
      const [x, y] = project(point);
      if (index) ctx.lineTo(x, y); else ctx.moveTo(x, y);
    });
    ctx.setLineDash(dashed ? [5, 5] : []);
    ctx.strokeStyle = color;
    ctx.lineWidth = width;
    ctx.stroke();
    ctx.setLineDash([]);
  }

  function normalize(vector, size) {
    const length = Math.hypot(vector[0], vector[1]);
    return length > 1e-8 ? [vector[0] / length * size, vector[1] / length * size] : [size, 0];
  }

  function vectorAt(project, sample, angle, length = 26) {
    const origin = project([sample.X, sample.Y, sample.Z]);
    const rad = angle * Math.PI / 180;
    const moved = project([sample.X + Math.cos(rad), sample.Y + Math.sin(rad), sample.Z]);
    return normalize([moved[0] - origin[0], moved[1] - origin[1]], length);
  }

  function arrow(ctx, x, y, dx, dy, color) {
    ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + dx, y + dy); ctx.stroke();
    const angle = Math.atan2(dy, dx);
    ctx.beginPath(); ctx.moveTo(x + dx, y + dy);
    ctx.lineTo(x + dx - 7 * Math.cos(angle - .45), y + dy - 7 * Math.sin(angle - .45));
    ctx.lineTo(x + dx - 7 * Math.cos(angle + .45), y + dy - 7 * Math.sin(angle + .45));
    ctx.closePath(); ctx.fill();
  }

  function nozzle(ctx, project, sample, profile) {
    const center = project([sample.X, sample.Y, sample.Z]);
    const physicalC=(sample.C_deg || 0)-(sample.c_reference_shift_deg || 0);
    const long = vectorAt(project, sample, physicalC, 18);
    const corners = nozzleFootprint(sample, profile).map(project);
    if($('show-nozzle').checked){
    ctx.beginPath();
    corners.forEach((p, i) => i ? ctx.lineTo(...p) : ctx.moveTo(...p));
    ctx.closePath(); ctx.strokeStyle = '#f6a66f'; ctx.lineWidth = 2; ctx.stroke();
    }
    ctx.beginPath(); ctx.arc(center[0], center[1], 3, 0, Math.PI * 2);
    ctx.fillStyle = '#fff'; ctx.fill();
    if (($('show-vectors').checked || $('debug-stage').value==='ORIENTATION') && sample.tangent_xy != null) {
      const tip=project([sample.X+sample.tangent_xy[0],sample.Y+sample.tangent_xy[1],sample.Z]);
      const dx=tip[0]-center[0],dy=tip[1]-center[1],norm=Math.hypot(dx,dy)||1;
      const tangent=[30*dx/norm,30*dy/norm];
      arrow(ctx, center[0], center[1], tangent[0], tangent[1], '#83bfe6');
      if($('show-nozzle').checked)arrow(ctx, center[0], center[1], long[0] * 1.6, long[1] * 1.6, '#f5b379');
    }
  }

  function draw(sim) {
    const canvas = $('sim-canvas');
    const {ctx, w, h} = resize(canvas);
    ctx.fillStyle = '#171c1f'; ctx.fillRect(0, 0, w, h);
    grid(ctx, w, h);
    if (!sim) return;
    const layer = $('layer-select').value, stroke = $('stroke-select').value;
    const include = item => (layer === 'all' || String(item.layer_id) === layer) &&
                            (stroke === 'all' || String(item.stroke_id) === stroke);
    const original = sim.original_strokes.filter(include);
    const stage = $('debug-stage').value;
    const optimized = (stage==='PRINTABLE' ? sim.printable_path : sim.final_strokes).filter(include);
    const all = [...original, ...optimized];
    const bounds = [Infinity, -Infinity, Infinity, -Infinity, Infinity, -Infinity];
    for (const path of all) for (const point of path.points)
      for (let axis = 0; axis < 3; axis++) {
        bounds[axis * 2] = Math.min(bounds[axis * 2], point[axis]);
        bounds[axis * 2 + 1] = Math.max(bounds[axis * 2 + 1], point[axis]);
      }
    if (!Number.isFinite(bounds[0])) return;
    const project = projection(bounds, w, h);
    const compare = stage==='RAW' || stage==='CLASSIFIED' ? 'original' : stage==='PRINTABLE' || stage==='ORIENTATION' ? 'optimized' : $('path-compare').value;
    if (compare !== 'optimized')
      for (const path of original)
        drawPath(ctx, path.points, project, path.cordon ? '#8874ae' : path.deposition ? '#68737a' : '#3f5867', 2, !path.deposition);
    if (compare !== 'original')
      for (const path of optimized)
        drawPath(ctx, path.points, project, path.cordon ? '#af92ef' : path.deposition ? '#ed8e52' : '#557587', path.cordon?3:1.6, !path.deposition);
    if(compare!=='original'){
      const colors={SOURCE_G2G3:'#81c9e0',CORNER_FILLET:'#e7cc6d',FREE_CURVE_FIT:'#80cfa6'};
      for(const p of (sim.machine_primitives||[]).filter(include))
        drawPath(ctx,primitivePoints(p),project,$('show-native-arcs').checked ? colors[p.arc_source] || (p.metadata?.fallback?'#dca0d6':'#ed8e52') : '#ed8e52',p.type==='ARC'?2.5:1.6,p.type==='TRAVEL');
    }
    const dots=(points,color,radius=2)=>{ctx.fillStyle=color;for(const p of points){const [x,y]=project(p);ctx.beginPath();ctx.arc(x,y,radius,0,Math.PI*2);ctx.fill();}};
    if($('show-raw-points').checked)for(const p of original)dots(p.points,'#8c969c');
    if($('show-optimized-points').checked)dots(sim.samples.filter(s=>s.deposition_active&&include(s)).map(s=>[s.X,s.Y,s.Z]),'#f8b581');
    if($('show-legacy').checked&&state.legacySimulation){
      for(const p of state.legacySimulation.final_strokes.filter(include)){drawPath(ctx,p.points,project,'#a495cd',1,true);dots(p.points,'#a495cd',1.2);}
    }
    if($('show-corners').checked){
      for(const f of sim.fillets.filter(include))if(f.vertex_xyz)dots([f.vertex_xyz],'#e97f87',4);
      for(const c of sim.corners.filter(include)){const p=sim.samples.find(s=>s.sample_id===c.sample_id);if(p)dots([[p.X,p.Y,p.Z]],'#e97f87',4);}
    }
    for (const fillet of sim.fillets.filter(f => f.status === 'PASS' && include(f) && $('show-fillets').checked)) {
      const arcPoints=sim.samples.filter(s=>s.stroke_id===fillet.stroke_id&&s.fillet_id===fillet.id).map(s=>[s.X,s.Y,s.Z]);
      drawPath(ctx,arcPoints,project,'#e7cc6d',2);
      const [x, y] = project(fillet.t2);
      ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2);
      ctx.fillStyle = '#f2ce65'; ctx.fill();
      if ($('show-debug').checked) {
        for (const [label, point, color] of [['V', fillet.vertex_xyz, '#b3b9bf'], ['T1', fillet.t1, '#80cfa6'], ['T2', fillet.t2, '#f2ce65']]) {
          const [px, py] = project(point);
          ctx.beginPath(); ctx.arc(px, py, 5, 0, Math.PI * 2); ctx.fillStyle = color; ctx.fill();
          ctx.font = '11px Segoe UI'; ctx.fillText(`${label} #${fillet.id}`, px + 7, py - 7);
        }
        const [cx, cy] = project([fillet.center[0], fillet.center[1], fillet.t1[2]]);
        ctx.strokeStyle = '#a9b4b8'; ctx.beginPath();
        ctx.moveTo(cx - 4, cy - 4); ctx.lineTo(cx + 4, cy + 4);
        ctx.moveTo(cx + 4, cy - 4); ctx.lineTo(cx - 4, cy + 4); ctx.stroke();
        ctx.fillStyle='#b3b9bf';ctx.fillText(`O · R ${fmt(fillet.radius_mm,2)}`,cx+8,cy+14);
      }
    }
    if(stage==='CLASSIFIED'){
      for(const span of sim.classified_geometry.filter(include)){
        const path=sim.design_path.find(s=>s.stroke_id===span.stroke_id);
        for(const item of span.vertices){
          const p=item.vertex_xyz||path?.points[item.point_index];if(!p)continue;
          const [x,y]=project(p);ctx.beginPath();ctx.arc(x,y,3,0,Math.PI*2);
          ctx.fillStyle=item.category==='SHARP_CORNER'?'#f18888':item.category==='SMOOTH_CURVE'?'#81c9b0':'#e8c674';ctx.fill();
        }
      }
    }
    if($('show-debug').checked){
      for(const seam of sim.safe_seams.filter(include)){
        if(seam.status!=='PASS')continue;
        const [x,y]=project(seam.selected_xyz);
        drawPath(ctx,[seam.guard_start_xyz,seam.guard_end_xyz],project,'#ab9ee3',4);
        ctx.fillStyle='#d3c8ff';ctx.font='11px Segoe UI';ctx.fillText(`SEAM · guard ${seam.corner_guard_mm} mm`,x+6,y-12);
      }
      for(const transition of sim.layer_transitions){
        if(transition.start_xyz)drawPath(ctx,[transition.start_xyz,transition.xyz],project,'#81c9e0',3,true);
      }
    }
    hitTargets=sim.samples.filter(s=>include(s)&&(s.emit||s.motion_type==='START')).map(s=>({sample:s,xy:project([s.X,s.Y,s.Z])}));
    for(const p of (sim.machine_primitives||[]).filter(p=>p.type==='ARC'&&include(p))){
      const q=primitivePose(p,.5), sample=sim.samples.find(s=>s.sample_id===p.sample_id);
      if(sample)hitTargets.push({sample,xy:project([q.X,q.Y,q.Z])});
    }
    if($('show-corners').checked)for(const f of sim.fillets.filter(include)){const sample=sim.samples.find(s=>s.sample_id===f.sample_id);if(sample&&f.vertex_xyz)hitTargets.push({sample,xy:project(f.vertex_xyz)});}
    const continuous = $('play-mode').value === 'continuous';
    const head = playbackAtCursor(playback, continuous ? playbackCursor : Math.floor(playbackCursor));
    const current = head?.entry.sample;
    $('timeline-label').textContent = head ? `${head.entryIndex + 1} / ${playback.entries.length} · ${fmt(head.seconds,1)} / ${fmt(playback.totalSeconds,1)} s` : 'Sin recorrido en esta selección';
    $('playback-hint').textContent = continuous
      ? `Continua: tiempo ideal por longitud y F; no incluye aceleración ni blending de la máquina. ${playback.estimatedCount ? `${playback.estimatedCount} movimientos G0, C-only o sin F usan un tiempo visual estimado. ` : ''}Velocidad visual ${$('play-speed').selectedOptions[0].textContent}.`
      : 'Por bloques: muestra los puntos del G-code uno a uno (2 bloques/s a 1×). El tiempo de pantalla en este modo no representa el tiempo de la impresora.';
    if (current) {
      if (compare !== 'original' && head.entry.duration > 0)
        drawPath(ctx, head.entry.primitive ? primitivePoints(head.entry.primitive,head.fraction) : [[head.entry.from.X,head.entry.from.Y,head.entry.from.Z], [head.pose.X,head.pose.Y,head.pose.Z]], project, '#8ce0c0', 4, !current.deposition_active);
      if (include(current)) nozzle(ctx, project, head.pose, sim.audit?.config?.nozzle);
      $('sample-id').value=current.sample_id;
      const f=sim.fillets.find(f=>f.stroke_id===current.stroke_id&&f.id===current.fillet_id);
      $('corner-detail').textContent=f?`Corner #${f.id} · Capa ${f.layer_id} · Giro ${fmt(f.turn_angle_deg)}° · R elegido ${fmt(f.radius_mm)} mm / preferido ${fmt(f.radius_preferred_mm)} / máximo ${fmt(f.radius_max_feasible_mm)}
T1 ${f.t1.map(x=>fmt(x)).join(', ')} · T2 ${f.t2.map(x=>fmt(x)).join(', ')} · Centro ${f.center.map(x=>fmt(x)).join(', ')}
Máquina: ${f.machine_block_count??f.sample_count} bloques · Pantalla: ${f.display_sample_count??f.sample_count} puntos · Legacy 2°: ${f.legacy_sample_count} bloques · Error de cuerda visual ${fmt(f.max_chord_error_mm,5)} mm · Error C ${fmt(f.max_orientation_interp_error_deg,5)}° · ω requerida ${fmt(f.required_omega_C_deg_s)} °/s · ${f.kinematic_status}`:'';
      const visual = continuous && head.fraction < 1 ? `Posición visual · Tramo ${fmt(head.fraction*100,1)} % · X ${fmt(head.pose.X)} Y ${fmt(head.pose.Y)} Z ${fmt(head.pose.Z)} C ${fmt(head.pose.C_deg)}°\nDestino del bloque: ` : '';
      $('sample-detail').textContent = `${visual}Muestra ${current.sample_id} · Línea ${current.source_line} · Movimiento origen ${current.source_motion_id ?? '—'} · Capa ${current.layer_id ?? '—'} / trazo ${current.stroke_id ?? '—'}\nX ${fmt(current.X)}  Y ${fmt(current.Y)}  Z ${fmt(current.Z)}  C ${fmt(current.C_deg, 3)}° · F origen ${fmt(current.feed_source,2)} / final ${fmt(current.feed_mm_min,2)} / recomendada ${fmt(current.feed_recommended,2)} mm/min · κ ${fmt(current.curvature_1_mm,6)} 1/mm\nError geometría ${fmt(current.geometry_error_mm,6)} mm · Error C ${fmt(current.orientation_error_deg,6)}° · ${current.generated_reason} · ${current.primitive_type ?? current.motion_type} ${current.primitive_id ?? ''} · ${current.emit?'bloque exportado':'estado de referencia'}`;
      const p=head.entry.primitive;
      if(p?.type==='ARC') $('sample-detail').textContent+=`\n${p.direction==='CCW'?'G3':'G2'} · ${p.arc_source} · R ${fmt(p.radius,4)} mm · Centro ${p.center_xyz.map(v=>fmt(v,4)).join(', ')}\nI ${fmt(p.I,4)} J ${fmt(p.J,4)} · Barrido ${fmt(p.sweep_deg)}° · C ${fmt(p.start_c)} → ${fmt(p.end_c)}° · ω ${fmt(p.required_c_velocity_deg_s)} °/s\nError ajuste ${fmt(p.metadata.max_fit_error_mm,6)} mm · Error C ${fmt(p.metadata.max_orientation_fit_error_deg,6)}° · Span ${p.metadata.source_span??'geometría original'}`;
    } else {
      $('sample-detail').textContent = 'Sin movimientos para la capa o el trazo seleccionado.';
      $('corner-detail').textContent = '';
    }
    const detail=$('stage-detail');detail.classList.toggle('hidden',!$('show-debug').checked&&stage==='SAMPLED');
    detail.textContent=stage==='RAW'?sim.raw_program:stage==='CLASSIFIED'?JSON.stringify(sim.classified_geometry.filter(include),null,2):JSON.stringify({state:stage,conversion_id:sim.conversion_id,stages:sim.stages,sample:current},null,2);
  }

  function jumpToFillet(fillet) {
    inspect(fillet.sample_id);
  }

  function inspect(sampleId){
    if(!state.simulation)return;
    const index=state.simulation.samples.findIndex(s=>s.sample_id===sampleId);
    if(index<0)return;
    const s=state.simulation.samples[index];
    $('layer-select').value=s.layer_id==null?'all':String(s.layer_id);
    $('stroke-select').value=s.stroke_id==null?'all':String(s.stroke_id);
    refreshPlayback();
    let cursor = playback.entries.findIndex(e=>e.index===index);
    // Inspection may request a metadata sample not used during playback.
    if (cursor < 0) {
      $('start-mode').value = 'program';
      refreshPlayback();
      cursor = playback.entries.findIndex(e=>e.index===index);
    }
    playbackCursor = Math.max(0,cursor);
    $('timeline').value=playbackCursor;
    camera.focus=[s.X,s.Y,s.Z];camera.zoom=2;camera.panX=0;camera.panY=0;
    draw(state.simulation);
  }

  $('layer-select').onchange = refreshPlayback;
  $('stroke-select').onchange = refreshPlayback;
  $('path-compare').onchange = () => draw(state.simulation);
  $('view-2d').onclick = () => setView('2d');
  $('view-3d').onclick = () => setView('3d');
  $('camera-reset').onclick = () => resetCamera('iso');
  $('camera-top').onclick = () => resetCamera('top');
  $('camera-front').onclick = () => resetCamera('front');
  $('camera-side').onclick = () => resetCamera('side');
  $('camera-iso').onclick = () => resetCamera('iso');
  $('camera-fit').onclick = fitCamera;
  $('camera-pan').onclick = () => {
    interactionMode = interactionMode === 'orbit' ? 'pan' : 'orbit';
    $('camera-pan').setAttribute('aria-pressed', String(interactionMode === 'pan'));
    $('camera-pan').textContent = interactionMode === 'pan' ? 'Modo órbita' : 'Modo pan';
  };
  $('start-mode').onchange = setStart;
  for(const id of ['show-raw-points','show-optimized-points','show-corners','show-fillets','show-nozzle','show-legacy','show-native-arcs'])$(id).onchange=()=>draw(state.simulation);
  $('show-vectors').onchange = () => draw(state.simulation);
  $('show-debug').onchange = () => draw(state.simulation);
  $('debug-stage').onchange = () => draw(state.simulation);
  $('inspect-sample').onclick = () => inspect(Number($('sample-id').value));
  $('timeline').oninput = () => {
    pausePlayback();
    playbackCursor = Number($('timeline').value);
    draw(state.simulation);
  };
  $('play-mode').onchange = () => {
    pausePlayback();
    if ($('play-mode').value === 'blocks') playbackCursor = Math.round(playbackCursor);
    $('timeline').step = $('play-mode').value === 'blocks' ? '1' : '0.001';
    $('timeline').value = playbackCursor;
    draw(state.simulation);
  };
  $('play-speed').onchange = () => {lastFrameTime = null; draw(state.simulation);};
  $('play-button').onclick = () => {
    if (!state.simulation || playback.entries.length < 2) return;
    if (state.playing) {pausePlayback(); return;}
    if (playbackCursor >= playback.entries.length - 1) playbackCursor = 0;
    state.playing = true;
    lastFrameTime = null;
    $('play-button').textContent = 'Ⅱ';
    $('play-button').setAttribute('aria-label', 'Pausar');
    state.timer = requestAnimationFrame(animate);
  };
  const canvas = $('sim-canvas');
  canvas.addEventListener('contextmenu', event => event.preventDefault());
  canvas.addEventListener('pointerdown', event => {
    if (state.view !== '3d') return;
    event.preventDefault();
    drag = {x: event.clientX, y: event.clientY, startX:event.clientX,startY:event.clientY,
            mode: interactionMode === 'pan' || event.button === 2 || event.button === 1 || event.shiftKey ? 'pan' : 'orbit'};
    canvas.setPointerCapture(event.pointerId);
  });
  canvas.addEventListener('pointermove', event => {
    if (!drag || state.view !== '3d') return;
    const dx = event.clientX - drag.x, dy = event.clientY - drag.y;
    if (drag.mode === 'pan') {camera.panX += dx; camera.panY += dy;}
    else {
      camera.yaw += dx * .008;
      camera.elevation = Math.max(-Math.PI / 2 + .03,
                                  Math.min(Math.PI / 2, camera.elevation + dy * .008));
    }
    drag.x = event.clientX; drag.y = event.clientY;
    draw(state.simulation);
  });
  canvas.addEventListener('pointerup',event=>{
    if(!drag || Math.hypot(event.clientX-drag.startX,event.clientY-drag.startY)<4){
      const box=canvas.getBoundingClientRect();let best=null,distance=14;
      for(const hit of hitTargets){const d=Math.hypot(hit.xy[0]-(event.clientX-box.left),hit.xy[1]-(event.clientY-box.top));if(d<distance){best=hit;distance=d;}}
      if(best)inspect(best.sample.sample_id);
    }
    drag=null;
  });
  canvas.addEventListener('pointercancel',()=>{drag=null;});
  canvas.addEventListener('wheel', event => {
    if (state.view !== '3d') return;
    event.preventDefault();
    camera.zoom = Math.max(.1, Math.min(20, camera.zoom * Math.exp(-event.deltaY * .001)));
    draw(state.simulation);
  }, {passive: false});

  return {draw, fillSelectors, jumpToFillet, inspect,
          cameraSnapshot: () => ({...camera})};
}
