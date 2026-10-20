import {createSimulator} from './simulation.js';
const $ = id => document.getElementById(id);
const state = {file:null, source:'', parsed:null, result:null, simulation:null, page:'archivo', view:'2d', playing:false, timer:null, busy:false};
const nav = [...document.querySelectorAll('#navigation button')];
function page(name){state.page=name;nav.forEach(b=>b.classList.toggle('active',b.dataset.page===name));document.querySelectorAll('.page').forEach(p=>p.classList.toggle('active',p.id===`page-${name}`));$('breadcrumb').textContent=`PROYECTO / ${name.toUpperCase()}`;if(name==='simulacion')drawSimulation();if(name==='auditoria')drawChart();}
nav.forEach(button=>button.onclick=()=>page(button.dataset.page));
const number=id=>Number($(id).value);
const optional=id=>$(id).value===''?null:Number($(id).value);
$('curve-reference-model').onchange=()=>{
  const parametric=$('curve-reference-model').value==='PARAMETRIC_HERMITE';
  $('tangent-model-label').value=parametric?'Derivada de la curva paramétrica':'Centrada por longitud de arco';
  $('angular-model-label').value=parametric?'Tangente XYZ / C unificada':'PCHIP sin overshoot';
  if(parametric){
    for(const id of ['xyz-decimals','c-decimals','ijk-decimals'])$(id).value=Math.max(number(id),6);
  }
};
function config(){return {geometry:{mode:$('geometry-mode').value,max_xy_deviation_mm:number('xy-tolerance')},corner_smoothing:{enabled:$('geometry-mode').value==='SMOOTH_CORNERS',detection_threshold_deg:number('fillet-threshold'),radius_mode:$('radius-mode').value,radius_mm:number('fillet-radius'),max_path_deviation_mm:optional('fillet-deviation'),preferred_radius_nozzle_factor:number('radius-factor'),max_radius_mm:optional('radius-max'),min_segment_remaining_mm:number('fillet-remaining'),max_delta_c_deg:number('max-delta'),max_chord_error_mm:number('fillet-chord-error')},orientation:{alignment_offset_deg:number('alignment'),calibration_offset_deg:number('calibration'),symmetry_period_deg:number('symmetry'),home_aligned_start:$('home-aligned-start').checked},corner_detection:{smooth_turn_max_deg:number('smooth'),sharp_corner_min_deg:number('sharp')},corner_planner:{strategy:$('strategy').value,max_pretransition_mm:number('pretransition'),max_fraction_of_incoming_segment:number('fraction'),allow_c_only_at_sharp_corner:$('allow-c-only').checked},sampling:{max_delta_c_deg:number('max-delta'),max_orientation_interp_error_deg:number('max-error'),max_linear_step_mm:number('max-step')},kinematics:{max_c_velocity_deg_s:optional('max-velocity'),max_c_acceleration_deg_s2:optional('max-acceleration')},serialization:{xyz_decimals:number('xyz-decimals'),c_decimals:number('c-decimals'),feed_decimals:number('f-decimals')}};}
function extendedConfig(){
  const value=config();
  value.simplification={enabled:$('simplify-enabled').checked,geometry_tolerance_mm:number('simplify-geometry'),orientation_tolerance_deg:number('simplify-angle')};
  value.layer_transition={safe_seam:$('safe-seam').checked,corner_guard_mm:number('corner-guard'),alternate_open_paths:$('alternate-open-paths').checked};
  value.process={feed_policy:$('feed-policy').value,experimental_feed_factor:number('feed-factor')};
  value.orientation.mode='alignment';
  value.nozzle={length_mm:number('nozzle-length')};
  value.z_conversion={enabled:$('z-enabled').checked,first_layer_z_mm:number('z-first'),subsequent_layer_height_mm:number('z-height')};
  value.sampling.mode='adaptive_global_curves';
  value.sampling.geometry_error_mm=number('simplify-geometry');
  value.sampling.preferred_min_block_length_mm=number('micro-length');
  value.native_arcs={enabled:$('native-arcs-enabled').checked};
  value.arc_output={format:$('arc-output-format').value};
  value.arc_fit={enabled:$('arc-fit-enabled').checked,adaptive_refinement:$('arc-fit-refinement').checked,
    reference_model:$('curve-reference-model').value,
    geometry_tolerance_mm:number('arc-fit-geometry'),
    radial_tolerance_mm:number('arc-fit-geometry'),tangent_tolerance_deg:number('arc-fit-angle'),
    orientation_tolerance_deg:number('arc-fit-angle'),min_radius_mm:number('arc-fit-radius')};
  value.serialization.ijk_decimals=number('ijk-decimals');
  value.serialization.radius_decimals=number('radius-decimals');
  return value;
}
$('feed-policy').onchange=()=>{$('feed-factor').disabled=$('feed-policy').value!=='ADAPTIVE_EXPERIMENTAL';};
async function api(path,body){const response=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const data=await response.json();if(!response.ok)throw Error(data.detail||`HTTP ${response.status}`);return data;}
function status(text,kind='neutral'){$('header-status').textContent=text;$('header-status').style.color=kind==='fail'?'#ed8787':kind==='warning'?'#f0b46e':'#75ce96';}
function escapeHtml(text){return String(text).replace(/[&<>"']/g,ch=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch]));}
function fmt(v,n=3){return v==null?'—':Number(v).toFixed(n);}
async function loadFile(file){if(!file||state.busy)return;state.file=file;state.source=await file.text();state.result=null;state.simulation=null;state.legacySimulation=null;$('legacy-status').textContent='';$('header-file').textContent=file.name.toUpperCase();$('file-summary').textContent='Analizando archivo…';$('parse-tag').textContent='ANALIZANDO';$('parse-tag').className='pill neutral';$('convert-button').disabled=true;$('downloads').classList.add('hidden');page('archivo');try{state.parsed=await api('/api/files/parse',{source:state.source,filename:file.name,config:extendedConfig()});renderFile();$('convert-button').disabled=false;$('convert-detail').textContent=`${state.parsed.lines} líneas · ${state.parsed.layers} capas · ${state.parsed.strokes} trazos`;status('ARCHIVO LISTO');}catch(e){state.parsed=null;$('file-summary').innerHTML=`<div class="message-box error">${escapeHtml(e.message)}</div>`;$('parse-tag').textContent='ERROR DE PARSER';$('parse-tag').className='pill fail';status('ERROR','fail');}}
$('load-example').onclick=async()=>{try{const response=await fetch('/examples/cuadrado.ngc');if(!response.ok)throw Error('HTTP '+response.status);const source=await response.text();await loadFile(new File([source],'cuadrado.ngc',{type:'text/plain'}));}catch(error){status('ERROR','fail');$('file-summary').textContent=error.message;}};
$('load-organic-example').onclick=async()=>{try{
  const response=await fetch('/examples/contorno_organico.ngc');if(!response.ok)throw Error('HTTP '+response.status);
  const source=await response.text();$('curve-reference-model').value='PARAMETRIC_HERMITE';$('curve-reference-model').onchange();
  await loadFile(new File([source],'contorno_organico.ngc',{type:'text/plain'}));
}catch(error){status('ERROR','fail');$('file-summary').textContent=error.message;}};
function renderFile(){const p=state.parsed;const items=[['ARCHIVO',p.filename],['TAMAÑO',`${(p.bytes/1024).toFixed(1)} KB`],['LÍNEAS',p.lines],['CAPAS',p.layers],['TRAZOS',p.strokes],['G0',p.counts.G0],['G1',p.counts.G1],['FEEDS',p.feeds_mm_min.map(x=>fmt(x,1)).join(', ')||'—'],['RANGO X',`${fmt(p.ranges.X[0])} → ${fmt(p.ranges.X[1])}`],['RANGO Y',`${fmt(p.ranges.Y[0])} → ${fmt(p.ranges.Y[1])}`],['RANGO Z',`${fmt(p.ranges.Z[0])} → ${fmt(p.ranges.Z[1])}`],['G2 / G3',`${p.counts.G2} / ${p.counts.G3}`]];$('file-summary').className='';$('file-summary').innerHTML=`<div class="summary-grid">${items.map(([label,value])=>`<div class="stat ${String(value).length>16?'wide':''}"><small>${label}</small><strong>${escapeHtml(value)}</strong></div>`).join('')}</div>${p.warnings.length?`<div class="warnings">${p.warnings.length} avisos · ${escapeHtml(p.warnings.slice(0,3).join(' · '))}</div>`:''}`;$('parse-tag').textContent='INTERPRETADO';$('parse-tag').className='pill pass';}
$('select-file').onclick=e=>{e.stopPropagation();$('file-input').click()};$('dropzone').onclick=()=>$('file-input').click();$('dropzone').onkeydown=e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();$('file-input').click()}};$('file-input').onchange=e=>loadFile(e.target.files[0]);const drop=$('dropzone');for(const name of ['dragenter','dragover'])drop.addEventListener(name,e=>{e.preventDefault();drop.classList.add('drag')});for(const name of ['dragleave','drop'])drop.addEventListener(name,e=>{e.preventDefault();drop.classList.remove('drag')});drop.addEventListener('drop',e=>loadFile(e.dataTransfer.files[0]));
function reviewDecisions(review){
  const dialog=$('closure-dialog');
  const cordon=review.cordon;
  const start=cordon.preview_start_xyz, seam=cordon.contour_start_xyz;
  $('cordon-stage').classList.remove('hidden');
  $('closure-stage').classList.add('hidden');
  dialog.setAttribute('aria-labelledby','closure-title');
  dialog.setAttribute('aria-describedby','closure-description');
  $('cordon-add').disabled=!cordon.can_add;
  $('cordon-keep').disabled=!cordon.can_keep;
  $('cordon-omit').disabled=!cordon.can_omit;
  $('cordon-keep').textContent=cordon.can_keep&&!cordon.can_add?'Mantener recorrido':'Mantener cordón';
  $('cordon-details').innerHTML=`<div><small>INICIO DE LA FIGURA</small><strong>${seam?seam.slice(0,2).map(v=>fmt(v,1)).join(', '):'—'}</strong></div><div><small>TRAMO DE ENTRADA ACTUAL</small><strong>${cordon.existing?`${fmt(cordon.existing_length_mm,1)} mm · cordón`:cordon.source_entry_length_mm?`${fmt(cordon.source_entry_length_mm,1)} mm · aproximación`:'No detectado'}</strong></div><div class="seam">${start?`Nuevo cordón: ${fmt(cordon.length_mm,0)} mm a 45° desde X ${fmt(start[0],2)}, Y ${fmt(start[1],2)} (${cordon.preview_direction}) hasta el inicio. Un G0 llega al punto del cordón; el G1 recorre los 65 mm mientras alimentas la mezcla manualmente.`:escapeHtml(cordon.reason)}</div>`;
  $('cordon-warning').textContent=!cordon.can_add?cordon.reason:cordon.negative_xy?`El punto propuesto tiene X o Y negativos. Comprueba que tu máquina alcance X ${fmt(start[0],2)}, Y ${fmt(start[1],2)} antes de usar el G-code.`:cordon.existing&&cordon.existing_length_mm>70?`Mantener cordón conserva los ${fmt(cordon.existing_length_mm,1)} mm originales. Añadir lo ajusta a 65 mm.`:'El cordón aparece separado en la simulación para identificar qué concreto retirar.';
  $('closure-details').innerHTML=review.review_required?`<div><small>CIERRE PENDIENTE · CAPA 1</small><strong>${fmt(review.missing_first_layer_length_mm)} mm</strong></div><div><small>CAPAS DE REFERENCIA</small><strong>${review.reference_layers.length}</strong></div><div class="seam">Inicio XY de origen: ${review.seam_xy.map(v=>fmt(v,2)).join(', ')} mm<br>${review.affected_layer_count} capas con el mismo contorno.</div>`:'';
  dialog.returnValue='';
  return new Promise(resolve=>{
    let choice=null;
    const buttons=['cordon-add','cordon-keep','cordon-omit'].map(id=>$(id));
    const choose=e=>{
      choice=e.currentTarget.value;
      if(review.review_required){
        $('cordon-stage').classList.add('hidden');
        $('closure-stage').classList.remove('hidden');
        dialog.setAttribute('aria-labelledby','closure-title-step2');
        dialog.setAttribute('aria-describedby','closure-description-step2');
        $('closure-preserve').focus();
      }else dialog.close('proceed');
    };
    buttons.forEach(b=>b.addEventListener('click',choose));
    dialog.addEventListener('close',()=>{
      buttons.forEach(b=>b.removeEventListener('click',choose));
      const closing=review.review_required?dialog.returnValue:'preserve';
      resolve(choice&&['complete','preserve'].includes(closing)?{cordon:choice,closure:closing}:null);
    },{once:true});
    dialog.showModal();
  });
}
$('convert-button').onclick=async()=>{
  if(!state.source||state.busy)return;
  state.busy=true;
  const request={source:state.source,filename:state.file.name,config:extendedConfig()};
  const button=$('convert-button');button.disabled=true;
  for(const id of ['select-file','load-example','file-input'])$(id).disabled=true;
  $('downloads').classList.add('hidden');
  $('convert-state').textContent='Revisando inicio y cierre…';
  $('convert-detail').textContent='Preparando las opciones del cordón y las capas';
  status('REVISANDO','warning');
  try{
    const review=await api('/api/files/closure-analysis',request);
    $('convert-state').textContent='Pendiente de elección';
    const decisions=await reviewDecisions(review);
    if(!decisions){
      $('convert-state').textContent='Conversión cancelada';
      $('convert-detail').textContent='Puedes volver a convertir y elegir cómo tratar el cordón y el cierre.';
      status('ARCHIVO LISTO');return;
    }
    request.config.layer_completion={decision:decisions.closure};
    request.config.cordon={mode:decisions.cordon,length_mm:65};
    $('convert-state').textContent='Convirtiendo…';
    $('convert-detail').textContent='Planificación de C y auditoría en curso';
    status('PROCESANDO','warning');
    state.legacySimulation=null;$('legacy-status').textContent='';
    const result=await api('/api/convert',request);
    const response=await fetch(`/api/simulate/${result.id}`);
    if(!response.ok)throw Error(`HTTP ${response.status}`);
    state.simulation=await response.json();state.result=result;
    $('convert-state').textContent=result.status;
    $('convert-detail').textContent=`${result.sample_count} muestras · ${result.report.counts.layers} capas · ${result.report.counts.fillets} fillets`;
    status(result.status,result.status==='FAIL'?'fail':result.status.includes('WARNING')?'warning':'pass');
    setDownloads();renderAudit();fillSelectors();page(result.status==='FAIL'?'auditoria':'simulacion');
  }catch(e){
    $('convert-state').textContent='Error en conversión';$('convert-detail').textContent=e.message;status('ERROR','fail');
  }finally{
    state.busy=false;button.disabled=false;
    for(const id of ['select-file','load-example','file-input'])$(id).disabled=false;
  }
};
async function chooseGcodeDestination(event){
  const info=$('download-status');
  if(typeof window.showSaveFilePicker!=='function'){
    info.textContent='Este navegador descargará el archivo según su configuración. Para elegir carpeta, usa clic derecho → Guardar enlace como, o activa “Preguntar dónde guardar cada archivo” en el navegador.';
    return;
  }
  event.preventDefault();
  const link=event.currentTarget;
  const url=link.href, filename=link.download;
  try{
    const handle=await window.showSaveFilePicker({suggestedName:filename,types:[{description:'G-code XYZC',accept:{'text/plain':['.ngc','.gcode']}}]});
    const response=await fetch(url);
    if(!response.ok)throw Error(`HTTP ${response.status}`);
    const writable=await handle.createWritable();
    try{await writable.write(await response.blob());await writable.close();}
    catch(error){await writable.abort().catch(()=>{});throw error;}
    info.textContent=`Guardado: ${handle.name}`;
  }catch(error){
    info.textContent=error.name==='AbortError'?'Guardado cancelado.':`No se pudo guardar (${error.message}). Usa clic derecho → Guardar enlace como.`;
  }
}
function setDownloads(){
  const id=state.result.id;
  for(const [element,path] of [['download-gcode','gcode'],['download-report','report'],['download-csv','samples']]){
    const el=$(element);
    el.href=`/api/conversion/${id}/${path}`;
    el.download=path==='gcode'?`${(state.file.name.replace(/\.[^.]+$/,'')||'pieza')}_xyzc.ngc`:path==='report'?'conversion_report.json':'samples.csv';
    el.style.display=path==='gcode'&&state.result.status==='FAIL'?'none':'';
  }
  $('download-gcode').onclick=chooseGcodeDestination;
  $('download-status').textContent='';
  $('downloads').classList.remove('hidden');
}
const simulator = createSimulator(state, fmt);
function fillSelectors(){simulator.fillSelectors(state.simulation);}
function drawSimulation(){simulator.draw(state.simulation);}
function resize(canvas){const box=canvas.getBoundingClientRect();const ratio=devicePixelRatio||1;canvas.width=Math.max(1,Math.round(box.width*ratio));canvas.height=Math.max(1,Math.round(box.height*ratio));const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);return {ctx,w:box.width,h:box.height};}
function drawGrid(ctx,w,h){ctx.strokeStyle='#252d31';ctx.lineWidth=1;for(let x=30;x<w;x+=60){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke()}for(let y=30;y<h;y+=60){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke()}}
function renderAudit(){
  const r=state.result.report;
  const labels={Geometry:'GEOMETRÍA',Corners:'ESQUINAS',Curves:'CURVAS',Layers:'CAPAS',Orientation:'ORIENTACIÓN',Kinematics:'CINEMÁTICA',Sampling:'MUESTREO',Serialization:'SERIALIZACIÓN'};
  const cards=Object.entries(labels).map(([key,label])=>[label,r.validators[key].status]);
  $('audit-summary').innerHTML=cards.map(([label,value])=>'<div class="audit-card '+(value==='PASS'?'pass':value==='FAIL'?'fail':value==='WARNING'?'warning':'neutral')+'"><small>'+label+'</small><strong>'+value+'</strong></div>').join('');
  const geom='<div class="message-box">Modo '+escapeHtml(r.geometry.mode)+' · XY modificado: '+(r.geometry.modified_locally?'sí':'no')+' · desviación máxima '+fmt(r.geometry.max_path_deviation_mm,3)+' mm · diferencia entre capas repetidas '+fmt(r.geometry.max_interlayer_xy_drift_mm,6)+' mm</div>';
  $('layer-audit').className='table-wrap';
  $('layer-audit').innerHTML='<table><thead><tr><th>CAPA</th><th>Z NOMINAL</th><th>TRAYECTO</th><th>DESV. XY MÁX.</th><th>DRIFT CENTROIDE</th><th>ESTADO</th></tr></thead><tbody>'+r.layers.map(x=>'<tr class="clickable" data-layer="'+x.layer_id+'"><td>'+x.layer_id+'</td><td>'+fmt(x.nominal_z)+'</td><td>'+fmt(x.path_length_mm,1)+' mm</td><td>'+fmt(x.max_xy_deviation_mm,4)+' mm</td><td>'+fmt(x.layer_xy_drift_mm,4)+' mm</td><td>'+x.status+'</td></tr>').join('')+'</tbody></table>';
  $('layer-audit').querySelectorAll('tr[data-layer]').forEach(row=>row.onclick=()=>{$('layer-select').value=row.dataset.layer;page('simulacion')});
  const active=r.fillets.filter(x=>x.status==='PASS');
  const rejected=r.fillets.filter(x=>x.status!=='PASS');
  $('corner-audit').className='table-wrap';
  let rows=active.map((f,i)=>'<tr class="clickable" data-fillet="'+r.fillets.indexOf(f)+'"><td>#'+f.id+'</td><td>'+f.layer_id+' / '+f.stroke_id+'</td><td>'+fmt(f.turn_angle_deg,1)+'°</td><td>R '+fmt(f.radius_mm,2)+' mm</td><td>'+fmt(f.t1[0],2)+', '+fmt(f.t1[1],2)+' → '+fmt(f.t2[0],2)+', '+fmt(f.t2[1],2)+'</td><td>'+fmt(f.max_path_deviation_mm,3)+' mm</td><td>'+fmt(f.corner_orientation_error_at_arc_exit_deg,3)+'°</td><td>'+fmt(f.corner_settling_distance_after_arc_mm,3)+' mm</td><td>'+f.status+'</td></tr>').join('');
  rows+=rejected.map(f=>'<tr><td>#'+f.id+'</td><td>'+f.layer_id+' / '+f.stroke_id+'</td><td>'+fmt(f.turn_angle_deg,1)+'°</td><td>R '+fmt(f.radius_mm,2)+' mm</td><td colspan="4">'+escapeHtml(f.reason||'Sin fillet')+'</td><td>'+f.status+'</td></tr>').join('');
  $('corner-audit').innerHTML='<table><thead><tr><th>#</th><th>CAPA / TRAZO</th><th>GIRO</th><th>RADIO</th><th>T1 → T2 (XY)</th><th>DESV. MÁX.</th><th>ERROR C EN T2</th><th>SETTLING</th><th>ESTADO</th></tr></thead><tbody>'+rows+'</tbody></table>';
  $('corner-audit').innerHTML+=active.map(f=>{
    const raw=state.simulation.original_strokes.find(s=>s.stroke_id===f.stroke_id);
    const vertex=raw?.points[f.point_index];
    return '<details class="fillet-details"><summary>Fillet #'+f.id+' · capa '+f.layer_id+' · trazo '+f.stroke_id+'</summary><div>'+
      'Vértice original: '+(vertex?fmt(vertex[0],3)+', '+fmt(vertex[1],3):'—')+
      ' · Centro: '+fmt(f.center[0],3)+', '+fmt(f.center[1],3)+
      ' · Barrido: '+fmt(f.sweep_deg,2)+'° · C inicial/final: '+fmt(f.c_start_deg,3)+'° / '+fmt(f.c_end_deg,3)+'°'+
      ' · Muestras generadas: '+f.generated_segments+' · Estado: '+f.status+'</div></details>';
  }).join('');
  $('corner-audit').querySelectorAll('tr[data-fillet]').forEach(row=>row.onclick=()=>{simulator.jumpToFillet(r.fillets[Number(row.dataset.fillet)]);page('simulacion')});
  if(r.corners.length){
    $('corner-audit').innerHTML+='<table><thead><tr><th>VÉRTICE</th><th>LÍNEA</th><th>TIPO</th><th>ESTRATEGIA</th><th>ERROR C</th><th>ESTADO</th></tr></thead><tbody>'+r.corners.map(c=>`<tr class="clickable" data-sample="${c.sample_id}"><td>${c.id}</td><td>${c.source_line}</td><td>${c.category}</td><td>${c.strategy}</td><td>${fmt(c.orientation_error_at_vertex_deg)}°</td><td>${c.status}</td></tr>`).join('')+'</tbody></table>';
  }
  // All statuses and selection targets come from the backend report.
  $('corner-audit').querySelectorAll('tr[data-fillet]').forEach(row=>row.onclick=()=>{page('simulacion');simulator.jumpToFillet(r.fillets[Number(row.dataset.fillet)])});
  $('corner-audit').querySelectorAll('tr[data-sample]').forEach(row=>row.onclick=()=>{page('simulacion');simulator.inspect(Number(row.dataset.sample))});
  $('messages').innerHTML=geom+r.issues.map(issue=>`<div class="message-box ${issue.status==='FAIL'?'error':''}">${escapeHtml(issue.message)}${issue.sample_id?`<button class="issue-link" data-sample="${issue.sample_id}">Ver muestra</button>`:''}</div>`).join('');
  $('messages').querySelectorAll('[data-sample]').forEach(button=>button.onclick=()=>{page('simulacion');simulator.inspect(Number(button.dataset.sample))});
  renderOptimization(r);
  renderCordonAudit(r.cordon);
  renderClosureAudit(r.layer_completion);
  drawChart();
}
function renderCordonAudit(c){
  const panel=$('cordon-report');
  if(!c){panel.className='hidden';return;}
  panel.className='';
  const labels={ADDED:'Cordón añadido',ADJUSTED:'Cordón original ajustado',KEPT:'Cordón original mantenido',OMITTED:'Sin cordón: entrada en G0',ABSENT:'Sin cordón: no había tramo de entrada',NO_EXISTING:'No había cordón que mantener',PRESERVED_SOURCE:'Recorrido de origen conservado',LEGACY:'Recorrido de origen'};
  const coords=c.actual_start_xyz&&c.actual_end_xyz?` · X ${fmt(c.actual_start_xyz[0],2)}, Y ${fmt(c.actual_start_xyz[1],2)} → X ${fmt(c.actual_end_xyz[0],2)}, Y ${fmt(c.actual_end_xyz[1],2)}`:'';
  const check=c.validation?` · Verificación ${c.validation.status} · unión al inicio ${fmt(c.validation.seam_gap_mm,6)} mm`:'';
  panel.innerHTML=`<div class="message-box">${labels[c.status]||escapeHtml(c.status)}${c.actual_length_mm!=null?` · ${fmt(c.actual_length_mm,2)} mm`:''}${coords}${check}</div>`;
}
function renderClosureAudit(c){
  const panel=$('closure-report');
  if(!c){panel.className='hidden';return;}
  panel.className='';
  if(!c.review_required){panel.innerHTML=`<div class="message-box">Revisión de cierre: ${escapeHtml(c.reason)}</div>`;return;}
  const completed=c.status==='COMPLETED';
  const summary=completed?`Cierre completado en ${c.affected_layer_count} capas · ${c.reassigned_return_count} regresos antes de subir · ${c.added_closure_count} cierre final añadido`:`Se eligió mantener el recorrido · cierre pendiente en la primera capa: ${fmt(c.missing_first_layer_length_mm)} mm`;
  const check=c.validation?` · Verificación: ${c.validation.status} · separación máxima ${fmt(c.validation.max_gap_mm,6)} mm`:'';
  panel.innerHTML=`<div class="message-box closure-summary">${summary}${check}</div><details class="fillet-details"><summary>Detalle del cierre por capa</summary><table><thead><tr><th>CAPA</th><th>Z ORIGEN</th><th>CIERRE</th><th>LÍNEA / REFERENCIA</th><th>ACCIÓN</th></tr></thead><tbody>${c.layers.map(x=>`<tr><td>${x.layer_index+1}</td><td>${fmt(x.source_z_mm)} mm</td><td>${fmt(x.closure_length_mm)} mm</td><td>${x.source_line} / ${x.reference_source_line}</td><td>${completed?(x.action==='ADD_FINAL_RETURN'?'Cierre final añadido':'Regreso antes de subir'):'Recorrido mantenido'}</td></tr>`).join('')}</tbody></table></details>`;
}
function renderOptimization(r){
  const d=r.density,z=r.z_conversion;
  $('density-report').textContent=`G1 ${d.output_G1_count} · G2 ${d.output_G2_count??0} · G3 ${d.output_G3_count??0} · Rectas G1 ${d.straight_G1_count} · Curvas G1 ${d.curve_G1_count} · Fillets nativos ${r.native_arcs.fillet_arc_count} · Curvas ajustadas ${r.native_arcs.fitted_arc_count} · Recorrido en arcos ${fmt(100*(r.native_arcs.native_arc_length_fraction??0),1)}% · Bloque mínimo ${fmt(d.min_block_length_mm)} mm · Microbloques < ${d.preferred_min_block_length_mm} mm: ${d.microblock_count}`;
  $('z-report').textContent=`Normalización Z ${z.enabled?'activa':'desactivada'} · Origen: ${z.source_layer_z_values.join(', ')} mm → Destino: ${z.target_layer_z_values.join(', ')} mm · Error de capa máximo ${fmt(z.max_layer_z_error_mm,6)} mm`;
  const m=r.metrics;
  const items=[['MOVIMIENTOS ENTRADA',m.input_motion_count],['MOVIMIENTOS SALIDA',m.output_motion_count],['COMPRESIÓN',fmt(m.compression_ratio,2)+'×'],['FILLETS',m.fillet_count],['ERROR GEOMETRÍA',fmt(r.simplification.max_geometry_error_mm,5)+' mm'],['ERROR ORIENTACIÓN',fmt(m.max_orientation_error_deg,5)+'°'],['ΔC MÁX.',fmt(m.max_delta_c_deg,3)+'°'],['ARCO → POLILÍNEA',fmt(m.max_fillet_polyline_deviation_mm,4)+' mm'],['VÉRTICE → ARCO',fmt(m.max_fillet_vertex_setback_mm,4)+' mm'],['ω C MÁX.',fmt(m.max_omega_c_deg_s,3)+' °/s'],['α C MÁX.',fmt(m.max_alpha_c_deg_s2,3)+' °/s²'],['DRIFT ENTRE CAPAS',fmt(m.max_layer_xy_drift_mm,7)+' mm'],['SAFE SEAM',m.safe_seam_status],['NaN / INFINITY',m.nan_count+' / '+m.infinity_count]];
  if(r.initial_c_reference)items.push(['INICIO DEL EJE C',r.initial_c_reference.mode==='HOME_ALIGNED_TANGENT'?'C0 · homing manual':'Ángulo absoluto']);
  items.push(['G2 / G3',`${m.output_G2_count} / ${m.output_G3_count}`],['COMPRESIÓN DE MÁQUINA',fmt(m.native_compression_ratio,2)+'×'],
    ['ERROR AJUSTE XY',fmt(r.native_arcs.max_fit_error_mm,6)+' mm'],['ERROR AJUSTE C',fmt(r.native_arcs.max_orientation_fit_error_deg,6)+'°'],
    ['G1 DE RESPALDO',r.native_arcs.fallback_G1_count],['RECORRIDO EN ARCOS',fmt(100*(r.native_arcs.native_arc_length_fraction??0),1)+'%']);
  if(r.curve_reconstruction?.applied)items.push(['REFERENCIA DE C','Tangente paramétrica'],
    ['ERROR TOTAL CERTIFICADO XY',fmt(r.curve_reconstruction.max_certified_geometry_error_mm,6)+' mm'],
    ['ERROR CERTIFICADO C',fmt(r.curve_reconstruction.max_certified_orientation_error_deg,6)+'°']);
  $('optimization-metrics').innerHTML=items.map(([label,value])=>`<div class="stat wide"><small>${label}</small><strong>${escapeHtml(value)}</strong></div>`).join('');
  $('conversion-identity').textContent=`Backend ${r.backend_version} · Conversión ${r.conversion_id}\nEntrada ${r.input_hash} · Configuración ${r.config_hash}`;
  const seams=r.safe_seams.map(s=>`<tr><td>${s.layer_id} / ${s.stroke_id}</td><td>${s.status}</td><td>${fmt(s.corner_distance_mm)} mm</td><td>${escapeHtml(s.reason)}</td></tr>`).join('');
  const openRows=(r.open_path_transitions||[]).map(s=>`<tr><td>${s.layer_id} / ${s.stroke_id}</td><td>${s.status}</td><td>${s.reversed?'INVERTIDO':'CONSERVADO'}</td><td>${s.transition_xy_held?'SÍ':'NO'}</td><td>${escapeHtml(s.reason)}</td></tr>`).join('');
  const openTable=openRows?`<table><thead><tr><th>CAPA / TRAZO ABIERTO</th><th>ESTADO</th><th>SENTIDO</th><th>XY/C FIJOS AL SUBIR</th><th>RAZÓN</th></tr></thead><tbody>${openRows}</tbody></table>`:'';
  const rows=r.feed_plan.primitives.slice(0,40).map(p=>`<tr><td>${p.primitive_id}</td><td>${p.source_line}</td><td>${fmt(p.feed_source,2)}</td><td>${fmt(p.max_feed_by_c_mm_min,2)}</td><td>${fmt(p.recommended_feed_mm_min,2)}</td><td>${escapeHtml(p.reason)}</td></tr>`).join('');
  $('process-audit').innerHTML=`<div class="message-box">Política: ${r.feed_plan.policy} · ${r.feed_plan.unknown_feed_segments} segmentos sin F conocida. El feed final de cada bloque está en el inspector y CSV.</div><table><thead><tr><th>CAPA / TRAZO</th><th>SEAM</th><th>DISTANCIA A ESQUINA</th><th>RAZÓN</th></tr></thead><tbody>${seams}</tbody></table>${openTable}<details class="fillet-details"><summary>Recomendaciones de feed · primeras ${Math.min(40,r.feed_plan.primitives.length)} primitivas</summary><table><thead><tr><th>PRIMITIVA</th><th>LÍNEA</th><th>F ORIGEN</th><th>F MÁX. POR C</th><th>F RECOMENDADA</th><th>RAZÓN</th></tr></thead><tbody>${rows}</tbody></table></details>`;
}
function drawChart(){
  const {ctx,w,h}=resize($('audit-chart'));ctx.fillStyle='#171c1f';ctx.fillRect(0,0,w,h);drawGrid(ctx,w,h);
  if(!state.simulation)return;
  const metric=$('chart-metric').value;
  const points=state.simulation.samples.filter(s=>s.emit&&Number.isFinite(s[metric]));
  if(points.length<2)return;
  let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity;
  for(const p of points){minX=Math.min(minX,p.s_global);maxX=Math.max(maxX,p.s_global);minY=Math.min(minY,p[metric]);maxY=Math.max(maxY,p[metric]);}
  const sx=(w-70)/Math.max(1,maxX-minX),sy=(h-55)/Math.max(.000001,maxY-minY);
  ctx.strokeStyle='#f0965e';ctx.lineWidth=1.6;ctx.beginPath();let previous=null;
  for(const p of points){const x=45+(p.s_global-minX)*sx,y=h-32-(p[metric]-minY)*sy;if(previous&&previous.stroke_id===p.stroke_id)ctx.lineTo(x,y);else ctx.moveTo(x,y);previous=p;}
  ctx.stroke();ctx.fillStyle='#a8b4b9';ctx.font='11px Segoe UI';ctx.fillText(fmt(maxY,2),6,22);ctx.fillText(fmt(minY,2),6,h-25);ctx.fillText(`${fmt(maxX,0)} mm`,w-78,h-10);
}
$('chart-metric').onchange=drawChart;window.addEventListener('resize',()=>{if(state.page==='simulacion')drawSimulation();if(state.page==='auditoria')drawChart()});

$('load-legacy').onclick=async()=>{
  if(!state.simulation)return;
  const button=$('load-legacy');button.disabled=true;$('legacy-status').textContent='Calculando en backend…';
  try{
    const source=state.simulation.raw_program;
    const profiles=await (await fetch('/api/config')).json();
    const profile=profiles.legacy;
    profile.z_conversion=state.result.report.config.z_conversion;
    profile.orientation=state.result.report.config.orientation;
    profile.layer_completion=state.result.report.config.layer_completion;
    profile.cordon=state.result.report.config.cordon;
    const result=await api('/api/convert',{source,config:profile});
    if(result.status==='FAIL')throw Error(result.report.errors.join('; '));
    const legacy=await (await fetch(`/api/simulate/${result.id}`)).json();
    if(state.simulation.raw_program!==source)return;
    state.legacySimulation=legacy;$('show-legacy').checked=true;
    $('legacy-status').textContent=`Legacy R10/2°: ${result.report.density.output_G1_count} G1 · Optimizado: ${state.result.report.density.output_G1_count} G1 · misma política Z`;
    drawSimulation();
  }catch(e){$('legacy-status').textContent=e.message;}finally{button.disabled=false;}
};
